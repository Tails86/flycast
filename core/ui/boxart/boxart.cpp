/*
	Copyright 2022 flyinghead
	Portions Copyright 2026 The Hollycast Authors

	This file is part of Flycast.

    Flycast is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 2 of the License, or
    (at your option) any later version.

    Flycast is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Flycast.  If not, see <https://www.gnu.org/licenses/>.
 */
#include "boxart.h"
#include "gamesdb.h"
#include "../game_scanner.h"
#include "oslib/oslib.h"
#include "oslib/storage.h"
#include "cfg/option.h"
#include "arcade_scraper.h"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cctype>
#include <cstring>
#include <memory>
#include <string_view>
#include <unordered_set>

namespace {

static constexpr size_t kOnlineBoxartBatchSize = 10;
static constexpr size_t kPhysicalBoxartBatchSize = 40;
static constexpr size_t kPhysicalBoxartWorkerCount = 4;

bool isSupportedBoxartExtension(const std::string& ext)
{
	std::string lowerExt = ext;
	std::transform(lowerExt.begin(), lowerExt.end(), lowerExt.begin(), [](unsigned char c) {
		return static_cast<char>(std::tolower(c));
	});
	return lowerExt == "png" || lowerExt == "jpg" || lowerExt == "jpeg" || lowerExt == "bmp";
}

bool isSupportedManualExtension(const std::string& ext)
{
	std::string lowerExt = ext;
	std::transform(lowerExt.begin(), lowerExt.end(), lowerExt.begin(), [](unsigned char c) {
		return static_cast<char>(std::tolower(c));
	});
	return isSupportedBoxartExtension(lowerExt) || lowerExt == "pdf" || lowerExt == "cbz" || lowerExt == "cbr";
}

std::string normalizeBoxartKey(const std::string& value)
{
	std::string out;
	out.reserve(value.size());
	bool prevSpace = true;
	for (unsigned char c : value)
	{
		if (c < 0x80 && std::isalnum(c))
		{
			out.push_back(static_cast<char>(std::tolower(c)));
			prevSpace = false;
		}
		else if (c >= 0x80)
		{
			out.push_back(static_cast<char>(c));
			prevSpace = false;
		}
		else if (!prevSpace)
		{
			out.push_back(' ');
			prevSpace = true;
		}
	}
	return trim_ws(out);
}

std::string makeBoxartKey(const std::string& filepath)
{
	return normalizeBoxartKey(get_file_basename(filepath));
}

GameBoxart makeMediaBoxart(const GameMedia& media)
{
	GameBoxart boxart;
	boxart.fileName = media.fileName;
	boxart.gamePath = media.path;
	boxart.name = media.name;
	boxart.searchName = media.gameName;
	boxart.arcade = media.arcade;
	return boxart;
}

bool isMissingBoxartFile(const GameBoxart& boxart)
{
	return !boxart.boxartPath.empty() && !file_exists(boxart.boxartPath);
}

bool isMissingScrapedBoxartFile(const GameBoxart& boxart)
{
	return boxart.scraped && !boxart.boxartUrl.empty() && (boxart.boxartPath.empty() || isMissingBoxartFile(boxart));
}

size_t findLastPathSeparator(const std::string& path)
{
#ifdef _WIN32
	// Windows accepts both separators in paths. POSIX treats backslash as a
	// valid filename character, so only split on '/' outside Windows.
	return path.find_last_of("/\\");
#else
	return path.find_last_of('/');
#endif
}

std::string parentPath(const std::string& path)
{
	const size_t pos = findLastPathSeparator(path);
	if (pos == std::string::npos)
		return {};
	return path.substr(0, pos + 1);
}

std::string fileNameFromPath(const std::string& path)
{
	const size_t pos = findLastPathSeparator(path);
	if (pos == std::string::npos)
		return path;
	return path.substr(pos + 1);
}

// Boxart database recovery may need to match copied artwork such as
// "Game (1).png" back to an older database entry that still points at
// "Game.png", or the other way around.
std::string removeCopySuffix(std::string name)
{
	const std::string ext = get_file_extension(name);
	if (ext.empty())
		return name;

	std::string base = get_file_basename(name);
	const size_t closeParen = base.rfind(')');
	const size_t openParen = base.rfind(" (");
	if (closeParen == base.length() - 1 && openParen != std::string::npos && openParen < closeParen)
	{
		bool allDigits = true;
		for (size_t i = openParen + 2; i < closeParen; i++)
			if (!std::isdigit(static_cast<unsigned char>(base[i])))
			{
				allDigits = false;
				break;
			}
		if (allDigits)
			base = base.substr(0, openParen);
	}
	return base + "." + ext;
}

std::string normalizePath(std::string path)
{
#ifdef _WIN32
	// Backslash is only a path separator on Windows. POSIX platforms can use it
	// in filenames, so do not rewrite it there.
	std::replace(path.begin(), path.end(), '\\', '/');
#endif
	std::transform(path.begin(), path.end(), path.begin(), [](unsigned char c) {
		return static_cast<char>(std::tolower(c));
	});
	return path;
}

bool isPathInDirectory(const std::string& path, const std::string& directory)
{
	const std::string normalizedPath = normalizePath(path);
	const std::string normalizedDirectory = normalizePath(directory);
	return normalizedPath.substr(0, normalizedDirectory.length()) == normalizedDirectory;
}

int boxartMergeScore(const GameBoxart& boxart)
{
	int score = 0;
	const bool hasBoxartFile = !boxart.boxartPath.empty() && file_exists(boxart.boxartPath);
	if (boxart.scraped && !boxart.boxartUrl.empty())
		score += hasBoxartFile ? 100 : 70;
	if (hasBoxartFile)
		score += 20;
	if (boxart.parsed)
		score += 10;
	if (boxart.scraped)
		score += 5;
	if (!boxart.overview.empty())
		score += 2;
	if (!boxart.name.empty())
		score += 1;
	return score;
}

void scrapePhysicalBoxart(std::vector<GameBoxart>& boxart, const std::string& saveDirectory)
{
	std::atomic<size_t> nextIndex{ 0 };
	const size_t workerCount = std::min(kPhysicalBoxartWorkerCount, boxart.size());
	std::vector<std::future<void>> workers;
	workers.reserve(workerCount);
	for (size_t i = 0; i < workerCount; i++)
	{
		workers.push_back(std::async(std::launch::async, [&boxart, &nextIndex, saveDirectory]() {
			OfflineScraper scraper;
			scraper.initialize(saveDirectory);
			while (true)
			{
				const size_t index = nextIndex.fetch_add(1);
				if (index >= boxart.size())
					break;
				scraper.scrape(boxart[index]);
			}
		}));
	}
	for (std::future<void>& worker : workers)
		worker.get();
}

int hexValue(char c)
{
	if (c >= '0' && c <= '9')
		return c - '0';
	if (c >= 'a' && c <= 'f')
		return c - 'a' + 10;
	if (c >= 'A' && c <= 'F')
		return c - 'A' + 10;
	return -1;
}

std::string decodeUriComponent(const std::string& value)
{
	// Android SAF encodes document IDs in URIs: "primary%3ARoms%2Fcovers" becomes "primary:Roms/covers".
	std::string decoded;
	decoded.reserve(value.size());
	for (size_t i = 0; i < value.size(); ++i)
	{
		if (value[i] == '%' && i + 2 < value.size())
		{
			const int hi = hexValue(value[i + 1]);
			const int lo = hexValue(value[i + 2]);
			if (hi >= 0 && lo >= 0)
			{
				decoded.push_back(static_cast<char>((hi << 4) | lo));
				i += 2;
				continue;
			}
		}
		decoded.push_back(value[i]);
	}
	return decoded;
}

std::string getAndroidDocumentId(const std::string& uri, const char* marker)
{
	// The shared media scanner uses this only for Android Storage Access Framework content:// URIs.
	const std::string decoded = decodeUriComponent(uri);
	const size_t markerPos = decoded.find(marker);
	if (markerPos == std::string::npos)
		return {};
	const size_t idStart = markerPos + std::strlen(marker);
	const size_t idEnd = std::strcmp(marker, "/tree/") == 0 ? decoded.find("/document/", idStart) : std::string::npos;
	return decoded.substr(idStart, idEnd == std::string::npos ? std::string::npos : idEnd - idStart);
}

std::string getFolderFirstComponent(const std::string& root, const std::string& path)
{
	if (root.find("content://") == 0 || path.find("content://") == 0)
	{
		const std::string rootId = getAndroidDocumentId(root, "/tree/");
		const std::string pathId = getAndroidDocumentId(path, "/document/");
		if (rootId.empty() || pathId.size() <= rootId.size() || pathId.compare(0, rootId.size(), rootId) != 0)
			return {};
		std::string relative = pathId.substr(rootId.size());
		while (!relative.empty() && relative.front() == '/')
			relative.erase(relative.begin());
		const size_t slash = relative.find('/');
		return slash == std::string::npos ? std::string{} : relative.substr(0, slash);
	}

	if (path.size() <= root.size() || path.compare(0, root.size(), root) != 0)
		return {};

	std::string relative = path.substr(root.size());
	while (!relative.empty() && (relative.front() == '/' || relative.front() == '\\'))
		relative.erase(relative.begin());
	const size_t slash = relative.find_first_of("/\\");
	return slash == std::string::npos ? std::string{} : relative.substr(0, slash);
}

struct MediaFolderAlias
{
	config::LibraryCoverMediaMode mode;
	std::vector<std::string_view> aliases;
};

std::string toLowerString(std::string_view value)
{
	std::string lower(value);
	std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char c) {
		return static_cast<char>(std::tolower(c));
	});
	return lower;
}

const MediaFolderAlias* findMediaFolderAlias(const std::string& folder)
{
	static const std::array<MediaFolderAlias, 9> mediaAliases{
		MediaFolderAlias {config::LibraryCoverMediaMode::MixImage,
				{"miximages", "miximage", "miximg", "miximgs", "mix", "mixrbv1", "image", "images"}},
		MediaFolderAlias {config::LibraryCoverMediaMode::Cover,
				{"covers", "cover", "box2dfront", "boxfront", "box-2d", "box2d"}},
		MediaFolderAlias {config::LibraryCoverMediaMode::Case,
				{"3dboxes", "3dbox", "box3d", "backcovers", "backcover", "boxtexture", "box-texture", "boxtextures", "case", "cases", "insert", "custom"}},
		MediaFolderAlias {config::LibraryCoverMediaMode::FanArt,
				{"fanart", "fanarts", "fan-art", "background", "backgrounds"}},
		MediaFolderAlias {config::LibraryCoverMediaMode::Screenshot,
				{"screenshots", "screenshot", "ss"}},
		MediaFolderAlias {config::LibraryCoverMediaMode::TitleScreen,
				{"titlescreens", "titlescreen", "title-screens", "title-screen", "sstitle"}},
		MediaFolderAlias {config::LibraryCoverMediaMode::Title,
				{"marquees", "marquee", "wheelhd", "wheel-hd", "wheel", "wheels", "screenmarquee", "screenmarqueesmall", "title"}},
		MediaFolderAlias {config::LibraryCoverMediaMode::Physical,
				{"physicalmedia", "physical-media", "support2d", "support-2d", "supporttexture", "support-texture", "support", "physical", "disc"}},
		MediaFolderAlias {config::LibraryCoverMediaMode::Manual,
				{"manuals", "manual", "manualss"}},
	};

	const std::string folderLower = toLowerString(folder);
	for (const MediaFolderAlias& aliasGroup : mediaAliases)
		for (const std::string_view alias : aliasGroup.aliases)
			if (folderLower == alias)
				return &aliasGroup;
	return nullptr;
}

bool isSupportedCustomMediaExtension(config::LibraryCoverMediaMode mediaMode, const std::string& ext)
{
	return mediaMode == config::LibraryCoverMediaMode::Manual
			? isSupportedManualExtension(ext)
			: isSupportedBoxartExtension(ext);
}

bool isGeneratedVmuIconPath(const std::string& root, const std::string& path)
{
	if (path.size() <= root.size() || path.compare(0, root.size(), root) != 0)
		return false;

	std::string relative = path.substr(root.size());
	while (!relative.empty() && (relative.front() == '/' || relative.front() == '\\'))
		relative.erase(relative.begin());
	const size_t slash = relative.find_first_of("/\\");
	const std::string firstComponent = slash == std::string::npos ? relative : relative.substr(0, slash);
	return firstComponent == "vmu-icons";
}

} // namespace

GameBoxart Boxart::getBoxart(const GameMedia& media)
{
	loadDatabase();
	const int sourceMode = config::BoxartSourceMode.get();
	if (sourceMode == static_cast<int>(BoxartSourceMode::PhysicalMediaOnly))
		return getPhysicalBoxart(media);

	GameBoxart boxart;
	{
		std::lock_guard<std::mutex> guard(mutex);
		auto it = games.find(media.fileName);
		if (it != games.end())
			boxart = it->second;
	}
	if (boxart.fileName.empty())
		boxart = makeMediaBoxart(media);
	else if (isMissingBoxartFile(boxart))
		boxart.boxartPath.clear();

	if (sourceMode == static_cast<int>(BoxartSourceMode::CustomThenScraped))
	{
		const std::string customPath = getCustomBoxartPath(media);
		if (!customPath.empty())
			boxart.boxartPath = customPath;
	}
	else if (sourceMode == static_cast<int>(BoxartSourceMode::ScrapedOnly))
	{
		if (boxart.boxartUrl.empty())
			boxart.boxartPath.clear();
	}

	return boxart;
}

GameBoxart Boxart::getBoxartAndLoad(const GameMedia& media)
{
	return getBoxartAndQueue(media, true);
}

void Boxart::queueBoxart(const GameMedia& media)
{
	getBoxartAndQueue(media, false);
}

void Boxart::startFetch()
{
	fetchBoxart();
}

GameBoxart Boxart::getBoxartAndQueue(const GameMedia& media, bool startFetch)
{
	loadDatabase();
	const int sourceMode = config::BoxartSourceMode.get();
	if (sourceMode == static_cast<int>(BoxartSourceMode::PhysicalMediaOnly))
		return getPhysicalBoxart(media);

	GameBoxart boxart;
	{
		std::lock_guard<std::mutex> guard(mutex);
		auto it = games.find(media.fileName);
		if (it != games.end())
		{
			boxart = it->second;
			const bool missingBoxartFile = isMissingBoxartFile(boxart);
			const bool missingScrapedBoxartFile = isMissingScrapedBoxartFile(boxart);
			const bool missingPhysicalBoxart = boxart.parsed && boxart.boxartPath.empty() && !boxart.scraped;
			if (missingBoxartFile || missingScrapedBoxartFile || missingPhysicalBoxart)
			{
				boxart.boxartPath.clear();
				if (missingBoxartFile || missingPhysicalBoxart)
					boxart.parsed = false;
				if (missingScrapedBoxartFile)
					boxart.scraped = false;
				it->second = boxart;
				databaseDirty = true;
			}
			const bool wantOnline = shouldFetchOnline();
			const bool needsOffline = !boxart.parsed;
			const bool needsOnline = wantOnline && !boxart.scraped;
			if (!boxart.busy && (needsOffline || needsOnline))
			{
				boxart.busy = it->second.busy = true;
				boxart.gamePath = media.path;
				boxart.arcade = media.arcade;
				toFetch.push_back(boxart);
			}
		}
		else
		{
			boxart = makeMediaBoxart(media);
			boxart.busy = true;
			games[boxart.fileName] = boxart;
			toFetch.push_back(boxart);
		}
	}

	if (sourceMode == static_cast<int>(BoxartSourceMode::CustomThenScraped))
	{
		std::string customPath = getCustomBoxartPath(media);
		if (!customPath.empty())
		{
			boxart.boxartPath = customPath;
			boxart.busy = false;
		}
	}
	else if (sourceMode == static_cast<int>(BoxartSourceMode::ScrapedOnly))
	{
		if (boxart.boxartUrl.empty())
			boxart.boxartPath.clear();
	}
	if (startFetch)
		fetchBoxart();

	return boxart;
}

bool Boxart::shouldFetchOnline() const
{
	return config::FetchBoxart && config::BoxartSourceMode.get() != static_cast<int>(BoxartSourceMode::PhysicalMediaOnly);
}

GameBoxart Boxart::getPhysicalBoxart(const GameMedia& media)
{
	{
		std::lock_guard<std::mutex> guard(mutex);
		auto it = physicalCache.find(media.fileName);
		if (it != physicalCache.end())
			return it->second;
	}

	GameBoxart boxart = makeMediaBoxart(media);

	OfflineScraper physicalScraper;
	physicalScraper.initialize(getSaveDirectory());
	physicalScraper.scrape(boxart);

	{
		std::lock_guard<std::mutex> guard(mutex);
		physicalCache[boxart.fileName] = boxart;
	}

	return boxart;
}

std::string Boxart::getCustomBoxartPath(const GameMedia& media)
{
	return getCustomBoxartPathForMediaMode(media, config::LibraryCoverMediaMode::CurrentArtwork);
}

std::string Boxart::getLibraryCoverMediaPath(const GameMedia& media)
{
	const auto coverMediaMode = static_cast<config::LibraryCoverMediaMode>(config::LibraryCoverMedia.get());
	if (coverMediaMode == config::LibraryCoverMediaMode::CurrentArtwork)
		return {};
	if (static_cast<size_t>(coverMediaMode) >= customBoxartByName.size())
		return {};
	return getCustomBoxartPathForMediaMode(media, coverMediaMode);
}

std::string Boxart::getCustomMediaPath(const GameMedia& media, config::LibraryCoverMediaMode mediaMode)
{
	return getCustomBoxartPathForMediaMode(media, mediaMode);
}

std::string Boxart::getCustomBoxartPathForMediaMode(const GameMedia& media, config::LibraryCoverMediaMode mediaMode)
{
	const size_t modeIndex = static_cast<size_t>(mediaMode);
	if (modeIndex >= customBoxartByName.size())
		return {};

	refreshCustomBoxartIndex(false);

	const std::string fileKey = makeBoxartKey(media.fileName);
	const std::string nameKey = makeBoxartKey(media.name);
	const std::string gameNameKey = makeBoxartKey(media.gameName);

	std::lock_guard<std::mutex> guard(mutex);
	const auto& mediaIndex = customBoxartByName[modeIndex];
	if (!fileKey.empty())
	{
		auto it = mediaIndex.find(fileKey);
		if (it != mediaIndex.end())
			return it->second;
	}
	if (!nameKey.empty())
	{
		auto it = mediaIndex.find(nameKey);
		if (it != mediaIndex.end())
			return it->second;
	}
	if (!gameNameKey.empty())
	{
		auto it = mediaIndex.find(gameNameKey);
		if (it != mediaIndex.end())
			return it->second;
	}
	return {};
}

void Boxart::fetchBoxart()
{
	if (physicalFetching.valid() && physicalFetching.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
	{
		try {
			physicalFetching.get();
		} catch (const std::exception& e) {
			ERROR_LOG(COMMON, "Physical boxart scraper thread exception: %s", e.what());
		} catch (...) {
			ERROR_LOG(COMMON, "Physical boxart scraper thread unknown exception");
		}
	}
	if (onlineFetching.valid() && onlineFetching.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
	{
		try {
			onlineFetching.get();
		} catch (const std::exception& e) {
			ERROR_LOG(COMMON, "Online boxart scraper thread exception: %s", e.what());
		} catch (...) {
			ERROR_LOG(COMMON, "Online boxart scraper thread unknown exception");
		}
	}

	if (!physicalFetching.valid())
	{
		bool hasPhysicalBoxart = false;
		{
			std::lock_guard<std::mutex> guard(mutex);
			for (const GameBoxart& item : toFetch)
				if (!item.parsed)
				{
					hasPhysicalBoxart = true;
					break;
				}
		}
		if (hasPhysicalBoxart)
		{
			physicalFetching = std::async(std::launch::async, [this]() {
				ThreadName _("BoxArt-physical");
				const bool wantOnline = shouldFetchOnline();
				while (true)
				{
					std::vector<GameBoxart> physicalBoxart;
					{
						std::lock_guard<std::mutex> guard(mutex);
						// Keep toFetch as a vector for online front-batching, but compact it
						// once here instead of erasing physical items one by one.
						std::vector<GameBoxart> remaining;
						remaining.reserve(toFetch.size());
						for (GameBoxart& item : toFetch)
						{
							if (!item.parsed && physicalBoxart.size() < kPhysicalBoxartBatchSize)
							{
								physicalBoxart.push_back(std::move(item));
							}
							else
							{
								remaining.push_back(std::move(item));
							}
						}
						toFetch = std::move(remaining);
					}
					if (physicalBoxart.empty())
						break;

					DEBUG_LOG(COMMON, "Scraping physical boxart for %d games", (int)physicalBoxart.size());
					scrapePhysicalBoxart(physicalBoxart, getSaveDirectory());
					{
						std::lock_guard<std::mutex> guard(mutex);
						for (GameBoxart& b : physicalBoxart)
							if (b.scraped || b.parsed)
							{
								auto existing = games.find(b.fileName);
								if (existing != games.end())
								{
									if (b.boxartPath.empty() && !existing->second.boxartPath.empty())
										b.boxartPath = existing->second.boxartPath;
									if (!existing->second.boxartUrl.empty())
										b.boxartUrl = existing->second.boxartUrl;
									if (existing->second.scraped)
										b.scraped = true;
								}
								if (wantOnline && !b.scraped)
									toFetch.push_back(b);
								else
									b.busy = false;
								games[b.fileName] = b;
								databaseDirty = true;
							}
					}
					saveDatabase();
				}
			});
		}
	}

	if (physicalFetching.valid() || onlineFetching.valid())
		return;
	if (toFetch.empty())
		return;

	std::vector<GameBoxart> boxart;
	{
		std::lock_guard<std::mutex> guard(mutex);
		for (const GameBoxart& item : toFetch)
			if (!item.parsed)
				return;

		size_t size = std::min(toFetch.size(), kOnlineBoxartBatchSize);
		boxart = std::vector<GameBoxart>(toFetch.begin(), toFetch.begin() + size);
		toFetch.erase(toFetch.begin(), toFetch.begin() + size);
	}
	onlineFetching = std::async(std::launch::async, [this, boxart = std::move(boxart)]() mutable {
		ThreadName _("BoxArt-online");
		const bool wantOnline = shouldFetchOnline();
		DEBUG_LOG(COMMON, "Scraping online boxart for %d games", (int)boxart.size());
		if (wantOnline)
		{
			if (scraper == nullptr)
			{
				scraper = std::unique_ptr<Scraper>(new TheGamesDb());
				if (!scraper->initialize(getSaveDirectory()))
				{
					ERROR_LOG(COMMON, "thegamesdb scraper initialization failed");
					scraper.reset();
					std::lock_guard<std::mutex> guard(mutex);
					toFetch.insert(toFetch.begin(), boxart.begin(), boxart.end());
					return;
				}
			}
			if (arcadeScraper == nullptr) {
				arcadeScraper = std::make_unique<ArcadeScraper>();
				arcadeScraper->initialize(getSaveDirectory());
			}
			try {
				arcadeScraper->scrape(boxart);
				scraper->scrape(boxart);
				{
					std::lock_guard<std::mutex> guard(mutex);
					for (GameBoxart& b : boxart)
					{
						b.busy = false;
						games[b.fileName] = b;
					}
				}
				databaseDirty = true;
			} catch (const std::runtime_error& e) {
				if (*e.what() != '\0')
					INFO_LOG(COMMON, "thegamesdb error: %s", e.what());
				{
					// put back failed items into toFetch array
					std::lock_guard<std::mutex> guard(mutex);
					for (GameBoxart& b : boxart)
						if (b.scraped)
						{
							b.busy = false;
							games[b.fileName] = b;
							databaseDirty = true;
						}
						else
						{
							toFetch.push_back(b);
						}
				}
			}
		}
		else
		{
			std::lock_guard<std::mutex> guard(mutex);
			for (GameBoxart& b : boxart)
			{
				b.busy = false;
				games[b.fileName] = b;
			}
		}
		saveDatabase();
	});
}

void Boxart::saveDatabase()
{
	if (!databaseDirty)
		return;
	std::string basePath = getSaveDirectory();
	std::string db_name = basePath + DB_NAME;
	DEBUG_LOG(COMMON, "Saving boxart database to %s", db_name.c_str());

	json array;
	{
		std::lock_guard<std::mutex> guard(mutex);
		for (const auto& game : games)
			if (game.second.scraped || game.second.parsed || !game.second.fileName.empty())
				array.push_back(game.second.to_json(basePath));
	}
	std::string serialized = array.dump(4, ' ', false, json::error_handler_t::replace);

	FILE *file = nowide::fopen(db_name.c_str(), "wt");
	if (file == nullptr) {
		WARN_LOG(COMMON, "Can't save boxart database to %s: error %d", db_name.c_str(), errno);
		return;
	}
	fwrite(serialized.c_str(), 1, serialized.size(), file);
	fclose(file);
	databaseDirty = false;
}

void Boxart::recoverDatabases(const std::string& saveDir)
{
	std::vector<std::string> databasePaths;
	std::unordered_map<std::string, std::string> imagePathsByName;
	try {
		hostfs::DirectoryTree tree(saveDir);
		for (auto it = tree.begin(); it != tree.end(); ++it)
		{
			const hostfs::FileInfo& entry = *it;
			if (entry.isDirectory)
				continue;
			if (get_file_extension(entry.name) == "json")
			{
				databasePaths.push_back(entry.path);
				continue;
			}
			if (isSupportedBoxartExtension(get_file_extension(entry.name)))
			{
				imagePathsByName.emplace(normalizePath(entry.name), entry.path);
				imagePathsByName.emplace(normalizePath(removeCopySuffix(entry.name)), entry.path);
			}
		}
	} catch (const std::exception& e) {
		WARN_LOG(COMMON, "Boxart database recovery scan failed: %s", e.what());
		return;
	}

	if (databasePaths.size() <= 1)
		return;

	const std::string mainDbPath = saveDir + DB_NAME;
	std::unordered_map<std::string, GameBoxart> mergedGames;
	std::unordered_map<std::string, std::string> candidateImagePaths;
	std::unordered_set<std::string> processedDatabasePaths;
	for (const std::string& dbPath : databasePaths)
	{
		std::unique_ptr<FILE, decltype(&fclose)> f(nowide::fopen(dbPath.c_str(), "rt"), &fclose);
		if (f == nullptr)
			continue;

		try {
			json v = json::parse(f.get());
			if (!v.is_array())
				continue;
			bool recoveredAnyGame = false;
			const std::string dbDir = parentPath(dbPath);
			for (const auto& o : v)
			{
				GameBoxart candidate(o, dbDir);
				if (candidate.fileName.empty() && candidate.name.empty())
					continue;

				if (!candidate.boxartPath.empty() && !file_exists(candidate.boxartPath))
				{
					const std::string imageName = normalizePath(removeCopySuffix(fileNameFromPath(candidate.boxartPath)));
					auto imageIt = imagePathsByName.find(imageName);
					if (imageIt != imagePathsByName.end())
						candidate.boxartPath = imageIt->second;
				}
				if (!candidate.boxartPath.empty() && file_exists(candidate.boxartPath) && isPathInDirectory(candidate.boxartPath, saveDir))
					candidateImagePaths[normalizePath(candidate.boxartPath)] = candidate.boxartPath;
				if (candidate.boxartPath.empty() && !candidate.scraped)
					candidate.parsed = false;

				const std::string key = candidate.fileName.empty() ? candidate.name : candidate.fileName;
				auto existing = mergedGames.find(key);
				if (existing == mergedGames.end() || boxartMergeScore(candidate) > boxartMergeScore(existing->second))
					mergedGames[key] = candidate;
				recoveredAnyGame = true;
			}
			if (recoveredAnyGame)
				processedDatabasePaths.insert(normalizePath(dbPath));
		} catch (const json::exception& e) {
			WARN_LOG(COMMON, "Skipping corrupt boxart database %s: %s", dbPath.c_str(), e.what());
		}
	}

	if (mergedGames.empty())
		return;

	json array;
	std::unordered_set<std::string> selectedImagePaths;
	for (const auto& game : mergedGames)
	{
		if (!game.second.boxartPath.empty() && file_exists(game.second.boxartPath) && isPathInDirectory(game.second.boxartPath, saveDir))
			selectedImagePaths.insert(normalizePath(game.second.boxartPath));
		array.push_back(game.second.to_json(saveDir));
	}

	FILE *file = nowide::fopen(mainDbPath.c_str(), "wt");
	if (file == nullptr)
	{
		WARN_LOG(COMMON, "Can't write recovered boxart database to %s: error %d", mainDbPath.c_str(), errno);
		return;
	}
	std::string serialized = array.dump(4, ' ', false, json::error_handler_t::replace);
	fwrite(serialized.c_str(), 1, serialized.size(), file);
	fclose(file);

	for (const std::string& dbPath : databasePaths)
		if (normalizePath(dbPath) != normalizePath(mainDbPath) && processedDatabasePaths.find(normalizePath(dbPath)) != processedDatabasePaths.end())
			nowide::remove(dbPath.c_str());
	for (const auto& imagePath : candidateImagePaths)
		if (selectedImagePaths.find(imagePath.first) == selectedImagePaths.end())
			nowide::remove(imagePath.second.c_str());

	INFO_LOG(COMMON, "Recovered %d games from %d boxart databases", (int)mergedGames.size(), (int)databasePaths.size());
}

void Boxart::loadDatabase()
{
	if (databaseLoaded)
		return;
	databaseLoaded = true;
	databaseDirty = false;
	std::string save_dir = getSaveDirectory();
	if (!file_exists(save_dir))
		make_directory(save_dir);
	recoverDatabases(save_dir);
	std::string db_name = save_dir + DB_NAME;
	std::unique_ptr<FILE, decltype(&fclose)> f(nowide::fopen(db_name.c_str(), "rt"), &fclose);
	if (f == nullptr)
	{
		refreshCustomBoxartIndex(false);
		return;
	}

	DEBUG_LOG(COMMON, "Loading boxart database from %s", db_name.c_str());
	try {
		std::lock_guard<std::mutex> guard(mutex);

		json v = json::parse(f.get());
		for (const auto& o : v)
		{
			GameBoxart game(o, save_dir);
			games[game.fileName] = game;
		}
	} catch (const json::exception& e) {
		WARN_LOG(COMMON, "Corrupted database file: %s", e.what());
	}
	refreshCustomBoxartIndex(false);
}

void Boxart::reviewDatabaseArtwork()
{
	std::lock_guard<std::mutex> guard(mutex);
	for (auto& game : games)
	{
		if (!game.second.boxartPath.empty())
		{
			const bool usableImage = isSupportedBoxartExtension(get_file_extension(game.second.boxartPath)) && file_exists(game.second.boxartPath);
			if (usableImage)
				continue;
			game.second.boxartPath.clear();
			if (!game.second.boxartUrl.empty())
				game.second.scraped = false;
			databaseDirty = true;
		}

		if (game.second.parsed && game.second.boxartPath.empty() && !game.second.scraped)
		{
			game.second.parsed = false;
			game.second.busy = false;
			databaseDirty = true;
		}
		if (game.second.boxartPath.empty() && game.second.scraped)
		{
			game.second.scraped = false;
			game.second.busy = false;
			databaseDirty = true;
		}
	}
}

void Boxart::refreshCache()
{
	if (physicalFetching.valid())
	{
		try {
			physicalFetching.get();
		} catch (const std::exception& e) {
			ERROR_LOG(COMMON, "Physical boxart scraper thread exception: %s", e.what());
		} catch (...) {
			ERROR_LOG(COMMON, "Physical boxart scraper thread unknown exception");
		}
	}
	if (onlineFetching.valid())
	{
		try {
			onlineFetching.get();
		} catch (const std::exception& e) {
			ERROR_LOG(COMMON, "Online boxart scraper thread exception: %s", e.what());
		} catch (...) {
			ERROR_LOG(COMMON, "Online boxart scraper thread unknown exception");
		}
	}
	saveDatabase();
	{
		std::lock_guard<std::mutex> guard(mutex);
		games.clear();
		physicalCache.clear();
		toFetch.clear();
		databaseLoaded = false;
		databaseDirty = false;
		customIndexLoaded = false;
	}
	loadDatabase();
	reviewDatabaseArtwork();
	refreshCustomBoxartIndex(true);
	saveDatabase();
}

void Boxart::term()
{
	if (physicalFetching.valid())
		physicalFetching.get();
	if (onlineFetching.valid())
		onlineFetching.get();
}

void Boxart::refreshCustomBoxartIndex(bool force)
{
	const std::string root = getSaveDirectory();
	if (!force && customIndexLoaded && root == customBoxartRoot)
		return;

	std::unordered_map<std::string, std::string> newIndex;
	CustomBoxartIndex mediaModeIndexes;
	if (!root.empty() && hostfs::storage().exists(root))
	{
		try {
			hostfs::DirectoryTree tree(root);
			for (auto it = tree.begin(); it != tree.end(); ++it)
			{
				const hostfs::FileInfo& entry = *it;
				if (entry.isDirectory)
					continue;
				if (isGeneratedVmuIconPath(root, entry.path))
					continue;
				const std::string ext = get_file_extension(entry.name);
				const std::string key = makeBoxartKey(entry.name);
				if (key.empty())
					continue;

				const std::string firstFolder = toLowerString(getFolderFirstComponent(root, entry.path));
				const MediaFolderAlias* alias = findMediaFolderAlias(firstFolder);
				if (alias == nullptr)
				{
					if (isSupportedBoxartExtension(ext))
						newIndex[key] = entry.path;
					continue;
				}
				if (!isSupportedCustomMediaExtension(alias->mode, ext))
					continue;
				if (isSupportedBoxartExtension(ext))
					newIndex[key] = entry.path;

				const size_t modeIndex = static_cast<size_t>(alias->mode);
				if (mediaModeIndexes[modeIndex].find(key) == mediaModeIndexes[modeIndex].end())
					mediaModeIndexes[modeIndex][key] = entry.path;
			}
		} catch (const std::exception& e) {
			WARN_LOG(COMMON, "Custom boxart scan failed: %s", e.what());
		}
	}

	{
		std::lock_guard<std::mutex> guard(mutex);
		if (root != customBoxartRoot)
			physicalCache.clear();
		customBoxartRoot = root;
		for (size_t i = 0; i < customBoxartByName.size(); ++i)
			customBoxartByName[i].swap(mediaModeIndexes[i]);
		customBoxartByName[static_cast<size_t>(config::LibraryCoverMediaMode::CurrentArtwork)] = std::move(newIndex);
		customIndexLoaded = true;
	}
}
