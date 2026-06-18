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
#include <array>
#include <chrono>
#include <cctype>
#include <cstring>
#include <string_view>

namespace {

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
			boxart.fileName = media.fileName;
			boxart.gamePath = media.path;
			boxart.name = media.name;
			boxart.searchName = media.gameName;	// for arcade games
			boxart.busy = true;
			boxart.arcade = media.arcade;
			games[boxart.fileName] = boxart;
			toFetch.push_back(boxart);
		}
	}
	fetchBoxart();

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

	GameBoxart boxart;
	boxart.fileName = media.fileName;
	boxart.gamePath = media.path;
	boxart.name = media.name;
	boxart.searchName = media.gameName;
	boxart.arcade = media.arcade;

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
	if (fetching.valid() && fetching.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
	{
		try {
			fetching.get();
		} catch (const std::exception& e) {
			ERROR_LOG(COMMON, "Boxart scraper thread exception: %s", e.what());
		} catch (...) {
			ERROR_LOG(COMMON, "Boxart scraper thread unknown exception");
		}
	}
	if (fetching.valid())
		return;
	if (toFetch.empty())
		return;
	fetching = std::async(std::launch::async, [this]() {
		ThreadName _("BoxArt-scraper");
		const bool wantOnline = shouldFetchOnline();
		if (offlineScraper == nullptr)
		{
			offlineScraper = std::unique_ptr<Scraper>(new OfflineScraper());
			offlineScraper->initialize(getSaveDirectory());
		}
		if (wantOnline)
		{
			if (scraper == nullptr)
			{
				scraper = std::unique_ptr<Scraper>(new TheGamesDb());
				if (!scraper->initialize(getSaveDirectory()))
				{
					ERROR_LOG(COMMON, "thegamesdb scraper initialization failed");
					scraper.reset();
					return;
				}
			}
			if (arcadeScraper == nullptr) {
				arcadeScraper = std::make_unique<ArcadeScraper>();
				arcadeScraper->initialize(getSaveDirectory());
			}
		}
		std::vector<GameBoxart> boxart;
		{
			std::lock_guard<std::mutex> guard(mutex);
			size_t size = std::min(toFetch.size(), (size_t)10);
			boxart = std::vector<GameBoxart>(toFetch.begin(), toFetch.begin() + size);
			toFetch.erase(toFetch.begin(), toFetch.begin() + size);
		}
		DEBUG_LOG(COMMON, "Scraping %d games", (int)boxart.size());
		offlineScraper->scrape(boxart);
		{
			std::lock_guard<std::mutex> guard(mutex);
			for (GameBoxart& b : boxart)
				if (b.scraped || b.parsed)
				{
					if (!wantOnline || b.scraped)
						b.busy = false;
					games[b.fileName] = b;
					databaseDirty = true;
				}
		}
		if (wantOnline)
		{
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
			if (game.second.scraped || game.second.parsed)
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

void Boxart::loadDatabase()
{
	if (databaseLoaded)
		return;
	databaseLoaded = true;
	databaseDirty = false;
	std::string save_dir = getSaveDirectory();
	if (!file_exists(save_dir))
		make_directory(save_dir);
	std::string db_name = save_dir + DB_NAME;
	FILE *f = nowide::fopen(db_name.c_str(), "rt");
	if (f == nullptr)
	{
		refreshCustomBoxartIndex(false);
		return;
	}

	DEBUG_LOG(COMMON, "Loading boxart database from %s", db_name.c_str());
	std::string all_data;
	char buf[4096];
	while (true)
	{
		int s = fread(buf, 1, sizeof(buf), f);
		if (s <= 0)
			break;
		all_data.append(buf, s);
	}
	fclose(f);
	try {
		std::lock_guard<std::mutex> guard(mutex);

		json v = json::parse(all_data);
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

void Boxart::term()
{
	if (fetching.valid())
		fetching.get();
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
