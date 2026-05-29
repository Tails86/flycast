/*
	Copyright 2026 flyinghead
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
#include "vmu_icon.h"
#include "../game_scanner.h"
#include "cfg/option.h"
#include "json.hpp"
#include "oslib/storage.h"
#include "stdclass.h"

#include <stb_image_write.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdio>
#include <ctime>
#include <mutex>
#include <unordered_map>
#include <vector>

using namespace nlohmann;

namespace {

constexpr size_t VMU_BLOCK_SIZE = 512;
constexpr size_t VMU_BLOCK_COUNT = 256;
constexpr size_t VMU_FLASH_SIZE = VMU_BLOCK_SIZE * VMU_BLOCK_COUNT;
constexpr u16 VMU_FAT_END = 0xfffa;
constexpr u16 VMU_FAT_FREE = 0xfffc;
std::mutex cacheMutex;
std::unordered_map<std::string, time_t> lastBootedByKey;
std::unordered_map<std::string, time_t> lastBootedByPath;
bool lastBootedIndexLoaded = false;
json sharedIconCache;
std::string sharedIconCachePath;
bool sharedIconCacheLoaded = false;

u16 readLe16(const u8 *p)
{
	return p[0] | (p[1] << 8);
}

std::string sanitizeCacheKey(std::string key)
{
	if (key.empty())
		return {};

	constexpr std::string_view INVALID_CHARS { " /\\:*?|<>\"" };
	for (char& c : key)
		if (INVALID_CHARS.find(c) != INVALID_CHARS.npos || static_cast<unsigned char>(c) < 0x20)
			c = '_';
	return key;
}

std::string makeCacheKey(const GameMedia& media, const std::string& gameId)
{
	std::string key = sanitizeCacheKey(gameId);
	if (!key.empty())
		return key;
	return sanitizeCacheKey(get_file_basename(media.fileName));
}

bool ensureDirectory(const std::string& path)
{
	return file_exists(path) || make_directory(path);
}

std::string appendPath(const std::string& path, const std::string& name)
{
	if (path.empty() || path.back() == '/' || path.back() == '\\')
		return path + name;
#ifdef _WIN32
	return path + "\\" + name;
#else
	return path + "/" + name;
#endif
}

std::string getVmuIconLibraryDir()
{
	std::string base = get_writable_data_path("boxart/");
	if (!ensureDirectory(base))
		return {};

	std::string icons = appendPath(base, "vmu-icons");
	if (!ensureDirectory(icons))
		return {};

	std::string library = appendPath(icons, "library");
	if (!ensureDirectory(library))
		return {};
	return library;
}

std::string getVmuIconCacheDir(const std::string& library, const std::string& key)
{
	std::string dir = appendPath(library, key);
	if (!ensureDirectory(dir))
		return {};
	return dir;
}

std::string findPerGameVmuPath(const GameMedia& media, const std::string& gameId)
{
	std::string key = sanitizeCacheKey(gameId);
	if (key.empty())
		key = sanitizeCacheKey(get_file_basename(media.fileName));
	if (key.empty())
		return {};

	std::vector<std::string> candidates { key + "_vmu_save_A1.bin" };
	if (!media.fileName.empty())
	{
		const std::string legacyName = media.fileName + "_vmu_save_A1.bin";
		if (legacyName != candidates.front())
			candidates.push_back(legacyName);
	}

	for (const std::string& vmuName : candidates)
	{
		if (!config::VMUPath.get().empty())
		{
			try {
				std::string fullpath = hostfs::storage().getSubPath(config::VMUPath, vmuName);
				if (hostfs::storage().exists(fullpath))
					return fullpath;
			} catch (const hostfs::StorageException&) {
			}
		}

		std::string path = get_writable_data_path(vmuName);
		if (file_exists(path))
			return path;

		path = get_readonly_data_path(vmuName);
		if (hostfs::storage().exists(path))
			return path;
	}
	return {};
}

hostfs::FileInfo getFileInfo(const std::string& path)
{
	try {
		return hostfs::storage().getFileInfo(path);
	} catch (const std::exception&) {
		return {};
	}
}

bool readFile(const std::string& path, std::vector<u8>& out)
{
	hostfs::File *file = hostfs::storage().openFile(path, "rb");
	if (file == nullptr)
		return false;

	const s64 size = file->size();
	if (size <= 0)
	{
		delete file;
		return false;
	}
	out.resize(static_cast<size_t>(size));
	const bool ok = file->read(out.data(), 1, out.size()) == out.size();
	delete file;
	return ok;
}

json loadIconCache(const std::string& path)
{
	std::vector<u8> data;
	if (!readFile(path, data))
		return json{
			{ "version", 1 },
			{ "games", json::object() },
		};

	try {
		json j = json::parse(data.begin(), data.end());
		if (!j.contains("games") || !j["games"].is_object())
			j["games"] = json::object();
		j["version"] = 1;
		return j;
	} catch (const json::exception&) {
		return json{
			{ "version", 1 },
			{ "games", json::object() },
		};
	}
}

bool saveIconCache(const std::string& path, const json& cache)
{
	std::string serialized = cache.dump(4, ' ', false, json::error_handler_t::replace);
	FILE *file = nowide::fopen(path.c_str(), "wt");
	if (file == nullptr)
		return false;
	fwrite(serialized.c_str(), 1, serialized.size(), file);
	fclose(file);
	return true;
}

json& getSharedIconCache(const std::string& path)
{
	if (!sharedIconCacheLoaded || sharedIconCachePath != path)
	{
		sharedIconCache = loadIconCache(path);
		sharedIconCachePath = path;
		sharedIconCacheLoaded = true;
		lastBootedByKey.clear();
		lastBootedByPath.clear();
		lastBootedIndexLoaded = false;
	}
	return sharedIconCache;
}

bool saveSharedIconCache()
{
	if (!sharedIconCacheLoaded || sharedIconCachePath.empty())
		return false;
	return saveIconCache(sharedIconCachePath, sharedIconCache);
}

bool cachedEntryMatches(const json& cache, const std::string& key, const hostfs::FileInfo& sourceInfo, bool& iconWasExtracted)
{
	try {
		const json& games = cache.at("games");
		const json& entry = games.at(key);
		if (entry.value("metadata_only", false) || !entry.contains("ok"))
			return false;
		if (entry.value("size", static_cast<size_t>(0)) != sourceInfo.size)
			return false;
		if (entry.value("mtime", static_cast<u64>(0)) != sourceInfo.updateTime)
			return false;
		iconWasExtracted = entry.value("ok", false);
		return true;
	} catch (const json::exception&) {
		return false;
	}
}

bool cachedAnyIconEntry(const json& cache, const std::string& key, u32& frameCount, u16& animationSpeed)
{
	try {
		const json& entry = cache.at("games").at(key);
		if (entry.value("source", "") == "live-vmu")
			return false;
		if (!entry.value("ok", false))
			return false;
		frameCount = entry.value("frames", 1);
		animationSpeed = entry.value("speed", 0);
		return true;
	} catch (const json::exception&) {
		return false;
	}
}

bool cachedLiveIconEntry(const json& cache, const std::string& key, u32& frameCount, u16& animationSpeed)
{
	try {
		const json& entry = cache.at("games").at(key);
		if (entry.value("source", "") != "live-vmu-a1" || !entry.value("ok", false))
			return false;
		frameCount = entry.value("frames", 1);
		animationSpeed = entry.value("speed", 0);
		return true;
	} catch (const json::exception&) {
		return false;
	}
}

void updateIconCacheEntry(json& cache, const std::string& key, const hostfs::FileInfo& sourceInfo,
		bool iconWasExtracted, u32 frameCount, u16 animationSpeed)
{
	cache["version"] = 1;
	if (!cache.contains("games") || !cache["games"].is_object())
		cache["games"] = json::object();
	json entry = cache["games"].value(key, json::object());
	if (!iconWasExtracted && entry.value("source", "") == "live-vmu-a1" && entry.value("ok", false))
		return;
	entry["metadata_only"] = false;
	entry["source"] = "file-vmu";
	entry["size"] = sourceInfo.size;
	entry["mtime"] = sourceInfo.updateTime;
	entry["ok"] = iconWasExtracted;
	entry["frames"] = frameCount;
	entry["speed"] = animationSpeed;
	cache["games"][key] = entry;
}

void updateLastBootedCacheEntry(json& cache, const std::string& key, const std::string& gamePath, time_t lastBooted)
{
	cache["version"] = 1;
	if (!cache.contains("games") || !cache["games"].is_object())
		cache["games"] = json::object();
	json entry = cache["games"].value(key, json::object());
	if (!entry.contains("ok"))
		entry["metadata_only"] = true;
	entry["path"] = gamePath;
	entry["last_booted"] = lastBooted;
	cache["games"][key] = entry;
	lastBootedByKey[key] = lastBooted;
	if (!gamePath.empty())
		lastBootedByPath[gamePath] = lastBooted;
}

time_t readLastBootedCacheEntry(const json& cache, const std::string& key, const std::string& gamePath)
{
	if (!lastBootedIndexLoaded)
	{
		try {
			const json& games = cache.at("games");
			for (auto it = games.begin(); it != games.end(); ++it)
			{
				const time_t lastBooted = static_cast<time_t>(it.value().value("last_booted", 0));
				if (lastBooted == 0)
					continue;
				lastBootedByKey[it.key()] = lastBooted;
				const std::string path = it.value().value("path", "");
				if (!path.empty())
					lastBootedByPath[path] = lastBooted;
			}
		} catch (const json::exception&) {
		}
		lastBootedIndexLoaded = true;
	}

	if (!key.empty())
	{
		const auto it = lastBootedByKey.find(key);
		if (it != lastBootedByKey.end())
			return it->second;
	}
	if (!gamePath.empty())
	{
		const auto it = lastBootedByPath.find(gamePath);
		if (it != lastBootedByPath.end())
			return it->second;
	}
	return 0;
}

void updateLiveIconCacheEntry(json& cache, const std::string& key, bool iconWasExtracted,
		u32 frameCount, u16 animationSpeed)
{
	cache["version"] = 1;
	if (!cache.contains("games") || !cache["games"].is_object())
		cache["games"] = json::object();
	if (!iconWasExtracted && cache["games"].contains(key) && cache["games"][key].value("ok", false))
		return;
	json entry = cache["games"].value(key, json::object());
	entry["metadata_only"] = false;
	entry["source"] = "live-vmu-a1";
	entry["size"] = VMU_FLASH_SIZE;
	entry["mtime"] = 0;
	entry["ok"] = iconWasExtracted;
	entry["frames"] = frameCount;
	entry["speed"] = animationSpeed;
	cache["games"][key] = entry;
}

void writePngData(void *context, void *data, int size)
{
	FILE *file = nowide::fopen(static_cast<const char *>(context), "wb");
	if (file == nullptr)
		return;
	fwrite(data, 1, size, file);
	fclose(file);
}

bool saveIconPng(const std::string& path, const std::vector<u8>& rgba)
{
	return stbi_write_png_to_func(writePngData, (void *)path.c_str(), 32, 32, 4, rgba.data(), 32 * 4) != 0;
}

std::string framePath(const std::string& cacheDir, u32 frameIndex)
{
	return appendPath(cacheDir, "frame-" + std::to_string(frameIndex) + ".png");
}

std::string selectCachedIconPath(const std::string& cacheDir, u32 frameCount, u16 animationSpeed, bool animate, double animationClock)
{
	const std::string iconPath = appendPath(cacheDir, "icon.png");
	if (!animate || frameCount <= 1)
		return file_exists(iconPath) ? iconPath : std::string();

	constexpr double LibraryIconFrameSeconds = 0.45;
	const u32 frame = static_cast<u32>(animationClock / LibraryIconFrameSeconds) % frameCount;
	const std::string path = framePath(cacheDir, frame);
	if (file_exists(path))
		return path;
	return file_exists(iconPath) ? iconPath : std::string();
}

bool hasCachedAnimationFrames(const std::string& cacheDir, u32 frameCount)
{
	for (u32 i = 0; i < frameCount; i++)
		if (!file_exists(framePath(cacheDir, i)))
			return false;
	return true;
}

std::string readVmuText(const u8 *data, size_t size)
{
	std::string text(reinterpret_cast<const char *>(data), size);
	const size_t end = text.find_last_not_of(std::string(" \0", 2));
	return end == std::string::npos ? std::string() : text.substr(0, end + 1);
}

std::string normalizeMatchText(const std::string& text, bool stripBracketed = false)
{
	std::string normalized;
	bool inBracket = false;
	for (char c : text)
	{
		if (stripBracketed && (c == '(' || c == '[' || c == '{'))
		{
			inBracket = true;
			continue;
		}
		if (stripBracketed && (c == ')' || c == ']' || c == '}'))
		{
			inBracket = false;
			continue;
		}
		if (inBracket)
			continue;

		const unsigned char ch = static_cast<unsigned char>(c);
		if (std::isalnum(ch))
			normalized += static_cast<char>(std::toupper(ch));
	}
	return normalized;
}

std::vector<std::string> makeTitleTokens(const std::string& title)
{
	static const std::array<std::string_view, 11> ignored {
		"USA", "EUR", "EUROPE", "JAPAN", "JPN", "DISC", "DISK", "REV", "THE", "AND", "VER",
	};

	std::vector<std::string> tokens;
	std::string current;
	for (char c : title)
	{
		const unsigned char ch = static_cast<unsigned char>(c);
		if (std::isalnum(ch))
			current += static_cast<char>(std::toupper(ch));
		else if (!current.empty())
		{
			if (current.size() >= 3 && std::find_if(ignored.begin(), ignored.end(),
					[&current](std::string_view value) { return value == current; }) == ignored.end())
				tokens.push_back(current);
			current.clear();
		}
	}
	if (current.size() >= 3 && std::find_if(ignored.begin(), ignored.end(),
			[&current](std::string_view value) { return value == current; }) == ignored.end())
		tokens.push_back(current);
	return tokens;
}

bool readIconHeaderInfo(const std::vector<u8>& fileData, size_t headerOffset, u32& frameCount, u16& animationSpeed)
{
	if (headerOffset + 0x80 > fileData.size())
		return false;

	const u8 *header = fileData.data() + headerOffset;
	frameCount = std::min<u32>(readLe16(header + 0x40), 3);
	animationSpeed = readLe16(header + 0x42);
	return frameCount != 0 && headerOffset + 0x80 + frameCount * VMU_BLOCK_SIZE <= fileData.size();
}

int scoreIconCandidate(const std::string& dirName, const std::vector<u8>& fileData,
		size_t headerOffset, const std::string& gameId, const std::string& gameTitle)
{
	const u8 *header = fileData.data() + headerOffset;
	const std::string metadata = normalizeMatchText(dirName
			+ " " + readVmuText(header, 16)
			+ " " + readVmuText(header + 0x10, 32)
			+ " " + readVmuText(header + 0x30, 16));

	int score = 0;
	const std::string gameIdText = normalizeMatchText(gameId);
	if (!gameIdText.empty() && metadata.find(gameIdText) != std::string::npos)
		score += 120;

	const std::string titleText = normalizeMatchText(gameTitle, true);
	if (titleText.size() >= 4 && !metadata.empty()
			&& (metadata.find(titleText) != std::string::npos || (metadata.size() >= 4 && titleText.find(metadata) != std::string::npos)))
		score += 100;

	for (const std::string& token : makeTitleTokens(gameTitle))
		if (metadata.find(token) != std::string::npos)
			score += 30;
	return score;
}

std::vector<u8> decodeIconFrame(const u8 *palette, const u8 *frame)
{
	std::array<u32, 16> colors;
	for (size_t i = 0; i < colors.size(); i++)
	{
		const u16 color = readLe16(palette + i * 2);
		const u8 a = ((color >> 12) & 0xf) * 17;
		const u8 r = ((color >> 8) & 0xf) * 17;
		const u8 g = ((color >> 4) & 0xf) * 17;
		const u8 b = (color & 0xf) * 17;
		colors[i] = r | (g << 8) | (b << 16) | (a << 24);
	}

	std::vector<u8> rgba(32 * 32 * 4);
	for (size_t y = 0; y < 32; y++)
	{
		for (size_t x = 0; x < 32; x += 2)
		{
			const u8 packed = frame[y * 16 + x / 2];
			const u8 indexes[2] = { static_cast<u8>(packed >> 4), static_cast<u8>(packed & 0xf) };
			for (size_t i = 0; i < 2; i++)
			{
				const u32 color = colors[indexes[i]];
				u8 *pixel = &rgba[(y * 32 + x + i) * 4];
				pixel[0] = color & 0xff;
				pixel[1] = (color >> 8) & 0xff;
				pixel[2] = (color >> 16) & 0xff;
				pixel[3] = (color >> 24) & 0xff;
			}
		}
	}
	return rgba;
}

bool extractIconFromVmsHeader(const std::vector<u8>& fileData, size_t headerOffset, const std::string& cacheDir,
		u32& frameCount, u16& animationSpeed)
{
	if (!readIconHeaderInfo(fileData, headerOffset, frameCount, animationSpeed))
		return false;

	const u8 *header = fileData.data() + headerOffset;
	bool wroteAny = false;
	for (u32 i = 0; i < frameCount; i++)
	{
		const std::vector<u8> rgba = decodeIconFrame(header + 0x60, header + 0x80 + i * VMU_BLOCK_SIZE);
		if (i == 0)
			wroteAny = saveIconPng(appendPath(cacheDir, "icon.png"), rgba);
		wroteAny = saveIconPng(framePath(cacheDir, i), rgba) || wroteAny;
	}
	return wroteAny;
}

bool readVmuFile(const std::vector<u8>& flash, const u8 *entry, const u16 *fat, std::vector<u8>& fileData)
{
	const u16 firstBlock = readLe16(entry + 0x02);
	const u16 blockCount = readLe16(entry + 0x18);
	if (firstBlock >= VMU_BLOCK_COUNT || blockCount == 0 || blockCount > 200)
		return false;

	fileData.clear();
	fileData.reserve(blockCount * VMU_BLOCK_SIZE);
	u16 block = firstBlock;
	for (u16 i = 0; i < blockCount; i++)
	{
		if (block >= VMU_BLOCK_COUNT)
			return false;
		fileData.insert(fileData.end(), flash.begin() + block * VMU_BLOCK_SIZE, flash.begin() + (block + 1) * VMU_BLOCK_SIZE);
		const u16 next = fat[block];
		if (next == VMU_FAT_END)
			break;
		if (next == VMU_FAT_FREE || next >= VMU_BLOCK_COUNT)
			return false;
		block = next;
	}
	return !fileData.empty();
}

bool extractBestIconFromFlash(const std::vector<u8>& flash, const std::string& cacheDir,
		const std::string& gameId, const std::string& gameTitle, u32& frameCount, u16& animationSpeed)
{
	if (flash.size() != VMU_FLASH_SIZE)
		return false;

	const u8 *root = flash.data() + 255 * VMU_BLOCK_SIZE;
	u16 fatStart = readLe16(root + 0x46);
	u16 dirStart = readLe16(root + 0x4a);
	u16 dirBlocks = readLe16(root + 0x4c);
	if (fatStart >= VMU_BLOCK_COUNT)
		fatStart = 254;
	if (dirStart >= VMU_BLOCK_COUNT)
		dirStart = 253;
	if (dirBlocks == 0 || dirBlocks > 13)
		dirBlocks = 13;

	std::array<u16, VMU_BLOCK_COUNT> fat;
	const u8 *fatData = flash.data() + fatStart * VMU_BLOCK_SIZE;
	for (size_t i = 0; i < fat.size(); i++)
		fat[i] = readLe16(fatData + i * 2);

	struct IconCandidate
	{
		std::vector<u8> fileData;
		size_t headerOffset = 0;
		int score = 0;
	};
	std::vector<IconCandidate> candidates;

	for (u16 dirBlockIndex = 0; dirBlockIndex < dirBlocks; dirBlockIndex++)
	{
		if (dirStart < dirBlockIndex)
			break;
		const u16 dirBlock = dirStart - dirBlockIndex;
		const u8 *dir = flash.data() + dirBlock * VMU_BLOCK_SIZE;
		for (size_t entryOffset = 0; entryOffset < VMU_BLOCK_SIZE; entryOffset += 32)
		{
			const u8 *entry = dir + entryOffset;
			if (entry[0] != 0x33 && entry[0] != 0xcc)
				continue;

			std::vector<u8> fileData;
			if (!readVmuFile(flash, entry, fat.data(), fileData))
				continue;

			u32 candidateFrameCount = 0;
			u16 candidateAnimationSpeed = 0;
			const size_t headerOffset = readLe16(entry + 0x1a) * VMU_BLOCK_SIZE;
			if (!readIconHeaderInfo(fileData, headerOffset, candidateFrameCount, candidateAnimationSpeed))
				continue;

			const std::string dirName = readVmuText(entry + 0x04, 12);
			const int score = scoreIconCandidate(dirName, fileData, headerOffset, gameId, gameTitle);
			candidates.push_back({ std::move(fileData), headerOffset, score });
		}
	}

	if (candidates.empty())
		return false;

	const auto best = std::max_element(candidates.begin(), candidates.end(),
			[](const IconCandidate& left, const IconCandidate& right) { return left.score < right.score; });
	if (best->score == 0 && candidates.size() > 1)
		return false;
	return extractIconFromVmsHeader(best->fileData, best->headerOffset, cacheDir, frameCount, animationSpeed);
}

} // namespace

bool cacheVmuIconFromFlash(const std::string& gameId, const std::string& gameTitle, const void *data, size_t size)
{
	std::lock_guard<std::mutex> lock(cacheMutex);

	const std::string key = sanitizeCacheKey(gameId);
	if (key.empty() || data == nullptr || size != VMU_FLASH_SIZE)
		return false;

	const std::string libraryDir = getVmuIconLibraryDir();
	if (libraryDir.empty())
		return false;

	const std::string cacheDir = getVmuIconCacheDir(libraryDir, key);
	if (cacheDir.empty())
		return false;

	const u8 *bytes = static_cast<const u8 *>(data);
	std::vector<u8> flash(bytes, bytes + size);
	u32 frameCount = 0;
	u16 animationSpeed = 0;
	const bool extracted = extractBestIconFromFlash(flash, cacheDir, gameId, gameTitle, frameCount, animationSpeed);

	const std::string cachePath = appendPath(libraryDir, "cache.json");
	json& cache = getSharedIconCache(cachePath);
	updateLiveIconCacheEntry(cache, key, extracted, frameCount, animationSpeed);
	saveSharedIconCache();
	return extracted;
}

void markLibraryGameBooted(const std::string& gameId, const std::string& gamePath)
{
	std::string key = sanitizeCacheKey(gameId);
	if (key.empty())
		key = sanitizeCacheKey(get_file_basename(gamePath));
	if (key.empty() || gamePath.empty())
		return;

	std::lock_guard<std::mutex> lock(cacheMutex);
	const std::string libraryDir = getVmuIconLibraryDir();
	if (libraryDir.empty())
		return;

	const std::string cachePath = appendPath(libraryDir, "cache.json");
	json& cache = getSharedIconCache(cachePath);
	updateLastBootedCacheEntry(cache, key, gamePath, std::time(nullptr));
	saveSharedIconCache();
}

time_t getLibraryGameLastBooted(const GameMedia& media, const std::string& gameId)
{
	std::lock_guard<std::mutex> lock(cacheMutex);
	const std::string key = makeCacheKey(media, gameId);
	if (key.empty() && media.path.empty())
		return 0;

	const std::string libraryDir = getVmuIconLibraryDir();
	if (libraryDir.empty())
		return 0;

	const std::string cachePath = appendPath(libraryDir, "cache.json");
	return readLastBootedCacheEntry(getSharedIconCache(cachePath), key, media.path);
}

std::string getCachedVmuIconPath(const GameMedia& media, const std::string& gameId, bool animate, double animationClock)
{
	std::lock_guard<std::mutex> lock(cacheMutex);

	if (!config::PerGameVmu)
		return {};

	const std::string key = makeCacheKey(media, gameId);
	if (key.empty())
		return {};

	const std::string libraryDir = getVmuIconLibraryDir();
	if (libraryDir.empty())
		return {};

	const std::string cacheDir = getVmuIconCacheDir(libraryDir, key);
	if (cacheDir.empty())
		return {};

	const std::string cachePath = appendPath(libraryDir, "cache.json");
	json& cache = getSharedIconCache(cachePath);
	u32 cachedFrameCount = 0;
	u16 cachedAnimationSpeed = 0;
	if (cachedLiveIconEntry(cache, key, cachedFrameCount, cachedAnimationSpeed))
		return selectCachedIconPath(cacheDir, cachedFrameCount, cachedAnimationSpeed, animate, animationClock);

	const std::string vmuPath = findPerGameVmuPath(media, gameId);
	if (vmuPath.empty())
	{
		u32 frameCount = 0;
		u16 animationSpeed = 0;
		if (!cachedAnyIconEntry(cache, key, frameCount, animationSpeed))
			return {};
		return selectCachedIconPath(cacheDir, frameCount, animationSpeed, animate, animationClock);
	}

	const hostfs::FileInfo vmuInfo = getFileInfo(vmuPath);

	bool iconWasExtracted = false;
	if (cachedEntryMatches(cache, key, vmuInfo, iconWasExtracted))
	{
		if (!iconWasExtracted)
			return {};
		const json& entry = cache["games"][key];
		const u32 frameCount = entry.value("frames", 1);
		const u16 animationSpeed = entry.value("speed", 0);
		if (!animate || frameCount <= 1 || hasCachedAnimationFrames(cacheDir, frameCount))
		{
			const std::string cachedPath = selectCachedIconPath(cacheDir, frameCount, animationSpeed, animate, animationClock);
			if (!cachedPath.empty())
				return cachedPath;
		}
	}

	std::vector<u8> flash;
	u32 frameCount = 0;
	u16 animationSpeed = 0;
	const bool extracted = readFile(vmuPath, flash) && extractBestIconFromFlash(flash, cacheDir, gameId, media.name, frameCount, animationSpeed);
	updateIconCacheEntry(cache, key, vmuInfo, extracted, frameCount, animationSpeed);
	saveSharedIconCache();
	return extracted ? selectCachedIconPath(cacheDir, frameCount, animationSpeed, animate, animationClock) : std::string();
}
