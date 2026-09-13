/*
	Copyright 2025 flyinghead

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
#include "settings.h"
#include "gui.h"
#include "settings_new.h"
#include "oslib/storage.h"

namespace {

void addContentPathCallback(const std::string& path)
{
	auto& contentPath = config::ContentPath.get();
	if (std::count(contentPath.begin(), contentPath.end(), path) == 0)
	{
		scanner.stop();
		contentPath.push_back(path);
		if (gui_state == GuiState::Main)
			SaveSettings();
		scanner.refresh();
	}
}

} // namespace

void addContentPath(bool start)
{
	const char* title = "Select a Content Folder";
	select_file_popup(title, [](bool cancelled, std::string selection) {
		if (!cancelled)
			addContentPathCallback(selection);
		return true;
	});
#ifdef __ANDROID__
	if (start)
	{
		bool supported = hostfs::addStorage(true, false, title, [](bool cancelled, std::string selection) {
			if (!cancelled)
				addContentPathCallback(selection);
		});
		if (!supported)
			ImGui::OpenPopup(title);
	}
#else
	if (start)
		ImGui::OpenPopup(title);
#endif
}

// Legacy compatibility entry point. The active General settings tab is implemented
// in settings_new.cpp; this symbol is preserved for compatibility and merge stability.
void gui_settings_general()
{
<<<<<<< HEAD
	SettingsNew::renderGeneralTab();
=======
	struct
	{
		const char* label;
		const char* value;
	}
	UILanguages[] = {
		{ T("System Default"), "" },
		{ "English", "en" },
		{ "Français", "fr" },
		{ "Magyar", "hu" },
		{ "日本語", "ja" },
		{ "Português (Brasil)", "pt_BR" },
		{ "Svenska", "sv" },
		{ "Türkçe", "tr" },
		{ "简体中文", "zh_CN" },
		{ "正體中文", "zh_TW" },
	};

	// Determine the preview text
	const std::string currentLanguage = config::UILanguage.get();
	std::string preview;
	for (const auto& it : UILanguages)
	{
		if (currentLanguage == it.value) {
			preview = it.label;
			break;
		}
	}

	if (ImGui::BeginCombo(T("UI Language"), preview.c_str()))
	{
		for (const auto& lang : UILanguages)
		{
			const bool selected = (currentLanguage == lang.value);

			if (ImGui::Selectable(lang.label, selected)) {
				config::UILanguage = lang.value;
				i18n::reloadLanguage();
			}
			if (selected)
				ImGui::SetItemDefaultFocus();
		}

		ImGui::EndCombo();
	}

	{
		DisabledScope scope(settings.platform.isArcade());

		const char *languages[] = { T("Japanese"), T("English"), T("German"), T("French"), T("Spanish"), T("Italian"), T("Default") };
		OptionComboBox(T("Dreamcast Language"), config::Language, languages, std::size(languages),
				T("The language as configured in the Dreamcast BIOS"));

		const char *broadcast[] = { "NTSC", "PAL", "PAL/M", "PAL/N", T("Default") };
		OptionComboBox(T("Broadcast"), config::Broadcast, broadcast, std::size(broadcast),
				T("TV broadcasting standard for non-VGA modes"));
	}

	const char *consoleRegion[] = { T("Japan"), T("USA"), T("Europe"), T("Default") };
	const char *arcadeRegion[] = { T("Japan"), T("USA"), i18n::translateCtx("region", "Export"), T("Korea") };
	const char **region = settings.platform.isArcade() ? arcadeRegion : consoleRegion;
	OptionComboBox(T("Region"), config::Region, region, std::size(consoleRegion),
			T("BIOS region"));

	const char *cable[] = { T("VGA"), T("RGB Component"), T("TV Composite") };
	{
		DisabledScope scope(config::Cable.isReadOnly() || settings.platform.isArcade());

		const char *value = config::Cable == 0 ? cable[0]
				: config::Cable > 0 && config::Cable <= (int)std::size(cable) ? cable[config::Cable - 1]
				: "?";
		if (ImGui::BeginCombo(T("Cable"), value, ImGuiComboFlags_None))
		{
			for (int i = 0; i < IM_ARRAYSIZE(cable); i++)
			{
				bool is_selected = i == 0 ? config::Cable <= 1 : config::Cable - 1 == i;
				if (ImGui::Selectable(cable[i], &is_selected))
					config::Cable = i == 0 ? 0 : i + 1;
				if (is_selected)
					ImGui::SetItemDefaultFocus();
			}
			ImGui::EndCombo();
		}
        ImGui::SameLine();
        ShowHelpMarker(T("Video connection type"));
	}

#if !defined(TARGET_IPHONE)
    ImVec2 size;
    size.x = 0.0f;
    size.y = (ImGui::GetTextLineHeightWithSpacing() + ImGui::GetStyle().FramePadding.y * 2.f)
    				* (config::ContentPath.get().size() + 1);

    ImVec2 childSize;
    if (beginFrame(T("Content Location"), size, &childSize))
    {
    	int to_delete = -1;
        for (u32 i = 0; i < config::ContentPath.get().size(); i++)
        {
        	ImguiID _(config::ContentPath.get()[i].c_str());
            ImGui::AlignTextToFramePadding();
            float maxW = childSize.x - ImGui::CalcTextSize(ICON_FA_TRASH_CAN).x - ImGui::GetStyle().FramePadding.x * 2
            		 - ImGui::GetStyle().ItemSpacing.x;
            std::string s = middleEllipsis(config::ContentPath.get()[i], maxW);
        	ImGui::Text("%s", s.c_str());
        	ImGui::SameLine(0, maxW - ImGui::CalcTextSize(s.c_str()).x + ImGui::GetStyle().ItemSpacing.x);
        	if (ImGui::Button(ICON_FA_TRASH_CAN))
        		to_delete = i;
        }

        ImguiStyleVar _(ImGuiStyleVar_FramePadding, ScaledVec2(24, 3));
        const bool addContent = ImGui::Button((T("Add") + std::string("##") + "ContentLocation").c_str());
        addContentPath(addContent);
        ImGui::SameLine();

        if (ImGui::Button(T("Rescan Content")))
			scanner.refresh();

		endFrame();
    	if (to_delete >= 0)
    	{
    		scanner.stop();
    		config::ContentPath.get().erase(config::ContentPath.get().begin() + to_delete);
			scanner.refresh();
    	}
    }
    ImGui::SameLine();
    ShowHelpMarker(T("The folders where your games are stored"));

    size.y = ImGui::GetTextLineHeightWithSpacing() + ImGui::GetStyle().FramePadding.y * 2.0f;
    ImGui::Spacing();

#if defined(__linux__) && !defined(__ANDROID__)
    if (beginFrame(T("Data Folder"), size, &childSize))
    {
    	float w = childSize.x - ImGui::GetStyle().FramePadding.x;
    	std::string s = middleEllipsis(get_writable_data_path(""), w);
        ImGui::Text("%s", s.c_str());
        endFrame();
    }
    ImGui::SameLine();
    ShowHelpMarker(T("The folder containing BIOS files, as well as saved VMUs and states"));
#else
#if defined(__ANDROID__) || defined(TARGET_MAC)
    size.y += ImGui::GetTextLineHeightWithSpacing();
#endif
    if (beginFrame(T("Home Folder"), size, &childSize))
    {
    	float w = childSize.x - ImGui::GetStyle().FramePadding.x;
    	std::string s = middleEllipsis(get_writable_config_path(""), w);
        ImGui::Text("%s", s.c_str());
        ImguiStyleVar _(ImGuiStyleVar_FramePadding, ScaledVec2(24, 3));
#ifdef __ANDROID__
        {
        	DisabledScope _(!config::UseSafFilePicker);
			if (ImGui::Button(i18n::translateCtx("action", "Import")))
				hostfs::importHomeDirectory();
			ImGui::SameLine();
			if (ImGui::Button(i18n::translateCtx("action", "Export")))
				hostfs::exportHomeDirectory();
        }
#endif
#ifdef TARGET_MAC
        if (ImGui::Button(T("Reveal in Finder")))
        {
            char temp[512];
            snprintf(temp, sizeof(temp), "open \"%s\"", get_writable_config_path("").c_str());
            system(temp);
        }
#endif
        endFrame();
    }
    ImGui::SameLine();
    ShowHelpMarker(T("The folder where Flycast saves configuration files and VMUs. BIOS files should be in a subfolder named \"data\""));
#endif // !linux
    ImGui::Spacing();
#else // TARGET_IPHONE
    {
    	ImguiStyleVar _(ImGuiStyleVar_FramePadding, ScaledVec2(24, 3));
		if (ImGui::Button(T("Rescan Content")))
			scanner.refresh();
    }
#endif
    ImGui::Spacing();
	OptionCheckbox(T("Box Art Game List"), config::BoxartDisplayMode,
			T("Display game cover art in the game list."));
	OptionCheckbox(T("Fetch Box Art"), config::FetchBoxart,
			T("Fetch cover images from TheGamesDB.net."));
	if (OptionSlider(T("UI Scaling"), config::UIScaling, 50, 200, T("Adjust the size of UI elements and fonts."), "%d%%"))
		uiUserScaleUpdated = true;
	if (uiUserScaleUpdated)
	{
		ImGui::SameLine();
		if (ImGui::Button(T("Apply"))) {
			mainui_reinit();
			uiUserScaleUpdated = false;
		}
	}

	const char *themes[] = { T("Dark"), T("Light"), T("Dreamcast"), T("High Contrast"), T("Nintendo"), T("Aqua Chill") };
	int previousUITheme = config::UITheme;
	OptionComboBox(T("UI Theme"), config::UITheme, themes, std::size(themes),
			T("Select the UI color theme."));
	// Auto-apply theme when selection changes
	if (previousUITheme != config::UITheme) {
		applyCurrentTheme();
	}

	if (OptionCheckbox(T("Hide Legacy Naomi Roms"), config::HideLegacyNaomiRoms,
			T("Hide .bin, .dat and .lst files from the content browser")))
		scanner.refresh();
#ifdef __ANDROID__
	OptionCheckbox(T("Use SAF File Picker"), config::UseSafFilePicker,
			T("Use Android Storage Access Framework file picker to select folders and files. Ignored on Android 10 and later."));
#endif

	ImGui::Text("%s", T("Automatic State:"));
	OptionCheckbox(T("Load"), config::AutoLoadState,
			T("Load the last saved state of the game when starting"));
	ImGui::SameLine();
	OptionCheckbox(T("Save"), config::AutoSaveState,
			T("Save the state of the game when stopping"));
	OptionCheckbox(T("Naomi Free Play"), config::ForceFreePlay, T("Configure Naomi games in Free Play mode."));
#if USE_DISCORD
	OptionCheckbox(T("Discord Presence"), config::DiscordPresence, T("Show which game you are playing on Discord"));
#endif
#ifdef USE_RACHIEVEMENTS
	OptionCheckbox(T("Enable RetroAchievements"), config::EnableAchievements, T("Track your game achievements using RetroAchievements.org"));
	{
		DisabledScope _(!config::EnableAchievements);
		ImGui::Indent();
		OptionCheckbox(T("Hardcore Mode"), config::AchievementsHardcoreMode,
				T("Enable RetroAchievements hardcore mode. Using cheats and loading a state are not allowed in this mode."));
		InputText(T("Username"), &config::AchievementsUserName.get(),
				achievements::isLoggedOn() ? ImGuiInputTextFlags_ReadOnly : ImGuiInputTextFlags_None);
		if (config::EnableAchievements)
		{
			static std::future<void> futureLogin;
			achievements::init();
			if (achievements::isLoggedOn())
			{
				ImGui::Text("%s", T("Authentication successful"));
				if (futureLogin.valid())
					futureLogin.get();
				if (ImGui::Button(T("Logout"), ScaledVec2(100, 0)))
					achievements::logout();
			}
			else
			{
				static char password[256];
				InputText(T("Password"), password, sizeof(password), ImGuiInputTextFlags_Password);
				if (futureLogin.valid())
				{
					if (futureLogin.wait_for(std::chrono::seconds::zero()) == std::future_status::timeout) {
						ImGui::Text("%s", T("Authenticating..."));
					}
					else
					{
						try {
							futureLogin.get();
						} catch (const FlycastException& e) {
							gui_error(e.what());
						}
					}
				}
				{
					DisabledScope _(config::AchievementsUserName.get().empty() || password[0] == '\0');
					if (ImGui::Button(T("Login"), ScaledVec2(100, 0)) && !futureLogin.valid())
					{
						futureLogin = achievements::login(config::AchievementsUserName.get().c_str(), password);
						memset(password, 0, sizeof(password));
					}
				}
			}
		}
		ImGui::Unindent();
	}
#endif

// Custom Paths section - hidden on Android and iOS
#if !defined(TARGET_IPHONE)
    ImGui::Spacing();
    header(T("Custom Paths"));

    managePathList(T("BIOS Folders"), T("Select a BIOS folder"), config::BiosPath.get(),
    		T("Folders containing BIOS files (e.g. dc_boot.bin or dc_bios.bin) and arcade BIOS"));
    ImGui::Spacing();

#if !defined(__ANDROID__)
    manageSinglePath(T("VMU Folder"), T("Select the VMU folder"), config::VMUPath,
    		T("Folder where VMU (.bin) saves are stored"));
    ImGui::Spacing();

    managePathList(T("Savestate Folders"), T("Select a savestate folder"), config::SavestatePath.get(),
    		T("Folders for save states. First path is used for new states; all are searched when loading"));
    ImGui::Spacing();

    manageSinglePath(T("Game Save Folder"), T("Select the game save folder"), config::SavePath,
    		T("Folder for game save data (e.g. arcade NVRAM)"));
    ImGui::Spacing();
#endif

    managePathList(T("Texture Pack Folders"), T("Select a texture pack folder"), config::TexturePath.get(),
    		T("Folders containing textures/<gameId> or <gameId> under a textures subfolder"));
    ImGui::Spacing();

#if !defined(__ANDROID__)
    manageSinglePath(T("Texture Dump Folder"), T("Select the texture dump folder"), config::TextureDumpPath,
    		T("Folder where texture dumps are saved. Game-specific subfolders will be created automatically"));
    ImGui::Spacing();
    
    manageSinglePath(T("Box Art Folder"), T("Select the box art folder"), config::BoxartPath,
    		T("Folder containing box art images (png/jpg). If empty, Flycast will use the default Home Folder/boxart for downloads and generated art"));
    ImGui::Spacing();

    managePathList(T("Controller Mapping Folders"), T("Select a controller mapping folder"), config::MappingsPath.get(),
    		T("Folders containing controller mapping files (.cfg). The emulator also looks in Home Folder/mappings. Per-game mappings are suffixed with _<gameId>.cfg"));
    ImGui::Spacing();

    managePathList(T("Cheat Folders"), T("Select a cheat folder"), config::CheatPath.get(),
    		T("Folders containing cheat files (.cht/.txt) named with the game ID. Flycast will auto-load matching files if present"));
    ImGui::Spacing();
#endif  // !ANDROID
#endif  // !IPHONE
>>>>>>> flycast/dev
}

static void applyDarkTheme()
{
	// Reset style state before applying the theme palette.
	ImGui::GetStyle() = ImGuiStyle{};
	ImGui::StyleColorsDark();

	// Theme-specific style adjustments.
	ImGuiStyle& style = ImGui::GetStyle();
	style.TabRounding = 5.0f;
	style.FrameRounding = 3.0f;
	style.ItemSpacing = ImVec2(8, 8);
	style.ItemInnerSpacing = ImVec2(4, 6);

	// Reset style properties to defaults to ensure clean theme switching
	style.TabBorderSize = 0.0f;
	style.FrameBorderSize = 0.0f;

	// Ensure menu bar uses dark theme colors
	style.Colors[ImGuiCol_MenuBarBg] = ImVec4(0.10f, 0.10f, 0.10f, 1.00f);
	style.Colors[ImGuiCol_Border] = ImVec4(0.30f, 0.30f, 0.35f, 0.50f);

	// High-contrast text colors.
	style.Colors[ImGuiCol_Text] = ImVec4(1.00f, 1.00f, 1.00f, 1.00f);
	style.Colors[ImGuiCol_TextDisabled] = ImVec4(0.70f, 0.70f, 0.70f, 1.00f);

	// Darker blue highlight/hover color for better text readability
	style.Colors[ImGuiCol_Header] = ImVec4(0.10f, 0.25f, 0.45f, 0.80f);
	style.Colors[ImGuiCol_HeaderHovered] = ImVec4(0.15f, 0.35f, 0.55f, 1.00f);
	style.Colors[ImGuiCol_HeaderActive] = ImVec4(0.20f, 0.40f, 0.60f, 1.00f);

	// Popup background - solid dark
	style.Colors[ImGuiCol_PopupBg] = ImVec4(0.12f, 0.12f, 0.14f, 1.00f);

	// Frame backgrounds
	style.Colors[ImGuiCol_FrameBg] = ImVec4(0.20f, 0.20f, 0.24f, 1.00f);
	style.Colors[ImGuiCol_FrameBgHovered] = ImVec4(0.15f, 0.35f, 0.55f, 1.00f);
	style.Colors[ImGuiCol_FrameBgActive] = ImVec4(0.20f, 0.40f, 0.60f, 1.00f);
}

static void applyLightTheme()
{
	ImGui::StyleColorsLight();
	ImGuiStyle& style = ImGui::GetStyle();

	// High-contrast text colors for light background.
	style.Colors[ImGuiCol_Text] = ImVec4(0.00f, 0.00f, 0.00f, 1.00f);
	style.Colors[ImGuiCol_TextDisabled] = ImVec4(0.40f, 0.40f, 0.40f, 1.00f);

	// Neutral light gray background. This keeps the light theme from looking
	// white-washed while still giving dark text a clear surface.
	style.Colors[ImGuiCol_WindowBg] = ImVec4(0.88f, 0.88f, 0.89f, 1.00f);
	style.Colors[ImGuiCol_ChildBg] = ImVec4(0.88f, 0.88f, 0.89f, 1.00f);

	// Keep dialogs on the light side too. PopupStyleScope derives popup text from
	// this surface, so a mid/dark popup color can make theme-switched dialogs look
	// muddy or leave text with poor contrast.
	style.Colors[ImGuiCol_PopupBg] = ImVec4(0.92f, 0.91f, 0.94f, 1.00f);

	style.Colors[ImGuiCol_Border] = ImVec4(0.56f, 0.56f, 0.64f, 0.60f);
	style.Colors[ImGuiCol_BorderShadow] = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);

	// Light lavender controls preserve the theme accent while staying safely
	// behind dark text in rows, inputs, sliders, and popup selections.
	style.Colors[ImGuiCol_FrameBg] = ImVec4(0.82f, 0.82f, 0.91f, 1.00f);
	style.Colors[ImGuiCol_FrameBgHovered] = ImVec4(0.76f, 0.77f, 0.88f, 1.00f);
	style.Colors[ImGuiCol_FrameBgActive] = ImVec4(0.68f, 0.70f, 0.82f, 1.00f);

	style.Colors[ImGuiCol_TitleBg] = ImVec4(0.70f, 0.70f, 0.85f, 1.00f);
	style.Colors[ImGuiCol_TitleBgActive] = ImVec4(0.60f, 0.60f, 0.80f, 1.00f);
	style.Colors[ImGuiCol_TitleBgCollapsed] = ImVec4(0.80f, 0.80f, 0.90f, 0.75f);

	// Menu bar - lighter purple (top toolbar color).
	style.Colors[ImGuiCol_MenuBarBg] = ImVec4(0.76f, 0.76f, 0.88f, 1.00f);

	style.Colors[ImGuiCol_ScrollbarBg] = ImVec4(0.84f, 0.84f, 0.85f, 1.00f);
	style.Colors[ImGuiCol_ScrollbarGrab] = ImVec4(0.58f, 0.58f, 0.72f, 1.00f);
	style.Colors[ImGuiCol_ScrollbarGrabHovered] = ImVec4(0.50f, 0.52f, 0.68f, 1.00f);
	style.Colors[ImGuiCol_ScrollbarGrabActive] = ImVec4(0.42f, 0.46f, 0.62f, 1.00f);

	style.Colors[ImGuiCol_CheckMark] = ImVec4(0.00f, 0.35f, 0.75f, 1.00f);

	style.Colors[ImGuiCol_SliderGrab] = ImVec4(0.36f, 0.45f, 0.78f, 1.00f);
	style.Colors[ImGuiCol_SliderGrabActive] = ImVec4(0.24f, 0.34f, 0.66f, 1.00f);

	style.Colors[ImGuiCol_Button] = ImVec4(0.78f, 0.78f, 0.90f, 1.00f);
	style.Colors[ImGuiCol_ButtonHovered] = ImVec4(0.70f, 0.72f, 0.86f, 1.00f);
	style.Colors[ImGuiCol_ButtonActive] = ImVec4(0.60f, 0.64f, 0.80f, 1.00f);

	// Header colors are row fills under normal text, including popup choices.
	style.Colors[ImGuiCol_Header] = ImVec4(0.78f, 0.80f, 0.91f, 1.00f);
	style.Colors[ImGuiCol_HeaderHovered] = ImVec4(0.68f, 0.72f, 0.86f, 1.00f);
	style.Colors[ImGuiCol_HeaderActive] = ImVec4(0.56f, 0.62f, 0.78f, 1.00f);

	style.Colors[ImGuiCol_Separator] = ImVec4(0.58f, 0.58f, 0.70f, 0.70f);

	// Tabs - using lighter purple for active tabs
	style.Colors[ImGuiCol_Tab] = ImVec4(0.80f, 0.80f, 0.90f, 1.00f);
	style.Colors[ImGuiCol_TabHovered] = ImVec4(0.70f, 0.72f, 0.86f, 1.00f);
	style.Colors[ImGuiCol_TabActive] = ImVec4(0.64f, 0.68f, 0.84f, 1.00f);
	style.Colors[ImGuiCol_TabUnfocused] = ImVec4(0.84f, 0.84f, 0.86f, 1.00f);
	style.Colors[ImGuiCol_TabUnfocusedActive] = ImVec4(0.74f, 0.76f, 0.86f, 1.00f);

	style.Colors[ImGuiCol_TextSelectedBg] = ImVec4(0.58f, 0.66f, 0.88f, 0.45f);

	// Table colors
	style.Colors[ImGuiCol_TableHeaderBg] = ImVec4(0.76f, 0.76f, 0.88f, 1.00f);
	style.Colors[ImGuiCol_TableBorderStrong] = ImVec4(0.56f, 0.56f, 0.72f, 1.00f);
	style.Colors[ImGuiCol_TableBorderLight] = ImVec4(0.70f, 0.70f, 0.82f, 0.80f);
	style.Colors[ImGuiCol_TableRowBg] = ImVec4(0.88f, 0.88f, 0.89f, 1.00f);
	style.Colors[ImGuiCol_TableRowBgAlt] = ImVec4(0.84f, 0.84f, 0.86f, 1.00f);

	style.TabBorderSize = 0.0f;
	style.FrameBorderSize = 0.0f;
}

static void applyDreamcastTheme()
{
	ImGui::StyleColorsDark();
	ImGuiStyle& style = ImGui::GetStyle();

	// Dreamcast-inspired palette.
	style.Colors[ImGuiCol_Text] = ImVec4(1.00f, 1.00f, 1.00f, 1.00f);
	style.Colors[ImGuiCol_TextDisabled] = ImVec4(0.70f, 0.70f, 0.70f, 0.50f);

	// Background and frame colors.
	style.Colors[ImGuiCol_WindowBg] = ImVec4(0.05f, 0.07f, 0.10f, 1.00f);
	style.Colors[ImGuiCol_ChildBg] = ImVec4(0.05f, 0.07f, 0.10f, 1.00f);
	style.Colors[ImGuiCol_TitleBg] = ImVec4(0.13f, 0.16f, 0.40f, 1.00f);
	style.Colors[ImGuiCol_TitleBgActive] = ImVec4(0.18f, 0.22f, 0.50f, 1.00f);
	style.Colors[ImGuiCol_TitleBgCollapsed] = ImVec4(0.00f, 0.43f, 0.73f, 0.70f);
	style.Colors[ImGuiCol_MenuBarBg] = ImVec4(0.10f, 0.12f, 0.15f, 1.00f);
	style.Colors[ImGuiCol_ScrollbarBg] = ImVec4(0.05f, 0.07f, 0.10f, 1.00f);
	style.Colors[ImGuiCol_ScrollbarGrab] = ImVec4(0.95f, 0.95f, 0.95f, 1.00f);
	style.Colors[ImGuiCol_ScrollbarGrabHovered] = ImVec4(0.95f, 0.95f, 0.95f, 1.00f);
	style.Colors[ImGuiCol_ScrollbarGrabActive] = ImVec4(0.95f, 0.95f, 0.95f, 1.00f);

	// Controller button colors - more vibrant
	style.Colors[ImGuiCol_Button] = ImVec4(0.90f, 0.50f, 0.10f, 1.00f);
	style.Colors[ImGuiCol_ButtonHovered] = ImVec4(1.00f, 0.70f, 0.40f, 1.00f);
	style.Colors[ImGuiCol_ButtonActive] = ImVec4(0.95f, 0.45f, 0.00f, 1.00f);

	style.Colors[ImGuiCol_PopupBg] = ImVec4(0.05f, 0.07f, 0.10f, 1.00f);
	style.Colors[ImGuiCol_FrameBg] = ImVec4(0.15f, 0.17f, 0.22f, 1.00f);
	style.Colors[ImGuiCol_FrameBgHovered] = ImVec4(0.10f, 0.70f, 0.50f, 1.00f);
	style.Colors[ImGuiCol_FrameBgActive] = ImVec4(0.06f, 0.60f, 0.35f, 1.00f);

	style.Colors[ImGuiCol_Header] = ImVec4(0.15f, 0.18f, 0.45f, 1.00f);
	style.Colors[ImGuiCol_HeaderHovered] = ImVec4(0.20f, 0.23f, 0.55f, 1.00f);
	style.Colors[ImGuiCol_HeaderActive] = ImVec4(0.10f, 0.13f, 0.33f, 1.00f);

	// A Button Red - more vibrant
	style.Colors[ImGuiCol_CheckMark] = ImVec4(1.00f, 0.30f, 0.30f, 1.00f);
	style.Colors[ImGuiCol_SliderGrab] = ImVec4(1.00f, 0.30f, 0.30f, 1.00f);
	style.Colors[ImGuiCol_SliderGrabActive] = ImVec4(0.90f, 0.20f, 0.20f, 1.00f);

	// Tabs - using X Button Blue - more vibrant
	style.Colors[ImGuiCol_Tab] = ImVec4(0.13f, 0.16f, 0.40f, 1.00f);
	style.Colors[ImGuiCol_TabHovered] = ImVec4(0.25f, 0.30f, 0.60f, 1.00f);
	style.Colors[ImGuiCol_TabActive] = ImVec4(0.30f, 0.35f, 0.65f, 1.00f);
	style.Colors[ImGuiCol_TabUnfocused] = ImVec4(0.13f, 0.16f, 0.40f, 1.00f);
	style.Colors[ImGuiCol_TabUnfocusedActive] = ImVec4(0.25f, 0.30f, 0.60f, 1.00f);

	// Other elements - more vibrant
	style.Colors[ImGuiCol_Border] = ImVec4(1.00f, 0.85f, 0.25f, 0.70f);
	style.Colors[ImGuiCol_BorderShadow] = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);
	style.Colors[ImGuiCol_Separator] = ImVec4(0.00f, 0.50f, 0.80f, 0.75f);
	style.Colors[ImGuiCol_TextSelectedBg] = ImVec4(1.00f, 0.50f, 0.00f, 0.35f);

	// Table colors - more vibrant
	style.Colors[ImGuiCol_TableHeaderBg] = ImVec4(0.15f, 0.18f, 0.45f, 1.00f);
	style.Colors[ImGuiCol_TableBorderStrong] = ImVec4(0.00f, 0.50f, 0.80f, 1.00f);
	style.Colors[ImGuiCol_TableBorderLight] = ImVec4(0.00f, 0.50f, 0.80f, 0.70f);
	style.Colors[ImGuiCol_TableRowBg] = ImVec4(0.05f, 0.07f, 0.10f, 1.00f);
	style.Colors[ImGuiCol_TableRowBgAlt] = ImVec4(0.10f, 0.12f, 0.15f, 1.00f);
}

static void applyHighContrastTheme()
{
	ImGui::StyleColorsDark();
	ImGuiStyle& style = ImGui::GetStyle();

	// High-contrast theme with Dreamcast accent colors.
	style.Colors[ImGuiCol_Text] = ImVec4(1.00f, 1.00f, 1.00f, 1.00f);
	style.Colors[ImGuiCol_TextDisabled] = ImVec4(0.70f, 0.70f, 0.70f, 1.00f);

	style.Colors[ImGuiCol_WindowBg] = ImVec4(0.00f, 0.00f, 0.00f, 1.00f);
	style.Colors[ImGuiCol_ChildBg] = ImVec4(0.05f, 0.05f, 0.05f, 1.00f);
	style.Colors[ImGuiCol_PopupBg] = ImVec4(0.10f, 0.10f, 0.12f, 1.00f);

	style.Colors[ImGuiCol_Border] = ImVec4(0.00f, 0.43f, 0.73f, 0.50f);
	style.Colors[ImGuiCol_BorderShadow] = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);

	// Frame elements (checkboxes, input fields) - darker blue for better text readability
	style.Colors[ImGuiCol_FrameBg] = ImVec4(0.15f, 0.15f, 0.20f, 1.00f);
	style.Colors[ImGuiCol_FrameBgHovered] = ImVec4(0.10f, 0.30f, 0.50f, 1.00f);
	style.Colors[ImGuiCol_FrameBgActive] = ImVec4(0.15f, 0.35f, 0.55f, 1.00f);

	// Title bars
	style.Colors[ImGuiCol_TitleBg] = ImVec4(0.10f, 0.10f, 0.10f, 1.00f);
	style.Colors[ImGuiCol_TitleBgActive] = ImVec4(0.00f, 0.43f, 0.73f, 1.00f);
	style.Colors[ImGuiCol_TitleBgCollapsed] = ImVec4(0.00f, 0.43f, 0.73f, 0.50f);

	style.Colors[ImGuiCol_MenuBarBg] = ImVec4(0.10f, 0.10f, 0.10f, 1.00f);

	// Scrollbars
	style.Colors[ImGuiCol_ScrollbarBg] = ImVec4(0.10f, 0.10f, 0.10f, 1.00f);
	style.Colors[ImGuiCol_ScrollbarGrab] = ImVec4(0.40f, 0.40f, 0.40f, 1.00f);
	style.Colors[ImGuiCol_ScrollbarGrabHovered] = ImVec4(0.56f, 0.56f, 0.56f, 1.00f);
	style.Colors[ImGuiCol_ScrollbarGrabActive] = ImVec4(0.00f, 0.65f, 0.90f, 1.00f);

	// Interactive elements
	style.Colors[ImGuiCol_CheckMark] = ImVec4(1.00f, 0.50f, 0.00f, 1.00f);

	style.Colors[ImGuiCol_SliderGrab] = ImVec4(0.00f, 0.65f, 0.90f, 1.00f);
	style.Colors[ImGuiCol_SliderGrabActive] = ImVec4(1.00f, 0.50f, 0.00f, 1.00f);

	// Buttons
	style.Colors[ImGuiCol_Button] = ImVec4(0.00f, 0.43f, 0.73f, 1.00f);
	style.Colors[ImGuiCol_ButtonHovered] = ImVec4(0.00f, 0.65f, 0.90f, 1.00f);
	style.Colors[ImGuiCol_ButtonActive] = ImVec4(0.00f, 0.33f, 0.60f, 1.00f);

	// Headers (settings screen highlights) - use DC Logo Blue (same as selected tab)
	style.Colors[ImGuiCol_Header] = ImVec4(0.00f, 0.43f, 0.73f, 0.90f);
	style.Colors[ImGuiCol_HeaderHovered] = ImVec4(0.00f, 0.50f, 0.80f, 1.00f);
	style.Colors[ImGuiCol_HeaderActive] = ImVec4(0.00f, 0.43f, 0.73f, 1.00f);

	// Tables
	style.Colors[ImGuiCol_TableHeaderBg] = ImVec4(0.15f, 0.15f, 0.15f, 1.00f);
	style.Colors[ImGuiCol_TableBorderStrong] = ImVec4(0.00f, 0.43f, 0.73f, 1.00f);
	style.Colors[ImGuiCol_TableBorderLight] = ImVec4(0.00f, 0.43f, 0.73f, 0.50f);

	// Tabs
	style.Colors[ImGuiCol_Tab] = ImVec4(0.15f, 0.15f, 0.15f, 1.00f);
	style.Colors[ImGuiCol_TabHovered] = ImVec4(0.00f, 0.65f, 0.90f, 1.00f);
	style.Colors[ImGuiCol_TabActive] = ImVec4(0.00f, 0.43f, 0.73f, 1.00f);
	style.Colors[ImGuiCol_TabUnfocused] = ImVec4(0.15f, 0.15f, 0.15f, 1.00f);
	style.Colors[ImGuiCol_TabUnfocusedActive] = ImVec4(0.00f, 0.43f, 0.73f, 1.00f);

	// Other UI elements
	style.Colors[ImGuiCol_Separator] = ImVec4(1.00f, 1.00f, 1.00f, 0.40f);
	style.Colors[ImGuiCol_TextSelectedBg] = ImVec4(1.00f, 0.50f, 0.00f, 0.35f);

	// Increase contrast even more
	style.Alpha = 1.0f;
	style.FrameBorderSize = 1.0f;
	style.WindowBorderSize = 1.0f;
	style.PopupBorderSize = 1.0f;
	style.TabBorderSize = 1.0f;
}

static void applyNintendoTheme()
{
	ImGui::StyleColorsDark();
	ImGuiStyle& style = ImGui::GetStyle();

	// Nintendo color palette
	ImVec4 nintendoRed = ImVec4(0.90f, 0.10f, 0.10f, 1.00f);
	ImVec4 nintendoRedLight = ImVec4(1.00f, 0.30f, 0.30f, 1.00f);
	ImVec4 nintendoRedDark = ImVec4(0.65f, 0.05f, 0.05f, 1.00f);

	ImVec4 luigiGreen = ImVec4(0.00f, 0.65f, 0.00f, 1.00f);
	ImVec4 luigiGreenLight = ImVec4(0.30f, 0.85f, 0.30f, 1.00f);

	ImVec4 gameboy = ImVec4(0.70f, 0.80f, 0.15f, 1.00f);
	ImVec4 gamecubePurple = ImVec4(0.35f, 0.20f, 0.65f, 1.00f);

	// Text colors
	style.Colors[ImGuiCol_Text] = ImVec4(1.00f, 1.00f, 1.00f, 1.00f);
	style.Colors[ImGuiCol_TextDisabled] = ImVec4(0.70f, 0.70f, 0.70f, 0.65f);

	// Window background and elements - darker blue-black like classic consoles
	style.Colors[ImGuiCol_WindowBg] = ImVec4(0.03f, 0.03f, 0.10f, 1.00f);
	style.Colors[ImGuiCol_ChildBg] = ImVec4(0.03f, 0.03f, 0.10f, 1.00f);
	style.Colors[ImGuiCol_PopupBg] = ImVec4(0.02f, 0.02f, 0.08f, 1.00f);

	// Frame elements - using GameCube purple for frames
	style.Colors[ImGuiCol_FrameBg] = ImVec4(0.12f, 0.08f, 0.20f, 1.00f);
	style.Colors[ImGuiCol_FrameBgHovered] = ImVec4(0.20f, 0.14f, 0.35f, 1.00f);
	style.Colors[ImGuiCol_FrameBgActive] = gamecubePurple;

	// Title elements - using Nintendo red
	style.Colors[ImGuiCol_TitleBg] = ImVec4(0.05f, 0.05f, 0.12f, 1.00f);
	style.Colors[ImGuiCol_TitleBgActive] = nintendoRed;
	style.Colors[ImGuiCol_TitleBgCollapsed] = ImVec4(0.05f, 0.05f, 0.12f, 0.75f);
	style.Colors[ImGuiCol_MenuBarBg] = ImVec4(0.08f, 0.08f, 0.15f, 1.00f);

	// Scrollbars - GameBoy inspired
	style.Colors[ImGuiCol_ScrollbarBg] = ImVec4(0.04f, 0.04f, 0.10f, 1.00f);
	style.Colors[ImGuiCol_ScrollbarGrab] = ImVec4(0.30f, 0.40f, 0.10f, 1.00f);
	style.Colors[ImGuiCol_ScrollbarGrabHovered] = ImVec4(0.50f, 0.60f, 0.15f, 1.00f);
	style.Colors[ImGuiCol_ScrollbarGrabActive] = gameboy;

	// Button elements - Nintendo red
	style.Colors[ImGuiCol_Button] = nintendoRed;
	style.Colors[ImGuiCol_ButtonHovered] = nintendoRedLight;
	style.Colors[ImGuiCol_ButtonActive] = nintendoRedDark;

	// Interactive elements - Luigi green for checkmarks and sliders
	style.Colors[ImGuiCol_CheckMark] = luigiGreenLight;
	style.Colors[ImGuiCol_SliderGrab] = luigiGreen;
	style.Colors[ImGuiCol_SliderGrabActive] = luigiGreenLight;

	// Headers (collapsing headers, tree nodes) - GameCube purple
	style.Colors[ImGuiCol_Header] = ImVec4(0.20f, 0.12f, 0.35f, 1.00f);
	style.Colors[ImGuiCol_HeaderHovered] = gamecubePurple;
	style.Colors[ImGuiCol_HeaderActive] = ImVec4(0.45f, 0.25f, 0.75f, 1.00f);

	// Tab elements - Red/green for Mario/Luigi contrast
	style.Colors[ImGuiCol_Tab] = ImVec4(0.15f, 0.10f, 0.30f, 1.00f);
	style.Colors[ImGuiCol_TabHovered] = luigiGreenLight;
	style.Colors[ImGuiCol_TabActive] = nintendoRed;
	style.Colors[ImGuiCol_TabUnfocused] = ImVec4(0.10f, 0.08f, 0.20f, 1.00f);
	style.Colors[ImGuiCol_TabUnfocusedActive] = ImVec4(0.40f, 0.10f, 0.10f, 1.00f);

	// Border and separator
	style.Colors[ImGuiCol_Border] = ImVec4(0.40f, 0.40f, 0.50f, 0.50f);
	style.Colors[ImGuiCol_BorderShadow] = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);
	style.Colors[ImGuiCol_Separator] = nintendoRed;

	// Table elements
	style.Colors[ImGuiCol_TableHeaderBg] = ImVec4(0.15f, 0.10f, 0.25f, 1.00f);
	style.Colors[ImGuiCol_TableBorderStrong] = nintendoRed;
	style.Colors[ImGuiCol_TableBorderLight] = ImVec4(0.40f, 0.10f, 0.10f, 0.70f);
	style.Colors[ImGuiCol_TableRowBg] = ImVec4(0.03f, 0.03f, 0.10f, 1.00f);
	style.Colors[ImGuiCol_TableRowBgAlt] = ImVec4(0.06f, 0.06f, 0.14f, 1.00f);

	// Selected text - keep transparency for selection highlight
	style.Colors[ImGuiCol_TextSelectedBg] = ImVec4(luigiGreen.x, luigiGreen.y, luigiGreen.z, 0.35f);

	// Reset to defaults for these
	style.TabBorderSize = 0.0f;
	style.FrameBorderSize = 0.0f;
}

static void applySoftTheme()
{
	ImGui::StyleColorsDark();
	ImGuiStyle& style = ImGui::GetStyle();

	// Soft, blue/turquoise theme that's easy on the eyes
	// Soft text colors
	style.Colors[ImGuiCol_Text] = ImVec4(0.85f, 0.90f, 0.92f, 1.00f);
	style.Colors[ImGuiCol_TextDisabled] = ImVec4(0.55f, 0.60f, 0.65f, 0.70f);

	// Soft dark backgrounds with turquoise tint
	style.Colors[ImGuiCol_WindowBg] = ImVec4(0.15f, 0.18f, 0.20f, 1.00f);
	style.Colors[ImGuiCol_ChildBg] = ImVec4(0.15f, 0.18f, 0.20f, 1.00f);
	style.Colors[ImGuiCol_TitleBg] = ImVec4(0.10f, 0.25f, 0.30f, 1.00f);
	style.Colors[ImGuiCol_TitleBgActive] = ImVec4(0.15f, 0.35f, 0.45f, 1.00f);
	style.Colors[ImGuiCol_TitleBgCollapsed] = ImVec4(0.10f, 0.25f, 0.30f, 0.75f);
	style.Colors[ImGuiCol_MenuBarBg] = ImVec4(0.17f, 0.21f, 0.24f, 1.00f);

	// Soft scrollbars
	style.Colors[ImGuiCol_ScrollbarBg] = ImVec4(0.15f, 0.18f, 0.20f, 1.00f);
	style.Colors[ImGuiCol_ScrollbarGrab] = ImVec4(0.25f, 0.45f, 0.50f, 1.00f);
	style.Colors[ImGuiCol_ScrollbarGrabHovered] = ImVec4(0.30f, 0.55f, 1.00f, 1.00f);
	style.Colors[ImGuiCol_ScrollbarGrabActive] = ImVec4(0.35f, 0.65f, 1.00f, 1.00f);

	// Soft, muted button colors
	style.Colors[ImGuiCol_Button] = ImVec4(0.20f, 0.40f, 0.45f, 1.00f);
	style.Colors[ImGuiCol_ButtonHovered] = ImVec4(0.25f, 0.50f, 0.55f, 1.00f);
	style.Colors[ImGuiCol_ButtonActive] = ImVec4(0.30f, 0.60f, 0.65f, 1.00f);

	style.Colors[ImGuiCol_PopupBg] = ImVec4(0.15f, 0.18f, 0.20f, 1.00f);

	// Frames (checkboxes, input fields)
	style.Colors[ImGuiCol_FrameBg] = ImVec4(0.20f, 0.28f, 0.33f, 1.00f);
	style.Colors[ImGuiCol_FrameBgHovered] = ImVec4(0.25f, 0.35f, 0.40f, 1.00f);
	style.Colors[ImGuiCol_FrameBgActive] = ImVec4(0.30f, 0.45f, 0.50f, 1.00f);

	// Headers (collapsing headers, tree nodes)
	style.Colors[ImGuiCol_Header] = ImVec4(0.20f, 0.35f, 0.45f, 1.00f);
	style.Colors[ImGuiCol_HeaderHovered] = ImVec4(0.25f, 0.40f, 0.50f, 1.00f);
	style.Colors[ImGuiCol_HeaderActive] = ImVec4(0.30f, 0.50f, 0.60f, 1.00f);

	// Accent colors - light turquoise
	style.Colors[ImGuiCol_CheckMark] = ImVec4(0.40f, 0.80f, 0.90f, 1.00f);
	style.Colors[ImGuiCol_SliderGrab] = ImVec4(0.35f, 0.65f, 0.75f, 1.00f);
	style.Colors[ImGuiCol_SliderGrabActive] = ImVec4(0.45f, 0.75f, 0.85f, 1.00f);

	// Tabs - soft blue palette
	style.Colors[ImGuiCol_Tab] = ImVec4(0.15f, 0.30f, 0.40f, 1.00f);
	style.Colors[ImGuiCol_TabHovered] = ImVec4(0.25f, 0.40f, 0.50f, 1.00f);
	style.Colors[ImGuiCol_TabActive] = ImVec4(0.30f, 0.50f, 0.60f, 1.00f);
	style.Colors[ImGuiCol_TabUnfocused] = ImVec4(0.10f, 0.25f, 0.35f, 1.00f);
	style.Colors[ImGuiCol_TabUnfocusedActive] = ImVec4(0.20f, 0.35f, 0.45f, 1.00f);

	// Other elements
	style.Colors[ImGuiCol_Border] = ImVec4(0.20f, 0.35f, 0.45f, 0.60f);
	style.Colors[ImGuiCol_Separator] = ImVec4(0.20f, 0.35f, 0.45f, 0.75f);
	style.Colors[ImGuiCol_TextSelectedBg] = ImVec4(0.30f, 0.50f, 0.60f, 0.35f);

	// Table colors
	style.Colors[ImGuiCol_TableHeaderBg] = ImVec4(0.15f, 0.30f, 0.40f, 1.00f);
	style.Colors[ImGuiCol_TableBorderStrong] = ImVec4(0.20f, 0.35f, 0.45f, 1.00f);
	style.Colors[ImGuiCol_TableBorderLight] = ImVec4(0.20f, 0.35f, 0.45f, 0.70f);
	style.Colors[ImGuiCol_TableRowBg] = ImVec4(0.15f, 0.18f, 0.20f, 1.00f);
	style.Colors[ImGuiCol_TableRowBgAlt] = ImVec4(0.17f, 0.21f, 0.24f, 1.00f);

	// Reset to defaults for these
	style.TabBorderSize = 0.0f;
	style.FrameBorderSize = 0.0f;
}

// Add the common function before gui_initFonts
void applyCurrentTheme()
{
	if (config::UITheme == 0)
		applyDarkTheme();
	else if (config::UITheme == 1)
		applyLightTheme();
	else if (config::UITheme == 2)
		applyDreamcastTheme();
	else if (config::UITheme == 3)
		applyHighContrastTheme();
	else if (config::UITheme == 4)
		applyNintendoTheme();
	else if (config::UITheme == 5)
		applySoftTheme();
	else
		applyDarkTheme();
}
