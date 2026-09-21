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
#include "IconsFontAwesome6.h"
#include "mainui.h"
#include "log/LogManager.h"
#include "hw/maple/maple_if.h"
#include "imgui_stdlib.h"
#include "input/dreampotato.h"

#ifdef GDB_SERVER
#include "hw/mem/addrspace.h"
#endif

#include <optional>

static void gui_settings_advanced()
{
#if FEAT_SHREC != DYNAREC_NONE
    header(T("CPU Mode"));
    {
		ImGui::Columns(2, "cpu_modes", false);
		OptionRadioButton(T("Dynarec"), config::DynarecEnabled, true,
				T("Use the dynamic recompiler. Recommended in most cases"));
		ImGui::NextColumn();
		OptionRadioButton(T("Interpreter"), config::DynarecEnabled, false,
				T("Use the interpreter. Very slow but may help in case of a dynarec problem"));
		ImGui::Columns(1, nullptr, false);

		OptionSlider(T("SH4 Clock"), config::Sh4Clock, 100, 300,
				T("Over/Underclock the main SH4 CPU. Default is 200 MHz. Other values may crash, freeze or trigger unexpected nuclear reactions."),
				"%d MHz");
    }
#ifdef GDB_SERVER
	ImGui::Spacing();
	header("Virtual memory addresses");
	{
		void *ram_base, *ram, *vram, *aram;
		addrspace::getAddress(&ram_base, &ram, &vram, &aram);

		ImGui::Text("Base Address: %p", ram_base);

		if (ram == nullptr) {
			const ImVec4 gray(0.75f, 0.75f, 0.75f, 1.f);
			ImGui::TextColored(gray, "%s", "RAM adresses are not available until the emulation is started");
		} else {
			ImGui::Columns(3, "virtualMemoryAddress", false);
			ImGui::Text("RAM: %p", ram);
			ImGui::NextColumn();
			ImGui::Text("VRAM64: %p", vram);
			ImGui::NextColumn();
			ImGui::Text("ARAM: %p", aram);
			ImGui::Columns(1, nullptr, false);
		}

	}
	ImGui::Spacing();
	header("Debugging");
	{
		OptionCheckbox("Enable GDB", config::GDB, "GDB debugging support, disables Dynarec and dramatically reduces performance when a debugger is connected.");
		OptionCheckbox("Wait for connection", config::GDBWaitForConnection, "Start emulation once the debugger is connected.");
#ifndef __ANDROID
		OptionCheckbox("Serial Console", config::SerialConsole, "Dump the Dreamcast serial console to stdout");
		OptionCheckbox("Serial PTY", config::SerialPTY, "Requires the option \"Serial Console\" to work");
#endif

		static int gdbport = config::GDBPort;
		if (ImGui::InputInt("GDB port", &gdbport))
		{
			config::GDBPort = gdbport;
		}
		const ImGuiStyle& style = ImGui::GetStyle();
		ImGui::SameLine(0, style.ItemInnerSpacing.x);
		ShowHelpMarker("Default port is 3263");
	}
#endif
	ImGui::Spacing();
#endif
    header(T("Other"));
    {
    	OptionCheckbox(T("HLE BIOS"), config::UseReios, T("Force high-level BIOS emulation"));
        OptionCheckbox(T("Multi-threaded emulation"), config::ThreadedRendering,
        		T("Run the emulated CPU and GPU on different threads"));
#if !defined(__ANDROID) && !defined(GDB_SERVER)
        OptionCheckbox(T("Serial Console"), config::SerialConsole,
        		T("Dump the Dreamcast serial console to stdout"));
#endif
		{
			DisabledScope scope(game_started);
			OptionCheckbox(T("Dreamcast 32MB RAM Mod"), config::RamMod32MB,
					T("Enables 32MB RAM Mod for Dreamcast. May affect compatibility"));
		}
        OptionCheckbox(T("Dump Textures"), config::DumpTextures,
        		T("Dump all textures into data/texdump/<game id>"));
		ImGui::Indent();
		{
			DisabledScope scope(!config::DumpTextures.get());
			OptionCheckbox(T("Dump Replaced Textures"), config::DumpReplacedTextures,
					T("Always dump textures that are already replaced by custom textures"));
			OptionCheckbox(T("Discard Video and Animated Textures"), config::DumpUniqueTextures,
					T("Skip dumping video (YUV) and already updated textures"));
		}
		ImGui::Unindent();
        bool logToFile = config::loadBool("log", "LogToFile", false);
		if (ImGui::Checkbox(T("Log to File"), &logToFile))
			config::saveBool("log", "LogToFile", logToFile);
        ImGui::SameLine();
        ShowHelpMarker(T("Log debug information to flycast.log"));
#ifdef SENTRY_UPLOAD
        OptionCheckbox(T("Automatically Report Crashes"), config::UploadCrashLogs,
        		T("Automatically upload crash reports to sentry.io to help in troubleshooting. No personal information is included."));
#endif
    }

#ifdef USE_LUA
	header(T("Lua Scripting"));
	{
		InputText(T("Lua Filename"), &config::LuaFileName.get(), ImGuiInputTextFlags_CharsNoBlank);
		ImGui::SameLine();
		ShowHelpMarker(T("Specify lua filename to use. Should be located in Flycast config folder. Defaults to flycast.lua when empty."));
	}
#endif
}

#if !defined(NDEBUG) || defined(DEBUGFAST) || FC_PROFILER

static void gui_debug_tab()
{
	header("Logging");
	{
		LogManager *logManager = LogManager::GetInstance();
		for (LogTypes::LOG_TYPE type = LogTypes::AICA; type < LogTypes::NUMBER_OF_LOGS; type = (LogTypes::LOG_TYPE)(type + 1))
		{
			bool enabled = logManager->IsEnabled(type, logManager->GetLogLevel());
			std::string name = std::string(logManager->GetShortName(type)) + " - " + logManager->GetFullName(type);
			if (ImGui::Checkbox(name.c_str(), &enabled) && logManager->GetLogLevel() > LogTypes::LWARNING) {
				logManager->SetEnable(type, enabled);
				config::saveBool("log", logManager->GetShortName(type), enabled);
			}
		}
		ImGui::Spacing();

		static const char *levels[] = { "Notice", "Error", "Warning", "Info", "Debug" };
		if (ImGui::BeginCombo("Log Verbosity", levels[logManager->GetLogLevel() - 1], ImGuiComboFlags_None))
		{
			for (std::size_t i = 0; i < std::size(levels); i++)
			{
				bool is_selected = logManager->GetLogLevel() - 1 == (int)i;
				if (ImGui::Selectable(levels[i], &is_selected)) {
					logManager->SetLogLevel((LogTypes::LOG_LEVELS)(i + 1));
					config::saveInt("log", "Verbosity", i + 1);
				}
				if (is_selected)
					ImGui::SetItemDefaultFocus();
			}
			ImGui::EndCombo();
		}
		InputText("Log Server", &config::LogServer.get(), ImGuiInputTextFlags_CharsNoBlank | ImGuiInputTextFlags_CallbackCharFilter, dnsCharFilter);
        ImGui::SameLine();
        ShowHelpMarker("Log to this hostname[:port] with UDP. Default port is 31667.");
	}
#if FC_PROFILER
	ImGui::Spacing();
	header("Profiling");
	{

		OptionCheckbox("Enable", config::ProfilerEnabled, "Enable the profiler.");
		if (!config::ProfilerEnabled)
		{
			ImGui::PushItemFlag(ImGuiItemFlags_Disabled, true);
			ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5f);
		}
		OptionCheckbox("Display", config::ProfilerDrawToGUI, "Draw the profiler output in an overlay.");
		OptionCheckbox("Output to terminal", config::ProfilerOutputTTY, "Write the profiler output to the terminal");
		// TODO frame warning time
		if (!config::ProfilerEnabled)
		{
			ImGui::PopItemFlag();
			ImGui::PopStyleVar();
		}
	}
#endif
}
#endif

class VerticalTabBar
{
private:
	//! ID of the Selectable representing the currently selected tab.
	std::optional<ImGuiID> activeSelectableID = std::nullopt;

public:
	bool BeginTabBar(const char* id)
	{
		ImGui::PushID(id);

		// Setup tab bar structure: tab list on the left, active tab content on the right.
		ImGui::BeginChild("##verticalTabBar", ImVec2(160, 0), ImGuiChildFlags_NavFlattened);
		ImGui::EndChild();
		ImGui::SameLine();
		ImGui::BeginChild("##activeTabContent", ImVec2(0, 0), ImGuiChildFlags_NavFlattened);
		ImGui::EndChild();

		return ImGui::BeginChild("##verticalTabBar");
	}

	void EndTabBar()
	{
		ImGui::EndChild(); // ##verticalTabBar
		ImGui::PopID();
	}

	bool BeginTab(const char* icon, const char* label)
	{
		std::string fullLabel = std::string(icon) + " " + label;
		ImGuiID selectableID = ImGui::GetID(fullLabel.c_str());

		// When we have no activeSelectableID stored, then the first tab becomes the active tab automatically
		if (!activeSelectableID.has_value()) {
			activeSelectableID = selectableID;
		}

		// TODO2: When tabs are switched, multiple tabs can render in a single frame.
		// We may need the Selectable() overload which takes a pointer to 'isActiveTab', to catch when a tab is deselected
		bool isActiveTab = activeSelectableID.value() == selectableID;
		bool pressed = ImGui::Selectable(fullLabel.c_str(), isActiveTab);
		IM_ASSERT(selectableID == ImGui::GetItemID());
		if (pressed) {
			isActiveTab = true;
			activeSelectableID = selectableID;
		}

		if (isActiveTab) {
			ImGui::EndChild(); // ##verticalTabBar
			ImGui::BeginChild("##activeTabContent", ImVec2(0, 0));
		}

		if (pressed) {
			// Move focus to ##activeTabContent
			// TODO2: If public APIs don't offer us a decent way to move focus back
			// to the tab bar on B-press, then, it's probably not worth moving focus to the active area on A-press here.
			// ImGui::SetKeyboardFocusHere();
		}

		return isActiveTab;
	}

	void EndTab()
	{
		ImGui::EndChild(); // ##activeTabContent
		ImGui::BeginChild("##verticalTabBar");
	}
};

void gui_display_settings_header(ImVec2 normal_padding, std::array<bool, 4>& mapleDevicesChanges, std::array<std::array<bool, 2>, 4>& expDevicesChanges)
{
	ImguiStyleVar _(ImGuiStyleVar_FramePadding, normal_padding);

	auto availableWidth = ImGui::GetContentRegionAvail().x;
    if (ImGui::Button(T("Done"), ImVec2(availableWidth, uiScaled(30))))
    {
    	if (uiUserScaleUpdated)
    	{
    		uiUserScaleUpdated = false;
    		mainui_reinit();
    	}
    	if (game_started)
    		gui_setState(GuiState::Commands);
    	else
    		gui_setState(GuiState::Main);
    	dreampotato::update();
 		if (game_started && settings.platform.isConsole())
		{
			for (unsigned bus = 0; bus < mapleDevicesChanges.size(); bus++)
			{
				if (mapleDevicesChanges[bus]) {
					maple_ReconnectDevice(bus, 5);
				}
				else
				{
					if (expDevicesChanges[bus][0]) {
						maple_ReconnectDevice(bus, 0);
						reset_vmus();
					}
					if (expDevicesChanges[bus][1]) {
						maple_ReconnectDevice(bus, 1);
						reset_vmus();
					}
				}
			}
    	}
    	mapleDevicesChanges = {};
    	expDevicesChanges = {};
       	SaveSettings();
    }
	if (game_started)
	{
		if (config::Settings::instance().hasPerGameConfig())
		{
			if (ImGui::Button(T("Delete Game Config"), ImVec2(availableWidth, uiScaled(30))))
			{
				config::Settings::instance().setPerGameConfig(false);
				config::Settings::instance().load(false);
				loadGameSpecificSettings();
			}
		}
		else
		{
			if (ImGui::Button(T("Make Game Config"), ImVec2(availableWidth, uiScaled(30))))
				config::Settings::instance().setPerGameConfig(true);
		}
	}
}

void gui_display_settings()
{
	static std::array<bool, 4> mapleDevicesChanges;
	static std::array<std::array<bool, 2>, 4> expDevicesChanges;

	fullScreenWindow(false);
	ImguiStyleVar _(ImGuiStyleVar_WindowRounding, 0);

    ImGui::Begin(T("Settings"), nullptr, ImGuiWindowFlags_DragScrolling | ImGuiWindowFlags_NoResize
    		| ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse);
	ImVec2 normal_padding = ImGui::GetStyle().FramePadding;

	if (ImGui::GetContentRegionAvail().x >= uiScaled(650.f))
		ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ScaledVec2(16, 6));
	else
		// low width
		ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ScaledVec2(4, 6));

	static VerticalTabBar SettingsTabBar;
    if (SettingsTabBar.BeginTabBar("settings"))
    {
		gui_display_settings_header(normal_padding, mapleDevicesChanges, expDevicesChanges);

		if (SettingsTabBar.BeginTab(ICON_FA_TOOLBOX, T("General")))
		{
			ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, normal_padding);
			gui_settings_general();
			ImGui::PopStyleVar();
			SettingsTabBar.EndTab();
		}
		if (SettingsTabBar.BeginTab(ICON_FA_GAMEPAD, T("Controls")))
		{
			ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, normal_padding);
			gui_settings_controls(mapleDevicesChanges, expDevicesChanges);
			ImGui::PopStyleVar();
			SettingsTabBar.EndTab();
		}
		if (SettingsTabBar.BeginTab(ICON_FA_DISPLAY, T("Video")))
		{
			ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, normal_padding);
			gui_settings_video();
			ImGui::PopStyleVar();
			SettingsTabBar.EndTab();
		}
		if (SettingsTabBar.BeginTab(ICON_FA_MUSIC, T("Audio")))
		{
			ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, normal_padding);
			gui_settings_audio();
			ImGui::PopStyleVar();
			SettingsTabBar.EndTab();
		}
		if (SettingsTabBar.BeginTab(ICON_FA_WIFI, T("Network")))
		{
			ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, normal_padding);
			gui_settings_network();
			ImGui::PopStyleVar();
			SettingsTabBar.EndTab();
		}
		if (SettingsTabBar.BeginTab(ICON_FA_MICROCHIP, T("Advanced")))
		{
			ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, normal_padding);
			gui_settings_advanced();
			ImGui::PopStyleVar();
			SettingsTabBar.EndTab();
		}
#if !defined(NDEBUG) || defined(DEBUGFAST) || FC_PROFILER
		if (SettingsTabBar.BeginTab(ICON_FA_BUG, "Debug"))
		{
			ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, normal_padding);
			gui_debug_tab();
			ImGui::PopStyleVar();
			SettingsTabBar.EndTab();
		}
#endif
		if (SettingsTabBar.BeginTab(ICON_FA_CIRCLE_INFO, T("About")))
		{
			ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, normal_padding);
			gui_settings_about();
			ImGui::PopStyleVar();
			SettingsTabBar.EndTab();
		}
		SettingsTabBar.EndTabBar();
    }
	ImGui::PopStyleVar();

    scrollWhenDraggingOnVoid();
    windowDragScroll();
    ImGui::End();
}

