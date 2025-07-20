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
#include "network/ggpo.h"
#include "network/ice.h"
#include "imgui_stdlib.h"

void gui_settings_network()
{
	ImGuiStyle& style = ImGui::GetStyle();
	header("Network Type");
	{
		DisabledScope scope(game_started);

		int netType = 0;
		if (config::GGPOEnable)
			netType = 1;
		else if (config::NetworkEnable)
			netType = 2;
		else if (config::BattleCableEnable)
			netType = 3;
		ImGui::Columns(4, "networkType", false);
		ImGui::RadioButton("Disabled##network", &netType, 0);
		ImGui::NextColumn();
		ImGui::RadioButton("GGPO", &netType, 1);
		ImGui::SameLine(0, style.ItemInnerSpacing.x);
		ShowHelpMarker("Enable networking using GGPO");
		ImGui::NextColumn();
		ImGui::RadioButton("Naomi", &netType, 2);
		ImGui::SameLine(0, style.ItemInnerSpacing.x);
		ShowHelpMarker("Enable networking for supported Naomi and Atomiswave games");
		ImGui::NextColumn();
		ImGui::RadioButton("Battle Cable", &netType, 3);
		ImGui::SameLine(0, style.ItemInnerSpacing.x);
		ShowHelpMarker("Emulate the Taisen (Battle) null modem cable for games that support it");
		ImGui::Columns(1, nullptr, false);

		config::GGPOEnable = false;
		config::NetworkEnable = false;
		config::BattleCableEnable = false;
		switch (netType) {
		case 1:
			config::GGPOEnable = true;
			break;
		case 2:
			config::NetworkEnable = true;
			break;
		case 3:
			config::BattleCableEnable = true;
			break;
		}
	}
	if (config::GGPOEnable || config::NetworkEnable || config::BattleCableEnable) {
		ImGui::Spacing();
		header("Configuration");
	}
	{
		if (config::GGPOEnable)
		{
			config::NetworkEnable = false;
			OptionCheckbox("Play as Player 1", config::ActAsServer,
					"Deselect to play as player 2");
			ImGui::InputText("Peer", &config::NetworkServer.get(), ImGuiInputTextFlags_CharsNoBlank, nullptr, nullptr);
			ImGui::SameLine();
			ShowHelpMarker("Your peer IP address and optional port");
			OptionSlider("Frame Delay", config::GGPODelay, 0, 20,
				"Sets Frame Delay, advisable for sessions with ping >100 ms");

			ImGui::Text("Left Thumbstick:");
			OptionRadioButton<int>("Disabled##analogaxis", config::GGPOAnalogAxes, 0, "Left thumbstick not used");
			ImGui::SameLine();
			OptionRadioButton<int>("Horizontal", config::GGPOAnalogAxes, 1, "Use the left thumbstick horizontal axis only");
			ImGui::SameLine();
			OptionRadioButton<int>("Full", config::GGPOAnalogAxes, 2, "Use the left thumbstick horizontal and vertical axes");

			OptionCheckbox("Enable Chat", config::GGPOChat, "Open the chat window when a chat message is received");
			if (config::GGPOChat)
			{
				OptionCheckbox("Enable Chat Window Timeout", config::GGPOChatTimeoutToggle, "Automatically close chat window after 20 seconds");
				if (config::GGPOChatTimeoutToggle)
				{
					char chatTimeout[256];
					snprintf(chatTimeout, sizeof(chatTimeout), "%d", (int)config::GGPOChatTimeout);
					ImGui::InputText("Chat Window Timeout (seconds)", chatTimeout, sizeof(chatTimeout), ImGuiInputTextFlags_CharsDecimal, nullptr, nullptr);
					ImGui::SameLine();
					ShowHelpMarker("Sets duration that chat window stays open after new message is received.");
					config::GGPOChatTimeout.set(atoi(chatTimeout));
				}
			}
			OptionCheckbox("Network Statistics", config::NetworkStats,
					"Display network statistics on screen");
		}
		else if (config::NetworkEnable)
		{
			OptionCheckbox("Act as Server", config::ActAsServer,
					"Create a local server for Naomi network games");
			if (!config::ActAsServer)
			{
				ImGui::InputText("Server", &config::NetworkServer.get(), ImGuiInputTextFlags_CharsNoBlank, nullptr, nullptr);
				ImGui::SameLine();
				ShowHelpMarker("The server to connect to. Leave blank to find a server automatically on the default port");
			}
			char localPort[256];
			snprintf(localPort, sizeof(localPort), "%d", (int)config::LocalPort);
			ImGui::InputText("Local Port", localPort, sizeof(localPort), ImGuiInputTextFlags_CharsDecimal, nullptr, nullptr);
			ImGui::SameLine();
			ShowHelpMarker("The local UDP port to use");
			config::LocalPort.set(atoi(localPort));
		}
		else if (config::BattleCableEnable)
		{
#ifdef USE_ICE
		    if (ImGui::BeginTabBar("battleMode", ImGuiTabBarFlags_NoTooltip))
		    {
				if (ImGui::BeginTabItem("Match Code"))
				{
					ice::State state = ice::getState();
					ImGuiInputTextFlags textFlags = state == ice::Offline ? ImGuiInputTextFlags_CharsNoBlank : ImGuiInputTextFlags_ReadOnly;
					static std::string matchCode;
					ImGui::InputText("Code", &matchCode, textFlags);
					ImGui::SameLine();
					ShowHelpMarker("Choose a unique word or number and share it with your opponent.");
					if (state == ice::Offline) {
						if (ImGui::Button("Connect") && !matchCode.empty())
							ice::init(matchCode, true);
					}
					else {
						if (ImGui::Button("Disconnect"))
							try { ice::term(); } catch (...) {}
					}
					std::string status;
					switch (state)
					{
					case ice::Offline:
						status = ice::getStatusText();
						break;
					case ice::Online:
						status = "Waiting at meeting point...";
						break;
					case ice::ChalAccepted:
						status = "Preparing game...";
						break;
					case ice::Playing:
						status = "Playing " + matchCode + " (" + ice::getStatusText() + ")";
						break;
					default:
						break;
					}
					ImGui::TextDisabled("%s", status.c_str());
					OptionCheckbox("Network Statistics", config::NetworkStats,
							"Display network statistics on screen");
					ImGui::EndTabItem();
				}
				if (ImGui::BeginTabItem("Manual"))
				{
#endif
					ImGui::InputText("Peer", &config::NetworkServer.get(), ImGuiInputTextFlags_CharsNoBlank, nullptr, nullptr);
					ImGui::SameLine();
					ShowHelpMarker("The peer to connect to. Leave blank to find a player automatically on the default port");
					char localPort[256];
					snprintf(localPort, sizeof(localPort), "%d", (int)config::LocalPort);
					ImGui::InputText("Local Port", localPort, sizeof(localPort), ImGuiInputTextFlags_CharsDecimal, nullptr, nullptr);
					ImGui::SameLine();
					ShowHelpMarker("The local UDP port to use");
					config::LocalPort.set(atoi(localPort));
#ifdef USE_ICE
					ImGui::EndTabItem();
				}
				ImGui::EndTabBar();
		    }
#endif
			OptionCheckbox("Act as Master", config::ActAsServer,
					"Only used for Maximum Speed. One of the peer must be master.");
		}
	}
	ImGui::Spacing();
	header("Network Options");
	{
		OptionCheckbox("Enable UPnP", config::EnableUPnP, "Automatically configure your network router for netplay");
		OptionCheckbox("Broadcast Digital Outputs", config::NetworkOutput, "Broadcast digital outputs and force-feedback state on TCP port 8000. "
				"Compatible with the \"-output network\" MAME option. Arcade games only.");
		{
			DisabledScope scope(game_started);

			OptionCheckbox("Broadband Adapter Emulation", config::EmulateBBA,
					"Emulate the Ethernet Broadband Adapter (BBA) instead of the Modem");
		}
		OptionCheckbox("Use DCNet", config::UseDCNet, "Use the DCNet cloud service for Dreamcast Internet access.");
		ImGui::InputText("ISP User Name", &config::ISPUsername.get(), ImGuiInputTextFlags_CharsNoBlank | ImGuiInputTextFlags_CallbackCharFilter,
				[](ImGuiInputTextCallbackData *data) { return static_cast<int>(data->EventChar <= ' ' || data->EventChar > '~'); }, nullptr);
		ImGui::SameLine();
		ShowHelpMarker("The ISP user name stored in the console Flash RAM. Used by some online games as the player name. Leave blank to keep the current Flash RAM value.");
#if !defined(NDEBUG) || defined(DEBUGFAST)
		{
			DisabledScope scope(config::UseDCNet);
			ImGui::InputText("DNS", &config::DNS.get(), ImGuiInputTextFlags_CharsNoBlank);
			ImGui::SameLine();
			ShowHelpMarker("DNS server name or IP address");
		}
#endif
	}
#ifdef NAOMI_MULTIBOARD
	ImGui::Spacing();
	header("Multiboard Screens");
	{
		//OptionRadioButton<int>("Disabled##multiboard", config::MultiboardSlaves, 0, "Multiboard disabled (when optional)");
		OptionRadioButton<int>("1 (Twin)", config::MultiboardSlaves, 1, "One screen configuration (F355 Twin)");
		ImGui::SameLine();
		OptionRadioButton<int>("3 (Deluxe)", config::MultiboardSlaves, 2, "Three screens configuration");
	}
#endif
}
