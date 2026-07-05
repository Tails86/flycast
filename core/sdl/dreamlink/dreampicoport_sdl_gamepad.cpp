/*
	Copyright 2024 flyinghead
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

#include "dreampicoport_sdl_gamepad.h"

#include "input/dreamlink/maplelink.h"
#include "input/dreamlink/maplelinkregistry.h"
#include "input/dreamlink/dreampicoport.h"

#include "hw/maple/maple_devs.h"
#include "hw/maple/maple_if.h"
#include "ui/gui.h"
#include "cfg/option.h"
#include "oslib/i18n.h"
#include "oslib/oslib.h"
#include "log/Log.h"
#include "emulator.h"

#include "DreamPicoPortApi.hpp"

// C++ standard library
#include <iomanip>
#include <sstream>
#include <thread>
#include <list>
#include <vector>
#include <array>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <optional>
#include <chrono>
#include <unordered_map>
#include <string_view>

#ifndef TARGET_UWP
#include <asio.hpp>
#endif

#if defined(__linux__) || (defined(__APPLE__) && defined(TARGET_OS_MAC))
#include <dirent.h>
#endif

#if defined(_WIN32) && !defined(TARGET_UWP)
#include <windows.h>
#include <setupapi.h>
#endif

//! Only to be called during instantiation to determine hardware information
//! @param[in] joystick_idx SDL joystick index
//! @param[in] sdl_joystick SDL joystick object
static DreamPicoPort::HardwareInfo parse_hw_info(int joystick_idx, SDL_Joystick* sdl_joystick) {
#if defined(_WIN32)
	// Workaround: Getting the instance ID here fixes some sort of L/R trigger bug in Windows dinput for some reason
	(void)SDL_JoystickGetDeviceInstanceID(joystick_idx);
#endif

	DreamPicoPort::HardwareInfo hw_info;

	// Set the serial number if found by SDL Joystick
	const char* joystick_serial = SDL_JoystickGetSerial(sdl_joystick);
	if (joystick_serial) {
		hw_info.serial_number = joystick_serial;
	} else {
		// Version 1.2.0 and later embeds serial in name as a workaround for MacOS and Linux
		// Serial is expected between a dash (-) and space ( ) character or until end of string
		const char* joystick_name = SDL_JoystickName(sdl_joystick);
		if (joystick_name) {
			std::string name_str(joystick_name);
			size_t dash_pos = name_str.find('-');
			if (dash_pos != std::string::npos) {
				size_t start_pos = dash_pos + 1;
				size_t end_pos = name_str.find(' ', start_pos);
				if (end_pos == std::string::npos) {
					end_pos = name_str.length();
				}
				// Serials are normally 16 characters, but check for at least 10 to account for any future changes
				if ((start_pos + 10) <= end_pos) {
					hw_info.serial_number = name_str.substr(start_pos, end_pos - start_pos);
				}
			}
		}
	}

#if defined(_WIN32)
	// This only works in Windows because the joystick_path is not given in other OSes
	const char* joystick_path = SDL_JoystickPath(sdl_joystick);

	struct SDL_hid_device_info* devs = SDL_hid_enumerate(DreamPicoPort::VID, DreamPicoPort::PID);
	if (devs) {
		struct SDL_hid_device_info* my_dev = nullptr;

		if (!devs->next) {
			// Only single device found, so this is simple (host-1p firmware used)
			hw_info.hardware_bus = 0;
			hw_info.is_hardware_bus_implied = false;
			hw_info.is_single_device = true;
			my_dev = devs;
		} else {
			struct SDL_hid_device_info* it = devs;

			if (joystick_path)
			{
				while (it)
				{
					// Note: hex characters will be differing case, so case-insensitive cmp is needed
					if (it->path && 0 == SDL_strcasecmp(it->path, joystick_path)) {
						my_dev = it;
						break;
					}
					it = it->next;
				}
			}

			if (my_dev) {
				it = devs;
				int count = 0;
				if (my_dev->serial_number) {
					while (it) {
						if (it->serial_number &&
							0 == wcscmp(it->serial_number, my_dev->serial_number))
						{
							++count;
						}
						it = it->next;
					}

					if (count == 1) {
						// Single device of this serial found
						hw_info.is_single_device = true;
						hw_info.hardware_bus = 0;
						hw_info.is_hardware_bus_implied = false;
					} else {
						hw_info.is_single_device = false;
						if (my_dev->release_number < 0x0102) {
							// Interfaces go in decending order
							hw_info.hardware_bus = (count - (my_dev->interface_number % 4) - 1);
							hw_info.is_hardware_bus_implied = false;
						} else {
							// Version 1.02 of interface will make interfaces in ascending order
							hw_info.hardware_bus = (my_dev->interface_number % 4);
							hw_info.is_hardware_bus_implied = false;
						}
					}
				}
			}
		}

		// Set serial number if found in SDL_hid
		if (my_dev) {
			if (hw_info.serial_number.empty() && my_dev->serial_number) {
				int len = WideCharToMultiByte(CP_UTF8, 0, my_dev->serial_number, -1, nullptr, 0, nullptr, nullptr);
				if (len > 0) {
					std::vector<char> buffer(len);
					WideCharToMultiByte(CP_UTF8, 0, my_dev->serial_number, -1, buffer.data(), len, nullptr, nullptr);
					hw_info.serial_number = std::string(buffer.data());
				}
			}
		}

		SDL_hid_free_enumeration(devs);
	}

#endif // #if defined(_WIN32)

	if (hw_info.hardware_bus < 0) {
		// The number of buttons gives a clue as to what index the controller is
		int nbuttons = SDL_JoystickNumButtons(sdl_joystick);

		if (nbuttons >= 32 || nbuttons <= 27) {
			// Older version of firmware or single player
			hw_info.hardware_bus = 0;
			hw_info.is_hardware_bus_implied = true;
			hw_info.is_single_device = true;
		}
		else {
			hw_info.hardware_bus = 31 - nbuttons;
			hw_info.is_hardware_bus_implied = false;
			hw_info.is_single_device = false;
		}
	}

	hw_info.unique_id.clear();
	hw_info.sort_id.clear();
	if (!hw_info.is_hardware_bus_implied && !hw_info.serial_number.empty()) {
		// Locking to name, which includes A-D, plus serial number will ensure correct enumeration every time
		hw_info.unique_id = std::string("sdl_") + hw_info.getName("") + std::string("_") + hw_info.serial_number;
		// Ensure this is ordered by SDL, product name, serial, and port char
		hw_info.sort_id = (
			std::string("sdl_") +
			hw_info.getProductName() + std::string("_") +
			hw_info.serial_number + std::string("_") +
			std::string(1, hw_info.getPortChar())
		);
	}

	return hw_info;
}

DreamPicoPortSDLGamepad::DreamPicoPortSDLGamepad(
	int maple_port,
	int joystick_idx,
	SDL_Joystick* sdl_joystick
) :
	DreamLinkSDLGamepad(
		std::make_shared<DreamPicoPort>(maple_port, parse_hw_info(joystick_idx, sdl_joystick)),
		maple_port,
		joystick_idx,
		sdl_joystick
)
{
	DreamPicoPort *picoPort = dynamic_cast<DreamPicoPort*>(dreamlink.get());
	_name = picoPort->getName();

	const std::string& sortId = picoPort->getSortId();
	if (!sortId.empty()) {
		_sort_id = sortId;
	}

	const std::string& uniqueId = picoPort->getUniqueId();
	if (!uniqueId.empty()) {
		_unique_id = uniqueId;
		loadMapping();
	}

	int bus = picoPort->getDefaultBus();
	if (DreamLink::isValidBus(bus))
		set_maple_port(bus);
}

bool DreamPicoPortSDLGamepad::identify(int deviceIndex)
{
	char guid_str[33] {};
	SDL_JoystickGetGUIDString(SDL_JoystickGetDeviceGUID(deviceIndex), guid_str, sizeof(guid_str));
	// Dreamcast Controller USB VID:1209 PID:2f07
	const char* pid_vid_guid_str = guid_str + 8;
	if (memcmp(VID_PID_GUID, pid_vid_guid_str, 16) == 0) {
		return true;
	}
	return false;
}

void DreamPicoPortSDLGamepad::setCustomMapping(const std::shared_ptr<InputMapping>& mapping)
{
	// Since this is a real DC controller, no deadzone adjustment is needed
	mapping->dead_zone = 0.0f;
	// Map the things not set by SDL
	mapping->set_button(DC_BTN_C, 2);
	mapping->set_button(DC_BTN_Z, 5);
	mapping->set_button(DC_BTN_D, 10);
	mapping->set_button(DC_DPAD2_UP, 9);
	mapping->set_button(DC_DPAD2_DOWN, 8);
	mapping->set_button(DC_DPAD2_LEFT, 7);
	mapping->set_button(DC_DPAD2_RIGHT, 6);
}

const char *DreamPicoPortSDLGamepad::get_button_name(u32 code)
{
	using namespace i18n;
	switch (code) {
		// Coincides with buttons setup in setDefaultMapping
		case 2: return "C";
		case 5: return "Z";
		case 10: return "D";
		case 9: return T("DPad2 Up");
		case 8: return T("DPad2 Down");
		case 7: return T("DPad2 Left");
		case 6: return T("DPad2 Right");

		// These buttons are normally not physically accessible but are mapped on DreamPicoPort
		case 12: return T("VMU1 A");
		case 15: return T("VMU1 B");
		case 16: return T("VMU1 Up");
		case 17: return T("VMU1 Down");
		case 18: return T("VMU1 Left");
		case 19: return T("VMU1 Right");

		default: return DreamLinkSDLGamepad::get_button_name(code); // default name
	}
}

bool DreamPicoPortSDLGamepad::gamepad_btn_input(u32 code, bool pressed)
{
	if (code == 20 && !pressed)
	{
		DreamPicoPort *picoPort = dynamic_cast<DreamPicoPort*>(dreamlink.get());
		if (picoPort)
		{
			picoPort->queryPeripherals(false);
		}
	}

	return DreamLinkSDLGamepad::gamepad_btn_input(code, pressed);
}
