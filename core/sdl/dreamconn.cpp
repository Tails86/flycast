/*
	Copyright 2024 flyinghead

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
#include "dreamconn.h"

#ifdef USE_DREAMCASTCONTROLLER
#include "hw/maple/maple_devs.h"
#include "ui/gui.h"
#include <cfg/option.h>
#include <SDL.h>
#include <asio.hpp>
#include <iomanip>
#include <sstream>
#include <optional>

#if defined(__linux__) || (defined(__APPLE__) && defined(TARGET_OS_MAC))
#include <dirent.h>
#endif

#if defined(_WIN32)
#include <windows.h>
#include <setupapi.h>
#endif

class DreamcastControllerConnection
{
	const int bus;
	const int dreamcastControllerType;
	asio::ip::tcp::iostream iostream;
	asio::io_context io_context;
	std::string serial_out_data;
	asio::serial_port serial_handler{io_context};
	bool serial_write_in_progress = false;
	asio::streambuf serial_read_buffer;

public:
	DreamcastControllerConnection(const DreamcastControllerConnection&) = delete;
	DreamcastControllerConnection() = delete;

	explicit DreamcastControllerConnection(int bus, int dreamcastControllerType) :
		bus(bus),
		dreamcastControllerType(dreamcastControllerType)
	{}

	~DreamcastControllerConnection()
	{
		disconnect();
	}

	std::optional<MapleMsg> connect()
	{
		asio::error_code ec;

		switch (dreamcastControllerType) {
			case TYPE_DREAMCONN:
			{
#if !defined(_WIN32)
				WARN_LOG(INPUT, "DreamcastController[%d] connection failed: DreamConn+ / DreamConn S Controller supported on Windows only", bus);
				return std::nullopt;
#endif
				iostream = asio::ip::tcp::iostream("localhost", std::to_string(DreamConn::BASE_PORT + bus));
				if (!iostream) {
					WARN_LOG(INPUT, "DreamcastController[%d] connection failed: %s", bus, iostream.error().message().c_str());
					disconnect();
					return std::nullopt;
				}
				iostream.expires_from_now(std::chrono::seconds(1));
				break;
			}
			case TYPE_DREAMCASTCONTROLLERUSB:
			{
				// the serial port isn't ready at this point, so we need to sleep briefly
				// we probably should have a better way to handle this
#if defined(_WIN32)
				Sleep(500);
#elif defined(__linux__) || (defined(__APPLE__) && defined(TARGET_OS_MAC))
				usleep(500000);
#endif

				serial_handler = asio::serial_port(io_context);
				io_context.reset();

				// select first available serial device
				std::string serial_device = getFirstSerialDevice();

				serial_handler.open(serial_device, ec);

				if (ec || !serial_handler.is_open()) {
					WARN_LOG(INPUT, "DreamcastController[%d] connection failed: %s", bus, ec.message().c_str());
					disconnect();
					return std::nullopt;
				}

				startSerialRead();

				break;
			}
			default:
			{
				return std::nullopt;
			}
		}

		// Now get the controller configuration
		MapleMsg msg;
		msg.command = MDCF_GetCondition;
		msg.destAP = (bus << 6) | 0x20;
		msg.originAP = bus << 6;
		msg.setData(MFID_0_Input);

		ec = sendMsg(msg);
		if (ec)
		{
			WARN_LOG(INPUT, "DreamcastController[%d] connection failed: %s", bus, ec.message().c_str());
			disconnect();
			return std::nullopt;
		}
		if (!receiveMsg(msg)) {
			WARN_LOG(INPUT, "DreamcastController[%d] read timeout", bus);
			disconnect();
			return std::nullopt;
		}
		if (dreamcastControllerType == TYPE_DREAMCONN) {
			iostream.expires_from_now(std::chrono::duration<u32>::max());	// don't use a 64-bit based duration to avoid overflow
		}

		return msg;
	}

	void disconnect()
	{
		if (dreamcastControllerType == TYPE_DREAMCONN)
		{
			if (iostream) {
				iostream.close();
			}
		}
		else if (dreamcastControllerType == TYPE_DREAMCASTCONTROLLERUSB)
		{
			io_context.stop();

			if (serial_handler.is_open()) {
				try
				{
					serial_handler.cancel();
				}
				catch(const asio::system_error&)
				{
					// Ignore cancel errors
				}
			}

			try
			{
				serial_handler.close();
			}
			catch(const asio::system_error&)
			{
				// Ignore closing errors
			}
		}
	}

	asio::error_code sendMsg(const MapleMsg& msg, std::chrono::milliseconds timeoutMs = std::chrono::milliseconds(5000))
	{
		std::ostringstream s;
		s.fill('0');
		if (dreamcastControllerType == TYPE_DREAMCASTCONTROLLERUSB)
		{
			// Messages to Dreamcast Controller USB need to be prefixed to trigger the correct parser
			s << "X ";
		}

		s << std::hex << std::uppercase
			<< std::setw(2) << (u32)msg.command << " "
			<< std::setw(2) << (u32)msg.destAP << " "
			<< std::setw(2) << (u32)msg.originAP << " "
			<< std::setw(2) << (u32)msg.size;
		const u32 sz = msg.getDataSize();
		for (u32 i = 0; i < sz; i++)
			s << " " << std::setw(2) << (u32)msg.data[i];
		s << "\r\n";

		asio::error_code ec;

		if (dreamcastControllerType == TYPE_DREAMCONN)
		{
			if (!iostream) {
				return asio::error::not_connected;
			}
			asio::ip::tcp::socket& sock = static_cast<asio::ip::tcp::socket&>(iostream.socket());
			asio::write(sock, asio::buffer(s.str()), ec);
		}
		else if (dreamcastControllerType == TYPE_DREAMCASTCONTROLLERUSB)
		{
			if (!serial_handler.is_open()) {
				return asio::error::not_connected;
			}

			const std::chrono::steady_clock::time_point expiration = std::chrono::steady_clock::now() + timeoutMs;

			// Wait for last write to complete
			while (serial_write_in_progress){
				if (!serial_handler.is_open()) {
					return asio::error::not_connected;
				}
				if (io_context.stopped()){
					return asio::error::shut_down;
				}
				else if (io_context.run_one_until(expiration) <= 0) {
					return asio::error::timed_out;
				}
			}

			// Poll to ensure all the waiting serial data is in serial_read_buffer
			io_context.poll_one();

			// Clear out the read buffer before writing next command
			if (serial_read_buffer.size() > 0) {
				serial_read_buffer.consume(serial_read_buffer.size());
			}

			serial_write_in_progress = true;
			serial_out_data = s.str();
			asio::async_write(serial_handler, asio::buffer(serial_out_data), asio::transfer_exactly(serial_out_data.size()), [this](const asio::error_code& error, size_t bytes_transferred)
			{
				serial_write_in_progress = false;
				if (error) {
					serial_handler.cancel();
				}
			});
		}

		return ec;
	}

	bool receiveMsg(MapleMsg& msg, std::chrono::milliseconds timeoutMs = std::chrono::milliseconds(5000))
	{
		std::string response;

		if (dreamcastControllerType == TYPE_DREAMCONN)
		{
			if (!std::getline(iostream, response))
				return false;
			sscanf(response.c_str(), "%hhx %hhx %hhx %hhx", &msg.command, &msg.destAP, &msg.originAP, &msg.size);
			if ((msg.getDataSize() - 1) * 3 + 13 >= response.length())
				return false;
			for (unsigned i = 0; i < msg.getDataSize(); i++)
				sscanf(&response[i * 3 + 12], "%hhx", &msg.data[i]);
			return !iostream.fail();
		}
		else if (dreamcastControllerType == TYPE_DREAMCASTCONTROLLERUSB)
		{
			const std::chrono::steady_clock::time_point expiration = std::chrono::steady_clock::now() + timeoutMs;

			for (int i = 0; i < 2; ++i)
			{
				// discard the first message as we are interested in the second only which returns the controller configuration
				response = "";

				// Keep pulling characters until this line is complete
				char c = '\0';
				do {
					// Wait until data is received or timeout has expired
					// serial_read_buffer is asynchronously filled by the serial_handler's io_context
					while (serial_read_buffer.size() <= 0){
						if (!serial_handler.is_open() || io_context.stopped() || io_context.run_one_until(expiration) <= 0){
							// connection closed, io_context was stopped, or failed to receive anything before timeout
							return false;
						}
					}

					// Consume characters until buffers are empty or \n found
					asio::const_buffers_1 data = serial_read_buffer.data();
					std::size_t consumed = 0;
					for (const asio::const_buffer& buff : data)
					{
						const char* buffDat = static_cast<const char*>(buff.data());
						for (std::size_t i = 0; i < buff.size(); ++i)
						{
							c = *buffDat++;
							++consumed;

							if (c == '\n') {
								// Stop reading now
								break;
							}

							response += c;
						}

						if (c == '\n') {
							// Stop reading now
							break;
						}
					}

					serial_read_buffer.consume(consumed);

				} while (c != '\n');

				// Remove carriage return if found
				if (response.size() > 0 && response[response.size() - 1] == '\r') {
					response.pop_back();
				}
			}

			sscanf(response.c_str(), "%hhx %hhx %hhx %hhx", &msg.command, &msg.destAP, &msg.originAP, &msg.size);

			if (serial_handler.is_open()) {
				return true;
			}
			else {
				return false;
			}
		}

		return false;
	}

private:
	static std::string getFirstSerialDevice() {

		// On Windows, we get the first serial device matching our VID/PID
#if defined(_WIN32)
		HDEVINFO deviceInfoSet = SetupDiGetClassDevs(NULL, "USB", NULL, DIGCF_PRESENT | DIGCF_ALLCLASSES);
		if (deviceInfoSet == INVALID_HANDLE_VALUE) {
			return "";
		}

		SP_DEVINFO_DATA deviceInfoData;
		deviceInfoData.cbSize = sizeof(SP_DEVINFO_DATA);

		for (DWORD i = 0; SetupDiEnumDeviceInfo(deviceInfoSet, i, &deviceInfoData); ++i) {
			DWORD dataType, bufferSize = 0;
			SetupDiGetDeviceRegistryProperty(deviceInfoSet, &deviceInfoData, SPDRP_HARDWAREID, &dataType, NULL, 0, &bufferSize);

			if (bufferSize > 0) {
				std::vector<char> buffer(bufferSize);
				if (SetupDiGetDeviceRegistryProperty(deviceInfoSet, &deviceInfoData, SPDRP_HARDWAREID, &dataType, (PBYTE)buffer.data(), bufferSize, NULL)) {
					std::string hardwareId(buffer.begin(), buffer.end());
					if (hardwareId.find("VID_1209") != std::string::npos && hardwareId.find("PID_2F07") != std::string::npos) {
						HKEY deviceKey = SetupDiOpenDevRegKey(deviceInfoSet, &deviceInfoData, DICS_FLAG_GLOBAL, 0, DIREG_DEV, KEY_READ);
						if (deviceKey != INVALID_HANDLE_VALUE) {
							char portName[256];
							DWORD portNameSize = sizeof(portName);
							if (RegQueryValueEx(deviceKey, "PortName", NULL, NULL, (LPBYTE)portName, &portNameSize) == ERROR_SUCCESS) {
								RegCloseKey(deviceKey);
								SetupDiDestroyDeviceInfoList(deviceInfoSet);
								return std::string(portName);
							}
							RegCloseKey(deviceKey);
						}
					}
				}
			}
		}

		SetupDiDestroyDeviceInfoList(deviceInfoSet);
		return "";
#endif

#if defined(__linux__) || (defined(__APPLE__) && defined(TARGET_OS_MAC))
	// On MacOS/Linux, we get the first serial device matching the device prefix
	std::string device_prefix = "";

#if defined(__linux__)
		device_prefix = "ttyACM";
#elif (defined(__APPLE__) && defined(TARGET_OS_MAC))
		device_prefix = "tty.usbmodem";
#endif

		std::string path = "/dev/";
		DIR *dir;
		struct dirent *ent;
		if ((dir = opendir(path.c_str())) != NULL) {
			while ((ent = readdir(dir)) != NULL) {
				std::string device = ent->d_name;
				if (device.find(device_prefix) != std::string::npos) {
					closedir(dir);
					return path + device;
				}
			}
			closedir(dir);
		}
		return "";
#endif
	}

	void startSerialRead()
	{
		serialReadHandler(asio::error_code(), 0);
	}

	void serialReadHandler(const asio::error_code& error, std::size_t size)
	{
		if (error) {
			serial_handler.cancel();
		}
		else {
			// Rearm the read
			asio::async_read_until(
				serial_handler,
				serial_read_buffer,
				'\n',
				[this](const asio::error_code& error, std::size_t size) -> void {
					// Auto reload read - io_context will always have work to do
					serialReadHandler(error, size);
				}
			);
		}
	}
};

void createDreamConnDevices(std::shared_ptr<DreamConn> dreamconn, bool gameStart);

DreamConn::DreamConn(int bus, int dreamcastControllerType) : bus(bus), dreamcastControllerType(dreamcastControllerType) {
	dreamcastControllerConnection = std::make_unique<DreamcastControllerConnection>(bus, dreamcastControllerType);
	connect();
}

DreamConn::~DreamConn() {
	disconnect();
}

void DreamConn::connect()
{
	maple_io_connected = false;
	expansionDevs = 0;

	std::optional<MapleMsg> msg = dreamcastControllerConnection->connect();
	if (msg)
	{
		expansionDevs = msg->originAP & 0x1f;
	}

	config::MapleExpansionDevices[bus][0] = hasVmu() ? MDT_SegaVMU : MDT_None;
	config::MapleExpansionDevices[bus][1] = hasRumble() ? MDT_PurupuruPack : MDT_None;

	if (hasVmu() || hasRumble())
	{
		NOTICE_LOG(INPUT, "Connected to DreamcastController[%d]: Type:%s, VMU:%d, Rumble Pack:%d", bus, dreamcastControllerType == 1 ? "DreamConn+ / DreamcConn S Controller" : "Dreamcast Controller USB", hasVmu(), hasRumble());
		maple_io_connected = true;
	}
	else
	{
		WARN_LOG(INPUT, "DreamcastController[%d] connection: no VMU or Rumble Pack connected", bus);
		disconnect();
		return;
	}
}

void DreamConn::disconnect()
{
	dreamcastControllerConnection->disconnect();

	maple_io_connected = false;

	NOTICE_LOG(INPUT, "Disconnected from DreamcastController[%d]", bus);
}

bool DreamConn::send(const MapleMsg& msg)
{
	asio::error_code ec;

	if (maple_io_connected)
		ec = dreamcastControllerConnection->sendMsg(msg);
	else
		return false;
	if (ec) {
		maple_io_connected = false;
		WARN_LOG(INPUT, "DreamcastController[%d] send failed: %s", bus, ec.message().c_str());
		disconnect();
		return false;
	}
	return true;
}

bool DreamConnGamepad::isDreamcastController(int deviceIndex)
{
	char guid_str[33] {};
	SDL_JoystickGetGUIDString(SDL_JoystickGetDeviceGUID(deviceIndex), guid_str, sizeof(guid_str));
	NOTICE_LOG(INPUT, "GUID: %s VID:%c%c%c%c PID:%c%c%c%c", guid_str,
			guid_str[10], guid_str[11], guid_str[8], guid_str[9],
			guid_str[18], guid_str[19], guid_str[16], guid_str[17]);

	// DreamConn VID:4457 PID:4443
	// Dreamcast Controller USB VID:1209 PID:2f07
	if (memcmp("5744000043440000", guid_str + 8, 16) == 0 || memcmp("09120000072f0000", guid_str + 8, 16) == 0)
	{
		NOTICE_LOG(INPUT, "Dreamcast controller found!");
		return true;
	}
	return false;
}

DreamConnGamepad::DreamConnGamepad(int maple_port, int joystick_idx, SDL_Joystick* sdl_joystick)
	: SDLGamepad(maple_port, joystick_idx, sdl_joystick)
{
	char guid_str[33] {};

	SDL_JoystickGetGUIDString(SDL_JoystickGetDeviceGUID(joystick_idx), guid_str, sizeof(guid_str));

	// DreamConn VID:4457 PID:4443
	// Dreamcast Controller USB VID:1209 PID:2f07
	if (memcmp("5744000043440000", guid_str + 8, 16) == 0)
	{
		dreamcastControllerType = TYPE_DREAMCONN;
		_name = "DreamConn+ / DreamConn S Controller";
	}
	else if (memcmp("09120000072f0000", guid_str + 8, 16) == 0)
	{
		dreamcastControllerType = TYPE_DREAMCASTCONTROLLERUSB;
		_name = "Dreamcast Controller USB";
	}

	EventManager::listen(Event::Start, handleEvent, this);
	EventManager::listen(Event::LoadState, handleEvent, this);
}

DreamConnGamepad::~DreamConnGamepad() {
	EventManager::unlisten(Event::Start, handleEvent, this);
	EventManager::unlisten(Event::LoadState, handleEvent, this);
}

void DreamConnGamepad::set_maple_port(int port)
{
	if (port < 0 || port >= 4) {
		dreamconn.reset();
	}
	else if (dreamconn == nullptr || dreamconn->getBus() != port) {
		dreamconn.reset();
		dreamconn = std::make_shared<DreamConn>(port, dreamcastControllerType);
	}
	SDLGamepad::set_maple_port(port);
}

void DreamConnGamepad::handleEvent(Event event, void *arg)
{
	DreamConnGamepad *gamepad = static_cast<DreamConnGamepad*>(arg);
	if (gamepad->dreamconn != nullptr)
		createDreamConnDevices(gamepad->dreamconn, event == Event::Start);
}

bool DreamConnGamepad::gamepad_btn_input(u32 code, bool pressed)
{
	if (!is_detecting_input() && input_mapper)
	{
		DreamcastKey key = input_mapper->get_button_id(0, code);
		if (key == DC_BTN_START) {
			startPressed = pressed;
			checkKeyCombo();
		}
	}
	else {
		startPressed = false;
	}
	return SDLGamepad::gamepad_btn_input(code, pressed);
}

bool DreamConnGamepad::gamepad_axis_input(u32 code, int value)
{
	if (!is_detecting_input())
	{
		if (code == leftTrigger) {
			ltrigPressed = value > 0;
			checkKeyCombo();
		}
		else if (code == rightTrigger) {
			rtrigPressed = value > 0;
			checkKeyCombo();
		}
	}
	else {
		ltrigPressed = false;
		rtrigPressed = false;
	}
	return SDLGamepad::gamepad_axis_input(code, value);
}

void DreamConnGamepad::checkKeyCombo() {
	if (ltrigPressed && rtrigPressed && startPressed)
		gui_open_settings();
}

#else // USE_DREAMCASTCONTROLLER

void DreamConn::connect() {
}
void DreamConn::disconnect() {
}

bool DreamConnGamepad::isDreamcastController(int deviceIndex) {
	return false;
}
DreamConnGamepad::DreamConnGamepad(int maple_port, int joystick_idx, SDL_Joystick* sdl_joystick)
	: SDLGamepad(maple_port, joystick_idx, sdl_joystick) {
}
DreamConnGamepad::~DreamConnGamepad() {
}
void DreamConnGamepad::set_maple_port(int port) {
	SDLGamepad::set_maple_port(port);
}
bool DreamConnGamepad::gamepad_btn_input(u32 code, bool pressed) {
	return SDLGamepad::gamepad_btn_input(code, pressed);
}
bool DreamConnGamepad::gamepad_axis_input(u32 code, int value) {
	return SDLGamepad::gamepad_axis_input(code, value);
}
#endif
