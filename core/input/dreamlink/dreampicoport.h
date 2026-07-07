/*
    Copyright 2026 The Hollycast Authors

    This file is part of Hollycast.

    Hollycast is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 2 of the License, or
    (at your option) any later version.

    Hollycast is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Hollycast.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "gamepad_dreamlink.h"
#include "../mapping.h"

#include <cstdint>
#include <chrono>
#include <memory>
#include <vector>
#include <mutex>

//! This class allows for communication to DreamPicoPort peripherals through libusb/WebUSB
class DreamPicoPort : public GamepadDreamLink
{
public:
    //! Static hardware information
    struct HardwareInfo
    {
        //! The bus index of the hardware connection which will differ from the software bus
        int hardware_bus = -1;
        //! true iff only a single devices was found when enumerating devices
        bool is_single_device = true;
        //! True when initial enumeration failed
        bool is_hardware_bus_implied = true;
        //! The located serial number of this device or empty string if could not be found
        std::string serial_number;
        //! If set, the determined unique ID of this device. If not set, the serial could not be parsed.
        std::string unique_id;
        //! The ID used for sorting on the UI
        std::string sort_id;

        //! @return The hardware port character identifier
        char getPortChar() const;

        //! @return the static product name
        static const char* getProductName();

        //! @param[in] separator Separator string to use between name and port char
        //! @return unique name of this device using the given separator
        std::string getName(const std::string& separator = " ") const;
    };

public:
    //! Default constructor = deleted
    DreamPicoPort() = delete;

    //! Constructor
    //! @param[in] bus Initial (software) bus
    //! @param[in] hw_info
    DreamPicoPort(int bus, HardwareInfo hw_info);

    //! Destructor (default)
    ~DreamPicoPort() = default;

    //! Send maple message, ignore response
    //! @param[in] msg The message to send
    //! @return true iff the message was sent
    bool send(const MapleMsg& msg) override;

    //! Send maple message and wait for a response
    //! @param[in] txMsg The message to send
    //! @param[out] rxMsg The response when the return value is true
    //! @return true iff the message was sent and a message was received
    bool sendReceive(const MapleMsg& txMsg, MapleMsg& rxMsg) override;

    //! Sends the current game id to a DreamLink backed expansion device if supported
    //! @param[in] expansion The expansion port to send to or -1 to send to all storage devices
    void sendGameId(int expansion = -1);

    //! Called when game has started
    void onGameStarted() override;

    //! Called when game has terminated
    void onGameTermination() override;

    //! Transform flycast port index into DreamPicoPort port index
    static int fcPortToDppPort(int forPort);

    //! Transform a DreamPicoPort port index into flycast port index
    static int dppPortToFcPort(int forPort);

    //! Retrieves the currently known function codes mask
    //! @param[in] forPort The port to query [0,5]
    //! @return a mask representing codes within MapleFunctionID
    u32 getFunctionCodesMask(int forPort) const override;

    //! Retrieves the function definitions (a u32 value for each function which contains header data)
    //! @param[in] forPort The port to query [0,5]
    //! @return the function definitions for each available function
    std::array<u32, 3> getFunctionDefinitions(int forPort) const;

    //! @return the default software bus index [0,3] to use when none is set is settings
    int getDefaultBus() const;

    //! @return the unique string ID of this DreamPicoPort
    const std::string& getUniqueId() const;

    //! @return the sorting ID of this DreamPicoPort
    const std::string& getSortId() const;

    //! Changes the software bus
    //! @param[in] newBus The new software bus to use [0,3]
    void changeBus(int newBus) override;

    //! Called once the gamepad associated with this DreamPicoPort has been registered
    void registered() override;

    //! @return the display name
    const char* getName() const override;

    //! @return the product name
    const char* getProductName() const override;

    //! Called on connection or reconfiguration to update state of hardware
    void setMapleDevices();

    //! @return true iff currently connected to a DreamPicoPort peripheral through libusb/WinUSB
    bool isConnected() override;

    //! @return the description of whatever issue is preventing connection
    const char* getIssueDescription() const override;

    //! @return a new maple_device to be used for this DreamPicoPort at the given bus and port
    std::shared_ptr<maple_device> createMapleDevice(int bus, int port) override;

    //! Attempt to connect to this DreamPicoPort through libusb/WinUSB
    void connect() override;

    //! Disconnect from device
    void disconnect() override;

    //! Send the current software bus to the DreamPicoPort so that it may update its default VMU display
    void sendPort();

    //! @return the hardware bus on the DreamPicoPort this instance is communicating with
    int hardwareBus() const;

    //! @return true iff the hardware bus is implied
    bool isHardwareBusImplied() const;

    //! @return true if the DreamPicoPort only contains one port
    bool isSingleDevice() const;

    //! @param[in] software_bus The software bus to use [0,3] or -1 to use the internally-known software bus
    //! @return a short description representing both the known hardware and software bus (for logging purposes)
    //!         ex: "A" if both hardware and software buses are 0
    //!         ex: "A,B" if software bus is set to 0 and hardware bus is 1
    std::string getLocDesc(int software_bus = -1) const;

    //! Refresh the internal state of function codes and definitions
    //! @param[in] clearOnFailure Set to true to clear out data if retrieval false or false to keep previously known
    //! @return true iff the query was successful
    bool queryPeripherals(bool clearOnFailure = true);

    //! Set custom mapping associated with the DreamPicoPort
    //! @param[in,out] mapping The mapping to update
    static void setCustomMapping(const std::shared_ptr<InputMapping>& mapping);

    //! Retrieves button name of non-standard button codes
    //! @param[in] code Button index
    //! @return button name for the given code
    //! @return nullptr if the default name should be used
    static const char *getButtonName(u32 code);

private:
    //! Internal connection call
    void internalConnect();

    //! Internal disconnection call
    void internalDisconnect();

    //! Schedules connection retry through the UI thread
    void scheduleConnectRetry();

    //! The connection callback executed from the UI thread
    void connectionCallback();

    //! Converts a function code mask to function name
    //! @param[in] fnCode The function code mask
    //! @return an associated function name
    static const char* fnToName(u32 fnCode);

public:
    //! Dreamcast Controller USB VID:1209 PID:2f07
    static constexpr std::uint16_t VID = 0x1209;
    static constexpr std::uint16_t PID = 0x2f07;

private:
    //! Duration to delay before trying to connect again
    static constexpr std::chrono::milliseconds CONNECT_RETRY_DELAY = std::chrono::milliseconds(1000);

    //! Serializes the externally-executed interfaces of this class
    mutable std::recursive_mutex mutex;
    //! Implements communication interface to DreamPicoPort
    std::unique_ptr<class ApiDreamPicoPortComms> dpp_comms;
    //! Set to true on first connection attempt
    bool connect_attempted = false;
    //! Set to true while connection was requested
    bool connect_requested = false;
    //! Set to true when connect retry has been scheduled
    bool connect_retry_scheduled = false;

    //! Current timeout in milliseconds
    std::chrono::milliseconds timeout_ms;
    //! The bus ID dictated by flycast
    int software_bus = -1;
    //! The queried interface version
    double interface_version = 0.0;
    //! Set to true if update is required
    bool update_required = false;
    //! The queried peripherals; for each function, index 0 is function code and index 1 is the function definition
    std::vector<std::vector<std::array<uint32_t, 2>>> peripherals;

    //! Hardware information determined on instantiation
    HardwareInfo hw_info;
    //! The name to return on getName
    const std::string device_name;
};