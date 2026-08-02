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

#include "android_dreampicoport_gamepad.h"
#include <input/dreamlink/dreampicoport.h>
#include <jni.h>

#include <optional>
#include <unordered_map>
#include <mutex>

class AndroidDreamPicoPort : public DreamPicoPort
{
public:
	struct UsbDeviceConnection
	{
		jni::Object connection_obj = {};
		intptr_t sys_dev = -1;
	};

	struct ExtendedHardwareInfo
	{
		HardwareInfo base_info;
		std::shared_ptr<UsbDeviceConnection> usb_device_connection = {};
	};

	AndroidDreamPicoPort(int bus, ExtendedHardwareInfo& hw_info) :
		DreamPicoPort(bus, hw_info.base_info),
		usb_device_connection(hw_info.usb_device_connection)
	{}

	~AndroidDreamPicoPort()
	{}

	void close(JNIEnv *env)
	{
		usb_device_connection.reset();
	}

	static std::shared_ptr<UsbDeviceConnection> openUsbDeviceAndGetFd(
		JNIEnv *env,
		jobject usbManager,
		jobject usbDev,
		const std::string& serial
	) {
		// A lookup is used because up to 4 controllers may have the same serial and don't need redundant connections
		static std::unordered_map<std::string, std::weak_ptr<UsbDeviceConnection>> lookup;
		static std::mutex lookupMutex;

		std::lock_guard<std::mutex> lock(lookupMutex);

		auto iter = lookup.find(serial);
		if (iter != lookup.end()) {
			std::shared_ptr<UsbDeviceConnection> ptr = iter->second.lock();
			if (ptr) {
				return ptr;
			}
			// Weak pointer no longer valid; remove it
			lookup.erase(iter);
		}

		jni::Class usbManagerClass(env->GetObjectClass(usbManager));
		jmethodID openDeviceMethod = env->GetMethodID(
			usbManagerClass,
			"openDevice",
			"(Landroid/hardware/usb/UsbDevice;)Landroid/hardware/usb/UsbDeviceConnection;"
		);

		jni::Object connection(env->CallObjectMethod(usbManager, openDeviceMethod, usbDev));

		if (connection == nullptr) {
			// openDevice failed — permission not granted, or device disconnected
			return nullptr;
		}

		jni::Class connectionClass(env->GetObjectClass(connection));
		jmethodID getFileDescriptorMethod = env->GetMethodID(connectionClass, "getFileDescriptor", "()I");

		jint fd = env->CallIntMethod(connection, getFileDescriptorMethod);

		std::shared_ptr<UsbDeviceConnection> newConnection = std::make_shared<UsbDeviceConnection>();
		// Promote to global ref so it survives past this call — required since
		// libusb will use the fd beyond the lifetime of this native call frame
		newConnection->connection_obj = connection.globalRef<jni::Object>();
		newConnection->sys_dev = (intptr_t)fd;

		// Save to lookup for future use
		lookup[serial] = newConnection;

		// Warning: this connection should not be retained once the USB device is disconnected
		return newConnection;
	}

private:
	std::shared_ptr<UsbDeviceConnection> usb_device_connection;
};

//! Get number of devices that contain the given serial at the specified ID direction
//! @param[in] env The Java Native Interface environment object
//! @param[in] id The ID of the target device
//! @param[in] serial The serial of the target device
//! @param[in] direction -1 to search backwards or 1 to search forwards
//! @param[in] inputDeviceClass The InputDevice class pointer
//! @param[in] getDeviceMethodId Method ID of InputDevice.getDevice()
//! @param[in] getNameMethodId Method ID of InputDevice.getName()
//! @param[in] maxCount The maximum count to return (default: 3)
//! @return the count
static int get_serial_count(
	JNIEnv *env,
	int id,
	const std::string& serial,
	int direction,
	const jni::Class& inputDeviceClass,
	jmethodID getDeviceMethodId,
	jmethodID getNameMethodId,
	int maxCount = 3
)
{
	const int increment = (direction < 0) ? -1 : 1;
	int count = 0;
	static const int kMaxMisses = 10;
	jint lastKnownId = id;
	jint testId = id + increment;
	while (testId >= 0 && ((testId - lastKnownId) * increment) < kMaxMisses && count < maxCount)
	{
		jni::Object device(env->CallStaticObjectMethod(inputDeviceClass, getDeviceMethodId, testId));
		if (!device.isNull())
		{
			jni::String jName(env->CallObjectMethod(device, getNameMethodId));

			if (!jName.isNull())
			{
				std::string deviceName(jName.to_string());
				if (deviceName.find(serial) != std::string::npos)
				{
					lastKnownId = testId;
					++count;
				}
			}
		}

		testId += increment;
	}

	return count;
}

static jni::Object findUsbDeviceByVidPidSerial(
	JNIEnv *env,
	jobject usbManager,
	jint targetVid,
	jint targetPid,
	const std::string& targetSerial
)
{
	jni::Class usbManagerClass(env->GetObjectClass(usbManager));
	jmethodID getDeviceListMethod = env->GetMethodID(usbManagerClass, "getDeviceList", "()Ljava/util/HashMap;");
	jni::Object deviceMap(env->CallObjectMethod(usbManager, getDeviceListMethod));

	if (deviceMap.isNull()) {
		return nullptr;
	}

	jni::Class mapClass(env->GetObjectClass(deviceMap));
	jmethodID valuesMethod = env->GetMethodID(mapClass, "values", "()Ljava/util/Collection;");
	jni::Object valuesCollection(env->CallObjectMethod(deviceMap, valuesMethod));

	jni::Class collectionClass(env->GetObjectClass(valuesCollection));
	jmethodID iteratorMethod = env->GetMethodID(collectionClass, "iterator", "()Ljava/util/Iterator;");
	jni::Object iterator(env->CallObjectMethod(valuesCollection, iteratorMethod));

	jni::Class iteratorClass(env->GetObjectClass(iterator));
	jmethodID hasNextMethod = env->GetMethodID(iteratorClass, "hasNext", "()Z");
	jmethodID nextMethod = env->GetMethodID(iteratorClass, "next", "()Ljava/lang/Object;");

	jni::Class usbDeviceClass(env->FindClass("android/hardware/usb/UsbDevice"));
	jmethodID getVendorIdMethod = env->GetMethodID(usbDeviceClass, "getVendorId", "()I");
	jmethodID getProductIdMethod = env->GetMethodID(usbDeviceClass, "getProductId", "()I");
	jmethodID getSerialNumberMethod = env->GetMethodID(usbDeviceClass, "getSerialNumber", "()Ljava/lang/String;");

	while (env->CallBooleanMethod(iterator, hasNextMethod)) {
		jni::Object device(env->CallObjectMethod(iterator, nextMethod));

		jint vid = env->CallIntMethod(device, getVendorIdMethod);
		jint pid = env->CallIntMethod(device, getProductIdMethod);

		bool matches = (vid == targetVid && pid == targetPid);

		if (matches) {
			jni::String deviceSerial(env->CallObjectMethod(device, getSerialNumberMethod));

			if (env->ExceptionCheck()) {
				// SecurityException: permission not granted for this device
				env->ExceptionClear();
				matches = false;
			} else if (deviceSerial.isNull()) {
				// Permission not yet granted, or device has no serial — can't match
				matches = false;
			} else {
				matches = (deviceSerial.to_string() == targetSerial);
			}
		}

		if (matches) {
			return device;
		}
	}

	return jni::Object(); // null
}

// Returns the interface ID at position n (0-based) in the sorted list of
// distinct interface IDs present on the device. Returns -1 if n is out of range.
static int getNthInterfaceId(JNIEnv *env, jobject usbDevice, int n) {
	jni::Class usbDeviceClass(env->GetObjectClass(usbDevice));
	jmethodID getInterfaceCountMethod = env->GetMethodID(usbDeviceClass, "getInterfaceCount", "()I");
	jmethodID getInterfaceMethod = env->GetMethodID(
		usbDeviceClass,
		"getInterface",
		"(I)Landroid/hardware/usb/UsbInterface;"
	);

	jint interfaceCount = env->CallIntMethod(usbDevice, getInterfaceCountMethod);

	jni::Class usbInterfaceClass(env->FindClass("android/hardware/usb/UsbInterface"));
	jmethodID getIdMethod = env->GetMethodID(usbInterfaceClass, "getId", "()I");

	std::vector<jint> interfaceIds;
	interfaceIds.reserve(interfaceCount);

	for (jint i = 0; i < interfaceCount; i++) {
		jni::Object usbInterface(env->CallObjectMethod(usbDevice, getInterfaceMethod, i));
		jint id = env->CallIntMethod(usbInterface, getIdMethod);
		interfaceIds.push_back(id);
	}

	std::sort(interfaceIds.begin(), interfaceIds.end());
	interfaceIds.erase(std::unique(interfaceIds.begin(), interfaceIds.end()), interfaceIds.end());

	if (n < 0 || n >= static_cast<jint>(interfaceIds.size())) {
		return -1; // out of range
	}

	return interfaceIds[n];
}

//! Only to be called during instantiation to determine hardware information
static std::optional<AndroidDreamPicoPort::ExtendedHardwareInfo> parse_hw_info(
	JNIEnv *env,
	jobject usbManager,
	int id,
	const std::string& name
)
{
	AndroidDreamPicoPort::ExtendedHardwareInfo hwInfo;
	hwInfo.base_info.serial_number = DreamPicoPort::getSerialFromName(name);

	if (hwInfo.base_info.serial_number.empty())
	{
		NOTICE_LOG(INPUT, "Failed to retrieve serial from DreamPicoPort name: %s", name.c_str());
		return std::nullopt;
	}

	// Attempt to retrieve the UsbDevice for this serial number
	jni::Object usbDev = findUsbDeviceByVidPidSerial(
		env,
		usbManager,
		DreamPicoPort::VID,
		DreamPicoPort::PID,
		hwInfo.base_info.serial_number
	);

	if (usbDev.isNull())
	{
		// Probably don't have permission yet
		NOTICE_LOG(INPUT, "Failed to retrieve DreamPicoPort UsbDevice for %s", hwInfo.base_info.serial_number.c_str());
		return std::nullopt;
	}

	jni::Class inputDeviceClass(env->FindClass("android/view/InputDevice"));
	if (inputDeviceClass.isNull())
	{
		NOTICE_LOG(INPUT, "Failed to locate android/view/InputDevice");
		return std::nullopt;
	}

	jmethodID getDeviceMethodId = env->GetStaticMethodID(inputDeviceClass, "getDevice", "(I)Landroid/view/InputDevice;");
	if (!getDeviceMethodId)
	{
		NOTICE_LOG(INPUT, "Failed to locate InputDevice.getDevice()");
		return std::nullopt;
	}

	jmethodID getNameMethodId = env->GetMethodID(inputDeviceClass, "getName", "()Ljava/lang/String;");
	if (!getNameMethodId)
	{
		NOTICE_LOG(INPUT, "Failed to locate InputDevice.getName()");
		return std::nullopt;
	}

	hwInfo.base_info.hardware_bus = get_serial_count(
		env,
		id,
		hwInfo.base_info.serial_number,
		-1,
		inputDeviceClass,
		getDeviceMethodId,
		getNameMethodId
	);

	if (hwInfo.base_info.hardware_bus == 0)
	{
		if (
			get_serial_count(
				env,
				id,
				hwInfo.base_info.serial_number,
				1,
				inputDeviceClass,
				getDeviceMethodId,
				getNameMethodId,
				1
			) > 0
		)
		{
			hwInfo.base_info.is_single_device = false;
		}
	}
	else
	{
		hwInfo.base_info.is_single_device = false;
	}

	// Interfaces 0-3 correspond to the four gamepad ports. If only ports B and D are connected,
	// their interface IDs would otherwise be seen as 0 and 1; remapping them to their true
	// slot indices (1 and 3) keeps hardware_bus consistent with the physical port layout.
	hwInfo.base_info.hardware_bus = getNthInterfaceId(env, usbDev, hwInfo.base_info.hardware_bus);

	hwInfo.usb_device_connection =
		AndroidDreamPicoPort::openUsbDeviceAndGetFd(env, usbManager, usbDev, hwInfo.base_info.serial_number);

	if (!hwInfo.usb_device_connection)
	{
		NOTICE_LOG(INPUT, "Failed to open file descriptor to DreamPicoPort device");
		return std::nullopt;
	}

	hwInfo.base_info.sys_dev = hwInfo.usb_device_connection->sys_dev;

	return hwInfo;
}

static std::shared_ptr<AndroidDreamPicoPort> make_dpp(
	JNIEnv *env,
	jobject usbManager,
	int maple_port,
	int id,
	const std::string& name
)
{
	std::optional<AndroidDreamPicoPort::ExtendedHardwareInfo> hwInfo = parse_hw_info(env, usbManager, id, name);

	if (!hwInfo.has_value())
	{
		return nullptr;
	}

	if (hwInfo->base_info.hardware_bus >= 0 && hwInfo->usb_device_connection)
	{
		return std::make_shared<AndroidDreamPicoPort>(maple_port, hwInfo.value());
	}

	return nullptr;
}

AndroidDreamPicoPortGamepad::AndroidDreamPicoPortGamepad(
	JNIEnv *env,
	jobject usbManager,
	int maple_port,
	const AndroidGamepadDevice::AndroidJoystickData& joystickData
) :
	DreamLinkAndroidGamepad(
		make_dpp(env, usbManager, maple_port, joystickData.id, joystickData.joyname),
		maple_port,
		joystickData
	),
	dpp(std::dynamic_pointer_cast<AndroidDreamPicoPort>(dreamlink)),
	android_name(joystickData.joyname)
{
	// The name will be the main device name, not the specific gamepad name
	// e.x. "OrangeFox86 DreamPicoPort-E66141040371972A v1.2.4"

	if (dpp) {
		_name = dpp->getName();

		const DreamPicoPort::HardwareInfo& hw_info = dpp->getHardwareInfo();

		if (!hw_info.serial_number.empty()) {
			// Ensure this is ordered by product name, serial, and port char
			_sort_id = (
				hw_info.getProductName() + std::string("_") +
				hw_info.serial_number + std::string("_") +
				std::string(1, hw_info.getPortChar())
			);
			// Locking to name, which includes A-D, plus serial number will ensure correct enumeration every time
			_unique_id = hw_info.getName("", true) + std::string("_") + hw_info.serial_number;
			// Reload mapping now that unique ID changed
			loadMapping();
		}

		int bus = dpp->getDefaultBus();
		if (DreamLink::isValidBus(bus))
			set_maple_port(bus);
	} else {
		_name = "DreamPicoPort";
	}
}

// Need to define destructor in source because DreamPicoPort has a forward declaration
AndroidDreamPicoPortGamepad::~AndroidDreamPicoPortGamepad()
{
}

const char *AndroidDreamPicoPortGamepad::get_button_name(u32 code)
{
	using namespace i18n;
	switch (static_cast<s32>(code)) {
		case static_cast<s32>(ButtonCode::D): return "D";
		case static_cast<s32>(ButtonCode::UP_B): return T("DPad2 Up");
		case static_cast<s32>(ButtonCode::DOWN_B): return T("DPad2 Down");
		case static_cast<s32>(ButtonCode::LEFT_B): return T("DPad2 Left");
		case static_cast<s32>(ButtonCode::RIGHT_B): return T("DPad2 Right");

		// Alternate directional buttons
		case static_cast<s32>(ButtonCode::ALT_UP): return T("Alt Up");
		case static_cast<s32>(ButtonCode::ALT_DOWN): return T("Alt Down");
		case static_cast<s32>(ButtonCode::ALT_LEFT): return T("Alt Left");
		case static_cast<s32>(ButtonCode::ALT_RIGHT): return T("Alt Right");

		// These buttons are normally not physically accessible but are mapped on DreamPicoPort
		case static_cast<s32>(ButtonCode::VMU1_BUTTON_A): return T("VMU1 A");
		case static_cast<s32>(ButtonCode::VMU1_BUTTON_B): return T("VMU1 B");
		case static_cast<s32>(ButtonCode::VMU1_BUTTON_UP): return T("VMU1 Up");
		case static_cast<s32>(ButtonCode::VMU1_BUTTON_DOWN): return T("VMU1 Down");
		case static_cast<s32>(ButtonCode::VMU1_BUTTON_LEFT): return T("VMU1 Left");
		case static_cast<s32>(ButtonCode::VMU1_BUTTON_RIGHT): return T("VMU1 Right");

		case static_cast<s32>(ButtonCode::CHANGE_EVENT): return T("Device Change");

		default: return DreamLinkAndroidGamepad::get_button_name(code); // use the default name
	}
}

const char *AndroidDreamPicoPortGamepad::get_axis_name(u32 code)
{
	using namespace i18n;
	switch (static_cast<s32>(code)) {
		case static_cast<s32>(AxisCode::LEFT_TRIGGER): return T("LT");
		case static_cast<s32>(AxisCode::RIGHT_TRIGGER): return T("RT");

		default: return DreamLinkAndroidGamepad::get_axis_name(code); // use the default name
	}
}

void AndroidDreamPicoPortGamepad::close(JNIEnv *env)
{
	DreamLinkAndroidGamepad::close(env);
	if (dpp) {
		dpp->close(env);
	}
}

bool AndroidDreamPicoPortGamepad::identify(int vendorId, int productId)
{
	return (vendorId == DreamPicoPort::VID && productId == DreamPicoPort::PID);
}

bool AndroidDreamPicoPortGamepad::gamepad_btn_input(u32 code, bool pressed)
{
	if (static_cast<s32>(code) == static_cast<s32>(ButtonCode::CHANGE_EVENT) && !pressed)
	{
		if (dpp)
		{
			dpp->queryPeripherals(false);
		}
	}

	return DreamLinkAndroidGamepad::gamepad_btn_input(code, pressed);
}

void AndroidDreamPicoPortGamepad::setCustomMapping(const std::shared_ptr<InputMapping>& mapping)
{
	// Since this is a real DC controller, no deadzone adjustment is needed
	mapping->dead_zone = 0.0f;
	// Map the things not set by default
	mapping->set_button(DC_BTN_C, static_cast<u32>(ButtonCode::C));
	mapping->set_button(DC_BTN_Z, static_cast<u32>(ButtonCode::Z));
	mapping->set_button(DC_BTN_D, static_cast<u32>(ButtonCode::D));
	mapping->set_button(DC_DPAD2_UP, static_cast<u32>(ButtonCode::UP_B));
	mapping->set_button(DC_DPAD2_DOWN, static_cast<u32>(ButtonCode::DOWN_B));
	mapping->set_button(DC_DPAD2_LEFT, static_cast<u32>(ButtonCode::LEFT_B));
	mapping->set_button(DC_DPAD2_RIGHT, static_cast<u32>(ButtonCode::RIGHT_B));
	mapping->set_axis(DC_AXIS_LT, static_cast<u32>(AxisCode::LEFT_TRIGGER), true);
	mapping->set_axis(DC_AXIS_RT, static_cast<u32>(AxisCode::RIGHT_TRIGGER), true);
	mapping->set_axis(DC_AXIS2_LEFT, static_cast<u32>(AxisCode::RIGHT_X), false);
	mapping->set_axis(DC_AXIS2_RIGHT, static_cast<u32>(AxisCode::RIGHT_X), true);
	mapping->set_axis(DC_AXIS2_UP, static_cast<u32>(AxisCode::RIGHT_Y), false);
	mapping->set_axis(DC_AXIS2_DOWN, static_cast<u32>(AxisCode::RIGHT_Y), true);
}
