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

class AndroidDreamPicoPort : public DreamPicoPort
{
public:
	struct ExtendedHardwareInfo
	{
		HardwareInfo base_info;
		jobject usb_device_connection = nullptr;
	};

	AndroidDreamPicoPort(int bus, ExtendedHardwareInfo hw_info) :
		DreamPicoPort(bus, hw_info.base_info),
		usb_device_connection(hw_info.usb_device_connection)
	{}

	~AndroidDreamPicoPort()
	{}

	void close(JNIEnv *env)
	{
		env->DeleteGlobalRef(usb_device_connection);
	}

private:
	const jobject usb_device_connection;
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
	jclass inputDeviceClass,
	jmethodID getDeviceMethodId,
	jmethodID getNameMethodId,
	int maxCount = 3
)
{
	// Normalize direction
	if (direction < 0)
	{
		direction = -1;
	}
	else
	{
		direction = 1;
	}

	int count = 0;

	static const int kMaxMisses = 10;
	jint lastKnownId = id;
	jint testId = id + direction;
	while (testId >= 0 && ((testId - lastKnownId) * direction) < kMaxMisses && count < maxCount)
	{
		jobject device = env->CallStaticObjectMethod(inputDeviceClass, getDeviceMethodId, testId);
		if (device)
		{
			jstring jName = (jstring)env->CallObjectMethod(device, getNameMethodId);

			if (jName)
			{
				const char* nativeString = env->GetStringUTFChars(jName, nullptr);

				if (nativeString)
				{
					std::string deviceName(nativeString);
					if (deviceName.find(serial) != std::string::npos)
					{
						lastKnownId = testId;
						++count;
					}
				}

				env->DeleteLocalRef(jName);
			}

			env->DeleteLocalRef(device);
		}

		testId += direction;
	}

	return count;
}

jobject findUsbDeviceByVidPidSerial(
	JNIEnv *env,
	jobject usbManager,
	jint targetVid,
	jint targetPid,
	const char *targetSerial
)
{
	jclass usbManagerClass = env->GetObjectClass(usbManager);
	jmethodID getDeviceListMethod = env->GetMethodID(
			usbManagerClass, "getDeviceList", "()Ljava/util/HashMap;");
	jobject deviceMap = env->CallObjectMethod(usbManager, getDeviceListMethod);

	if (deviceMap == nullptr) {
		return nullptr;
	}

	jclass mapClass = env->GetObjectClass(deviceMap);
	jmethodID valuesMethod = env->GetMethodID(mapClass, "values", "()Ljava/util/Collection;");
	jobject valuesCollection = env->CallObjectMethod(deviceMap, valuesMethod);

	jclass collectionClass = env->GetObjectClass(valuesCollection);
	jmethodID iteratorMethod = env->GetMethodID(
			collectionClass, "iterator", "()Ljava/util/Iterator;");
	jobject iterator = env->CallObjectMethod(valuesCollection, iteratorMethod);

	jclass iteratorClass = env->GetObjectClass(iterator);
	jmethodID hasNextMethod = env->GetMethodID(iteratorClass, "hasNext", "()Z");
	jmethodID nextMethod = env->GetMethodID(iteratorClass, "next", "()Ljava/lang/Object;");

	jclass usbDeviceClass = env->FindClass("android/hardware/usb/UsbDevice");
	jmethodID getVendorIdMethod = env->GetMethodID(usbDeviceClass, "getVendorId", "()I");
	jmethodID getProductIdMethod = env->GetMethodID(usbDeviceClass, "getProductId", "()I");
	jmethodID getSerialNumberMethod = env->GetMethodID(
			usbDeviceClass, "getSerialNumber", "()Ljava/lang/String;");

	jobject result = nullptr;

	while (env->CallBooleanMethod(iterator, hasNextMethod)) {
		jobject device = env->CallObjectMethod(iterator, nextMethod);

		jint vid = env->CallIntMethod(device, getVendorIdMethod);
		jint pid = env->CallIntMethod(device, getProductIdMethod);

		bool matches = (vid == targetVid && pid == targetPid);

		if (matches && targetSerial != nullptr) {
			auto deviceSerial = (jstring) env->CallObjectMethod(device, getSerialNumberMethod);

			if (deviceSerial == nullptr) {
				// Permission not yet granted, or device has no serial — can't match
				matches = false;
			} else {
				const char *serialChars = env->GetStringUTFChars(deviceSerial, nullptr);
				matches = (strcmp(serialChars, targetSerial) == 0);
				env->ReleaseStringUTFChars(deviceSerial, serialChars);
				env->DeleteLocalRef(deviceSerial);
			}
		}

		if (matches) {
			result = device;
			break;
		}

		env->DeleteLocalRef(device);
	}

	env->DeleteLocalRef(usbManagerClass);
	env->DeleteLocalRef(deviceMap);
	env->DeleteLocalRef(mapClass);
	env->DeleteLocalRef(valuesCollection);
	env->DeleteLocalRef(collectionClass);
	env->DeleteLocalRef(iterator);
	env->DeleteLocalRef(iteratorClass);
	env->DeleteLocalRef(usbDeviceClass);

	return result; // local ref, or nullptr — caller must DeleteLocalRef when done
}

// Returns the interface ID at position n (0-based) in the sorted list of
// distinct interface IDs present on the device. Returns -1 if n is out of range.
static int getNthInterfaceId(JNIEnv *env, jobject usbDevice, int n) {
	jclass usbDeviceClass = env->GetObjectClass(usbDevice);
	jmethodID getInterfaceCountMethod = env->GetMethodID(usbDeviceClass, "getInterfaceCount", "()I");
	jmethodID getInterfaceMethod = env->GetMethodID(
		usbDeviceClass,
		"getInterface",
		"(I)Landroid/hardware/usb/UsbInterface;"
	);

	jint interfaceCount = env->CallIntMethod(usbDevice, getInterfaceCountMethod);

	jclass usbInterfaceClass = env->FindClass("android/hardware/usb/UsbInterface");
	jmethodID getIdMethod = env->GetMethodID(usbInterfaceClass, "getId", "()I");

	std::vector<jint> interfaceIds;
	interfaceIds.reserve(interfaceCount);

	for (jint i = 0; i < interfaceCount; i++) {
		jobject usbInterface = env->CallObjectMethod(usbDevice, getInterfaceMethod, i);
		jint id = env->CallIntMethod(usbInterface, getIdMethod);
		interfaceIds.push_back(id);
		env->DeleteLocalRef(usbInterface);
	}

	env->DeleteLocalRef(usbDeviceClass);
	env->DeleteLocalRef(usbInterfaceClass);

	std::sort(interfaceIds.begin(), interfaceIds.end());
	interfaceIds.erase(std::unique(interfaceIds.begin(), interfaceIds.end()), interfaceIds.end());

	if (n < 0 || n >= static_cast<jint>(interfaceIds.size())) {
		return -1; // out of range
	}

	return interfaceIds[n];
}

// Returns the native fd for the device, or -1 on failure.
// Also outputs the UsbDeviceConnection via outConnection (as a global ref) —
// you MUST keep this alive for as long as libusb is using the fd.
static intptr_t openUsbDeviceAndGetFd(JNIEnv *env, jobject usbManager, jobject usbDev, jobject *outConnection) {
	jclass usbManagerClass = env->GetObjectClass(usbManager);
	jmethodID openDeviceMethod = env->GetMethodID(
			usbManagerClass, "openDevice",
			"(Landroid/hardware/usb/UsbDevice;)Landroid/hardware/usb/UsbDeviceConnection;");

	jobject connection = env->CallObjectMethod(usbManager, openDeviceMethod, usbDev);
	env->DeleteLocalRef(usbManagerClass);

	if (connection == nullptr) {
		// openDevice failed — permission not granted, or device disconnected
		return -1;
	}

	jclass connectionClass = env->GetObjectClass(connection);
	jmethodID getFileDescriptorMethod = env->GetMethodID(
			connectionClass, "getFileDescriptor", "()I");

	jint fd = env->CallIntMethod(connection, getFileDescriptorMethod);
	env->DeleteLocalRef(connectionClass);

	// Promote to global ref so it survives past this call — required since
	// libusb will use the fd beyond the lifetime of this native call frame
	*outConnection = env->NewGlobalRef(connection);
	env->DeleteLocalRef(connection);

	return (intptr_t)fd;
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
	jobject usbDev = findUsbDeviceByVidPidSerial(
		env,
		usbManager,
		DreamPicoPort::VID,
		DreamPicoPort::PID,
		hwInfo.base_info.serial_number.c_str()
	);

	if (!usbDev)
	{
		// Probably don't have permission yet
		NOTICE_LOG(INPUT, "Failed to retrieve DreamPicoPort UsbDevice for %s", hwInfo.base_info.serial_number.c_str());
		return std::nullopt;
	}

	jclass inputDeviceClass = env->FindClass("android/view/InputDevice");
	if (!inputDeviceClass)
	{
		NOTICE_LOG(INPUT, "Failed to locate android/view/InputDevice");
		env->DeleteLocalRef(usbDev);
		return std::nullopt;
	}

	jmethodID getDeviceMethodId = env->GetStaticMethodID(inputDeviceClass, "getDevice", "(I)Landroid/view/InputDevice;");
	if (!getDeviceMethodId)
	{
		NOTICE_LOG(INPUT, "Failed to locate InputDevice.getDevice()");
		env->DeleteLocalRef(inputDeviceClass);
		env->DeleteLocalRef(usbDev);
		return std::nullopt;
	}

	jmethodID getNameMethodId = env->GetMethodID(inputDeviceClass, "getName", "()Ljava/lang/String;");
	if (!getNameMethodId)
	{
		NOTICE_LOG(INPUT, "Failed to locate InputDevice.getName()");
		env->DeleteLocalRef(inputDeviceClass);
		env->DeleteLocalRef(usbDev);
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

	env->DeleteLocalRef(inputDeviceClass);

	// Interfaces 0-3 correspond to the four gamepad ports. If only ports B and D are connected,
	// their interface IDs would otherwise be seen as 0 and 1; remapping them to their true
	// slot indices (1 and 3) keeps hardware_bus consistent with the physical port layout.
	hwInfo.base_info.hardware_bus = getNthInterfaceId(env, usbDev, hwInfo.base_info.hardware_bus);

	// TODO: it may be a good idea to cache serial -> usb_device_connection so it's not opened 4 times
	hwInfo.base_info.sys_dev = openUsbDeviceAndGetFd(env, usbManager, usbDev, &hwInfo.usb_device_connection);

	env->DeleteLocalRef(usbDev);

	if (hwInfo.base_info.sys_dev < 0)
	{
		NOTICE_LOG(INPUT, "Failed to open file descriptor to DreamPicoPort device");
		if (hwInfo.usb_device_connection)
		{
			env->DeleteGlobalRef(hwInfo.usb_device_connection);
		}
		return std::nullopt;
	}

	return hwInfo;
}

static std::shared_ptr<AndroidDreamPicoPort> make_dpp(
	JNIEnv *env,
	jobject usbManager,
	int maple_port,
	int id,
	const char *name
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
	int maple_port,
	int id,
	const char *name,
	const char *unique_id,
	const std::vector<int>& fullAxes,
	const std::vector<int>& halfAxes,
	jobject usbManager
) :
	DreamLinkAndroidGamepad(
		make_dpp(env, usbManager, maple_port, id, name),
		maple_port,
		id,
		name,
		unique_id,
		fullAxes,
		halfAxes
	),
	dpp(std::dynamic_pointer_cast<AndroidDreamPicoPort>(dreamlink)),
	android_name(name)
{
	// The name will be the main device name, not the specific gamepad name
	// e.x. "OrangeFox86 DreamPicoPort-E66141040371972A v1.2.4"

	if (dpp) {
		_name = dpp->getName();

		const DreamPicoPort::HardwareInfo& hw_info = dpp->getHardwareInfo();

		if (!hw_info.serial_number.empty()) {
			// TODO: need to handle sort ID
			// Ensure this is ordered by product name, serial, and port char
			// _sort_id = (
			// 	hw_info.getProductName() + std::string("_") +
			// 	hw_info.serial_number + std::string("_") +
			// 	std::string(1, hw_info.getPortChar())
			// );
			// Locking to name, which includes A-D, plus serial number will ensure correct enumeration every time
			_unique_id = hw_info.getName("", true) + std::string("_") + hw_info.serial_number;
			// Reload mapping now that unique ID changed
			loadMapping();
		}

		int bus = dpp->getDefaultBus();
		if (DreamLink::isValidBus(bus))
			set_maple_port(bus);
	}
}

// Need to define destructor in source because DreamPicoPort has a forward declaration
AndroidDreamPicoPortGamepad::~AndroidDreamPicoPortGamepad()
{
}

void AndroidDreamPicoPortGamepad::close(JNIEnv *env)
{
	DreamLinkAndroidGamepad::close(env);
	dpp->close(env);
	dpp.reset();
}

void AndroidDreamPicoPortGamepad::permissionGranted(JNIEnv *env, jobject usbManager)
{
	if (!dpp) {
		// Try to recreate the device and update parent if this was successful
		dpp = make_dpp(env, usbManager, maple_port(), get_android_id(), android_name.c_str());
		if (dpp) {
			updateDreamLink(dpp);
		}
	}
}

bool AndroidDreamPicoPortGamepad::identify(int vendorId, int productId)
{
	return (vendorId == DreamPicoPort::VID && productId == DreamPicoPort::PID);
}
