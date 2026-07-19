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

//! Only to be called during instantiation to determine hardware information
static DreamPicoPort::HardwareInfo parse_hw_info(JNIEnv *env, int id, const std::string& name)
{
	DreamPicoPort::HardwareInfo hwInfo;
	hwInfo.serial_number = DreamPicoPort::getSerialFromName(name);

	if (hwInfo.serial_number.empty())
	{
		NOTICE_LOG(INPUT, "Failed to retrieve serial from DreamPicoPort name: %s", name.c_str());
		return hwInfo;
	}

	jclass inputDeviceClass = env->FindClass("android/view/InputDevice");
	if (!inputDeviceClass)
	{
		NOTICE_LOG(INPUT, "Failed to locate android/view/InputDevice");
		return hwInfo;
	}

	jmethodID getDeviceMethodId = env->GetStaticMethodID(inputDeviceClass, "getDevice", "(I)Landroid/view/InputDevice;");
	if (!getDeviceMethodId)
	{
		NOTICE_LOG(INPUT, "Failed to locate InputDevice.getDevice()");
		return hwInfo;
	}

	jmethodID getNameMethodId = env->GetMethodID(inputDeviceClass, "getName", "()Ljava/lang/String;");
	if (!getNameMethodId)
	{
		NOTICE_LOG(INPUT, "Failed to locate InputDevice.getName()");
		return hwInfo;
	}

	hwInfo.hardware_bus = get_serial_count(
		env,
		id,
		hwInfo.serial_number,
		-1,
		inputDeviceClass,
		getDeviceMethodId,
		getNameMethodId
	);

	if (hwInfo.hardware_bus == 0)
	{
		if (
			get_serial_count(
				env,
				id,
				hwInfo.serial_number,
				1,
				inputDeviceClass,
				getDeviceMethodId,
				getNameMethodId,
				1
			) > 0
		)
		{
			hwInfo.is_single_device = false;
		}
	}
	else
	{
		hwInfo.is_single_device = false;
	}

	// TODO: Need to interrogate what interfaces the USB device has - this will require permission

	return hwInfo;
}

AndroidDreamPicoPortGamepad::AndroidDreamPicoPortGamepad(
	JNIEnv *env,
	int maple_port,
	int id,
	const char *name,
	const char *unique_id,
	const std::vector<int>& fullAxes,
	const std::vector<int>& halfAxes,
	jobject device
) :
	DreamLinkAndroidGamepad(
		std::make_shared<DreamPicoPort>(maple_port, parse_hw_info(env, id, name)),
		maple_port,
		id,
		name,
		unique_id,
		fullAxes,
		halfAxes
	),
	dpp(std::dynamic_pointer_cast<DreamPicoPort>(dreamlink))
{
	// The name will be the main device name, not the specific gamepad name
	// e.x. "OrangeFox86 DreamPicoPort-E66141040371972A v1.2.4"

	assert(dpp != nullptr);
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

// Need to define destructor in source because DreamPicoPort has a forward declaration
AndroidDreamPicoPortGamepad::~AndroidDreamPicoPortGamepad()
{
}

bool AndroidDreamPicoPortGamepad::identify(int vendorId, int productId)
{
	return (vendorId == DreamPicoPort::VID && productId == DreamPicoPort::PID);
}
