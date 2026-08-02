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
#pragma once

#include "dreamlink_android_gamepad.h"

#include <jni.h>

#include <memory>

class AndroidDreamPicoPortGamepad : public DreamLinkAndroidGamepad
{
public:
	//! Enumerates the raw button codes in Android for DreamPicoPort
	enum class ButtonCode : s32
	{
		A = AKEYCODE_BUTTON_A,
		B = AKEYCODE_BUTTON_B,
		C = AKEYCODE_BUTTON_C,
		X = AKEYCODE_BUTTON_X,
		Y = AKEYCODE_BUTTON_Y,
		Z = AKEYCODE_BUTTON_Z,
		RIGHT_B = AKEYCODE_BUTTON_L1,
		LEFT_B = AKEYCODE_BUTTON_R1,
		DOWN_B = AKEYCODE_BUTTON_L2,
		UP_B = AKEYCODE_BUTTON_R2,
		D = AKEYCODE_BUTTON_SELECT,
		START = AKEYCODE_BUTTON_START,
		VMU1_BUTTON_A = AKEYCODE_BUTTON_MODE,
		// The following come through as scan codes
		VMU1_BUTTON_B = -319,
		VMU1_BUTTON_UP = -704,
		VMU1_BUTTON_DOWN = -705,
		VMU1_BUTTON_LEFT = -706,
		VMU1_BUTTON_RIGHT = -707,
		CHANGE_EVENT = -708,
		ALT_UP = -712,
		ALT_DOWN = -713,
		ALT_LEFT = -714,
		ALT_RIGHT = -715
	};

	//! Enumerates the raw axis codes in Android for DreamPicoPort
	enum class AxisCode : s32
	{
		LEFT_X = AMOTION_EVENT_AXIS_X,
		LEFT_Y = AMOTION_EVENT_AXIS_Y,
		RIGHT_X = AMOTION_EVENT_AXIS_RX,
		RIGHT_Y = AMOTION_EVENT_AXIS_RY,
		LEFT_TRIGGER = AMOTION_EVENT_AXIS_Z,
		RIGHT_TRIGGER = AMOTION_EVENT_AXIS_RZ
	};

public:
	AndroidDreamPicoPortGamepad(
		JNIEnv *env,
		jobject usbManager,
		int maple_port,
		const AndroidGamepadDevice::AndroidJoystickData& joystickData
	);
	~AndroidDreamPicoPortGamepad();

	const char *get_button_name(u32 code) override;
	const char *get_axis_name(u32 code) override;
	void close(JNIEnv *env) override;
	void permissionGranted(JNIEnv *env, jobject usbManager) override;

	static bool identify(int vendorId, int productId);

	inline const std::string& sort_id() override
	{
		return !_sort_id.empty() ? _sort_id : DreamLinkAndroidGamepad::sort_id();
	}

	bool gamepad_btn_input(u32 code, bool pressed) override;

protected:
	void setCustomMapping(const std::shared_ptr<InputMapping>& mapping) override;

private:
	void updateNames();

private:
	std::shared_ptr<class AndroidDreamPicoPort> dpp;
	const std::string android_name;
	//! ID used for sorting on the UI
	std::string _sort_id;
};