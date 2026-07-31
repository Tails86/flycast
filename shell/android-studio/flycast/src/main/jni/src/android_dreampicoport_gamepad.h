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
	AndroidDreamPicoPortGamepad(
		JNIEnv *env,
		int maple_port,
		int id,
		const char *name,
		const char *unique_id,
		const std::vector<int>& fullAxes,
		const std::vector<int>& halfAxes,
		jobject usbManager
	);
	~AndroidDreamPicoPortGamepad();

	const char *get_button_name(u32 code) override;
	void close(JNIEnv *env) override;
	void permissionGranted(JNIEnv *env, jobject usbManager) override;

	static bool identify(int vendorId, int productId);

	bool gamepad_btn_input(u32 code, bool pressed) override;

protected:
	void setCustomMapping(const std::shared_ptr<InputMapping>& mapping) override;

private:
	void updateNames();

private:
	std::shared_ptr<class AndroidDreamPicoPort> dpp;
	const std::string android_name;
};