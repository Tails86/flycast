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

#include "types.h"
#include "emulator.h"
#include "android_gamepad.h"
#include "input/dreamlink/gamepad_dreamlink.h"

#include <functional>
#include <memory>
#include <array>

class DreamLinkAndroidGamepad : public AndroidGamepadDevice
{
public:
	virtual ~DreamLinkAndroidGamepad();
	const char* status() override;
	void set_maple_port(int port) override;
	void registered() override;
	static bool isDreamLinkGamepad(int vid, int pid);
	static bool isPermissionRequired(int vid, int pid);
	void resetMappingToDefault(bool arcade, bool gamepad) override;
	virtual void permissionGranted(JNIEnv *env, jobject usbManager) = 0;

protected:
	DreamLinkAndroidGamepad(
		std::shared_ptr<GamepadDreamLink> dreamlink,
		int maple_port,
		int id,
		const char *name,
		const char *unique_id,
		const std::vector<int>& fullAxes,
		const std::vector<int>& halfAxes
	);
	std::shared_ptr<InputMapping> getDefaultMapping() override;
	void setBaseDefaultMapping(const std::shared_ptr<InputMapping>& mapping) const;
	virtual void setCustomMapping(const std::shared_ptr<InputMapping>& mapping) {}
	void updateDreamLink(std::shared_ptr<GamepadDreamLink> dreamlink);

	std::shared_ptr<GamepadDreamLink> dreamlink;
};

std::shared_ptr<DreamLinkAndroidGamepad> createDreamLinkAndroidGamepad(
	JNIEnv *env,
	int maple_port,
	int id,
	const char *name,
	const char *unique_id,
	const std::vector<int>& fullAxes,
	const std::vector<int>& halfAxes,
	int vid,
	int pid,
	jobject usbManager
);
