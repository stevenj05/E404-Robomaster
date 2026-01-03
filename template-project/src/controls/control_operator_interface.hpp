/*
 * Copyright (c) 2020-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruw-edu.
 *
 * aruw-edu is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruw-edu is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruw-edu.  If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include "tap/util_macros.hpp"

namespace tap::communication::serial
{
    class Remote;
    class Drivers;
}

namespace control
{
    // Control Operator Interface: Accesses inputs provided from user remote
    class ControlOperatorInterface
    {
        public:
            ControlOperatorInterface(tap::communication::serial::Remote &remote);

            // Tank Drive functions. Return values between -1 and 1, inclusive, that indicate
            // the speed and direction of chassis movement denoted by the left joystick.
            mockable float getChassisTankXInput();
            mockable float getChassisTankYInput();

            // Gimbal turning functions. Return values between -1 and 1, inclusive, that indicate
            // the speed and direction of gimbal movement denoted by the right joystick.
            mockable float getGimbalTankXInput();
            mockable float getGimbalTankYInput();

            // Flywheel activation function.
            mockable tap::communication::serial::Remote::SwitchState getFlyWheelInput();

        private:

            // Pointer to the remote for taking controller input
            tap::communication::serial::Remote &remote;
    };
}  // namespace control
