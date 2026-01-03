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

#include "control_operator_interface.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/communication/serial/remote.hpp"

using tap::algorithms::limitVal;
using tap::communication::serial::Remote;
using tap::communication::serial::Drivers;

namespace control
{
    // Constructor that initializes the remote reference
    ControlOperatorInterface::ControlOperatorInterface(Remote &remote) : remote(remote) {}

    // Call remote's getChannel function, left horizontal switch gets user input for horizontal tank drive
    mockable float ControlOperatorInterface::getChassisTankXInput()
    {
        return limitVal<float>(remote.getChannel(Remote::Channel::LEFT_HORIZONTAL), -1.0f, 1.0f);
    }

    // Call remote's getChannel function, left vertical switch gets user input for forward/backward tank drive
    mockable float ControlOperatorInterface::getChassisTankYInput()
    {
        return limitVal<float>(remote.getChannel(Remote::Channel::LEFT_VERTICAL), -1.0f, 1.0f);
    }

    // Call remote's getChannel function, right horizontal switch gets user input for yaw control
    mockable float ControlOperatorInterface::getGimbalTankXInput()
    {
        return limitVal<float>(remote.getChannel(Remote::Channel::RIGHT_HORIZONTAL), -1.0f, 1.0f);
    }
    
    // Call remote's getChannel function, right vertical switch gets user input for pitch control
    mockable float ControlOperatorInterface::getGimbalTankYInput()
    {
        return limitVal<float>(remote.getChannel(Remote::Channel::RIGHT_VERTICAL), -1.0f, 1.0f);
    }

    // Call remote's getChannel function for the flywheel activation. Returns 
    mockable Remote::SwitchState ControlOperatorInterface::getFlyWheelInput()
    {
        return remote.getSwitch(Remote::Switch::LEFT_SWITCH);
    }

}  // namespace control
