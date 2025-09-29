#pragma once
#include "tap/communication/serial/remote.hpp"
#include "../Constants.hpp"

using namespace Constants;

class Gimbal {
public:
    Gimbal(tap::communication::serial::Remote& remote);

    void initialize();
    void update();
    void tick();

private:
    tap::motor::DjiMotor motorPitch{drivers, Constants::M_ID6, Constants::CAN_BUS1, true, "motor6"};
    tap::motor::DjiMotor MotorYaw{drivers, Constants::M_ID7, Constants::CAN_BUS1, true, "motor7"};

    tap::algorithms::SmoothPid pidPitch{Constants::pidConfig5};
    tap::algorithms::SmoothPid pidYaw{Constants::pidConfig6};

    tap::communication::serial::Remote& remote;

    int32_t targetPos{0};
};
