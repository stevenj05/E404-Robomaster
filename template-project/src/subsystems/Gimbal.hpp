#pragma once
#include <sys/types.h>
#include "tap/communication/serial/remote.hpp"
#include "../Constants.hpp"

using namespace Constants;

class Gimbal {
public:
    Gimbal(tap::communication::serial::Remote& remote, double& _yaw, double& _pitch);

    void initialize();
    void update();
    void tick(float scale = 1.0f);

private:
    tap::motor::DjiMotor motorPitch{drivers, Constants::M_ID6, Constants::CAN_BUS1, true, "motorPitch"};
    tap::motor::DjiMotor motorYaw{drivers, Constants::M_ID7, Constants::CAN_BUS1, true, "motorYaw"};

    tap::algorithms::SmoothPid pidPitch{Constants::pidConfig5};
    tap::algorithms::SmoothPid pidYaw{Constants::pidConfig6};

    tap::communication::serial::Remote& remote;
    double yaw, pitch;

    int32_t targetPitch{0};
    int32_t targetYaw{0};
};
