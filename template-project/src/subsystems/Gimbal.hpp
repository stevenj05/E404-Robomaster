#pragma once

#include "../drivers_singleton.hpp"
#include "../Constants.hpp"

using namespace Constants;

class Gimbal {
public:
    Gimbal(src::Drivers* _drivers, tap::communication::serial::Remote& remote, double& _yaw, double& _pitch);

    void initialize();
    void update();
    void tick(float scale = 1.0f);

private:
    src::Drivers* drivers;
   
    std::optional<tap::motor::DjiMotor> motorPitch, motorYaw;

    tap::algorithms::SmoothPid pidPitch{gimbal_pid_pitch};
    tap::algorithms::SmoothPid pidYaw{gimbal_pid_yaw};

    tap::communication::serial::Remote& remote;
    double& yaw;
    double& pitch;

    int32_t targetPitch{0};
    int32_t targetYaw{0};
};
