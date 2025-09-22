#pragma once
#include "tap/motor/dji_motor.hpp"
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/communication/serial/remote.hpp"

class Gimbal {
public:
    Gimbal(tap::motor::DjiMotor& pitchMotor,
           tap::communication::serial::Remote& remote);

    void initialize();
    void update();
    void tick();

private:
    tap::motor::DjiMotor& motorPitch;
    tap::algorithms::SmoothPid pidPitch;
    tap::communication::serial::Remote& remote;

    int32_t targetPos{0};
};
