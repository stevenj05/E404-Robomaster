#pragma once
#include "tap/motor/dji_motor.hpp"
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/communication/serial/remote.hpp"

class Flywheels {
public:
    Flywheels(tap::motor::DjiMotor& f1,
              tap::motor::DjiMotor& f2,
              tap::communication::serial::Remote& remote);

    void initialize();
    void update();
    void tick();

private:
    tap::motor::DjiMotor& flywheel1;
    tap::motor::DjiMotor& flywheel2;
    tap::algorithms::SmoothPid pid1, pid2;
    tap::communication::serial::Remote& remote;

    float desiredRPM{0.0f};
};
