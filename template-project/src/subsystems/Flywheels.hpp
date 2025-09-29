#pragma once
#include "tap/motor/dji_motor.hpp"
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/communication/serial/remote.hpp"
#include "../Constants.hpp"

using namespace Constants;

class Flywheels {
public:
    Flywheels(tap::communication::serial::Remote& remote);

    void initialize();
    void update();
    void tick();

private:
    tap::motor::DjiMotor flywheel1{drivers, Constants::M_ID2, Constants::CAN_BUS1, false, "flywheel1"};
    tap::motor::DjiMotor flywheel2{drivers, Constants::M_ID1, Constants::CAN_BUS1, false, "flywheel2"};

    tap::algorithms::SmoothPid pid1{Constants::flywheelPidConfig1};
    tap::algorithms::SmoothPid pid2{Constants::flywheelPidConfig2};

    tap::communication::serial::Remote& remote;

    float desiredRPM{0.0f};
};
