#pragma once

#include "../drivers_singleton.hpp"
#include "../Constants.hpp"

using namespace Constants;

class Flywheels {
public:
    Flywheels(src::Drivers* _drivers, tap::communication::serial::Remote& remote);

    void initialize();
    void update();
    void tick(float scale = 1.0f);

private:
    src::Drivers* drivers;

    std::optional<tap::motor::DjiMotor> flywheel1, flywheel2;

    tap::algorithms::SmoothPid pid1{flywheel_pid_1};
    tap::algorithms::SmoothPid pid2{flywheel_pid_2};

    tap::communication::serial::Remote& remote;

    float desiredRPM{0.0f};
};
