#include "Flywheels.hpp"

Flywheels::Flywheels(tap::communication::serial::Remote& remoteIn)
:  remote(remoteIn) {}

void Flywheels::initialize() {
    flywheel1.initialize();
    flywheel2.initialize();
}

void Flywheels::update() {
    auto state = remote.getSwitch(tap::communication::serial::Remote::Switch::LEFT_SWITCH);

    if (state == tap::communication::serial::Remote::SwitchState::UP) {
        desiredRPM = 4000; // full speed
    } else if (state == tap::communication::serial::Remote::SwitchState::MID) {
        desiredRPM = 0;
    } else if (state == tap::communication::serial::Remote::SwitchState::DOWN) {
        desiredRPM = -4000; // reverse
    }


    pid1.runControllerDerivateError(desiredRPM - flywheel1.getShaftRPM(), 1);
    pid2.runControllerDerivateError(desiredRPM - flywheel2.getShaftRPM(), 1);
}

void Flywheels::tick(float scale) {
    flywheel1.setDesiredOutput(static_cast<int32_t>(pid1.getOutput()*scale));
    flywheel2.setDesiredOutput(static_cast<int32_t>(pid2.getOutput()*scale));
}