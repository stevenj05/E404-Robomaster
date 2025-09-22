#include "Flywheels.hpp"
#include "../Constants.hpp"

Flywheels::Flywheels(tap::motor::DjiMotor& f1,
                     tap::motor::DjiMotor& f2,
                     tap::communication::serial::Remote& remoteIn)
: flywheel1(f1), flywheel2(f2),
  pid1(Constants::flywheel1Pid), pid2(Constants::flywheel2Pid),
  remote(remoteIn) {}

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
}

void Flywheels::tick() {
    pid1.runControllerDerivateError(desiredRPM - flywheel1.getShaftRPM(), 1);
    pid2.runControllerDerivateError(desiredRPM - flywheel2.getShaftRPM(), 1);

    flywheel1.setDesiredOutput(static_cast<int32_t>(pid1.getOutput()));
    flywheel2.setDesiredOutput(static_cast<int32_t>(pid2.getOutput()));
}