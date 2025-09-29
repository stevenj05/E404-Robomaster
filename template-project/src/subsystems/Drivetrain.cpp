#include "Drivetrain.hpp"

Drivetrain::Drivetrain(tap::communication::serial::Remote& remoteIn)
: remote(remoteIn) {}

void Drivetrain::initialize() {
    motorFL.initialize();
    motorFR.initialize();
    motorBL.initialize();
    motorBR.initialize();
    lastToggleTime = modm::chrono::milli_clock::now();
}

void Drivetrain::update() {
    fwdInput    = remote.getChannel(tap::communication::serial::Remote::Channel::LEFT_VERTICAL);
    strafeInput = remote.getChannel(tap::communication::serial::Remote::Channel::LEFT_HORIZONTAL);
    turnInput   = remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_HORIZONTAL);

    auto now = modm::chrono::milli_clock::now();
    bool switchDown = (remote.getSwitch(tap::communication::serial::Remote::Switch::RIGHT_SWITCH)
                       == tap::communication::serial::Remote::SwitchState::DOWN);

    if (switchDown && (now - lastToggleTime).count() > 250) {
        beybladeMode = !beybladeMode;
        lastToggleTime = now;
    }

    pidFL.runControllerDerivateError(((fwdInput + strafeInput + turnInput) * 4000) - motorFL.getShaftRPM(), 1);
    pidFR.runControllerDerivateError(((fwdInput - strafeInput - turnInput) * 4000) - motorFR.getShaftRPM(), 1);
    pidBL.runControllerDerivateError(((-fwdInput - strafeInput + turnInput) * 4000) - motorBL.getShaftRPM(), 1);
    pidBR.runControllerDerivateError(((-fwdInput + strafeInput - turnInput) * 4000) - motorBR.getShaftRPM(), 1);
}

void Drivetrain::tick() {
    if (beybladeMode) {
        motorFL.setDesiredOutput(12000);
        motorFR.setDesiredOutput(-12000);
        motorBL.setDesiredOutput(12000);
        motorBR.setDesiredOutput(-12000);
    } else {
        motorFL.setDesiredOutput(static_cast<int32_t>(pidFL.getOutput()));
        motorFR.setDesiredOutput(static_cast<int32_t>(pidFR.getOutput()));
        motorBL.setDesiredOutput(static_cast<int32_t>(pidBL.getOutput()));
        motorBR.setDesiredOutput(static_cast<int32_t>(pidBR.getOutput()));
    }
}