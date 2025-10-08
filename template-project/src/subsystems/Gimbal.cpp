#include "Gimbal.hpp"

Gimbal::Gimbal(tap::communication::serial::Remote& remoteIn, double& _yaw, double& _pitch)
: remote(remoteIn), yaw(_yaw), pitch(_pitch) {}

void Gimbal::initialize() {
    motorPitch.initialize();
    motorPitch.resetEncoderValue();
    targetPitch = 0;
}

void Gimbal::update() {
    int32_t pitchInput = remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_VERTICAL);
    if (std::abs(pitchInput) > 0) {
        targetPitch += pitchInput * 10; // sensitivity multiplier
    }

    pidPitch.runControllerDerivateError(targetPitch - motorPitch.getEncoderUnwrapped(), 1);
}

void Gimbal::tick(float scale) {
    motorPitch.setDesiredOutput(static_cast<int32_t>(pidPitch.getOutput() * scale));
}