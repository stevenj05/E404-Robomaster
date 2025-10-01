#include "Gimbal.hpp"

Gimbal::Gimbal(tap::communication::serial::Remote& remoteIn)
: remote(remoteIn) {}

void Gimbal::initialize() {
    motorPitch.initialize();
    motorPitch.resetEncoderValue();
    targetPos = 0;
}

void Gimbal::update() {
    int32_t pitchInput = remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_VERTICAL);
    if (std::abs(pitchInput) > 0) {
        targetPos += pitchInput * 10; // sensitivity multiplier
    }

    pidPitch.runControllerDerivateError(targetPos - motorPitch.getEncoderUnwrapped(), 1);
}

void Gimbal::tick(float scale) {
    motorPitch.setDesiredOutput(static_cast<int32_t>(pidPitch.getOutput() * scale));
}