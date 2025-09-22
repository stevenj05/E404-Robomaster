#include "Gimbal.hpp"
#include "../Constants.hpp"

Gimbal::Gimbal(tap::motor::DjiMotor& pitchMotor,
               tap::communication::serial::Remote& remoteIn)
: motorPitch(pitchMotor), pidPitch(Constants::pidController5), remote(remoteIn) {}

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
}

void Gimbal::tick() {
    pidPitch.runControllerDerivateError(targetPos - motorPitch.getEncoderUnwrapped(), 1);
    motorPitch.setDesiredOutput(static_cast<int32_t>(pidPitch.getOutput()));
}