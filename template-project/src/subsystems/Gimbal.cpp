#include "Gimbal.hpp"

Gimbal::Gimbal(tap::communication::serial::Remote& remoteIn, double& _yaw, double& _pitch)
: remote(remoteIn), yaw(_yaw), pitch(_pitch) {}

void Gimbal::initialize() {
    motorPitch.initialize();
    motorPitch.resetEncoderValue();
    MotorYaw.initialize();
    MotorYaw.resetEncoderValue();

    // Initialize target pitch and yaw to 0
    targetPitch = 0;
    targetYaw = 0;
}
    //Gets the vertical component of the right tick
    //Applies it to pitch
void Gimbal::update() {
    int32_t pitchInput = remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_VERTICAL);
    if (std::abs(pitchInput) > 0) {
        // Pitch control
        targetPitch += pitchInput * 10; // sensitivity multiplier
        
    }

    int32_t yawInput = remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_HORIZONTAL);
    if (std::abs(yawInput) > 0) {
        // Yaw control
        targetYaw += yawInput * 10; // yaw sensitivity multiplier
    }

    pidPitch.runControllerDerivateError(targetPitch - motorPitch.getEncoderUnwrapped(), 1);
    pidYaw.runControllerDerivateError(targetYaw - MotorYaw.getEncoderUnwrapped(), 1);
}

//this returns a runController inside it 
        //runController runs the pid controller
//runControllerDerivativeError
//accounts for error between desired value and actual real output

//sets value to motor
void Gimbal::tick(float scale) {
    motorPitch.setDesiredOutput(static_cast<int32_t>(pidPitch.getOutput() * scale));
    MotorYaw.setDesiredOutput(static_cast<int32_t>(pidPitch.getOutput() * scale));
}