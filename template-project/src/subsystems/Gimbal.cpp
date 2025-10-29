#include "Gimbal.hpp"

Gimbal::Gimbal(src::Drivers*& _drivers, tap::communication::serial::Remote& remoteIn, double& _yaw, double& _pitch)
: drivers(_drivers), remote(remoteIn), yaw(_yaw), pitch(_pitch) {}

void Gimbal::initialize() {
    // Initialize the gimbal motor, setting its pitch, yaw, and the corresponding measures'
    // encoding angles to 0.
    motorPitch.initialize();
    motorPitch.resetEncoderValue();
    motorYaw.initialize();
    motorYaw.resetEncoderValue();

    // Initialize target pitch and yaw to 0
    targetPitch = 0;
    targetYaw = 0;
}
    //Gets the vertical component of the right stick, applies it to pitch
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
    pidYaw.runControllerDerivateError(targetYaw - motorYaw.getEncoderUnwrapped(), 1);
}

//this returns a runController inside it 
        //runController runs the pid controller
//runControllerDerivativeError
//accounts for error between desired value and actual real output

//sets value to motor
void Gimbal::tick(float scale) {
    motorPitch.setDesiredOutput(static_cast<int32_t>(pidPitch.getOutput() * scale));
    motorYaw.setDesiredOutput(static_cast<int32_t>(pidYaw.getOutput() * scale));
}