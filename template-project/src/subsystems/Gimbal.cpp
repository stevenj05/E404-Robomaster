#include "Gimbal.hpp"

Gimbal::Gimbal(tap::communication::serial::Remote& remoteIn, double& _yaw, double& _pitch)
: remote(remoteIn), yaw(_yaw), pitch(_pitch) {}

void Gimbal::initialize() {
    motorPitch.initialize();
    motorPitch.resetEncoderValue();
    MotorYaw.initialize();
    MotorYaw.resetEncoderValue();
    targetPitch = 0;
}
    //Gets the vertical component of the right tick
    //Applies it to pitch
void Gimbal::update() {
    int32_t pitchInput = remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_VERTICAL);
    if (std::abs(pitchInput) > 0) {
        targetPitch += pitchInput * 10; // sensitivity multiplier
        
    }
//this returns a runController inside it 
        //runController runs the pid controller
//runControllerDerivativeError
//accounts for error between desired value and actual real output
    pidPitch.runControllerDerivateError(targetPitch - motorPitch.getEncoderUnwrapped(), 1);
}

//sets value to motor
void Gimbal::tick(float scale) {
    motorPitch.setDesiredOutput(static_cast<int32_t>(pidPitch.getOutput() * scale));
}