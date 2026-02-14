#include "gimbal.hpp"

Gimbal::Gimbal(src::Drivers *drivers)
    : drivers(drivers),
      // Initialize motors on CAN1 - MOTOR5=Yaw, MOTOR8=Pitch (both INVERTED)
      yawMotor(new tap::motor::DjiMotor(drivers, tap::motor::MOTOR5, CAN_BUS, true, "Yaw Motor")),
      pitchMotor(new tap::motor::DjiMotor(drivers, tap::motor::MOTOR8, CAN_BUS, true, "Pitch Motor")),
      // Initialize PID controllers matching the working code
      pidYaw(new tap::algorithms::SmoothPid(tap::algorithms::SmoothPidConfig{70, 0, 0.2, 0, 13300, 1, 0, 1, 0})),
      pidPitch(new tap::algorithms::SmoothPid(tap::algorithms::SmoothPidConfig{100, 0, 0.5, 0, 15000, 1, 0, 1, 0}))
{
}

void Gimbal::initialize()
{
    yawMotor->initialize();
    pitchMotor->initialize();
    
    // Capture initial yaw encoder for turret-centric drive calibration
    initialYawEncoder = yawMotor->getEncoderUnwrapped();
}

void Gimbal::update()
{
    // Get joystick input from DR16 remote
    float yawInput = drivers->remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_HORIZONTAL);
    float pitchInput = drivers->remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_VERTICAL);
    
    // Increment counter for PID activation timing
    updateCounter++;
    
    // Check if PID should be activated (after 3 second delay)
    if (!pidYawActive && updateCounter >= PID_ACTIVATION_DELAY)
    {
        pidYawActive = true;
        gimbalYawTargetPos = yawMotor->getEncoderUnwrapped();
    }
    if (!pidPitchActive && updateCounter >= PID_ACTIVATION_DELAY)
    {
        pidPitchActive = true;
        gimbalPitchTargetPos = pitchMotor->getEncoderUnwrapped();
    }
    
    // === YAW CONTROL (MOTOR5) ===
    if (!pidYawActive)
    {
        // During startup (first 3 seconds): direct output control, no jerk
        yawMotor->setDesiredOutput(static_cast<int32_t>(-yawInput * 13300));
    }
    else
    {
        // After 3 seconds: activate PID for position holding
        // Update target position based on joystick input
        gimbalYawTargetPos += -yawInput * 12.0f;  // Faster accumulation for quicker yaw response
        
        // Update PID controller with error derivative computation
        float yawError = gimbalYawTargetPos - yawMotor->getEncoderUnwrapped();
        pidYaw->runControllerDerivateError(yawError, 0.002f);  // 0.002s = 500Hz update rate
        
        // Apply PID output
        yawMotor->setDesiredOutput(static_cast<int32_t>(pidYaw->getOutput()));
    }
    
    // === PITCH CONTROL (MOTOR8) ===
    if (!pidPitchActive)
    {
        // During startup (first 3 seconds): direct output control, no jerk
        pitchMotor->setDesiredOutput(static_cast<int32_t>(-pitchInput * 13300));
    }
    else
    {
        // After 3 seconds: activate PID for position holding
        // Update target position based on joystick input
        gimbalPitchTargetPos += -pitchInput * 3.0f;  // Inverted for intuitive up = up
        
        // Update PID controller with error derivative computation
        float pitchError = gimbalPitchTargetPos - pitchMotor->getEncoderUnwrapped();
        pidPitch->runControllerDerivateError(pitchError, 0.002f);  // 0.002s = 500Hz update rate
        
        // Apply PID output
        pitchMotor->setDesiredOutput(static_cast<int32_t>(pidPitch->getOutput()));
    }
}

void Gimbal::sendMotorCommands()
{
    drivers->djiMotorTxHandler.encodeAndSendCanData();
}

float Gimbal::getPitchEncoderPosition() const
{
    return pitchMotor->getEncoderUnwrapped();
}

float Gimbal::getYawAngleDegrees() const
{
    float encoderDiff = yawMotor->getEncoderUnwrapped() - initialYawEncoder;
    // Account for 2:1 gear ratio: motor spins 2x per turret rotation
    return (encoderDiff / ENCODER_COUNTS_PER_DEGREE) / YAW_GEAR_RATIO;
}


