#include "gimbal_subsystem.hpp"
#include "drivers.hpp"
#include "standard/standard_constants.hpp"
#include "tap/algorithms/math_user_utils.hpp"

using tap::algorithms::limitVal;

namespace gimbal
{
    // Gimbal Subsystem constructor
    GimbalSubsystem::GimbalSubsystem(Drivers* drivers, const GimbalConfig& config) : Subsystem(drivers), drivers(drivers), 
    yawMotor(drivers, YAWMOTOR, CAN_TURRET, false, "Yaw motor"), 
    pitchMotor(drivers, PITCHMOTOR, CAN_TURRET, false, "Pitch motor"),
    yawMotorPid(config.SmoothpidConfigYaw),
    pitchMotorPid(config.SmoothpidConfigPitch),
    currentYawAngle(0.0f),
    currentPitchAngle(0.0f),
    currentYawSpeed(0.0f),
    currentPitchSpeed(0.0f)
    {}

    // Initializing function, called once the Subsystem is registered in the CommandScheduler
    // to initialize motor values.
    void GimbalSubsystem::initialize()
    {
        // Initialize the Gimbal Subsystem's yaw and pitch motors and set their outputs to 0 on startup.
        // Sets pitch and yaw of the gimbal to the current pitch and yaw of the gimbal at this moment.
        yawMotor.initialize();
        yawMotor.setDesiredOutput(0);
        pitchMotor.initialize();
        pitchMotor.setDesiredOutput(0);
        startYaw = drivers->mpu6500.getYaw();
        startPitch = drivers->mpu6500.getPitch();
        targetYawAngle = startYaw;
        targetPitchAngle = startPitch;
        currentYawAngle = startYaw;
        currentPitchAngle = startPitch;
    }

    // "Periodic" function, called continously as long as the robot is running. Updates speeds
    // and angles of the yaw and pitch motors.
    void GimbalSubsystem::refresh()
    {
        // Update the current yaw and pitch angles based on the motor encoder values
        currentYawAngle = encoderToRadians(yawMotor.getEncoderUnwrapped());
        currentPitchAngle = encoderToRadians(pitchMotor.getEncoderUnwrapped());

        // Update the yaw and pitch motor speeds
        currentYawSpeed = yawMotor.getShaftRPM();
        currentPitchSpeed = pitchMotor.getShaftRPM();

        // If no right joystick input is given, set the target yaw and pitch to the current yaw and pitch.
        if(drivers->controlOperatorInterface.getGimbalTankXInput() == 0 && drivers->controlOperatorInterface.getGimbalTankYInput() == 0)
        {
            targetYawAngle = currentYawAngle;
            targetPitchAngle = currentPitchAngle;
        }
        // Otherwise, use the PID controllers to move pitch and yaw towards the target angles
        else
        {
            float dt = modm::PreciseClock::now().time_since_epoch().count();
            yawMotorPid.runControllerDerivateError(targetYawAngle - currentYawAngle, dt);
            yawMotor.setDesiredOutput(limitVal<float>(yawMotorPid.getOutput(), -MAX_GIMBAL_RPM, MAX_GIMBAL_RPM));
            pitchMotorPid.runControllerDerivateError(targetPitchAngle - currentPitchAngle, dt);
            pitchMotor.setDesiredOutput(limitVal<float>(pitchMotorPid.getOutput(), -MAX_GIMBAL_RPM, MAX_GIMBAL_RPM));
        }
    }

    // Processes remote input from the right joystick for gimbal movement, yaw and pitch inputs
    // are limited to [-1, 1]
    void GimbalSubsystem::remoteInput(float yawInput, float pitchInput)
    {
        setTargetYawAngle(currentYawAngle + yawInput * GIMBAL_RPM_SCALE);
        setTargetPitchAngle(currentPitchAngle + pitchInput * GIMBAL_RPM_SCALE);
    }
}