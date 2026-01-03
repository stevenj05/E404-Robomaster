#ifndef GIMBAL_SUBSYSTEM_HPP_
#define GIMBAL_SUBSYSTEM_HPP_

#include "tap/control/subsystem.hpp"
#include "tap/util_macros.hpp"
#include "modm/math/filter/pid.hpp"
#include "modm/math/geometry/angle.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/util_macros.hpp"
#include "standard/standard_constants.hpp"
#include "drivers.hpp"

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
    #include "tap/mock/dji_motor_mock.hpp"
#else
    #include "tap/motor/dji_motor.hpp"
#endif

namespace gimbal
{

    // Gimbal configuration struct for holding Smooth PID configurations
    struct GimbalConfig
    {
        tap::algorithms::SmoothPidConfig SmoothpidConfigPitch;
        tap::algorithms::SmoothPidConfig SmoothpidConfigYaw;
    };

    class GimbalSubsystem : public tap::control::Subsystem
    {
        public:

            // Gimbal Subsystem constructor with default parameters for the 
            // private variables of this class
            GimbalSubsystem(Drivers *drivers, const GimbalConfig& config);

            // Prevent all Gimbal Subsystem objects from being directly copied
            // to ensure only one Gimbal Subsystem can reference the required
            // hardware.
            GimbalSubsystem(const GimbalSubsystem &other) = delete;

            GimbalSubsystem &operator=(const GimbalSubsystem &other) = delete;

            // Default C++ deconstructor for cleaning up the Object reference once it's done
            // being used
            ~GimbalSubsystem() = default;

            // Initializing function, called once the Subsystem is registered in the CommandScheduler
            // to initialize motor values.
            void initialize() override;

            // "Periodic" function, called continously as long as the robot is running.
            // Updates speeds and angles of the yaw and pitch motors.
            void refresh() override;

            // Setter functions for changing target yaw and pitch angles
            inline void setTargetYawAngle(float yaw)
            {
                targetYawAngle = yaw;
            }

            inline void setTargetPitchAngle(float pitch)
            {
                targetPitchAngle = pitch;
            }

            // Processes remote input from the right joystick for gimbal movement, yaw and pitch inputs
            // are limited to [-1, 1]
            void remoteInput(float yawInput, float pitchInput);

            // Idea from NYU. Converts encoder angles to radians.
            static inline float GimbalSubsystem::encoderToRadians(int64_t encoderVal) 
            {
                return (M_TWOPI * static_cast<float>(encoderVal)) / tap::motor::DjiMotor::ENC_RESOLUTION;
            }

        private:

            // Pointer to the robot Drivers object
            Drivers* drivers;

            // DJI motor objects for the gimbal's yaw and pitch
            tap::motor::DjiMotor yawMotor;
            tap::motor::DjiMotor pitchMotor;

            // Smooth PIDs for adjusting gimbal motor outputs
            tap::algorithms::SmoothPid yawMotorPid;
            tap::algorithms::SmoothPid pitchMotorPid;
            
            // Starting pitch and yaw of the gimbal in radians, taken from the motor's encoder
            // on startup to avoid having the gimbal jerk to the zero position
            float startYaw;
            float startPitch;

            // Desired yaw and pitch angles of the gimbal in radians, taken from the motor encoders
            float targetYawAngle;
            float targetPitchAngle;

            // Current yaw and pitch angles of the gimbal in radians
            float currentYawAngle;
            float currentPitchAngle;

            // Current (target) yaw and pitch velocities in revolutions per minute (RPM)
            float currentYawSpeed;
            float currentPitchSpeed;

    }; // Gimbal Subsystem Class
}

#endif // GIMBAL_SUBSYSTEM_HPP