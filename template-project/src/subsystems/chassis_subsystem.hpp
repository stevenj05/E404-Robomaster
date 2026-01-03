#ifndef CHASSIS_SUBSYSTEM_HPP_
#define CHASSIS_SUBSYSTEM_HPP_

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

namespace chassis
{
    // Chassis configuration struct for holding Motor IDs and PID configurations
    struct ChassisConfig
    {
        tap::motor::MotorId frontLeftId;
        tap::motor::MotorId frontRightId;
        tap::motor::MotorId backLeftId;
        tap::motor::MotorId backRightId;

        tap::algorithms::SmoothPidConfig SmoothpidConfigFL;
        tap::algorithms::SmoothPidConfig SmoothpidConfigFR;
        tap::algorithms::SmoothPidConfig SmoothpidConfigBL;
        tap::algorithms::SmoothPidConfig SmoothpidConfigBR;
    };

    class ChassisSubsystem : public tap::control::Subsystem
    {
        public:

            // Drive train motor max speed
            static constexpr float MAX_DRIVETRAIN_RPM = DTRAIN_MAX_RPM;

            // Chassis Subsystem constructor with default parameters for the 
            // private variables of this class
            ChassisSubsystem(Drivers *drivers, const ChassisConfig& config);

            // Prevent all Chassis Subsystem objects from being directly copied
            // to ensure only one Chassis Subsystem can reference the required
            // hardware. This prevents codes like
            //
            // ChassisSubsystem a;
            // ChassisSubsystem b = a;
            //
            // from running.
            ChassisSubsystem(const ChassisSubsystem &other) = delete;

            ChassisSubsystem &operator=(const ChassisSubsystem &other) = delete;

            // Default C++ deconstructor for cleaning up the Object reference once it's done
            // being used
            ~ChassisSubsystem() = default;

            // Initializing function, called once the Subsystem is registered in the CommandScheduler
            // to initialize motor values.
            void initialize() override;

            // "Periodic" function, called continously as long as the robot is running.
            // Updates the PID calculator with desired outputs.
            void refresh() override;

            // Updates a PID with a desired RPM value.
            void ChassisSubsystem::updatePID(tap::algorithms::SmoothPid* pidCont, tap::motor::DjiMotor* const dTrainMotor, float targetRPM, float dt);

            // Sets the Chassis motor outputs based on horizontal/vertical linear velocities defined by the remote's left joystick and
            // the turning input (usually from beyblading). Implements translational motion driving.
            void ChassisSubsystem::setMotorRPMs(float x, float y, float rot);

            // Returns the current driving mode of the chassis
            DriveMode getDriveMode()
            {
                return dMode;
            }

        
        private:

            // Converts robot linear speed in meters per second to a desired rotations per minute value (RPM) for each of the 
            // Drivetrain DJI motor wheels.
            inline float mpsToRpm(float mps)
            {
                static constexpr float GEAR_RATIO = 19.0f;
                static constexpr float WHEEL_DIAMETER_M = WHEEL_RADIUS * 2;
                static constexpr float WHEEL_CIRCUMFERANCE_M = M_PI * WHEEL_DIAMETER_M;
                static constexpr float SEC_PER_M = 60.0f;

                return (mps / WHEEL_CIRCUMFERANCE_M) * SEC_PER_M * GEAR_RATIO;
            }

            // DJI motor objects for each of the four drivetrain motors
            tap::motor::DjiMotor FLMotor;
            tap::motor::DjiMotor FRMotor;
            tap::motor::DjiMotor BLMotor;
            tap::motor::DjiMotor BRMotor;

            // Smooth PID velocity controller objects for each of the drivetrain motors 
            tap::algorithms::SmoothPid pidControllerFL;
            tap::algorithms::SmoothPid pidControllerFR;
            tap::algorithms::SmoothPid pidControllerBR;
            tap::algorithms::SmoothPid pidControllerBL;

            // Target RPMs for each of the drivetrain motors
            float FLTargetRPM;
            float FRTargetRPM;
            float BLTargetRPM;
            float BRTargetRPM;

            // Current driving mode of the chassis (translational by default)
            DriveMode dMode = TRANS;

    }; // Chassis Subsystem Class
}

#endif // CHASSIS_SUBSYSTEM_HPP_