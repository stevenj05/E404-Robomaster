#include "chassis_movement_command.hpp"
#include "standard/standard_constants.hpp"
#include "tap/algorithms/math_user_utils.hpp"

namespace chassis
{
    // Chassis Movement Command constructor
    ChassisMovementCommand::ChassisMovementCommand(ChassisSubsystem *const chassis, Drivers *drivers) : chassis(chassis), drivers(drivers)
    {
        if (chassis == nullptr)
        {
            throw std::invalid_argument("ChassisMovementCommand: chassis subsystem pointer is null");
            return;
        }

        // Set the Drivetrain Subsystem as a requirement for this command
        this->addSubsystemRequirement(dynamic_cast<tap::control::Subsystem *>(chassis));
    }

    // Code that runs once, when the Command is first given to the CommandScheduler.
    // Sets all DriveTrain motor speeds to 0 (stops movement)
    void ChassisMovementCommand::initialize()
    {
        chassis->setMotorRPMs(0, 0, 0);
    }

    // ===== WORK IN PROGRESS =====
    // Must have a working Gimbal interface to rotate the vector input, needs different driving modes
    void ChassisMovementCommand::execute()
    {
        /*
        if (chassis->getDriveMode() == TRANS)
        {
            drivers->controlOperatorInterface.getChassisTankXInput();
        }
        */
       float xInput = drivers->controlOperatorInterface.getChassisTankXInput();
       float yInput = drivers->controlOperatorInterface.getChassisTankYInput();

       // Get the yaw angle from the gimbal and perform a rotation on the x-y inputs
       // using a 2D rotation "matrix".
       float yaw = drivers->mpu6500.getYaw();
       float xVel = ((cosf(yaw) * xInput) - (sinf(yaw) * yInput));
       float yVel = ((cosf(yaw) * yInput) + (sinf(yaw) * yInput));

       // ===== FIX THIS. Needs to account for rotational input.
       // Set the motor RPMs to the calculated values after rotation
       chassis->setMotorRPMs(xVel, yVel, 0);
    }

    // Code that runs once the Command has finished and is descheduled.
    // Stops all Drivetrain motors.
    void  ChassisMovementCommand::end(bool) 
    { 
        chassis->setMotorRPMs(0, 0, 0); 
    }

    // Returns a boolean indicating when the Command has finished running. Default value is false.
    bool ChassisMovementCommand::isFinished() const 
    { 
        return false; 
    }

}