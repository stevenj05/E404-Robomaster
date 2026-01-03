#ifndef CHASSIS_MOVEMENT_COMMAND_HPP_
#define CHASSIS_MOVEMENT_COMMAND_HPP_

#include "tap/control/command.hpp"
#include "chassis_subsystem.hpp"
#include "drivers.hpp"

namespace chassis
{
    class ChassisMovementCommand : public tap::control::Command
    {
        public:

            // Drivetrain Subsystem movement command constructor. MUST have a pointer to an existing
            // ChassisSubsystem object in order to set the drivetrain as its required subsystem.
            ChassisMovementCommand(ChassisSubsystem *const chassis, Drivers *drivers);

            // Prevent the movement command object from being copied and used in more than one place
            // due to assignment to another variable
            ChassisMovementCommand(const ChassisMovementCommand &other) = delete;
            ChassisMovementCommand &operator=(const ChassisMovementCommand &other) = delete;

            // Code that runs once, when the Command is first given to the CommandScheduler.
            // Sets all DriveTrain motor speeds to 0
            void initialize() override;

            // Code that runs continuously while the Command is scheduled in the CommandScheduler.
            // Takes input from the controller (remote or keyboard) and sends it to the Chassis
            // Subsystem to move the robot
            void execute() override;

            // Code that runs once the Command has finished and is descheduled.
            // Stops all Drivetrain motors.
            void end(bool interrupted) override;

            // Returns a boolean indicating when the Command has finished running
            bool isFinished() const override;

        private:

            // Pointer to the Drivetrain Subsystem for setting it as a requirement of this Command
            ChassisSubsystem *const chassis;

            // Drivers pointer
            Drivers *drivers;
    }; // ChassisMovementCommand
} //namespace Chassis
#endif //CHASSIS_MOVEMENT_COMMAND_HPP_ 