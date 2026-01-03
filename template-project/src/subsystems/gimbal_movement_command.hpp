#ifndef GIMBAL_MOVEMENT_COMMAND_HPP_
#define GIMBAL_MOVEMENT_COMMAND_HPP_

#include "tap/control/command.hpp"
#include "gimbal_subsystem.hpp"
#include "drivers.hpp"

namespace gimbal
{
    class GimbalMovementCommand : public tap::control::Command 
    {
        public:

            // Gimbal movement command constructor
            GimbalMovementCommand(GimbalSubsystem *const gimbal, Drivers *drivers);

            // Prevent the movement command object from being copied and used in more than one place
            // due to assignment to another variable
            GimbalMovementCommand(const GimbalMovementCommand &other) = delete;
            GimbalMovementCommand &operator=(const GimbalMovementCommand &other) = delete;

            // Code that runs once, when the Command is first given to the CommandScheduler.
            // Sets all Gimbal motor speeds to 0
            void initialize() override;

            // Code that runs continuously while the Command is scheduled in the CommandScheduler.
            // Takes input from the controller (remote or keyboard) and sends it to the Gimbal
            // Subsystem to move the gimbal/turret
            void execute() override;

            // Code that runs once the Command has finished and is descheduled.
            // Returns pitch and yaw angles back to 0 radians.
            void end(bool) override;

            // Returns a boolean indicating when the Command has finished running.
            bool isFinished() const override;
        
            private:

                Drivers* drivers;
                GimbalSubsystem* gimbal;
            
    }; // GimbalMovementCommand
} // namespace gimbal
#endif // GIMBAL_MOVEMENT_COMMAND_HPP_ 