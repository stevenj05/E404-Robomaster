#include "gimbal_movement_command.hpp"
#include "standard/standard_constants.hpp"
#include "tap/algorithms/math_user_utils.hpp"

namespace gimbal
{
    // Gimbal Movement Command constructor
    GimbalMovementCommand::GimbalMovementCommand(GimbalSubsystem *const gimbal, Drivers *drivers) : gimbal(gimbal), drivers(drivers)
    {
        // Check if the gimbal Subsystem pointer exists, throw an error and return immediately if not
        if(gimbal == nullptr)
        {
            throw std::invalid_argument("GimbalMovementCommand: gimbal subsystem pointer is null");
            return;
        }

        // Add the gimbal Subsystem as a requirement
        this->addSubsystemRequirement(dynamic_cast<tap::control::Subsystem *>(gimbal));
    }

    // Code that runs once, when the Command is first given to the CommandScheduler.
    // Sets Gimbal yaw and pitch angles to 0
    void GimbalMovementCommand::initialize()
    {
        gimbal->setTargetYawAngle(0);
        gimbal->setTargetPitchAngle(0);
    }

    // Code that runs continuously while the Command is scheduled in the CommandScheduler.
    // Takes input from the controller (remote or keyboard) and sends it to the Gimbal
    // Subsystem to move the gimbal/turret.
    void GimbalMovementCommand::execute()
    {
        gimbal->remoteInput(drivers->controlOperatorInterface.getGimbalTankXInput(), drivers->controlOperatorInterface.getGimbalTankYInput());
    }

    // Code that runs once the Command has finished and is descheduled.
    // Returns pitch and yaw angles back to 0 radians.
    void GimbalMovementCommand::end(bool)
    {
        gimbal->remoteInput(0, 0);
    }

    // Returns a boolean indicating when the Command has finished running. False by default (means
    // the command continues running until interrupted)
    bool GimbalMovementCommand::isFinished() const
    {
        return false;
    }
    
}