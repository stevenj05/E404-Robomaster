#include "tap/algorithms/math_user_utils.hpp"
#include "drivers.hpp"
#include "chassis_subsystem.hpp"
#include "standard/standard_constants.hpp"
#include "tap/algorithms/smooth_pid.hpp"
#include "modm/architecture/interface/clock.hpp"

using tap::algorithms::limitVal;

namespace chassis
{
    // Chassis Subsytem Constructor. Needs a SmoothpidConfig struct to be made and given as an argument
    // beforehand to insert Motor IDs and PID values for each of the motors.
    ChassisSubsystem::ChassisSubsystem(Drivers *drivers, const ChassisConfig& config)
        : Subsystem(drivers), FLMotor(drivers, config.frontLeftId, CAN_CHASSIS, false, "frontLeft"), 
        FRMotor(drivers, config.frontRightId, CAN_CHASSIS, false, "frontRight"), 
        BLMotor(drivers, config.backLeftId, CAN_CHASSIS,  false, "backLeft"),
        BRMotor(drivers, config.backRightId, CAN_CHASSIS, false, "backRight"),
        pidControllerFL(config.SmoothpidConfigFL),
        pidControllerFR(config.SmoothpidConfigFR),
        pidControllerBR(config.SmoothpidConfigBR),
        pidControllerBL(config.SmoothpidConfigBL),
        FLTargetRPM(0),
        FRTargetRPM(0),
        BLTargetRPM(0),
        BRTargetRPM(0)
        {}
    
    // Initializes the Chassis's motors
    void ChassisSubsystem::initialize()
    {
        FLMotor.initialize();
        FRMotor.initialize();
        BLMotor.initialize();
        BRMotor.initialize();
    }

    // ===== WORK IN PROGRESS =====
    // Corrects and updates DJI Chassis motor outputs using the PIDs
    void ChassisSubsystem::refresh()
    {
        // NOT sure if this works, but it *should* be measuring the time since this function was
        // last called.
        float dt = modm::PreciseClock::now().time_since_epoch().count();

        // Update the PIDs of each of the motors with the desired RPMs
        updatePID(&pidControllerFL, &FLMotor, FLTargetRPM, dt);
        updatePID(&pidControllerFR, &FRMotor, FRTargetRPM, dt);
        updatePID(&pidControllerBL, &BLMotor, BLTargetRPM, dt);
        updatePID(&pidControllerBR, &BRMotor, BRTargetRPM, dt);
    }

    // Updates a PID with a desired RPM value
    // - pidCont: Pointer to a SmoothPid object
    // - dTrainMotor: Pointer to a Drivertrain DJI Motor
    // - targetRPM: Target RPM (a float)
    // - dt: Time elapsed since the last time this function was called
    void ChassisSubsystem::updatePID(tap::algorithms::SmoothPid* pidCont, tap::motor::DjiMotor* const dTrainMotor, float targetRPM, float dt)
    {
        pidCont->runControllerDerivateError(targetRPM - dTrainMotor->getShaftRPM(), dt);
        dTrainMotor->setDesiredOutput((int32_t)pidCont->getOutput());
    }

    // ===== WORK IN PROGRESS =====
    // *** MAY USE THE mpsToRpm(float mps) FUNCTION TO CONVERT LINEAR VELOCITY INPUT TO RPM
    // Sets each of the motors to their desired RPMs for a given horizontal/vertical linear velocity defined by remote input and the
    // turning input (usually from beyblading)
    void ChassisSubsystem::setMotorRPMs(float x, float y, float rot)
    {
        FLTargetRPM = limitVal<float>(DTRAIN_RPM_SCALE * (-x + y + rot), -DTRAIN_MAX_RPM, DTRAIN_MAX_RPM);
        FRTargetRPM = limitVal<float>(DTRAIN_RPM_SCALE * (x + y - rot), -DTRAIN_MAX_RPM, DTRAIN_MAX_RPM);
        BLTargetRPM = limitVal<float>(DTRAIN_RPM_SCALE * (x + y + rot), -DTRAIN_MAX_RPM, DTRAIN_MAX_RPM);
        BRTargetRPM = limitVal<float>(DTRAIN_RPM_SCALE * (-x + y - rot), -DTRAIN_MAX_RPM, DTRAIN_MAX_RPM);
    }

}