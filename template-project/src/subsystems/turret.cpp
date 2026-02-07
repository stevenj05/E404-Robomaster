#include "turret.hpp"

Turret::Turret(src::Drivers *drivers, tap::communication::serial::Remote *remote)
    : drivers(drivers),
      remote(remote),
      agitator(new tap::motor::DjiMotor(drivers, tap::motor::MOTOR1, CAN_BUS, false, "agitator")),
      flywheel_left(new tap::motor::DjiMotor(drivers, tap::motor::MOTOR2, CAN_BUS, true, "flywheel_left")),
      flywheel_right(new tap::motor::DjiMotor(drivers, tap::motor::MOTOR3, CAN_BUS, false, "flywheel_right")),
      pidController1(new tap::algorithms::SmoothPid(tap::algorithms::SmoothPidConfig{15, 0.5, 0, 0, 16000, 1, 0, 1, 0})),
      pidController2(new tap::algorithms::SmoothPid(tap::algorithms::SmoothPidConfig{15, 0.5, 0, 0, 16000, 1, 0, 1, 0})),
      pidController3(new tap::algorithms::SmoothPid(tap::algorithms::SmoothPidConfig{15, 0.5, 0, 0, 16000, 1, 0, 1, 0}))
{
}

void Turret::initialize()
{
    agitator->initialize();
    flywheel_left->initialize();
    flywheel_right->initialize();
    red_dot.init();
}

void Turret::update()
{
    // Read remote input to determine desired RPMs
    float agitatorDesiredRpm = remote->getChannel(tap::communication::serial::Remote::Channel::WHEEL) * MAX_AGITATOR_RPM;
    float flywheelDesiredRpm = 0.0f;
    if (remote->getSwitch(tap::communication::serial::Remote::Switch::LEFT_SWITCH) ==
        tap::communication::serial::Remote::SwitchState::UP)
    {
        flywheelDesiredRpm = MAX_FLYWHEEL_RPM;
    }
    // Update agitator PID controller
    pidController1->runControllerDerivateError(agitatorDesiredRpm - agitator->getShaftRPM(), 1);
    agitator->setDesiredOutput(static_cast<int32_t>(pidController1->getOutput()));
    // Update flywheel left PID controller
    pidController2->runControllerDerivateError(flywheelDesiredRpm - flywheel_left->getShaftRPM(), 1);
    flywheel_left->setDesiredOutput(static_cast<int32_t>(pidController2->getOutput()));
    // Update flywheel right PID controller
    pidController3->runControllerDerivateError(flywheelDesiredRpm - flywheel_right->getShaftRPM(), 1);
    flywheel_right->setDesiredOutput(static_cast<int32_t>(pidController3->getOutput()));
    // Toggle laser on when flywheels are spinning, off when idle
    bool laserOn = (flywheelDesiredRpm > 0);
    red_dot.set(tap::gpio::Digital::OutputPin::Laser, laserOn);
}

void Turret::sendMotorCommands()
{
    drivers->djiMotorTxHandler.encodeAndSendCanData();
}
