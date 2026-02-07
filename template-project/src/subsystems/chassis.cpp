#include "chassis.hpp"
#include <algorithm>
#include <cmath>

Chassis::Chassis(src::Drivers *drivers)
    : drivers(drivers),
      // Initialize motors on CAN2 with IDs 1-4
      // ID 1 = Front Right, ID 2 = Front Left, ID 3 = Back Left, ID 4 = Back Right
      motorFL(new tap::motor::DjiMotor(drivers, tap::motor::MOTOR2, CAN_BUS, false, "FL Motor")),
      motorFR(new tap::motor::DjiMotor(drivers, tap::motor::MOTOR1, CAN_BUS, false, "FR Motor")),
      motorBL(new tap::motor::DjiMotor(drivers, tap::motor::MOTOR3, CAN_BUS, false, "BL Motor")),
      motorBR(new tap::motor::DjiMotor(drivers, tap::motor::MOTOR4, CAN_BUS, false, "BR Motor")),
      // Initialize PID controllers for each motor (matching turret pattern)
      pidFL(new tap::algorithms::SmoothPid(tap::algorithms::SmoothPidConfig{15, 0.5, 0, 0, 16000, 1, 0, 1, 0})),
      pidFR(new tap::algorithms::SmoothPid(tap::algorithms::SmoothPidConfig{15, 0.5, 0, 0, 16000, 1, 0, 1, 0})),
      pidBL(new tap::algorithms::SmoothPid(tap::algorithms::SmoothPidConfig{15, 0.5, 0, 0, 16000, 1, 0, 1, 0})),
      pidBR(new tap::algorithms::SmoothPid(tap::algorithms::SmoothPidConfig{15, 0.5, 0, 0, 16000, 1, 0, 1, 0}))
{
}

void Chassis::initialize()
{
    // Initialize all motors
    motorFL->initialize();
    motorFR->initialize();
    motorBL->initialize();
    motorBR->initialize();
}

void Chassis::update()
{
    // Get left joystick input from DR16 receiver for basic movement
    float vx = drivers->remote.getChannel(tap::communication::serial::Remote::Channel::LEFT_VERTICAL);   // forward/backward
    float vy = drivers->remote.getChannel(tap::communication::serial::Remote::Channel::LEFT_HORIZONTAL);  // strafe left/right
    float omega = 0.0f;  // rotation (not used yet)
    
    // X-drive kinematics: wheels at 45 degree angles
    // Forward: FL and BL spin forward, FR and BR spin backward
    // Strafe right: FL and BL spin same direction (away from each other), FR and BR opposite (toward each other)
    float fl_speed = vx - vy + omega;
    float fr_speed = -vx + vy + omega;
    float bl_speed = vx - vy + omega;
    float br_speed = -vx - vy + omega;
    
    // Scale to MAX_CHASSIS_RPM
    desiredRpmFL = fl_speed * MAX_CHASSIS_RPM;
    desiredRpmFR = fr_speed * MAX_CHASSIS_RPM;
    desiredRpmBL = bl_speed * MAX_CHASSIS_RPM;
    desiredRpmBR = br_speed * MAX_CHASSIS_RPM;
    
    // Update PID controllers and apply motor outputs
    pidFL->runControllerDerivateError(desiredRpmFL - motorFL->getShaftRPM(), 1);
    motorFL->setDesiredOutput(static_cast<int32_t>(pidFL->getOutput()));
    
    pidFR->runControllerDerivateError(desiredRpmFR - motorFR->getShaftRPM(), 1);
    motorFR->setDesiredOutput(static_cast<int32_t>(pidFR->getOutput()));
    
    pidBL->runControllerDerivateError(desiredRpmBL - motorBL->getShaftRPM(), 1);
    motorBL->setDesiredOutput(static_cast<int32_t>(pidBL->getOutput()));
    
    pidBR->runControllerDerivateError(desiredRpmBR - motorBR->getShaftRPM(), 1);
    motorBR->setDesiredOutput(static_cast<int32_t>(pidBR->getOutput()));
}

void Chassis::sendMotorCommands()
{
    drivers->djiMotorTxHandler.encodeAndSendCanData();
}

void Chassis::setVelocity(float vx, float vy, float omega)
{
    // X-drive kinematics: wheels at 45 degree angles
    // vx: forward/backward
    // vy: strafe left/right
    // omega: rotation
    // Strafe right: FL and BL spin same direction (away), FR and BR opposite (toward)
    float fl_speed = vx - vy + omega;
    float fr_speed = -vx + vy + omega;
    float bl_speed = vx - vy + omega;
    float br_speed = -vx - vy + omega;
    
    // Convert to desired RPM for PID control
    desiredRpmFL = fl_speed * MAX_CHASSIS_RPM;
    desiredRpmFR = fr_speed * MAX_CHASSIS_RPM;
    desiredRpmBL = bl_speed * MAX_CHASSIS_RPM;
    desiredRpmBR = br_speed * MAX_CHASSIS_RPM;
}
