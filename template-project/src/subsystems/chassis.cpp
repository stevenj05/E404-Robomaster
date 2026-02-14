#include "chassis.hpp"
#include <algorithm>
#include <cmath>

Chassis::Chassis(src::Drivers *drivers)
    : drivers(drivers), gimbal(nullptr),
      // Initialize motors on CAN1 with correct IDs
      // Front Left = ID 2, Front Right = ID 4, Back Left = ID 1, Back Right = ID 3
      motorFL(new tap::motor::DjiMotor(drivers, tap::motor::MOTOR2, CAN_BUS, false, "FL Motor")),
      motorFR(new tap::motor::DjiMotor(drivers, tap::motor::MOTOR4, CAN_BUS, false, "FR Motor")),
      motorBL(new tap::motor::DjiMotor(drivers, tap::motor::MOTOR1, CAN_BUS, false, "BL Motor")),
      motorBR(new tap::motor::DjiMotor(drivers, tap::motor::MOTOR3, CAN_BUS, false, "BR Motor")),
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
    
    // Auto-calibration: capture the gimbal's initial yaw angle on startup
    // This will be used as the reference position for turret-centric drive
    if (gimbal != nullptr)
    {
        initialGimbalYawOffset = gimbal->getYawAngleDegrees();
        isCalibrated = true;
    }
}

void Chassis::update()
{
    // 1. Get raw joystick inputs
    float raw_vx = drivers->remote.getChannel(tap::communication::serial::Remote::Channel::LEFT_VERTICAL); 
    float raw_vy = drivers->remote.getChannel(tap::communication::serial::Remote::Channel::LEFT_HORIZONTAL);
    
    // 2. Handle Rotation via RIGHT_SWITCH
    float omega = 0.0f;
    
    tap::communication::serial::Remote::SwitchState currentRightSwitchState = 
        drivers->remote.getSwitch(tap::communication::serial::Remote::Switch::RIGHT_SWITCH);
    
    // RIGHT_SWITCH is DOWN - no rotation
    // RIGHT_SWITCH is MID - no rotation  
    // RIGHT_SWITCH is UP - no rotation
    // Rotation controlled by left joystick horizontal movement
    
    // RIGHT_SWITCH can be used for other features in the future
    
    // Tell gimbal about the current chassis angular velocity (unused in turret-centric mode)
    if (gimbal != nullptr)
    {
        gimbal->setChassisAngularVelocity(0.0f);
    } 

    // 3. Get Yaw Angle from gimbal for turret-centric drive
    float turret_angle_deg = 0.0f;
    if (gimbal != nullptr && isCalibrated)
    {
        // Use auto-calibrated offset: subtract initial position so startup position = 0 degrees
        turret_angle_deg = gimbal->getYawAngleDegrees() - initialGimbalYawOffset - 10.0f;
    }
    float theta = -turret_angle_deg * (M_PI / 180.0f);  // Negate for correct rotation direction 

    // 4. Transform inputs to Turret-Centric
    float vx = 0.0f;
    float vy = 0.0f;
    
    // Always apply turret-centric transform
    vx = raw_vx * cos(theta) - raw_vy * sin(theta);
    vy = raw_vx * sin(theta) + raw_vy * cos(theta);
    
    // 5. Your existing X-drive kinematics
    float fl_speed = vx + vy + omega;
    float fr_speed = -vx + vy - omega;
    float bl_speed = vx - vy + omega;
    float br_speed = -vx - vy - omega;
    
    // 6. Scale to MAX_CHASSIS_RPM (Your existing logic)
    desiredRpmFL = fl_speed * MAX_CHASSIS_RPM;
    desiredRpmFR = fr_speed * MAX_CHASSIS_RPM;
    desiredRpmBL = bl_speed * MAX_CHASSIS_RPM;
    desiredRpmBR = br_speed * MAX_CHASSIS_RPM;
    
    // 7. KEEP YOUR PID CONTROLLERS (Do not delete these!)
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
    // Apply turret-centric transform to input velocities
    float turret_angle_deg = 0.0f;
    if (gimbal != nullptr && isCalibrated)
    {
        // Use auto-calibrated offset: subtract initial position so startup position = 0 degrees
        turret_angle_deg = gimbal->getYawAngleDegrees() - initialGimbalYawOffset - 10.0f;
    }
    float theta = -turret_angle_deg * (M_PI / 180.0f);  // Negate for correct rotation direction
    
    // Transform inputs to Turret-Centric
    float vx_transformed = vx * cos(theta) - vy * sin(theta);
    float vy_transformed = vx * sin(theta) + vy * cos(theta);
    
    // X-drive kinematics: wheels at 45 degree angles
    float fl_speed = vx_transformed + vy_transformed + omega;
    float fr_speed = -vx_transformed + vy_transformed - omega;
    float bl_speed = vx_transformed - vy_transformed + omega;
    float br_speed = -vx_transformed - vy_transformed - omega;
    
    // Convert to desired RPM for PID control
    desiredRpmFL = fl_speed * MAX_CHASSIS_RPM;
    desiredRpmFR = fr_speed * MAX_CHASSIS_RPM;
    desiredRpmBL = bl_speed * MAX_CHASSIS_RPM;
    desiredRpmBR = br_speed * MAX_CHASSIS_RPM;
}
float Chassis::getChassisAngularVelocity() const
{
    // Get actual measured motor speeds in RPM
    // For X-drive spin mode: FL and BL spin CW, FR and BR spin CCW
    float flRpm = motorFL->getShaftRPM();
    float frRpm = motorFR->getShaftRPM();
    float blRpm = motorBL->getShaftRPM();
    float brRpm = motorBR->getShaftRPM();
    
    // During pure rotation, opposite pairs spin opposite directions
    // Take average magnitude to get chassis spin rate
    float avgMotorRpmMagnitude = (std::abs(flRpm) + std::abs(frRpm) + std::abs(blRpm) + std::abs(brRpm)) / 4.0f;
    
    // Convert motor RPM to motor degrees per second
    // 1 RPM = 360 degrees / 60 seconds = 6 degrees/second
    float motorAngularVelocityDegPerSec = avgMotorRpmMagnitude * 6.0f;
    
    // For X-drive with 6-inch VEX Pro omni wheels to chassis angular velocity conversion:
    // Wheel diameter: 6" = 0.1524m, circumference = 0.4787m
    // Wheelbase (center to wheel): ~0.38m (RoboMaster Type C typical)
    // Motor RPM → linear velocity: RPM × 0.00798 m/s
    // Chassis angular velocity: (RPM × 0.00798) / 0.38 = RPM × 1.21 deg/s
    static constexpr float MOTOR_TO_CHASSIS_SCALING = 1.2f;
    
    float chassisAngularVelocityDegPerSec = motorAngularVelocityDegPerSec * MOTOR_TO_CHASSIS_SCALING;
    
    // Preserve sign based on left/right motor rotation directions
    // If FR/BR are spinning (main spin direction), use that sign
    if (frRpm != 0.0f)
    {
        // Return negative if FR/BR are negative (counterclockwise)
        if (frRpm < 0.0f)
        {
            chassisAngularVelocityDegPerSec = -chassisAngularVelocityDegPerSec;
        }
    }
    else if (flRpm < 0.0f)
    {
        // If FL/BL are spinning opposite (shouldn't happen in pure spin)
        chassisAngularVelocityDegPerSec = -chassisAngularVelocityDegPerSec;
    }
    
    return chassisAngularVelocityDegPerSec;
}