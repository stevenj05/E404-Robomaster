#include "chassis.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>

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
    
    // NOTE: Calibration is now ONLY done via LEFT_SWITCH button press
    // This ensures gimbal is stable and turret points where user wants before calibration
    // See gimbal.checkAndHandleRecalibration() for the actual calibration logic
}

void Chassis::update()
{
    // === BEYBLADE MODE CHECK ===
    // Can activate with Shift key OR right switch on remote
    bool keyboardBeyblade = drivers->remote.keyPressed(tap::communication::serial::Remote::Key::SHIFT);
    bool remoteBeyblade = (drivers->remote.getSwitch(tap::communication::serial::Remote::Switch::RIGHT_SWITCH) == 
                          tap::communication::serial::Remote::SwitchState::UP);
    bool beybladeActive = keyboardBeyblade || remoteBeyblade;
    
    // === HYBRID INPUT: Keyboard + Joystick ===
    // Get keyboard input for WASD movement
    float keyboard_vx = 0.0f;
    float keyboard_vy = 0.0f;
    
    if (drivers->remote.keyPressed(tap::communication::serial::Remote::Key::W))
        keyboard_vx += 1.0f;
    if (drivers->remote.keyPressed(tap::communication::serial::Remote::Key::S))
        keyboard_vx -= 1.0f;
    if (drivers->remote.keyPressed(tap::communication::serial::Remote::Key::A))
        keyboard_vy -= 1.0f;
    if (drivers->remote.keyPressed(tap::communication::serial::Remote::Key::D))
        keyboard_vy += 1.0f;
    
    // Get joystick input from remote
    float joystick_vx = drivers->remote.getChannel(tap::communication::serial::Remote::Channel::LEFT_VERTICAL);
    float joystick_vy = drivers->remote.getChannel(tap::communication::serial::Remote::Channel::LEFT_HORIZONTAL);
    
    // Combine inputs: add them together for hybrid control
    float raw_vx = keyboard_vx + joystick_vx;
    float raw_vy = keyboard_vy + joystick_vy;
    
    // Clamp to [-1, 1] range
    raw_vx = std::clamp(raw_vx, -1.0f, 1.0f);
    raw_vy = std::clamp(raw_vy, -1.0f, 1.0f);
    
    // 2. Adaptive Shuriken Speed (based on translation magnitude)
    // When moving fast: reduce spin for stability
    // When stationary: full spin speed
    float translationMagnitude = std::sqrt(raw_vx * raw_vx + raw_vy * raw_vy);
    float downscale = calculateDownscale(translationMagnitude);
    float shurikenRpm = MAX_SHURIKEN_SPEED * downscale;
    // Convert RPM to normalized omega (0 to 1 range), negative for counterclockwise
    float omega = beybladeActive ? -(shurikenRpm / MAX_CHASSIS_RPM) : 0.0f;
    
    // DEBUG: Print beyblade status
    static uint32_t beybladeDebugCounter = 0;
    if (beybladeActive && beybladeDebugCounter++ % 100 == 0)
    {
        printf("[BEYBLADE] TransMag=%.2f | Downscale=%.3f | DesiredSpin=%.0f RPM | Omega=%.3f\n",
               (double)translationMagnitude, (double)downscale, (double)shurikenRpm, (double)omega);
        printf("  Motor speeds - FL: %.0f | FR: %.0f | BL: %.0f | BR: %.0f\n",
               (double)motorFL->getShaftRPM(), (double)motorFR->getShaftRPM(), (double)motorBL->getShaftRPM(), (double)motorBR->getShaftRPM());
    }
    
    // === PASS ACTUAL CHASSIS SPIN TO GIMBAL FOR FEED-FORWARD COMPENSATION ===
    // Calculate actual measured chassis angular velocity from motors
    float actualChassisAngularVelocity = getChassisAngularVelocity();  // degrees/sec
    
    if (gimbal != nullptr)
    {
        // Pass the MEASURED angular velocity to gimbal for Feed-Forward in Beyblade mode
        gimbal->setChassisAngularVelocity(actualChassisAngularVelocity);
    }

    // 3. Get Yaw Angle from gimbal for turret-centric drive
    float turret_angle_deg = 0.0f;
    if (gimbal != nullptr && gimbal->isCalibrated())
    {
        // Use corrected encoder angle from gimbal (accounts for mounting offset between gyro and encoder)
        turret_angle_deg = gimbal->getCorrectedEncoderYawDegrees();
    }
    float theta = -turret_angle_deg * (M_PI / 180.0f);  // Negate for correct rotation direction

    // 4. Turret-Centric Translation
    // Use the relative angle between turret and chassis to rotate movement vectors
    float vx = 0.0f;
    float vy = 0.0f;
    
    // Only apply turret-centric transform if gimbal is calibrated
    if (gimbal != nullptr && gimbal->isCalibrated())
    {
        vx = raw_vx * cos(theta) - raw_vy * sin(theta);
        vy = raw_vx * sin(theta) + raw_vy * cos(theta);
    }
    else
    {
        // Before calibration, drive is disabled to prevent unpredictable behavior
        vx = 0.0f;
        vy = 0.0f;
    }
    
    // DEBUG: Print transform values when joystick is being moved
    static uint32_t debugCounter = 0;
    if ((std::abs(raw_vx) > 0.1f || std::abs(raw_vy) > 0.1f) && gimbal != nullptr && gimbal->isCalibrated() && debugCounter++ % 50 == 0)
    {
        printf("[CHASSIS] Turret: %.1f deg | Raw(vx=%.2f, vy=%.2f) → Transform(vx=%.2f, vy=%.2f)\n",
               (double)turret_angle_deg, (double)raw_vx, (double)raw_vy, (double)vx, (double)vy);
    }
    
    // 5. Your existing X-drive kinematics
    // All motors get same omega contribution for all 4 spinning same direction
    float fl_speed = vx + vy + omega;
    float fr_speed = -vx + vy + omega;
    float bl_speed = vx - vy + omega;
    float br_speed = -vx - vy + omega;
    
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
    if (gimbal != nullptr && gimbal->isCalibrated())
    {
        // Use corrected encoder angle from gimbal (accounts for mounting offset)
        turret_angle_deg = gimbal->getCorrectedEncoderYawDegrees();
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
float Chassis::calculateDownscale(float translationMagnitude) const
{
    // Adaptive downscaling based on FANG Robotics' Shuriken implementation
    // When stationary (magnitude = 0): downscale = 1.0 (full spin speed)
    // As robot moves: downscale reduces inversely, reducing spin for stability
    // 
    // Formula: downscale = coefficient / (coefficient + magnitude)
    // Higher coefficient means LESS aggressive downscaling
    // 
    // Example with DOWNSCALE_COEFFICIENT = 2.0:
    // - At magnitude 0.0: downscale = 2.0 / 2.0 = 1.0 (100% speed)
    // - At magnitude 1.0: downscale = 2.0 / 3.0 = 0.667 (67% speed)
    // - At magnitude 2.0: downscale = 2.0 / 4.0 = 0.5 (50% speed)
    // - At magnitude 4.0: downscale = 2.0 / 6.0 = 0.333 (33% speed)
    
    float downscale = DOWNSCALE_COEFFICIENT / (DOWNSCALE_COEFFICIENT + translationMagnitude);
    
    // Clamp to [0, 1] range for safety
    if (downscale > 1.0f) downscale = 1.0f;
    if (downscale < 0.0f) downscale = 0.0f;
    
    return downscale;
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