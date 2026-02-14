#ifndef GIMBAL_HPP
#define GIMBAL_HPP

#include "../drivers_singleton.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/algorithms/smooth_pid.hpp"

class Gimbal
{
public:
    Gimbal(src::Drivers *drivers);
    ~Gimbal() = default;

    /**
     * Initialize gimbal subsystem
     */
    void initialize();

    /**
     * Update gimbal control
     */
    void update();

    /**
     * Send motor commands via CAN
     */
    void sendMotorCommands();
    
    /**
     * Get current pitch encoder position
     */
    float getPitchEncoderPosition() const;
    
    /**
     * Get relative yaw angle from startup (degrees)
     * Positive = right/counterclockwise
     */
    float getYawAngleDegrees() const;
    
    /**
     * Set chassis angular velocity (currently unused in turret-centric mode)
     * @param angularVelocityDegPerSec Chassis angular velocity in degrees/second
     */
    void setChassisAngularVelocity(float angularVelocityDegPerSec) { 
        (void)angularVelocityDegPerSec;  // Unused in turret-centric mode
    }


private:
    static constexpr tap::can::CanBus CAN_BUS = tap::can::CanBus::CAN_BUS1;
    
    // Motor IDs: Yaw = MOTOR5 (ID 0x205), Pitch = MOTOR8 (ID 0x208)
    // Both motors are INVERTED (true)
    tap::motor::DjiMotor *yawMotor;
    tap::motor::DjiMotor *pitchMotor;
    
    // PID controllers for each motor
    tap::algorithms::SmoothPid *pidYaw;
    tap::algorithms::SmoothPid *pidPitch;
    
    // Gimbal target positions (encoder-based)
    float gimbalYawTargetPos = 0.0f;
    float gimbalPitchTargetPos = 0.0f;
    bool firstPitchUpdate = true;
    
    // PID activation delay (3 seconds at ~500Hz update rate = 1500 ticks)
    static constexpr uint32_t PID_ACTIVATION_DELAY = 1500;
    uint32_t updateCounter = 0;
    bool pidYawActive = false;
    bool pidPitchActive = false;
    
    // Yaw calibration for turret-centric drive
    float initialYawEncoder = 0.0f;
    static constexpr float ENCODER_COUNTS_PER_DEGREE = 8192.0f / 360.0f;  // GM6020
    static constexpr float YAW_GEAR_RATIO = 2.5f;  // 2.5:1 gear ratio (motor spins 2.5x per turret rotation)
    
    // Motor constants
    static constexpr int MAX_GIMBAL_RPM = 500;
    
    src::Drivers *drivers;
};

#endif  // GIMBAL_HPP
