#ifndef CHASSIS_HPP
#define CHASSIS_HPP

#include "../drivers_singleton.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/algorithms/smooth_pid.hpp"
#include "gimbal.hpp"

class Chassis
{
public:
    Chassis(src::Drivers *drivers);
    ~Chassis() = default;

    /**
     * Initialize chassis subsystem
     */
    void initialize();

    /**
     * Update chassis control
     */
    void update();

    /**
     * Send motor commands via CAN
     */
    void sendMotorCommands();

    /**
     * Set the desired velocity for the chassis
     * @param vx velocity in the x direction
     * @param vy velocity in the y direction
     * @param omega angular velocity
     */
    void setVelocity(float vx, float vy, float omega);
    
    /**
     * Set gimbal pointer for turret-centric drive
     * Does NOT trigger calibration—calibration must be done manually via LEFT_SWITCH
     */
    void setGimbal(Gimbal *gimbal_ptr) 
    { 
        gimbal = gimbal_ptr;
    }
    
    /**
     * Get actual chassis angular velocity in degrees per second
     * Calculated from measured motor RPM, accounts for geometry scaling
     * @return chassis angular velocity (degrees per second, positive = counterclockwise)
     */
    float getChassisAngularVelocity() const;

private:
    src::Drivers *drivers;
    Gimbal *gimbal;
    
    // RM3508 motors on CAN2 in X-drive configuration
    // Front-left motor
    tap::motor::DjiMotor *motorFL;
    // Front-right motor
    tap::motor::DjiMotor *motorFR;
    // Back-left motor
    tap::motor::DjiMotor *motorBL;
    // Back-right motor
    tap::motor::DjiMotor *motorBR;
    
    // PID Controllers for each motor
    tap::algorithms::SmoothPid *pidFL;
    tap::algorithms::SmoothPid *pidFR;
    tap::algorithms::SmoothPid *pidBL;
    tap::algorithms::SmoothPid *pidBR;
    
    // Store desired RPM for each motor
    float desiredRpmFL = 0.0f;
    float desiredRpmFR = 0.0f;
    float desiredRpmBL = 0.0f;
    float desiredRpmBR = 0.0f;
    
    // Constants
    static constexpr tap::can::CanBus CAN_BUS = tap::can::CanBus::CAN_BUS1;
    static constexpr float MAX_CHASSIS_RPM = 4800.0f; // Reduced by 20% (was 6000)
    // INCREASED to 3300 RPM for faster beyblade spin
    static constexpr float MAX_SHURIKEN_SPEED = 3300.0f;  
    static constexpr float DOWNSCALE_COEFFICIENT = 2.0f;  // Higher = less downscaling at high speed
    
    /**
     * Calculate downscale factor based on translation magnitude
     * When moving slowly: downscale ≈ 1.0 (full spin speed)
     * When moving fast: downscale reduces (less spin, better stability)
     * @param translationMagnitude magnitude of velocity vector
     * @return downscale factor between 0.0 and 1.0
     */
    float calculateDownscale(float translationMagnitude) const;
};

#endif  // CHASSIS_HPP
