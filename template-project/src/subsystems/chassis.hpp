#ifndef CHASSIS_HPP
#define CHASSIS_HPP

#include "../drivers_singleton.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/algorithms/smooth_pid.hpp"

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

private:
    src::Drivers *drivers;
    
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
    static constexpr float MAX_CHASSIS_RPM = 6000.0f;
};

#endif  // CHASSIS_HPP
