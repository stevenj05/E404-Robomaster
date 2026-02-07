#ifndef TURRET_HPP
#define TURRET_HPP

#include "../drivers_singleton.hpp"
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/communication/gpio/digital.hpp"
#include "tap/communication/serial/remote.hpp"
#include "tap/motor/dji_motor.hpp"

class Turret
{
public:
    Turret(src::Drivers *drivers, tap::communication::serial::Remote *remote);
    ~Turret() = default;

    //Initialize all turret motors and subsystems
    void initialize();
    /**
     * Update turret control - reads remote input and controls motors
     * Called periodically from main loop
     */
    void update();
    /**
     * Send motor commands via CAN
     */
    void sendMotorCommands();

private:
    src::Drivers *drivers;
    tap::communication::serial::Remote *remote;

    // Motors
    tap::motor::DjiMotor *agitator;
    tap::motor::DjiMotor *flywheel_left;
    tap::motor::DjiMotor *flywheel_right;

    // PID Controllers
    tap::algorithms::SmoothPid *pidController1;
    tap::algorithms::SmoothPid *pidController2;
    tap::algorithms::SmoothPid *pidController3;

    // Red Dot Laser control
    tap::gpio::Digital red_dot;

    // Constants
    static constexpr tap::can::CanBus CAN_BUS = tap::can::CanBus::CAN_BUS2;
    static constexpr float MAX_AGITATOR_RPM = 5000.0f;
    static constexpr float MAX_FLYWHEEL_RPM = 6500.0f;
};

#endif  // TURRET_HPP
