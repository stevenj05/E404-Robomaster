#pragma once
#include "tap/motor/dji_motor.hpp"
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/communication/serial/remote.hpp"
#include "modm/processing/timer.hpp"

class Drivetrain {
public:
    Drivetrain(tap::motor::DjiMotor& fl,
               tap::motor::DjiMotor& fr,
               tap::motor::DjiMotor& bl,
               tap::motor::DjiMotor& br,
               tap::communication::serial::Remote& remote);

    void initialize();
    void update();
    void tick();

private:
    // Hardware
    tap::motor::DjiMotor& motorFL;
    tap::motor::DjiMotor& motorFR;
    tap::motor::DjiMotor& motorBL;
    tap::motor::DjiMotor& motorBR;

    // PID
    tap::algorithms::SmoothPid pidFL, pidFR, pidBL, pidBR;

    // Remote
    tap::communication::serial::Remote& remote;

    // Inputs
    int32_t fwdInput{0}, strafeInput{0}, turnInput{0};

    // Beyblade
    bool beybladeMode{false};
    modm::chrono::milli_clock::time_point lastToggleTime;
};
