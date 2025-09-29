#pragma once

#include <tap/communication/serial/remote.hpp>
#include "modm/processing/timer.hpp"
#include "../Constants.hpp"

using namespace Constants;

class Drivetrain {
public:
    Drivetrain(tap::communication::serial::Remote& remote);

    void initialize();
    void update();
    void tick();

private:
    // Hardware
    tap::motor::DjiMotor motorFL{drivers, M_ID1, CAN_BUS2, false, "motor1"};
    tap::motor::DjiMotor motorFR{drivers, M_ID2, CAN_BUS2, true, "motor2"};
    tap::motor::DjiMotor motorBL{drivers, M_ID3, CAN_BUS2, false, "motor3"};
    tap::motor::DjiMotor motorBR{drivers, M_ID4, CAN_BUS2, true, "motor4"};
    // PID
    tap::algorithms::SmoothPid pidFL{pidConfig1};
    tap::algorithms::SmoothPid pidFR{pidConfig2};
    tap::algorithms::SmoothPid pidBL{pidConfig3};
    tap::algorithms::SmoothPid pidBR{pidConfig4};

    // Remote
    tap::communication::serial::Remote& remote;

    // Inputs
    int32_t fwdInput{0}, strafeInput{0}, turnInput{0};

    // Beyblade
    bool beybladeMode{false};
    modm::chrono::milli_clock::time_point lastToggleTime;
};
