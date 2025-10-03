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
    void tick(float scale = 1.0f);

private:
    // Helper to determine if motors are behaving correctly
    bool motorsHealthy();
    float safetyScale{1.0f};

    // Hardware
    tap::motor::DjiMotor motorFL{drivers, M_ID1, CAN_BUS2, false, "motorFL"};
    tap::motor::DjiMotor motorFR{drivers, M_ID2, CAN_BUS2, true, "motorFR"};
    tap::motor::DjiMotor motorBL{drivers, M_ID3, CAN_BUS2, false, "motorBL"};
    tap::motor::DjiMotor motorBR{drivers, M_ID4, CAN_BUS2, true, "motorBR"};

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
