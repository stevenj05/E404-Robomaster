#pragma once

#include <tap/communication/serial/remote.hpp>
#include "modm/processing/timer.hpp"
#include "../Constants.hpp"
#include <functional>
#include <cmath>
#include <random>
#include <array>

using namespace Constants;

struct DriveOutputs {
    int32_t fl, fr, bl, br;
};

class Drivetrain {
public:
    Drivetrain(tap::communication::serial::Remote& remote, double& _yaw);

    void initialize();
    void update();
    void tick(float scale = 1.0f);

private:
    // --- Internal helpers ---
    bool motorsHealthy();
    DriveOutputs computeDriveOutputs(float scale);
    void applyMotorOutputs(const DriveOutputs& drive);
    void applyBeybladeSpin(DriveOutputs drive, float scale);

    void mecanumDrive();
    void gimbleOrientedDrive();

    // --- State ---
    std::function<void()> driveFunc;
    float safetyScale{1.0f};
    bool beybladeMode{false};

    // --- Hardware ---
    tap::motor::DjiMotor motorFL{drivers, M_ID1, CAN_BUS2, false, "motorFL"};
    tap::motor::DjiMotor motorFR{drivers, M_ID2, CAN_BUS2, true,  "motorFR"};
    tap::motor::DjiMotor motorBL{drivers, M_ID3, CAN_BUS2, false, "motorBL"};
    tap::motor::DjiMotor motorBR{drivers, M_ID4, CAN_BUS2, true,  "motorBR"};

    // --- PID Controllers ---
    tap::algorithms::SmoothPid pidFL{pidConfig1};
    tap::algorithms::SmoothPid pidFR{pidConfig2};
    tap::algorithms::SmoothPid pidBL{pidConfig3};
    tap::algorithms::SmoothPid pidBR{pidConfig4};

    // --- Remote & Orientation ---
    tap::communication::serial::Remote& remote;
    double yaw;

    // --- Inputs ---
    int32_t fwdInput{0};
    int32_t strafeInput{0};
    int32_t turnInput{0};

    // --- Beyblade spin ---
    float beybladeSpin{8000.0f};           // current spin
    float targetSpin{8000.0f};             // next random target
    float spinSmoothFactor{0.1f};          // how fast spin moves toward target
    modm::chrono::micro_clock::time_point lastSpinUpdate{}; // last target update

    // Random engine
    std::mt19937 rng{std::random_device{}()};
    std::uniform_real_distribution<float> spinDist{6000.0f, 12000.0f};
};
