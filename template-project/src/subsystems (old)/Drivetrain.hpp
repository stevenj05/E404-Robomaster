#pragma once

#include "../drivers_singleton.hpp"
#include "../Constants.hpp"
#include <random>

using namespace Constants;

struct DriveOutputs {
    int32_t fl, fr, bl, br;
};

class Drivetrain {
public:
    Drivetrain(src::Drivers* _drivers, tap::communication::serial::Remote& remote, double& _yaw);

    void initialize();
    void update();
    void tick(float scale = 1.0f);

private:
    src::Drivers* drivers;

    // --- Internal helpers ---
    bool motorsHealthy();
    DriveOutputs computeDriveOutputs(float scale);
    void applyMotorOutputs(const DriveOutputs& drive);
    void applyBeybladeSpin(DriveOutputs drive, float scale);

    void mecanumDrive();
    void gimbalOrientedDrive();

    // --- State ---
    std::function<void()> mecanumFunc;
    std::function<void()> gimbalFunc;
    std::function<void()> driveFunc;

    float safetyScale{1.0f};
    bool beybladeMode{false}, gimbalMode{false};

    // --- Hardware ---
    std::optional<tap::motor::DjiMotor> motorFL, motorFR, motorBL, motorBR;

    // --- PID Controllers ---
    tap::algorithms::SmoothPid pidFL{chassis_pid_fl};
    tap::algorithms::SmoothPid pidFR{chassis_pid_fr};
    tap::algorithms::SmoothPid pidBL{chassis_pid_bl};
    tap::algorithms::SmoothPid pidBR{chassis_pid_br};

    // --- Remote & Orientation ---
    tap::communication::serial::Remote& remote;
    double& yaw;

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
