#include "Drivetrain.hpp"
#include <array>
#include <cmath>
#include <algorithm>
#include "tap/algorithms/smooth_pid.hpp"
#include "Constants.hpp"

Drivetrain::Drivetrain(tap::communication::serial::Remote& remoteIn, double& _yaw)
    : remote(remoteIn), yaw(_yaw) {}

void Drivetrain::initialize() {
    for (auto* motor : {&motorFL, &motorFR, &motorBL, &motorBR})
        motor->initialize();

    driveFunc = [&]() { mecanumDrive(); };
}

// --- internal helper ---
constexpr inline bool motorHealthy(int rpm, int out, int threshold) noexcept {
    return rpm > threshold || out < threshold;
}

bool Drivetrain::motorsHealthy() {
    constexpr int minRpmResponse = 50;
    return
        motorHealthy(std::abs(motorFL.getShaftRPM()), std::abs(pidFL.getOutput()), minRpmResponse) &&
        motorHealthy(std::abs(motorFR.getShaftRPM()), std::abs(pidFR.getOutput()), minRpmResponse) &&
        motorHealthy(std::abs(motorBL.getShaftRPM()), std::abs(pidBL.getOutput()), minRpmResponse) &&
        motorHealthy(std::abs(motorBR.getShaftRPM()), std::abs(pidBR.getOutput()), minRpmResponse);
}

void Drivetrain::mecanumDrive() {
    // Precompute combined factors once
    const double flFactor =  fwdInput +  strafeInput + turnInput;
    const double frFactor =  fwdInput -  strafeInput - turnInput;
    const double blFactor = -fwdInput -  strafeInput + turnInput;
    const double brFactor = -fwdInput +  strafeInput - turnInput;

    // Scale constant — compile-time if constexpr
    constexpr double MAX_RPM = 4000.0;

    // Compute desired RPM for each wheel
    const double flTarget = (flFactor * MAX_RPM) - motorFL.getShaftRPM();
    const double frTarget = (frFactor * MAX_RPM) - motorFR.getShaftRPM();
    const double blTarget = (blFactor * MAX_RPM) - motorBL.getShaftRPM();
    const double brTarget = (brFactor * MAX_RPM) - motorBR.getShaftRPM();

    // Run PID controllers directly
    pidFL.runControllerDerivateError(flTarget, 1);
    pidFR.runControllerDerivateError(frTarget, 1);
    pidBL.runControllerDerivateError(blTarget, 1);
    pidBR.runControllerDerivateError(brTarget, 1);
}

void Drivetrain::gimbalOrientedDrive() {
    const double gyroRadians = yaw * DEG_TO_RAD;

    const float temp = fwdInput * cos(gyroRadians) +
                       strafeInput * sin(gyroRadians);

    strafeInput = -fwdInput * sin(gyroRadians) +
                   strafeInput * cos(gyroRadians);

    fwdInput = temp;

    mecanumDrive();
}

void Drivetrain::update() {
    fwdInput    = remote.getChannel(tap::communication::serial::Remote::Channel::LEFT_VERTICAL);
    strafeInput = remote.getChannel(tap::communication::serial::Remote::Channel::LEFT_HORIZONTAL);
    turnInput   = remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_HORIZONTAL);

    beybladeMode = (remote.getSwitch(tap::communication::serial::Remote::Switch::RIGHT_SWITCH)
                    == tap::communication::serial::Remote::SwitchState::DOWN);

    gimbalMode = beybladeMode || false; //temp for if they want a switch for just this
    driveFunc = gimbalMode ? gyroFunc : mecanumFunc;

    driveFunc();

    // --- health-driven scaling ---
    const float delta = motorsHealthy() ? 0.05f : -0.05f;
    safetyScale = std::clamp(safetyScale + delta, 0.2f, 1.0f);

    // --- update Beyblade spin dynamically ---
    auto now = modm::chrono::micro_clock::now();

    if ((now - lastSpinUpdate).count() > 200000) { // 200ms in microseconds
        targetSpin = spinDist(rng);
        lastSpinUpdate = now;
    }

    beybladeSpin += (targetSpin - beybladeSpin) * spinSmoothFactor;
}

inline DriveOutputs Drivetrain::computeDriveOutputs(float scale) {
    DriveOutputs out;
    out.fl = static_cast<int32_t>(pidFL.getOutput() * scale);
    out.fr = static_cast<int32_t>(pidFR.getOutput() * scale);
    out.bl = static_cast<int32_t>(pidBL.getOutput() * scale);
    out.br = static_cast<int32_t>(pidBR.getOutput() * scale);
    return out;
}

inline void Drivetrain::applyMotorOutputs(const DriveOutputs& drive) {
    motorFL.setDesiredOutput(drive.fl);
    motorFR.setDesiredOutput(drive.fr);
    motorBL.setDesiredOutput(drive.bl);
    motorBR.setDesiredOutput(drive.br);
}

void Drivetrain::applyBeybladeSpin(DriveOutputs drive, float scale) {
    constexpr int32_t MAX_OUTPUT = 12000;
    const int32_t spinPower = static_cast<int32_t>(beybladeSpin * scale);

    drive.fl = std::clamp(drive.fl + spinPower, -MAX_OUTPUT, MAX_OUTPUT);
    drive.fr = std::clamp(drive.fr - spinPower, -MAX_OUTPUT, MAX_OUTPUT);
    drive.bl = std::clamp(drive.bl + spinPower, -MAX_OUTPUT, MAX_OUTPUT);
    drive.br = std::clamp(drive.br - spinPower, -MAX_OUTPUT, MAX_OUTPUT);

    applyMotorOutputs(drive);
}

void Drivetrain::tick(float scale) {
    scale *= safetyScale;

    auto driveOutputs = computeDriveOutputs(scale);

    if (beybladeMode)
        applyBeybladeSpin(driveOutputs, scale);
    else
        applyMotorOutputs(driveOutputs);
}