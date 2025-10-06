#include "Drivetrain.hpp"
#include <array>
#include <cmath>
#include <algorithm>

Drivetrain::Drivetrain(tap::communication::serial::Remote& remoteIn, double& _yaw)
    : remote(remoteIn), yaw(_yaw) {}

void Drivetrain::initialize() {
    for (auto* motor : {&motorFL, &motorFR, &motorBL, &motorBR})
        motor->initialize();

    driveFunc = [&]() { mecanumDrive(); };
}

// --- internal helper ---
bool Drivetrain::motorsHealthy() {
    constexpr int minRpmResponse = 50; // acceptable threshold

    struct MotorPidPair {
        decltype(motorFL)& motor;
        decltype(pidFL)& pid;
    };

    const std::array<MotorPidPair, 4> pairs {{
        {motorFL, pidFL}, {motorFR, pidFR},
        {motorBL, pidBL}, {motorBR, pidBR}
    }};

    return std::all_of(pairs.begin(), pairs.end(), [&](const auto& p) {
        const int rpm = std::abs(p.motor.getShaftRPM());
        const int out = std::abs(p.pid.getOutput());
        return rpm > minRpmResponse || out < minRpmResponse;
    });
}

void Drivetrain::mecanumDrive() {
    struct WheelNode {
        decltype(pidFL)& pid;
        decltype(motorFL)& motor;
        double factor;
    };

    const std::array<WheelNode, 4> nodes {{
        {pidFL, motorFL,  ( fwdInput +  strafeInput + turnInput)},
        {pidFR, motorFR,  ( fwdInput -  strafeInput - turnInput)},
        {pidBL, motorBL,  (-fwdInput -  strafeInput + turnInput)},
        {pidBR, motorBR,  (-fwdInput +  strafeInput - turnInput)}
    }};

    for (auto& n : nodes)
        n.pid.runControllerDerivateError((n.factor * 4000) - n.motor.getShaftRPM(), 1);
}

void Drivetrain::gimbleOrientedDrive() {
    const float gyroRadians = static_cast<float>(yaw * pi / 180.0);

    const float temp = fwdInput * std::cos(gyroRadians) +
                       strafeInput * std::sin(gyroRadians);

    strafeInput = -fwdInput * std::sin(gyroRadians) +
                   strafeInput * std::cos(gyroRadians);

    fwdInput = temp;

    mecanumDrive();
}

void Drivetrain::update() {
    fwdInput    = remote.getChannel(tap::communication::serial::Remote::Channel::LEFT_VERTICAL);
    strafeInput = remote.getChannel(tap::communication::serial::Remote::Channel::LEFT_HORIZONTAL);
    turnInput   = remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_HORIZONTAL);

    beybladeMode = (remote.getSwitch(tap::communication::serial::Remote::Switch::RIGHT_SWITCH)
                    == tap::communication::serial::Remote::SwitchState::DOWN);

    driveFunc();

    // --- health-driven scaling ---
    const float delta = motorsHealthy() ? 0.05f : -0.05f;
    safetyScale = std::clamp(safetyScale + delta, 0.2f, 1.0f);
}

DriveOutputs Drivetrain::computeDriveOutputs(float scale) {
    DriveOutputs out;
    out.fl = static_cast<int32_t>(pidFL.getOutput() * scale);
    out.fr = static_cast<int32_t>(pidFR.getOutput() * scale);
    out.bl = static_cast<int32_t>(pidBL.getOutput() * scale);
    out.br = static_cast<int32_t>(pidBR.getOutput() * scale);
    return out;
}

void Drivetrain::applyMotorOutputs(const DriveOutputs& drive) {
    motorFL.setDesiredOutput(drive.fl);
    motorFR.setDesiredOutput(drive.fr);
    motorBL.setDesiredOutput(drive.bl);
    motorBR.setDesiredOutput(drive.br);
}

void Drivetrain::applyBeybladeSpin(DriveOutputs drive, float scale) {
    constexpr int32_t MAX_OUTPUT = 12000;
    const int32_t spinPower = static_cast<int32_t>(8000 * scale);

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
