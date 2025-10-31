#include "Drivetrain.hpp"

Drivetrain::Drivetrain(src::Drivers* _drivers, tap::communication::serial::Remote& remoteIn, double& _yaw)
    : drivers(_drivers), remote(remoteIn), yaw(_yaw) {}

void Drivetrain::initialize() {
    motorFL.emplace(drivers, chassis_fl, can_chassis, false, "motorFL");
    motorFR.emplace(drivers, chassis_fr, can_chassis, true,  "motorFR");
    motorBL.emplace(drivers, chassis_bl, can_chassis, false, "motorBL");
    motorBR.emplace(drivers, chassis_br, can_chassis, true,  "motorBR");

    for (auto* motor : {&*motorFL, &*motorFR, &*motorBL, &*motorBR})
        motor->initialize();

    mecanumFunc = [this]() { this->mecanumDrive(); };
    gimbalFunc  = [this]() { this->gimbalOrientedDrive(); };
    driveFunc   = mecanumFunc;
}

// --- internal helper ---
constexpr inline bool motorHealthy(int rpm, int out, int threshold) noexcept {
    return rpm > threshold || out < threshold;
}

bool Drivetrain::motorsHealthy() {
    constexpr int minRpmResponse = 50;
    return
        motorHealthy(std::abs(motorFL->getShaftRPM()), std::abs(pidFL.getOutput()), minRpmResponse) &&
        motorHealthy(std::abs(motorFR->getShaftRPM()), std::abs(pidFR.getOutput()), minRpmResponse) &&
        motorHealthy(std::abs(motorBL->getShaftRPM()), std::abs(pidBL.getOutput()), minRpmResponse) &&
        motorHealthy(std::abs(motorBR->getShaftRPM()), std::abs(pidBR.getOutput()), minRpmResponse);
}

void Drivetrain::mecanumDrive() {
    // Precompute combined factors once
    const float flFactor =  fwdInput +  strafeInput + turnInput;
    const float frFactor =  fwdInput -  strafeInput - turnInput;
    const float blFactor = -fwdInput -  strafeInput + turnInput;
    const float brFactor = -fwdInput +  strafeInput - turnInput;

    // Scale constant — compile-time if constexpr
    constexpr float MAX_RPM = 4000.0;

    // Compute desired RPM for each wheel
    const float flTarget = (flFactor * MAX_RPM) - motorFL->getShaftRPM();
    const float frTarget = (frFactor * MAX_RPM) - motorFR->getShaftRPM();
    const float blTarget = (blFactor * MAX_RPM) - motorBL->getShaftRPM();
    const float brTarget = (brFactor * MAX_RPM) - motorBR->getShaftRPM();

    // Run PID controllers directly
    pidFL.runControllerDerivateError(flTarget, 1);
    pidFR.runControllerDerivateError(frTarget, 1);
    pidBL.runControllerDerivateError(blTarget, 1);
    pidBR.runControllerDerivateError(brTarget, 1);
}

void Drivetrain::gimbalOrientedDrive() {
    const double gyroRadians = yaw * deg_to_rad;

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
    driveFunc = gimbalMode ? gimbalFunc : mecanumFunc;

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
    motorFL->setDesiredOutput(drive.fl);
    motorFR->setDesiredOutput(drive.fr);
    motorBL->setDesiredOutput(drive.bl);
    motorBR->setDesiredOutput(drive.br);
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