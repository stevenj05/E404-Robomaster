#include "Drivetrain.hpp"

Drivetrain::Drivetrain(tap::communication::serial::Remote& remoteIn)
: remote(remoteIn) {}

void Drivetrain::initialize() {
    motorFL.initialize();
    motorFR.initialize();
    motorBL.initialize();
    motorBR.initialize();

    lastToggleTime   = modm::chrono::milli_clock::now();
}

// --- internal helper ---
bool Drivetrain::motorsHealthy() {
    constexpr int minRpmResponse = 50; // acceptable difference

    bool flOk = std::abs(motorFL.getShaftRPM()) > minRpmResponse || std::abs(pidFL.getOutput()) < minRpmResponse;
    bool frOk = std::abs(motorFR.getShaftRPM()) > minRpmResponse || std::abs(pidFR.getOutput()) < minRpmResponse;
    bool blOk = std::abs(motorBL.getShaftRPM()) > minRpmResponse || std::abs(pidBL.getOutput()) < minRpmResponse;
    bool brOk = std::abs(motorBR.getShaftRPM()) > minRpmResponse || std::abs(pidBR.getOutput()) < minRpmResponse;

    return (flOk && frOk && blOk && brOk);
}

void Drivetrain::update() {
    fwdInput    = remote.getChannel(tap::communication::serial::Remote::Channel::LEFT_VERTICAL);
    strafeInput = remote.getChannel(tap::communication::serial::Remote::Channel::LEFT_HORIZONTAL);
    turnInput   = remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_HORIZONTAL);

    auto now = modm::chrono::milli_clock::now();
    bool switchDown = (remote.getSwitch(tap::communication::serial::Remote::Switch::RIGHT_SWITCH)
                       == tap::communication::serial::Remote::SwitchState::DOWN);

    if (switchDown && (now - lastToggleTime).count() > 250) {
        beybladeMode = !beybladeMode;
        lastToggleTime = now;
    }

    // Update PID controllers
    pidFL.runControllerDerivateError(((fwdInput + strafeInput + turnInput) * 4000) - motorFL.getShaftRPM(), 1);
    pidFR.runControllerDerivateError(((fwdInput - strafeInput - turnInput) * 4000) - motorFR.getShaftRPM(), 1);
    pidBL.runControllerDerivateError(((-fwdInput - strafeInput + turnInput) * 4000) - motorBL.getShaftRPM(), 1);
    pidBR.runControllerDerivateError(((-fwdInput + strafeInput - turnInput) * 4000) - motorBR.getShaftRPM(), 1);

    // --- health-driven scaling ---
    if (motorsHealthy()) {
        // Gradually restore scale back toward 1.0
        safetyScale += 0.05f;
        if (safetyScale > 1.0f) safetyScale = 1.0f;
    } else {
        // Gradually reduce scale toward minimum safe output
        safetyScale -= 0.05f;
        if (safetyScale < 0.2f) safetyScale = 0.2f; // never fully off
    }
}

void Drivetrain::tick(float scale) {
    // Apply safety scaling directly on the provided scale
    scale *= safetyScale;

    if (beybladeMode) {
        motorFL.setDesiredOutput(12000 * scale);
        motorFR.setDesiredOutput(-12000 * scale);
        motorBL.setDesiredOutput(12000 * scale);
        motorBR.setDesiredOutput(-12000 * scale);
    } else {
        motorFL.setDesiredOutput(static_cast<int32_t>(pidFL.getOutput() * scale));
        motorFR.setDesiredOutput(static_cast<int32_t>(pidFR.getOutput() * scale));
        motorBL.setDesiredOutput(static_cast<int32_t>(pidBL.getOutput() * scale));
        motorBR.setDesiredOutput(static_cast<int32_t>(pidBR.getOutput() * scale));
    }
}
