#include "drivetrain/Drivetrain.hpp"
#include <cmath>
#include <algorithm>

namespace robot {

// Make a default configuration for the drivetrain (motors, geometry, PID gains, limits)
DrivetrainConfig makeDefaultDrivetrainConfig() {
    DrivetrainConfig c{};
    c.mFL = tap::motor::MOTOR2; // front left motor ID
    c.mFR = tap::motor::MOTOR3; // front right
    c.mBL = tap::motor::MOTOR4; // back left
    c.mBR = tap::motor::MOTOR1; // back right

    c.invFL = false; // motor direction sign
    c.invFR = true;
    c.invBL = false;
    c.invBR = true;

    c.bus = tap::can::CanBus::CAN_BUS2; // all on CAN2

    c.wheelRadius = 0.048f; // meters
    c.Lx = 0.14f;           // half front-back length
    c.Ly = 0.14f;           // half left-right length
    c.maxWheelRpm = 4500.0f;

    // P-only PID (Kp = 10) to start safe
    c.pidCfg = tap::algorithms::SmoothPidConfig(10, 0, 0, 0, 8000, 1, 0, 1, 0);
    return c;
}

// Helper to create a DjiMotor object with given settings
static tap::motor::DjiMotor makeMotor(src::Drivers* d,
                                      tap::motor::MotorId id,
                                      tap::can::CanBus bus,
                                      bool inverted,
                                      const char* name)
{
    return tap::motor::DjiMotor(d, id, bus, inverted, name);
}

// Constructor: build 4 motors + 4 PID controllers from config
Drivetrain::Drivetrain(src::Drivers* drivers, const DrivetrainConfig& cfg)
: d_(drivers), c_(cfg),
  fl_(makeMotor(drivers, cfg.mFL, cfg.bus, cfg.invFL, "FL")),
  fr_(makeMotor(drivers, cfg.mFR, cfg.bus, cfg.invFR, "FR")),
  bl_(makeMotor(drivers, cfg.mBL, cfg.bus, cfg.invBL, "BL")),
  br_(makeMotor(drivers, cfg.mBR, cfg.bus, cfg.invBR, "BR")),
  pidFL_(cfg.pidCfg), pidFR_(cfg.pidCfg), pidBL_(cfg.pidCfg), pidBR_(cfg.pidCfg)
{}

// Initialize all motors
void Drivetrain::init() {
    fl_.initialize();
    fr_.initialize();
    bl_.initialize();
    br_.initialize();
}

// Take desired robot speeds (vx, vy, w) → compute each wheel’s target RPM
void Drivetrain::setCommand(float vx, float vy, float w) {
    const float k = c_.Lx + c_.Ly; // rotational lever arm
    const float v_fl =  (vx - vy - k * w);
    const float v_fr =  (vx + vy + k * w);
    const float v_bl =  (vx + vy - k * w);
    const float v_br =  (vx - vy + k * w);

    // Convert linear speeds → RPM, clamp to safe limits
    tgtFlRpm_ = clampAbs(linearToRpm(v_fl, c_.wheelRadius), c_.maxWheelRpm);
    tgtFrRpm_ = clampAbs(linearToRpm(v_fr, c_.wheelRadius), c_.maxWheelRpm);
    tgtBlRpm_ = clampAbs(linearToRpm(v_bl, c_.wheelRadius), c_.maxWheelRpm);
    tgtBrRpm_ = clampAbs(linearToRpm(v_br, c_.wheelRadius), c_.maxWheelRpm);
}

// Run PIDs each loop: measure error, compute output, send to motors
void Drivetrain::update(float dt_s) {
    const float eFL = tgtFlRpm_ - fl_.getShaftRPM();
    const float eFR = tgtFrRpm_ - fr_.getShaftRPM();
    const float eBL = tgtBlRpm_ - bl_.getShaftRPM();
    const float eBR = tgtBrRpm_ - br_.getShaftRPM();

    pidFL_.runControllerDerivateError(eFL, dt_s);
    pidFR_.runControllerDerivateError(eFR, dt_s);
    pidBL_.runControllerDerivateError(eBL, dt_s);
    pidBR_.runControllerDerivateError(eBR, dt_s);

    fl_.setDesiredOutput(static_cast<int32_t>(pidFL_.getOutput()));
    fr_.setDesiredOutput(static_cast<int32_t>(pidFR_.getOutput()));
    bl_.setDesiredOutput(static_cast<int32_t>(pidBL_.getOutput()));
    br_.setDesiredOutput(static_cast<int32_t>(pidBR_.getOutput()));
}

// Immediately stop all motors
void Drivetrain::stop() {
    tgtFlRpm_ = tgtFrRpm_ = tgtBlRpm_ = tgtBrRpm_ = 0.0f;
    fl_.setDesiredOutput(0);
    fr_.setDesiredOutput(0);
    bl_.setDesiredOutput(0);
    br_.setDesiredOutput(0);
}

// Helper: convert linear velocity (m/s) → wheel RPM
float Drivetrain::linearToRpm(float v_mps, float radius_m) {
    return (v_mps / (2.0f * static_cast<float>(M_PI) * radius_m)) * 60.0f;
}

// Helper: clamp value to ±limit
float Drivetrain::clampAbs(float x, float limit) {
    if (x >  limit) return  limit;
    if (x < -limit) return -limit;
    return x;
}

} // namespace robot
