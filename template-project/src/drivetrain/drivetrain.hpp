/*#pragma once
#include "drivers.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/algorithms/smooth_pid.hpp"

namespace robot {

struct DrivetrainConfig {
    // Motor IDs and inversions
    tap::motor::MotorId mFL, mFR, mBL, mBR;
    bool invFL{false}, invFR{true}, invBL{false}, invBR{true};
    tap::can::CanBus bus{tap::can::CanBus::CAN_BUS2};

    // Geometry (meters)
    float wheelRadius{0.048f};
    float Lx{0.14f};   // half front-back distance
    float Ly{0.14f};   // half left-right distance

    // Limits
    float maxWheelRpm{4500.0f};

    // PID config (start P-only)
    tap::algorithms::SmoothPidConfig pidCfg{10, 0, 0, 0, 8000, 1, 0, 1, 0};
};

// Factory with your default IDs & signs
DrivetrainConfig makeDefaultDrivetrainConfig();

class Drivetrain {
public:
    Drivetrain(src::Drivers* drivers, const DrivetrainConfig& cfg);

    void init();                                  // initialize motors
    void setCommand(float vx, float vy, float w); // store targets (m/s, m/s, rad/s)
    void update(float dt_s);                      // run PIDs toward targets
    void stop();                                  // zero outputs

private:
    src::Drivers* d_;
    DrivetrainConfig c_;

    tap::motor::DjiMotor fl_, fr_, bl_, br_;
    tap::algorithms::SmoothPid pidFL_, pidFR_, pidBL_, pidBR_;

    float tgtFlRpm_{0}, tgtFrRpm_{0}, tgtBlRpm_{0}, tgtBrRpm_{0};

    static float linearToRpm(float v_mps, float radius_m);
    static float clampAbs(float x, float limit);
};

} // namespace robot
*/