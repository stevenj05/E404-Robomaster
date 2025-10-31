#pragma once
/**
 * Global robot-wide constants.
 * All values are constexpr so they are compile-time known and shared across subsystems.
 */
#include "tap/motor/dji_motor.hpp"
#include "tap/algorithms/smooth_pid.hpp"
#include <numbers>

namespace Constants {

// -- control loop & timing ----------------------------------------------------
static constexpr float main_loop_freq_hz = 500.0f;           // target control loop frequency
static constexpr float main_loop_dt_sec  = 1.0f / main_loop_freq_hz; // Δt for PID controllers

static constexpr float pwm_freq_hz = 500.0f;                 // PWM output frequency

// -- can bus assignment -------------------------------------------------------
static constexpr tap::can::CanBus can_chassis = tap::can::CanBus::CAN_BUS2; // drivetrain motors
static constexpr tap::can::CanBus can_turret  = tap::can::CanBus::CAN_BUS1; // gimbal + flywheels

// -- motor id map -------------------------------------------------------------
/*
 * Logical motor names → actual DJI CAN IDs.
 * can_chassis (CAN_BUS2):
 *   MOTOR2 → front-left
 *   MOTOR3 → front-right
 *   MOTOR4 → back-left
 *   MOTOR1 → back-right
 * can_turret (CAN_BUS1):
 *   MOTOR6 → pitch
 *   MOTOR8 → yaw
 *   MOTOR2 → flywheel #1 (left)
 *   MOTOR1 → flywheel #2 (right)
 */
static constexpr tap::motor::MotorId chassis_fl = tap::motor::MOTOR2;
static constexpr tap::motor::MotorId chassis_fr = tap::motor::MOTOR3;
static constexpr tap::motor::MotorId chassis_bl = tap::motor::MOTOR4;
static constexpr tap::motor::MotorId chassis_br = tap::motor::MOTOR1;

static constexpr tap::motor::MotorId gimbal_pitch = tap::motor::MOTOR6;
static constexpr tap::motor::MotorId gimbal_yaw   = tap::motor::MOTOR8;

static constexpr tap::motor::MotorId flywheel_1 = tap::motor::MOTOR2;
static constexpr tap::motor::MotorId flywheel_2 = tap::motor::MOTOR1;

// -- remote input scaling -----------------------------------------------------
static constexpr float remote_channel_max = 660.0f;   // raw max from DJI remote (±660)
static constexpr float remote_deadzone    = 30.0f;    // ignore inputs below this

// -- drivetrain parameters ----------------------------------------------------
static constexpr float chassis_max_rpm = 4000.0f;     // desired max wheel speed

// -- flywheel parameters ------------------------------------------------------
static constexpr float flywheel_rpm_full    = 7200.0f;  // full-speed shooting
static constexpr float flywheel_rpm_stop    = 0.0f;     // stopped
static constexpr float flywheel_rpm_reverse = -4000.0f; // reverse for un-jamming

// -- gimbal sensitivity -------------------------------------------------------
static constexpr float gimbal_pitch_sensitivity_deg_per_sec = 180.0f; // max pitch speed
static constexpr float gimbal_yaw_sensitivity_deg_per_sec   = 270.0f; // max yaw speed

// -- pid configurations -------------------------------------------------------
/*
 * SmoothPidConfig: kp, ki, kd, maxI, maxOut, iZone, wrap, derivativeFilter, outputFilter
 */

// chassis wheels
static constexpr tap::algorithms::SmoothPidConfig chassis_pid_fl{
    100.0f, 1.0f, 20.0f, 0.0f, 12000.0f, 1.0f, 0, 1, 0};

static constexpr tap::algorithms::SmoothPidConfig chassis_pid_fr{
    10.0f, 1.0f, 1.0f, 0.0f, 8000.0f, 1.0f, 0, 1, 0};

static constexpr tap::algorithms::SmoothPidConfig chassis_pid_bl{
    10.0f, 1.0f, 1.0f, 0.0f, 8000.0f, 1.0f, 0, 1, 0};

static constexpr tap::algorithms::SmoothPidConfig chassis_pid_br{
    10.0f, 1.0f, 1.0f, 0.0f, 8000.0f, 1.0f, 0, 1, 0};

// gimbal
static constexpr tap::algorithms::SmoothPidConfig gimbal_pid_pitch{
    10.0f, 1.0f, 1.0f, 0.0f, 8000.0f, 1.0f, 0, 1, 0};

static constexpr tap::algorithms::SmoothPidConfig gimbal_pid_yaw{
    10.0f, 1.0f, 1.0f, 0.0f, 8000.0f, 1.0f, 0, 1, 0};

// flywheels
static constexpr tap::algorithms::SmoothPidConfig flywheel_pid_1{
    20.0f, 0.0f, 0.0f, 100.0f, tap::motor::DjiMotor::MAX_OUTPUT_C620, 1.0f, 0, 1, 0};

static constexpr tap::algorithms::SmoothPidConfig flywheel_pid_2{
    20.0f, 0.0f, 0.0f, 100.0f, tap::motor::DjiMotor::MAX_OUTPUT_C620, 1.0f, 0, 1, 0};

// -- math helpers -------------------------------------------------------------
constexpr float deg_to_rad = std::numbers::pi_v<float> / 180.0f;

} // namespace constants