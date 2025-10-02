#pragma once

#include "tap/motor/dji_motor.hpp"
#include "tap/algorithms/smooth_pid.hpp"
#include "drivers_singleton.hpp"

namespace Constants {
    static constexpr float MAIN_LOOP_FREQUENCY = 500.0f;
    static constexpr float PWM_FREQUENCY = 500.0f;
    static constexpr tap::can::CanBus CAN_BUS1 = tap::can::CanBus::CAN_BUS1;
    static constexpr tap::can::CanBus CAN_BUS2 = tap::can::CanBus::CAN_BUS2;
    
    // Motor IDs
    static constexpr tap::motor::MotorId M_ID1 = tap::motor::MOTOR2;
    static constexpr tap::motor::MotorId M_ID2 = tap::motor::MOTOR3;
    static constexpr tap::motor::MotorId M_ID3 = tap::motor::MOTOR4;
    static constexpr tap::motor::MotorId M_ID4 = tap::motor::MOTOR1;

    static constexpr tap::motor::MotorId M_ID5 = tap::motor::MOTOR5;
    static constexpr tap::motor::MotorId M_ID6 = tap::motor::MOTOR6;
    
    static constexpr tap::motor::MotorId M_ID7 = tap::motor::MOTOR8;
    static constexpr tap::motor::MotorId M_ID8 = tap::motor::MOTOR2;
    static constexpr tap::motor::MotorId M_ID9 = tap::motor::MOTOR1;
    
    constexpr float kFlywheelSpeed{7200};
    
    // PID Configuration constants
    static constexpr tap::algorithms::SmoothPidConfig pidConfig1(100, 1, 20, 0, 12000, 1, 0, 1, 0);
    static constexpr tap::algorithms::SmoothPidConfig pidConfig2(10, 1, 1, 0, 8000, 1, 0, 1, 0);
    static constexpr tap::algorithms::SmoothPidConfig pidConfig3(10, 1, 1, 0, 8000, 1, 0, 1, 0);
    static constexpr tap::algorithms::SmoothPidConfig pidConfig4(10, 1, 1, 0, 8000, 1, 0, 1, 0);
    static constexpr tap::algorithms::SmoothPidConfig pidConfig5(10, 1, 1, 0, 8000, 1, 0, 1, 0);
    static constexpr tap::algorithms::SmoothPidConfig pidConfig6(10, 1, 1, 0, 8000, 1, 0, 1, 0);

    static constexpr tap::algorithms::SmoothPidConfig flywheelPidConfig1{
        20, 0, 0, 100, tap::motor::DjiMotor::MAX_OUTPUT_C620, 1, 0, 1, 0};
    static constexpr tap::algorithms::SmoothPidConfig flywheelPidConfig2{
        20, 0, 0, 100, tap::motor::DjiMotor::MAX_OUTPUT_C620, 1, 0, 1, 0};
}