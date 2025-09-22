#pragma once

#include "referenceHead.hpp"

namespace Constants {

    static constexpr float MAIN_LOOP_FREQUENCY = 500.0f;
    static constexpr float PWM_FREQUENCY       = 500.0f;

    static constexpr tap::can::CanBus CAN_BUS1 = tap::can::CanBus::CAN_BUS1;
    static constexpr tap::can::CanBus CAN_BUS2 = tap::can::CanBus::CAN_BUS2;

    static constexpr tap::motor::MotorId M_ID1 = tap::motor::MOTOR2;
    static constexpr tap::motor::MotorId M_ID2 = tap::motor::MOTOR3;
    static constexpr tap::motor::MotorId M_ID3 = tap::motor::MOTOR4;
    static constexpr tap::motor::MotorId M_ID4 = tap::motor::MOTOR1;
    static constexpr tap::motor::MotorId M_ID5 = tap::motor::MOTOR5;
    static constexpr tap::motor::MotorId M_ID6 = tap::motor::MOTOR6;
    static constexpr tap::motor::MotorId M_ID7 = tap::motor::MOTOR8;
    static constexpr tap::motor::MotorId M_ID8 = tap::motor::MOTOR2;
    static constexpr tap::motor::MotorId M_ID9 = tap::motor::MOTOR1;

    // Drive train: motors 1–4
    // Motor 6: 2006 agitator
    // Motors 5 & 7: Yaw and Pitch 6020
    constexpr float kFlywheelSpeed{7200};


    // -----------------------------------------------------------------------------
    // PID Configurations
    // -----------------------------------------------------------------------------
    static constexpr tap::algorithms::SmoothPidConfig pidConfig1(100, 1, 20, 0, 12000, 1, 0, 1, 0);
    static constexpr tap::algorithms::SmoothPidConfig pidConfig2(10, 1, 1, 0, 8000, 1, 0, 1, 0);
    static constexpr tap::algorithms::SmoothPidConfig pidConfig3(10, 1, 1, 0, 8000, 1, 0, 1, 0);
    static constexpr tap::algorithms::SmoothPidConfig pidConfig4(10, 1, 1, 0, 8000, 1, 0, 1, 0);
    static constexpr tap::algorithms::SmoothPidConfig pidConfig5(10, 1, 1, 0, 8000, 1, 0, 1, 0);
    static constexpr tap::algorithms::SmoothPidConfig pidConfig6(10, 1, 1, 0, 8000, 1, 0, 1, 0);

    static const tap::algorithms::SmoothPidConfig flywheelPidConfig1{
        20, 0, 0, 100, tap::motor::DjiMotor::MAX_OUTPUT_C620, 1, 0, 1, 0};
    static const tap::algorithms::SmoothPidConfig flywheelPidConfig2{
        20, 0, 0, 100, tap::motor::DjiMotor::MAX_OUTPUT_C620, 1, 0, 1, 0};

    // PID Controllers
    static tap::algorithms::SmoothPid pidController1(pidConfig1);
    static tap::algorithms::SmoothPid pidController2(pidConfig2);
    static tap::algorithms::SmoothPid pidController3(pidConfig3);
    static tap::algorithms::SmoothPid pidController4(pidConfig4);
    static tap::algorithms::SmoothPid pidController5(pidConfig5);
    static tap::algorithms::SmoothPid pidController6(pidConfig6);

    static tap::algorithms::SmoothPid flywheel1Pid(flywheelPidConfig1);
    static tap::algorithms::SmoothPid flywheel2Pid(flywheelPidConfig2);

    // -----------------------------------------------------------------------------
    // Motors
    // -----------------------------------------------------------------------------
    
    static tap::motor::DjiMotor motor(src::DoNotUse_getDrivers(), M_ID1,  CAN_BUS2, false, "motor1");
    static tap::motor::DjiMotor motor2(src::DoNotUse_getDrivers(), M_ID2, CAN_BUS2, true,  "motor2");
    static tap::motor::DjiMotor motor3(src::DoNotUse_getDrivers(), M_ID3, CAN_BUS2, false, "motor3");
    static tap::motor::DjiMotor motor4(src::DoNotUse_getDrivers(), M_ID4, CAN_BUS2, true,  "motor4");
    static tap::motor::DjiMotor motor5(src::DoNotUse_getDrivers(), M_ID5, CAN_BUS1,  true,  "motor5");
    static tap::motor::DjiMotor motor6(src::DoNotUse_getDrivers(), M_ID6, CAN_BUS1,  true,  "motor6");
    static tap::motor::DjiMotor motor7(src::DoNotUse_getDrivers(), M_ID7, CAN_BUS1,  true,  "motor7");
    static tap::motor::DjiMotor flywheel1(src::DoNotUse_getDrivers(), M_ID2, CAN_BUS1, false, "flywheel1");
    static tap::motor::DjiMotor flywheel2(src::DoNotUse_getDrivers(), M_ID1,  CAN_BUS1, false, "flywheel2");

}