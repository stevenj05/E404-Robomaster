// #include <stdio.h>
// #include <iostream>

// #ifdef PLATFORM_HOSTED
// #include "tap/communication/tcp-server/tcp_server.hpp"
// #include "tap/motor/motorsim/dji_motor_sim_handler.hpp"
// #endif

// #include "refrenceHead.hpp"
// #include "modm/architecture/interface/clock.hpp"
// #include "stuffh.hpp"


// /* Constants --------------------------------------------------------*/
// static constexpr float MAIN_LOOP_FREQUENCY = 500.0f;
// static constexpr float PWM_FREQUENCY = 500.0f;
// constexpr float k_flywheelSpeed{7200};

// // CAN & Motor IDs
// static constexpr tap::can::CanBus CAN_BUS = tap::can::CanBus::CAN_BUS1;
// static constexpr tap::can::CanBus CAN_BUS2 = tap::can::CanBus::CAN_BUS2;
// static constexpr tap::motor::MotorId MOTOR_ID = tap::motor::MOTOR2;
// static constexpr tap::motor::MotorId MOTOR_ID2 = tap::motor::MOTOR3;
// static constexpr tap::motor::MotorId MOTOR_ID3 = tap::motor::MOTOR4;
// static constexpr tap::motor::MotorId MOTOR_ID4 = tap::motor::MOTOR1;
// static constexpr tap::motor::MotorId MOTOR_ID5 = tap::motor::MOTOR5;
// static constexpr tap::motor::MotorId MOTOR_ID6 = tap::motor::MOTOR6;
// static constexpr tap::motor::MotorId MOTOR_ID7 = tap::motor::MOTOR8;
// static constexpr tap::motor::MotorId MOTOR_ID8 = tap::motor::MOTOR2;
// static constexpr tap::motor::MotorId MOTOR_ID9 = tap::motor::MOTOR1;

// /* Timers -----------------------------------------------------------*/
// tap::arch::PeriodicMilliTimer sendMotorTimeout(1000.0f / MAIN_LOOP_FREQUENCY);
// tap::arch::PeriodicMilliTimer updateImuTimeout(2);

// /* PID Configs ------------------------------------------------------*/
// tap::algorithms::SmoothPid pidController1({55,1,5,0,16000,1,0,1,0});
// tap::algorithms::SmoothPid pidController2({10,1,1,0,8000,1,0,1,0});
// tap::algorithms::SmoothPid pidController3({10,1,1,0,8000,1,0,1,0});
// tap::algorithms::SmoothPid pidController4({10,1,1,0,8000,1,0,1,0});
// tap::algorithms::SmoothPid pidController5({10,1,1,0,8000,1,0,1,0});
// tap::algorithms::SmoothPid pidController6({10,1,1,0,8000,1,0,1,0});
// tap::algorithms::SmoothPid pidController7({35,1,4,0,16000,1,0,1,0});
// tap::algorithms::SmoothPid flywheel1Pid({20,0,0,100, tap::motor::DjiMotor::MAX_OUTPUT_C620,1,0,1,0});
// tap::algorithms::SmoothPid flywheel2Pid({20,0,0,100, tap::motor::DjiMotor::MAX_OUTPUT_C620,1,0,1,0});

// /* Motors -----------------------------------------------------------*/
// // tap::motor::DjiMotor motor(src::DoNotUse_getDrivers(), MOTOR_ID, CAN_BUS2, false, "motor1");
// // tap::motor::DjiMotor motor2(src::DoNotUse_getDrivers(), MOTOR_ID2, CAN_BUS2, true, "motor2");
// // tap::motor::DjiMotor motor3(src::DoNotUse_getDrivers(), MOTOR_ID3, CAN_BUS2, false, "motor3");
// // tap::motor::DjiMotor motor4(src::DoNotUse_getDrivers(), MOTOR_ID4, CAN_BUS2, true, "motor4");
// // tap::motor::DjiMotor motor5(src::DoNotUse_getDrivers(), MOTOR_ID5, CAN_BUS, true, "gimbalYaw");
// // tap::motor::DjiMotor motor6(src::DoNotUse_getDrivers(), MOTOR_ID6, CAN_BUS, true, "agitator");
// // tap::motor::DjiMotor motor7(src::DoNotUse_getDrivers(), MOTOR_ID7, CAN_BUS, true, "gimbalPitch");
// // tap::motor::DjiMotor flywheel1(src::DoNotUse_getDrivers(), MOTOR_ID2, CAN_BUS, false, "flywheel1");
// // tap::motor::DjiMotor flywheel2(src::DoNotUse_getDrivers(), MOTOR_ID, CAN_BUS, true, "flywheel2");

// /* Global control variables -----------------------------------------*/
// float flywheel1DesiredRPM{0}, flywheel2DesiredRPM{0};
// float gimbalYawTargetPos{0}, gimbalTargetPos{0};
// float k = 0; // agitator setpoint
// modm::PreciseClock theClock{};
// modm::chrono::micro_clock::time_point epoch;

// static void initializeIo(src::Drivers *drivers);
// static void updateIo(src::Drivers *drivers);
// void updateFlywheels(float deltaTime);

// int main_2()
// {
// #ifdef PLATFORM_HOSTED
//     std::cout << "Simulation starting...\n";
// #endif

//     src::Drivers *drivers = src::DoNotUse_getDrivers();
//     tap::communication::serial::Remote remote(drivers);

//     Board::initialize();
//     tap::motor::DjiMotor motor(drivers, MOTOR_ID, CAN_BUS2, false, "motor1");
//     tap::motor::DjiMotor motor2(drivers, MOTOR_ID2, CAN_BUS2, true, "motor2");
//     tap::motor::DjiMotor motor3(drivers, MOTOR_ID3, CAN_BUS2, false, "motor3");
//     tap::motor::DjiMotor motor4(drivers, MOTOR_ID4, CAN_BUS2, true, "motor4");
//     tap::motor::DjiMotor motor5(drivers, MOTOR_ID5, CAN_BUS, true, "gimbalYaw");
//     tap::motor::DjiMotor motor6(drivers, MOTOR_ID6, CAN_BUS, true, "agitator");
//     tap::motor::DjiMotor motor7(drivers, MOTOR_ID7, CAN_BUS, true, "gimbalPitch");
//     tap::motor::DjiMotor flywheel1(drivers, MOTOR_ID2, CAN_BUS, false, "flywheel1");
//     tap::motor::DjiMotor flywheel2(drivers, MOTOR_ID, CAN_BUS, true, "flywheel2");

//     motor.initialize(); motor2.initialize(); motor3.initialize(); motor4.initialize();
//     motor5.initialize(); motor6.initialize(); motor7.initialize();
//     flywheel1.initialize(); flywheel2.initialize();
//     remote.initialize();

//     motor5.resetEncoderValue();
//     drivers->bmi088.init(500.f, 0.1f, 0.0f);
//     initializeIo(drivers);
//     drivers->pwm.setTimerFrequency(tap::gpio::Pwm::Timer::TIMER8, PWM_FREQUENCY);

// #ifdef PLATFORM_HOSTED
//     tap::motor::motorsim::DjiMotorSimHandler::getInstance()->resetMotorSims();
//     tap::communication::TCPServer::MainServer()->getConnection();
// #endif

//     epoch = theClock.now();

//     while (1)
//     {
//         // Map keyboard/mouse to control channels
//         /**
//         if (left_switch == down)
//             kb/m logic
//         else (left_switch == mid)
//             joystick
//         else (left_switch == up)
//             disable or joystick + mouse
//          */

//         /*
//         left joystick = chassis translation
//         right joystick = turret movement
//         wheel = chassis rotation
//         left switch
//             down = kb/m
//             mid = controller (chassis oriented)
//             up = controller (turret oriented)
//         right switch
//             down = flywheel off
//             mid = flywheel on
//             up = flywheel on, agitator on
//         */
//         //remote.keyPressed(tap::communication::serial::Remote::Key::W)
//         float FWDJoy = (remote.keyPressed(tap::communication::serial::Remote::Key::W) ? +1.0f : 0.0f) - (remote.keyPressed(tap::communication::serial::Remote::Key::S) ? +1.0f : 0.0f);
//         float StrafeJoy = (remote.keyPressed(tap::communication::serial::Remote::Key::D) ? +1.0f : 0.0f)
//                         - (remote.keyPressed(tap::communication::serial::Remote::Key::A) ? +1.0f : 0.0f);
//         float Tturn = (remote.keyPressed(tap::communication::serial::Remote::Key::Q) ? -1.0f : 0.0f)
//                     + (remote.keyPressed(tap::communication::serial::Remote::Key::E) ? +1.0f : 0.0f);
//         bool shift = remote.keyPressed(tap::communication::serial::Remote::Key::SHIFT) ||
//                      remote.keyPressed(tap::communication::serial::Remote::Key::SHIFT);
//         // bool leftClick = mouse.isButtonDown(tap::input::MouseButton::LEFT);
//         bool leftClick = remote.getMouseL();
//         bool rightClick = remote.getMouseR();
//         // float mouse_dx = mouse.getDeltaX();
//         int16_t mouse_dx = remote.getMouseX();
//         int16_t mouse_dy = remote.getMouseY();

//         remote.read();
//         PROFILE(drivers->profiler, updateIo, (drivers));
//         drivers->bmi088.read();

//         if (updateImuTimeout.execute()) {
//             drivers->bmi088.periodicIMUUpdate();
//         }

//         if (sendMotorTimeout.execute())
//         {
//             drivers->bmi088.periodicIMUUpdate();
//             drivers->commandScheduler.run();
//             drivers->djiMotorTxHandler.encodeAndSendCanData();
//             drivers->terminalSerial.update();

//             float rotation = 0.0f;
//             if (shift) {
//                 rotation = 1.0f;
//                 motor5.setDesiredOutput(14800 + Tturn * 4000);
//             } else {
//                 // gimbal yaw via mouse
//                 gimbalYawTargetPos += mouse_dx * 50.0f; // tune sensitivity
//             }

//             // Drive PID loop
//             float dm[4] = {(FWDJoy + StrafeJoy + Tturn + rotation),
//                            (FWDJoy - StrafeJoy - Tturn - rotation),
//                            (-FWDJoy - StrafeJoy + Tturn + rotation),
//                            (-FWDJoy + StrafeJoy - Tturn - rotation)};
//             tap::algorithms::SmoothPid *dpids[4] = {&pidController3, &pidController4,
//                                                     &pidController5, &pidController6};
//             tap::motor::DjiMotor *dmotors[4] = {&motor, &motor2, &motor3, &motor4};
//             for (int i = 0; i < 4; ++i) {
//                 dpids[i]->runControllerDerivateError(dm[i] * 4000 - dmotors[i]->getShaftRPM(), 1);
//                 dmotors[i]->setDesiredOutput((int32_t)dpids[i]->getOutput());
//             }

//             // Gimbal pitch via mouse Y
//             gimbalTargetPos += mouse_dy * -6.0f;
//             pidController1.runControllerDerivateError(gimbalTargetPos - motor7.getEncoderUnwrapped(), 1);
//             motor7.setDesiredOutput((int32_t)pidController1.getOutput());

//             // Agitator via mouse
//             if (leftClick)       k = +40000;
//             else if (rightClick) k = -3200;
//             else                 k = 0;
//             pidController2.runControllerDerivateError(k - motor6.getShaftRPM(), 1);
//             motor6.setDesiredOutput((int32_t)pidController2.getOutput());

//             // Flywheel control via switch (still using remote)
//             if (remote.getSwitch(tap::communication::serial::Remote::Switch::LEFT_SWITCH) ==
//                 tap::communication::serial::Remote::SwitchState::MID)
//             {
//                 flywheel1DesiredRPM = flywheel2DesiredRPM = k_flywheelSpeed;
//             }
//             else if (remote.getSwitch(tap::communication::serial::Remote::Switch::LEFT_SWITCH) ==
//                      tap::communication::serial::Remote::SwitchState::DOWN)
//             {
//                 flywheel1DesiredRPM = flywheel2DesiredRPM = 0.0f;
//             }

//             drivers->djiMotorTxHandler.encodeAndSendCanData();
//         }

//         drivers->canRxHandler.pollCanData();
//         modm::chrono::micro_clock::time_point now = theClock.now();
//         updateFlywheels((now - epoch).count());
//         modm::delay_us(100);
//     }

//     return 0;
// }

// void updateFlywheels(float deltaTime, tap::motor::DjiMotor &flywheel1, tap::motor::DjiMotor &flywheel2)
// {
//     auto control = [&](tap::motor::DjiMotor &fw, tap::algorithms::SmoothPid &pid, float target){
//         double err = target - fw.getShaftRPM();
//         double derr = pid.runControllerDerivateError(err, deltaTime);
//         double out = pid.runController(err, derr, deltaTime);
//         fw.setDesiredOutput((int32_t)out);
//     };
//     control(flywheel1, flywheel1Pid, flywheel1DesiredRPM);
//     control(flywheel2, flywheel2Pid, flywheel2DesiredRPM);
// }

// static void initializeIo(src::Drivers *d)
// {
//     d->analog.init(); d->pwm.init(); d->digital.init(); d->leds.init();
//     d->can.initialize(); d->errorController.init(); d->remote.initialize();
//     d->bmi088.init(MAIN_LOOP_FREQUENCY, 0.1, 0);
//     d->refSerial.initialize(); d->terminalSerial.initialize();
//     d->schedulerTerminalHandler.init();
//     d->djiMotorTerminalSerialHandler.init();
// }

// static void updateIo(src::Drivers *d)
// {
// #ifdef PLATFORM_HOSTED
//     tap::motor::motorsim::DjiMotorSimHandler::getInstance()->updateSims();
// #endif
//     d->canRxHandler.pollCanData();
//     d->refSerial.updateSerial();
//     d->remote.read();
//     d->bmi088.read();
// }
