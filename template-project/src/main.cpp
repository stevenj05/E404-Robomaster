#include <stdio.h>

#include <iostream>

#ifdef PLATFORM_HOSTED
/* hosted environment (simulator) includes --------------------------------- */

#include "tap/communication/tcp-server/tcp_server.hpp"
#include "tap/motor/motorsim/dji_motor_sim_handler.hpp"

#endif

#include "refrenceHead.hpp"
#include "modm/architecture/interface/clock.hpp"

/* Our Headers -------------------------------------------------------*/
#include "stuffh.hpp"

/* define timers here -------------------------------------------------------*/

static constexpr float MAIN_LOOP_FREQUENCY = 500.0f;

static constexpr float PWM_FREQUENCY = 500.0f;

static constexpr tap::can::CanBus CAN_BUS = tap::can::CanBus::CAN_BUS1;
static constexpr tap::can::CanBus CAN_BUS2 = tap::can::CanBus::CAN_BUS2;

static constexpr tap::motor::MotorId MOTOR_ID = tap::motor::MOTOR2;

static constexpr tap::motor::MotorId MOTOR_ID2 = tap::motor::MOTOR3;

static constexpr tap::motor::MotorId MOTOR_ID3 = tap::motor::MOTOR4;

static constexpr tap::motor::MotorId MOTOR_ID4 = tap::motor::MOTOR1;

static constexpr tap::motor::MotorId MOTOR_ID5 = tap::motor::MOTOR5;

static constexpr tap::motor::MotorId MOTOR_ID6 = tap::motor::MOTOR6;

static constexpr tap::motor::MotorId MOTOR_ID7 = tap::motor::MOTOR8;

static constexpr tap::motor::MotorId MOTOR_ID8 = tap::motor::MOTOR2;

static constexpr tap::motor::MotorId MOTOR_ID9 = tap::motor::MOTOR1;

tap::arch::PeriodicMilliTimer sendMotorTimeout(1000.0f / MAIN_LOOP_FREQUENCY);
tap::arch::PeriodicMilliTimer updateImuTimeout(2);

//motor1-4:Drive train motors | motor 6: 2006 Agitator | motor 5&7: Yaw and Pitch 6020

constexpr float k_flywheelSpeed{7200};

modm::PreciseClock theClock{};
modm::chrono::micro_clock::time_point epoch;
using MicrosecondDuration = modm::PreciseClock::duration;
// Place any sort of input/output initialization here. For example, place
// serial init stuff here.
static void initializeIo(src::Drivers *drivers);

// Anything that you would like to be called place here. It will be called
// very frequently. Use PeriodicMilliTimers if you don't want something to be
// called as frequently.
static void updateIo(src::Drivers *drivers);
void updateFlywheels(float deltaTime, tap::motor::DjiMotor &flywheel1, tap::motor::DjiMotor &flywheel2);

tap::algorithms::SmoothPidConfig SmoothpidConfig1(28, 0, 4.5, 0, 19000, 1, 0, 1, 0);
tap::algorithms::SmoothPidConfig SmoothpidConfig2(10, 1, 1, 0, 8000, 1, 0, 1, 0);
tap::algorithms::SmoothPidConfig SmoothpidConfig3(10, 1, 1, 0, 8000, 1, 0, 1, 0);
tap::algorithms::SmoothPidConfig SmoothpidConfig4(10, 1, 1, 0, 8000, 1, 0, 1, 0);
tap::algorithms::SmoothPidConfig SmoothpidConfig5(10, 1, 1, 0, 8000, 1, 0, 1, 0);
tap::algorithms::SmoothPidConfig SmoothpidConfig6(10, 1, 1, 0, 8000, 1, 0, 1, 0);
tap::algorithms::SmoothPidConfig SmoothpidConfig7(10, 0, 1.2, 0, 10000, 1, 0, 1, 0);
tap::algorithms::SmoothPidConfig flywheel1PidConfig{20, 0, 0, 100, tap::motor::DjiMotor::MAX_OUTPUT_C620, 1, 0, 1, 0};//PID tuning for flywheels
tap::algorithms::SmoothPidConfig flywheel2PidConfig{20, 0, 0, 100, tap::motor::DjiMotor::MAX_OUTPUT_C620, 1, 0, 1, 0};//PID tuning for flywheels
tap::algorithms::SmoothPid pidController1(SmoothpidConfig1);
tap::algorithms::SmoothPid pidController2(SmoothpidConfig2);
tap::algorithms::SmoothPid pidController3(SmoothpidConfig3);
tap::algorithms::SmoothPid pidController4(SmoothpidConfig4);
tap::algorithms::SmoothPid pidController5(SmoothpidConfig5);
tap::algorithms::SmoothPid pidController6(SmoothpidConfig6);
tap::algorithms::SmoothPid pidController7(SmoothpidConfig7);
tap::algorithms::SmoothPid flywheel1Pid(flywheel1PidConfig);
tap::algorithms::SmoothPid flywheel2Pid(flywheel2PidConfig);

// tap::motor::DjiMotor motor(src::DoNotUse_getDrivers(), MOTOR_ID, CAN_BUS2, false, "cool motor");
// tap::motor::DjiMotor motor2(src::DoNotUse_getDrivers(), MOTOR_ID2, CAN_BUS2, true, "cool motor");
// tap::motor::DjiMotor motor3(src::DoNotUse_getDrivers(), MOTOR_ID3, CAN_BUS2, false, "cool motor");
// tap::motor::DjiMotor motor4(src::DoNotUse_getDrivers(), MOTOR_ID4, CAN_BUS2, true, "cool motor");
// tap::motor::DjiMotor motor5(src::DoNotUse_getDrivers(), MOTOR_ID5, CAN_BUS, true, "cool motor");
// tap::motor::DjiMotor motor6(src::DoNotUse_getDrivers(), MOTOR_ID6, CAN_BUS, true, "cool motor");
// tap::motor::DjiMotor motor7(src::DoNotUse_getDrivers(), MOTOR_ID7, CAN_BUS, true, "cool motor");
// tap::motor::DjiMotor flywheel1(src::DoNotUse_getDrivers(), MOTOR_ID2, CAN_BUS, false, "cool motor");
// tap::motor::DjiMotor flywheel2(src::DoNotUse_getDrivers(), MOTOR_ID, CAN_BUS, true, "cool motor");

float flywheel1DesiredRPM{0.0f};
float flywheel2DesiredRPM{0.0f};

float heading, move, MotorA, MotorB, MotorC, MotorD, yaw, HPower;

float FWDJoy, StrafeJoy, TXJoy, TYJoy, Tturn, k;
bool done = false, f = true, d = true, beybladeMode = false, leftClick = false, rightClick = false, pitchInit = false;
int16_t mouse_dx = 0, mouse_dy = 0;
float DESIRED_RPM, DESIRED_RPM2, DESIRED_RPM3, DESIRED_RPM4,gimbalYawTargetPos, gimbalTargetPos;

void joystick_control(tap::communication::serial::Remote &remote) {
    FWDJoy = remote.getChannel(tap::communication::serial::Remote::Channel::LEFT_VERTICAL);
    StrafeJoy = remote.getChannel(tap::communication::serial::Remote::Channel::LEFT_HORIZONTAL);
    TXJoy = remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_HORIZONTAL);
    TYJoy = remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_VERTICAL);
    Tturn = remote.getChannel(tap::communication::serial::Remote::Channel::WHEEL);
}

template<typename T>
T clamp(T value, T min_val, T max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

void keyboard_control(tap::communication::serial::Remote &remote, tap::motor::DjiMotor*pitchMotor) {
    FWDJoy = (remote.keyPressed(tap::communication::serial::Remote::Key::W) ? +0.5f : 0.0f) - (remote.keyPressed(tap::communication::serial::Remote::Key::S) ? +0.5f : 0.0f);
    StrafeJoy = (remote.keyPressed(tap::communication::serial::Remote::Key::E) ? +0.5f : 0.0f)
                    - (remote.keyPressed(tap::communication::serial::Remote::Key::Q) ? +0.5f : 0.0f);
    TXJoy = (remote.keyPressed(tap::communication::serial::Remote::Key::D) ? +0.5f : 0.0f)
                    - (remote.keyPressed(tap::communication::serial::Remote::Key::A) ? +0.5f : 0.0f);
    // Tturn = (remote.keyPressed(tap::communication::serial::Remote::Key::Q) ? -1.0f : 0.0f)
    //             + (remote.keyPressed(tap::communication::serial::Remote::Key::E) ? +1.0f : 0.0f);
    constexpr float MOUSE_SENS_YAW = 2.0f;   // Higher = faster
    constexpr float DT = 1.0f / 60.0f;

    float yawInput = static_cast<float>(remote.getMouseX()) * MOUSE_SENS_YAW * DT;
    Tturn = -clamp(yawInput, -1.0f, 1.0f);
    
    //Gimbal Pitch Keyboard
   // Declare outside
       

        // Function
       
            if (!pitchInit) {
                gimbalTargetPos = pitchMotor->getEncoderUnwrapped();  // Use -> notation
                pitchInit = true;
            }

            constexpr float MOUSE_SENS_PITCH = 20.0f;
            constexpr float DT2 = 1.0f / 60.0f;

            float pitchInput = static_cast<float>(remote.getMouseY()) * MOUSE_SENS_PITCH * DT2;
            gimbalTargetPos += pitchInput;
            gimbalTargetPos = std::clamp(gimbalTargetPos, -1000.0f, 5000.0f);
        



    beybladeMode = remote.keyPressed(tap::communication::serial::Remote::Key::SHIFT) ||
                remote.keyPressed(tap::communication::serial::Remote::Key::SHIFT);
    // bool leftClick = mouse.isButtonDown(tap::input::MouseButton::LEFT);
    leftClick = remote.getMouseL();
    rightClick = remote.getMouseR();
    // float mouse_dx = mouse.getDeltaX();
    mouse_dx = remote.getMouseX();
    mouse_dy = remote.getMouseY();

    //Agitator Keyboard Control
    if (remote.getMouseL()) {
    k = -6000;  // Left click → counterclockwise
    } else if (remote.getMouseR()) {
        k = 6000;   // Right click → clockwise
    } else {
        k = 0;      // No click → stop
    }
}




int main()
{
#ifdef PLATFORM_HOSTED
    std::cout << "Simulation starting..." << std::endl;
#endif

    /*
     * NOTE: We are using DoNotUse_getDrivers here because in the main
     *      robot loop we must access the singleton drivers to update
     *      IO states and run the scheduler.
     */
    src::Drivers *drivers = src::DoNotUse_getDrivers();

    tap::communication::serial::Remote remote(drivers);

    Board::initialize();

    tap::motor::DjiMotor motor(drivers, MOTOR_ID, CAN_BUS2, false, "cool motor");
    tap::motor::DjiMotor motor2(drivers, MOTOR_ID2, CAN_BUS2, true, "cool motor");
    tap::motor::DjiMotor motor3(drivers, MOTOR_ID3, CAN_BUS2, false, "cool motor");
    tap::motor::DjiMotor motor4(drivers, MOTOR_ID4, CAN_BUS2, true, "cool motor");
    tap::motor::DjiMotor motor5(drivers, MOTOR_ID5, CAN_BUS, true, "cool motor");
    tap::motor::DjiMotor motor6(drivers, MOTOR_ID6, CAN_BUS, true, "cool motor");
    tap::motor::DjiMotor motor7(drivers, MOTOR_ID7, CAN_BUS, true, "cool motor");
    tap::motor::DjiMotor flywheel1(drivers, MOTOR_ID2, CAN_BUS, false, "cool motor");
    tap::motor::DjiMotor flywheel2(drivers, MOTOR_ID, CAN_BUS, true, "cool motor");

    motor.initialize();
    motor2.initialize();
    motor3.initialize();
    motor4.initialize();
    motor5.initialize();
    motor6.initialize();
    motor7.initialize();
    flywheel1.initialize();
    flywheel2.initialize();

    remote.initialize();

    motor5.resetEncoderValue();

    drivers->mpu6500.init(500.f, 0.1f, 0.0f);

    initializeIo(drivers);

    drivers->pwm.setTimerFrequency(tap::gpio::Pwm::Timer::TIMER8, PWM_FREQUENCY);

#ifdef PLATFORM_HOSTED
    tap::motor::motorsim::DjiMotorSimHandler::getInstance()->resetMotorSims();
    // Blocking call, waits until Windows Simulator connects.
    tap::communication::TCPServer::MainServer()->getConnection();
#endif

        while (1)
    {

        

        remote.read();
        PROFILE(drivers->profiler, updateIo, (drivers));
        drivers->mpu6500.read();

        if (updateImuTimeout.execute())
        {
            drivers->mpu6500.periodicIMUUpdate();
            yaw = drivers->mpu6500.getYaw();
            // TODO use yaw
        }

        if (sendMotorTimeout.execute())
        {
            PROFILE(drivers->profiler, drivers->mpu6500.periodicIMUUpdate, ());
            PROFILE(drivers->profiler, drivers->commandScheduler.run, ());
            PROFILE(drivers->profiler, drivers->djiMotorTxHandler.encodeAndSendCanData, ());
            PROFILE(drivers->profiler, drivers->terminalSerial.update, ());

            if (remote.getSwitch(tap::communication::serial::Remote::Switch::RIGHT_SWITCH) ==
                tap::communication::serial::Remote::SwitchState::MID) {
                joystick_control(remote);
            }

            else if (remote.getSwitch(tap::communication::serial::Remote::Switch::RIGHT_SWITCH) ==
                tap::communication::serial::Remote::SwitchState::DOWN) {
                keyboard_control(remote, &motor7);
            }
            
            // === Beyblade Mode: Spin chassis while holding gimbal ===
            static float yawHoldTarget = 0.0f;
            static bool lastBeybladeState = false;
            static bool waitingForStabilize = false;
            static int stabilizeCounter = 0;

            bool beybladeMode = (
                remote.getSwitch(tap::communication::serial::Remote::Switch::RIGHT_SWITCH) ==
                tap::communication::serial::Remote::SwitchState::UP) || remote.keyPressed(tap::communication::serial::Remote::Key::SHIFT);
                
            if (beybladeMode)
            {
                if (!lastBeybladeState)
                {
                    lastBeybladeState = true;
                    waitingForStabilize = false;
                    stabilizeCounter = 0;
                }

                // Manual control logic during beyblade
                motor5.setDesiredOutput(14800 + (Tturn * 4000));
                
                float rotation = 1.0f;  // full spin commanded in beyblade mode

                float A = FWDJoy + StrafeJoy + rotation;
                float B = FWDJoy - StrafeJoy - rotation;
                float C = -FWDJoy - StrafeJoy + rotation;
                float D = -FWDJoy + StrafeJoy - rotation;

                constexpr float DRIVE_SCALE = 4000.0f;

                pidController3.runControllerDerivateError(A * DRIVE_SCALE - motor.getShaftRPM(), 1);
                pidController4.runControllerDerivateError(B * DRIVE_SCALE - motor2.getShaftRPM(), 1);
                pidController5.runControllerDerivateError(C * DRIVE_SCALE - motor3.getShaftRPM(), 1);
                pidController6.runControllerDerivateError(D * DRIVE_SCALE - motor4.getShaftRPM(), 1);

                motor.setDesiredOutput((int32_t)pidController3.getOutput());
                motor2.setDesiredOutput((int32_t)pidController4.getOutput());
                motor3.setDesiredOutput((int32_t)pidController5.getOutput());
                motor4.setDesiredOutput((int32_t)pidController6.getOutput());



            }
            else
            {
                // === Transition: just exited beyblade ===
                if (lastBeybladeState)
                {
                    waitingForStabilize = true;
                    stabilizeCounter = 0;
                    lastBeybladeState = false;
                }

                // === Stabilization phase: wait until yaw stops ===
                if (waitingForStabilize)
                {
                    // Actively brake using proportional damping
                    float brakingTorque = -motor5.getShaftRPM() * 10.0f;  // Tweak this factor as needed
                    brakingTorque = std::clamp(brakingTorque, -8000.0f, 8000.0f);  // Safe limit
                    motor5.setDesiredOutput(static_cast<int32_t>(brakingTorque));

                    // Check if stable
                    if (fabsf(motor5.getShaftRPM()) < 2.0f)
                    {
                        stabilizeCounter++;
                        if (stabilizeCounter >= 10)
                        {
                            gimbalYawTargetPos = motor5.getEncoderUnwrapped();
                            waitingForStabilize = false;
                        }
                    }
                    else
                    {
                        stabilizeCounter = 0;
                    }
                }


                // === Only adjust target if NOT stabilizing ===
                const float scalingFactor = 40.0f;
                if (!waitingForStabilize && fabs(Tturn) > 0.01f)
                {
                    gimbalYawTargetPos += Tturn * scalingFactor;
                }

                // === Only run PID once target is stable ===
                if (!waitingForStabilize)
                {
                    pidController7.runControllerDerivateError(
                        gimbalYawTargetPos - motor5.getEncoderUnwrapped(),
                        1
                    );
                    motor5.setDesiredOutput(static_cast<int32_t>(pidController7.getOutput()));
                }
            }


            /*
                //move direcion
                move= yaw+180;

                //quadrant
                if((StrafeJoy>0) && (FWDJoy>0))
                {heading = abs(atan(FWDJoy/StrafeJoy));}
                if((StrafeJoy<0) && (FWDJoy>0))
                {heading = 180 + abs(atan(FWDJoy/StrafeJoy));}
                if((StrafeJoy<0) && (FWDJoy<0))
                {heading = 180 + abs(atan(FWDJoy/StrafeJoy));}
                if((StrafeJoy>0) && (FWDJoy<0))
                {heading = 360 - abs(atan(FWDJoy/StrafeJoy));}

                //heading Power

                HPower = sqrt(pow(FWDJoy,2) + pow(StrafeJoy,2));

                // motor position

                MotorA = yaw;

                MotorB = yaw+90;
                if(MotorB > 360)
                {MotorB = MotorB-360;}

                MotorC = yaw+180;
                if(MotorC >360)
                {MotorC = MotorC-360;}

                MotorD = yaw+270;
                if(MotorD>360)
                {MotorD= MotorD-360;}

                //movement

                if(MotorA>move)
                {
                motor.setDesiredOutput(HPower*(sin(MotorA)));
                }
                else
                {
                    //some const. speed (make sure you have pid tuned and integrated )
                    motor.setDesiredOutput();
                }
                if(MotorB>move)
                {
                motor2.setDesiredOutput(HPower*(sin(MotorA)));
                }
                else
                {
                    //some const. speed (make sure you have pid tuned and integrated )
                    motor2.setDesiredOutput();
                }
                if(MotorC>move)
                {
                motor3.setDesiredOutput(HPower*(sin(MotorA)));
                }
                else
                {
                    //some const. speed (make sure you have pid tuned and integrated )
                    motor3.setDesiredOutput();
                }
                if(MotorD>move)
                {
                motor4.setDesiredOutput(HPower*(sin(MotorA)));
                }
                else
                {
                    //some const. speed (make sure you have pid tuned and integrated )
                    motor4.setDesiredOutput();
                }
    */
            float rotation = beybladeMode ? 1.0f : 0.0f;
            pidController3.runControllerDerivateError(
                ((FWDJoy + StrafeJoy + TXJoy + rotation) * 4000) - (motor.getShaftRPM()),
                1);
            pidController4.runControllerDerivateError(
                ((FWDJoy - StrafeJoy - TXJoy - rotation) * 4000) - (motor2.getShaftRPM()),
                1);
            pidController5.runControllerDerivateError(
                ((-FWDJoy - StrafeJoy + TXJoy + rotation) * 4000) - (motor3.getShaftRPM()),
                1);
            pidController6.runControllerDerivateError(
                ((-FWDJoy + StrafeJoy - TXJoy - rotation) * 4000) - (motor4.getShaftRPM()),
                1);

            motor.setDesiredOutput((static_cast<int32_t>(pidController3.getOutput())));
            motor2.setDesiredOutput((static_cast<int32_t>(pidController4.getOutput())));
            motor3.setDesiredOutput((static_cast<int32_t>(pidController5.getOutput())));
            motor4.setDesiredOutput((static_cast<int32_t>(pidController6.getOutput())));
            //motor5.setDesiredOutput((Tturn) * (30000));//Gimble Yaw speed controll/wheel toggle

            // if (abs(TYJoy) > 0)
            // {
            //     gimbalTargetPos = gimbalTargetPos + TYJoy * -6;//value to change for Gimbal Pitch
            // }

            // if (motor.isMotorOnline() && d)
            // {
            //     //motor7.resetEncoderValue();
            //     gimbalTargetPos = motor7.getEncoderUnwrapped();
            //     d = false;
            // }
            // else if (d == false)
            // {
                pidController1.runControllerDerivateError(
                    (gimbalTargetPos) - (motor7.getEncoderUnwrapped()),
                    1);
                motor7.setDesiredOutput(static_cast<int32_t>(pidController1.getOutput()));
            // }

            /*
            pidController2.runControllerDerivateError(DESIRED_RPM - motor6.getShaftRPM(), 1);
            motor6.setDesiredOutput((pidController2.getOutput()));
            */
            pidController2.runControllerDerivateError((k) - (motor6.getShaftRPM()), 1);
            motor6.setDesiredOutput(static_cast<int32_t>(pidController2.getOutput()));
            if (remote.getSwitch(tap::communication::serial::Remote::Switch::LEFT_SWITCH) ==
                tap::communication::serial::Remote::SwitchState::UP)
            {
                k = -6000;
            }
            
            else if (
                remote.getSwitch(tap::communication::serial::Remote::Switch::LEFT_SWITCH) ==
                tap::communication::serial::Remote::SwitchState::DOWN)
            {
                k = 0;
            }
            // else if (
            //     remote.getSwitch(tap::communication::serial::Remote::Switch::RIGHT_SWITCH) ==
            //         tap::communication::serial::Remote::SwitchState::MID ||
            //     remote.getSwitch(tap::communication::serial::Remote::Switch::RIGHT_SWITCH) ==
            //         tap::communication::serial::Remote::SwitchState::UNKNOWN)
            // {
            //     k = 0;
            // }

            if (remote.getSwitch(tap::communication::serial::Remote::Switch::LEFT_SWITCH) ==
                tap::communication::serial::Remote::SwitchState::MID)
            {
                flywheel1DesiredRPM = k_flywheelSpeed;
                flywheel2DesiredRPM = k_flywheelSpeed;
            }
            else if (
                remote.getSwitch(tap::communication::serial::Remote::Switch::LEFT_SWITCH) ==
                tap::communication::serial::Remote::SwitchState::MID)
                
            {
            }
            else if ((remote.getSwitch(tap::communication::serial::Remote::Switch::RIGHT_SWITCH) ==
                      tap::communication::serial::Remote::SwitchState::MID))
            {
                flywheel1DesiredRPM = 0.0f;
                flywheel2DesiredRPM = 0.0f;
            }

            drivers->djiMotorTxHandler.encodeAndSendCanData();
        }

        drivers->canRxHandler.pollCanData();

        modm::chrono::micro_clock::time_point now{theClock.now()};
        modm::PreciseClock::duration duration{now - epoch};
        updateFlywheels(duration.count(), flywheel1, flywheel2);

        modm::delay_us(100);
    }
    return 0;
}

void updateFlywheels(float deltaTime, tap::motor::DjiMotor &flywheel1, tap::motor::DjiMotor &flywheel2)
{
    const double flywheel1Error{flywheel1DesiredRPM - flywheel1.getShaftRPM()};
    const double flywheel1derivativeError{flywheel1Pid.runControllerDerivateError(flywheel1Error, deltaTime)};
    const double flywheel1output{flywheel1Pid.runController(flywheel1Error, flywheel1derivativeError, deltaTime)};
    flywheel1.setDesiredOutput(static_cast<int32_t>(flywheel1output));

    const double flywheel2Error{flywheel2DesiredRPM - flywheel2.getShaftRPM()};
    const double flywheel2derivativeError{flywheel2Pid.runControllerDerivateError(flywheel2Error, deltaTime)};
    const double flywheel2output{flywheel2Pid.runController(flywheel2Error, flywheel2derivativeError, deltaTime)};
    flywheel2.setDesiredOutput(static_cast<int32_t>(flywheel2output));
}

static void initializeIo(src::Drivers *drivers)
{
    drivers->analog.init();
    drivers->pwm.init();
    drivers->digital.init();
    drivers->leds.init();
    drivers->can.initialize();
    drivers->errorController.init();
    drivers->remote.initialize();
    drivers->mpu6500.init(MAIN_LOOP_FREQUENCY, 0.1, 0);
    drivers->refSerial.initialize();
    drivers->terminalSerial.initialize();
    drivers->schedulerTerminalHandler.init();
    drivers->djiMotorTerminalSerialHandler.init();
}

static void updateIo(src::Drivers *drivers)
{
#ifdef PLATFORM_HOSTED
    tap::motor::motorsim::DjiMotorSimHandler::getInstance()->updateSims();
#endif

    drivers->canRxHandler.pollCanData();
    drivers->refSerial.updateSerial();
    drivers->remote.read();
    drivers->mpu6500.read();
}
