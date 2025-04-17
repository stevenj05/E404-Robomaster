#include <iostream>
#include<stdio.h>

#ifdef PLATFORM_HOSTED
/* hosted environment (simulator) includes --------------------------------- */

#include "tap/communication/tcp-server/tcp_server.hpp"
#include "tap/motor/motorsim/dji_motor_sim_handler.hpp"

#endif

#include "refrenceHead.hpp"

/* Our Headers -------------------------------------------------------*/
#include "stuffh.hpp"

/* define timers here -------------------------------------------------------*/

static constexpr float MAIN_LOOP_FREQUENCY = 500.0f;

static constexpr float PWM_FREQUENCY = 500.0f;

static constexpr tap::can::CanBus CAN_BUS = tap::can::CanBus::CAN_BUS1;

static constexpr tap::motor::MotorId MOTOR_ID = tap::motor::MOTOR2;

static constexpr tap::motor::MotorId MOTOR_ID2 = tap::motor::MOTOR3;

static constexpr tap::motor::MotorId MOTOR_ID3 = tap::motor::MOTOR4;

static constexpr tap::motor::MotorId MOTOR_ID4 = tap::motor::MOTOR1;

static constexpr tap::motor::MotorId MOTOR_ID5 = tap::motor::MOTOR5;

static constexpr tap::motor::MotorId MOTOR_ID6 = tap::motor::MOTOR6;

static constexpr tap::motor::MotorId MOTOR_ID7 = tap::motor::MOTOR7;

tap::arch::PeriodicMilliTimer sendMotorTimeout(1000.0f / MAIN_LOOP_FREQUENCY);
tap::arch::PeriodicMilliTimer updateImuTimeout(2);

// Place any sort of input/output initialization here. For example, place
// serial init stuff here.
static void initializeIo(src::Drivers *drivers);


// Anything that you would like to be called place here. It will be called
// very frequently. Use PeriodicMilliTimers if you don't want something to be
// called as frequently.
static void updateIo(src::Drivers *drivers);

tap::algorithms::SmoothPidConfig SmoothpidConfig1(10, 1, 1, 0, 8000, 1, 0, 1, 0);
tap::algorithms::SmoothPidConfig SmoothpidConfig2(10, 1, 1, 0, 8000, 1, 0, 1, 0);
tap::algorithms::SmoothPidConfig SmoothpidConfig3(10, 1, 1, 0, 8000, 1, 0, 1, 0);
tap::algorithms::SmoothPidConfig SmoothpidConfig4(10, 1, 1, 0, 8000, 1, 0, 1, 0);
tap::algorithms::SmoothPidConfig SmoothpidConfig5(10, 1, 1, 0, 8000, 1, 0, 1, 0);
tap::algorithms::SmoothPidConfig SmoothpidConfig6(10, 1, 1, 0, 8000, 1, 0, 1, 0);
tap::algorithms::SmoothPid pidController1(SmoothpidConfig1);
tap::algorithms::SmoothPid pidController2(SmoothpidConfig2);
tap::algorithms::SmoothPid pidController3(SmoothpidConfig3);
tap::algorithms::SmoothPid pidController4(SmoothpidConfig4);
tap::algorithms::SmoothPid pidController5(SmoothpidConfig5);
tap::algorithms::SmoothPid pidController6(SmoothpidConfig6);

tap::motor::DjiMotor motor(src::DoNotUse_getDrivers(), MOTOR_ID, CAN_BUS, false, "cool motor");
tap::motor::DjiMotor motor2(src::DoNotUse_getDrivers(), MOTOR_ID2, CAN_BUS, true, "cool motor");
tap::motor::DjiMotor motor3(src::DoNotUse_getDrivers(), MOTOR_ID3, CAN_BUS, false, "cool motor");
tap::motor::DjiMotor motor4(src::DoNotUse_getDrivers(), MOTOR_ID4, CAN_BUS, true, "cool motor");
tap::motor::DjiMotor motor5(src::DoNotUse_getDrivers(), MOTOR_ID5, CAN_BUS, true, "cool motor");
tap::motor::DjiMotor motor6(src::DoNotUse_getDrivers(), MOTOR_ID6, CAN_BUS, true, "cool motor");
tap::motor::DjiMotor motor7(src::DoNotUse_getDrivers(), MOTOR_ID7, CAN_BUS, true, "cool motor");

tap::gpio::Pwm::Pin pwmPin = tap::gpio::Pwm::Pin::Z;
tap::gpio::Pwm::Pin pwmPin2 = tap::gpio::Pwm::Pin::Y;


float heading, move, MotorA, MotorB, MotorC, MotorD, yaw,HPower;

float FWDJoy, StrafeJoy, TXJoy, TYJoy, Tturn,k;
bool done=false, f =true, d= true;
float DESIRED_RPM, DESIRED_RPM2, DESIRED_RPM3, DESIRED_RPM4, poop;

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

    motor.initialize();
    motor2.initialize();
    motor3.initialize();
    motor4.initialize();
    motor5.initialize();
    motor6.initialize();
    motor7.initialize();

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

            FWDJoy = remote.getChannel(tap::communication::serial::Remote::Channel::LEFT_VERTICAL);
            StrafeJoy = remote.getChannel(tap::communication::serial::Remote::Channel::LEFT_HORIZONTAL);
            TXJoy = remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_HORIZONTAL);
            TYJoy = remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_VERTICAL);
            Tturn = remote.getChannel(tap::communication::serial::Remote::Channel::WHEEL);


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
            // thiss only controls amperage must introduce PID for specific RPM values
            pidController3.runControllerDerivateError(((FWDJoy+StrafeJoy+TXJoy)*1000) - (motor.getShaftRPM() ), 1);
            pidController4.runControllerDerivateError(((FWDJoy-StrafeJoy-TXJoy)*1000) - (motor2.getShaftRPM() ), 1);
            pidController5.runControllerDerivateError(((-FWDJoy-StrafeJoy+TXJoy)*1000) - (motor3.getShaftRPM() ), 1);
            pidController6.runControllerDerivateError(((-FWDJoy+StrafeJoy-TXJoy)*1000) - (motor4.getShaftRPM() ), 1);
           
            motor.setDesiredOutput((static_cast<int32_t>(pidController3.getOutput())));
            motor2.setDesiredOutput((static_cast<int32_t>(pidController4.getOutput())));
            motor3.setDesiredOutput((static_cast<int32_t>(pidController5.getOutput())));
            motor4.setDesiredOutput((static_cast<int32_t>(pidController6.getOutput())));
            motor5.setDesiredOutput((Tturn)*(10000)); 


            if(abs(TYJoy) >0)
            {
                poop = poop + TYJoy*-4;
            }

            if(motor.isMotorOnline() && d)
            {
                motor7.resetEncoderValue();
                d = false;
            }
            else if (d == false)
        {
            pidController1.runControllerDerivateError((poop) - (motor7.getEncoderUnwrapped() ), 1);
            motor7.setDesiredOutput(static_cast<int32_t>(pidController1.getOutput()));
        }
            

            /*
            pidController2.runControllerDerivateError(DESIRED_RPM - motor6.getShaftRPM(), 1);
            motor6.setDesiredOutput((pidController2.getOutput()));
            */
           pidController2.runControllerDerivateError((k) - (motor6.getShaftRPM() ), 1);
           motor6.setDesiredOutput(static_cast<int32_t>(pidController2.getOutput()));
            if ( remote.getSwitch(tap::communication::serial::Remote::Switch::RIGHT_SWITCH) == 
            tap::communication::serial::Remote::SwitchState::UP )
            {
            k = 7200;
            }
            else if (remote.getSwitch(tap::communication::serial::Remote::Switch::RIGHT_SWITCH) == tap::communication::serial::Remote::SwitchState::MID || remote.getSwitch(tap::communication::serial::Remote::Switch::RIGHT_SWITCH) == tap::communication::serial::Remote::SwitchState::UNKNOWN)
            {
                k=0;
            }
            else if (remote.getSwitch(tap::communication::serial::Remote::Switch::RIGHT_SWITCH) == tap::communication::serial::Remote::SwitchState::DOWN )
            {
                k= -7200;
            }

            if(f)
            {
            drivers->pwm.write(.9,pwmPin);
            drivers->pwm.write(.9,pwmPin2);   
            }


            if ( remote.getSwitch(tap::communication::serial::Remote::Switch::LEFT_SWITCH) == 
            tap::communication::serial::Remote::SwitchState::UP  && f)
            {
                drivers->pwm.write(.1,pwmPin);
                drivers->pwm.write(.1,pwmPin2);
                f= false;
            }
            else if (remote.getSwitch(tap::communication::serial::Remote::Switch::LEFT_SWITCH) == 
            tap::communication::serial::Remote::SwitchState::DOWN)
            {
                drivers->pwm.write(.9,pwmPin);
                drivers->pwm.write(.9,pwmPin2);
            }
            else if ((remote.getSwitch(tap::communication::serial::Remote::Switch::LEFT_SWITCH) == 
            tap::communication::serial::Remote::SwitchState::MID && !f) )
            {
                drivers->pwm.write(.1,pwmPin);
                drivers->pwm.write(.1,pwmPin2);
            }



            drivers->djiMotorTxHandler.encodeAndSendCanData();
        }

        drivers->canRxHandler.pollCanData();

        modm::delay_us(100);
    }
    return 0;
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
