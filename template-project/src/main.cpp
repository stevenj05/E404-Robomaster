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
tap::algorithms::SmoothPid pidController(SmoothpidConfig1);


tap::motor::DjiMotor motor(src::DoNotUse_getDrivers(), MOTOR_ID, CAN_BUS, false, "cool motor");
tap::motor::DjiMotor motor2(src::DoNotUse_getDrivers(), MOTOR_ID2, CAN_BUS, true, "cool motor");
tap::motor::DjiMotor motor3(src::DoNotUse_getDrivers(), MOTOR_ID3, CAN_BUS, false, "cool motor");
tap::motor::DjiMotor motor4(src::DoNotUse_getDrivers(), MOTOR_ID4, CAN_BUS, true, "cool motor");
tap::motor::DjiMotor motor5(src::DoNotUse_getDrivers(), MOTOR_ID5, CAN_BUS, true, "cool motor");

tap::gpio::Pwm::Pin pwmPin = tap::gpio::Pwm::Pin::Z;
tap::gpio::Pwm::Pin pwmPin2 = tap::gpio::Pwm::Pin::Y;


float heading, move, MotorA, MotorB, MotorC, MotorD, yaw,HPower;

float FWDJoy, StrafeJoy, TXJoy, TYJoy;
bool done=false;
float DESIRED_RPM, DESIRED_RPM2, DESIRED_RPM3, DESIRED_RPM4;
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
    remote.initialize();
    

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

            /*
            motor.setDesiredOutput((FWDJoy+StrafeJoy+TXJoy)*(1684));
            motor2.setDesiredOutput((FWDJoy-StrafeJoy-TXJoy)*(1684));
            motor3.setDesiredOutput((-FWDJoy-StrafeJoy+TXJoy)*(1684));
            motor4.setDesiredOutput((-FWDJoy+StrafeJoy-TXJoy)*(1684));
            motor5.setDesiredOutput((-TYJoy)*(10000)); 
            */

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
