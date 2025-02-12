#include <iostream>
#include <stdio.h>

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

static constexpr tap::can::CanBus CAN_BUS = tap::can::CanBus::CAN_BUS1;

static constexpr tap::motor::MotorId MOTOR_ID = tap::motor::MOTOR1;

static constexpr tap::motor::MotorId MOTOR_ID2 = tap::motor::MOTOR2;

static constexpr tap::motor::MotorId MOTOR_ID3 = tap::motor::MOTOR3;

static constexpr tap::motor::MotorId MOTOR_ID4 = tap::motor::MOTOR4;


static constexpr int DESIRED_RPM = 3000;

tap::arch::PeriodicMilliTimer sendMotorTimeout(1000.0f / MAIN_LOOP_FREQUENCY);

// Place any sort of input/output initialization here. For example, place
// serial init stuff here.
static void initializeIo(src::Drivers *drivers);

// Anything that you would like to be called place here. It will be called
// very frequently. Use PeriodicMilliTimers if you don't want something to be
// called as frequently.
static void updateIo(src::Drivers *drivers);

tap::algorithms::SmoothPidConfig SmoothpidConfig(20, 0, 0, 0, 8000, 1, 0, 1, 0);
tap::algorithms::SmoothPid pidController(SmoothpidConfig);

tap::motor::DjiMotor motor(src::DoNotUse_getDrivers(), MOTOR_ID, CAN_BUS, false, "cool motor");
tap::motor::DjiMotor motor2(src::DoNotUse_getDrivers(), MOTOR_ID2, CAN_BUS, true, "cool motor");
tap::motor::DjiMotor motor3(src::DoNotUse_getDrivers(), MOTOR_ID3, CAN_BUS, false, "cool motor");
tap::motor::DjiMotor motor4(src::DoNotUse_getDrivers(), MOTOR_ID4, CAN_BUS, true, "cool motor");

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
    remote.initialize();
   // drivers -> djiMotorTxHandler.encodeAndSendCanData();
    initializeIo(drivers);
    

#ifdef PLATFORM_HOSTED
    tap::motor::motorsim::DjiMotorSimHandler::getInstance()->resetMotorSims();
    // Blocking call, waits until Windows Simulator connects.
    tap::communication::TCPServer::MainServer()->getConnection();
#endif

    while (1)
    {
        // do this as fast as you can
       remote.read();

        //PROFILE(drivers->profiler, updateIo, (drivers));

        if (sendMotorTimeout.execute())
        {
            PROFILE(drivers->profiler, drivers->mpu6500.periodicIMUUpdate, ());
            PROFILE(drivers->profiler, drivers->commandScheduler.run, ());
            PROFILE(drivers->profiler, drivers->djiMotorTxHandler.encodeAndSendCanData, ());
            PROFILE(drivers->profiler, drivers->terminalSerial.update, ());
            
            //pidController.runControllerDerivateError(DESIRED_RPM - 0, 1);
            
            float FWDJoy = remote.getChannel(tap::communication::serial::Remote::Channel::LEFT_VERTICAL);
            float StrafeJoy = remote.getChannel(tap::communication::serial::Remote::Channel::LEFT_HORIZONTAL);
            float TXJoy = remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_HORIZONTAL);
            float TYJoy = remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_VERTICAL);
            
            
            
            motor.setDesiredOutput((FWDJoy+StrafeJoy+TXJoy)*(1684));
            motor2.setDesiredOutput((FWDJoy-StrafeJoy-TXJoy)*(1684));
            motor3.setDesiredOutput((FWDJoy-StrafeJoy+TXJoy)*(1684));
            motor4.setDesiredOutput((FWDJoy+StrafeJoy-TXJoy)*(1684));
            
           //motor.setDesiredOutput((remote.getChannel(tap::communication::serial::Remote::Channel::LEFT_VERTICAL))*(1000));

            //std::cout << FWDJoy << std::endl;
            
            drivers->djiMotorTxHandler.encodeAndSendCanData();
            
        }

        drivers->canRxHandler.pollCanData();

        modm::delay_us(10);
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

