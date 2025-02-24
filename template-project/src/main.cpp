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

static constexpr int DESIRED_RPM = 3000;

tap::arch::PeriodicMilliTimer sendMotorTimeout(1000.0f / MAIN_LOOP_FREQUENCY);

// Place any sort of input/output initialization here. For example, place
// serial init stuff here.
static void initializeIo(src::Drivers *drivers);
/*
void initializePwmSequence(src::Drivers *drivers,tap::gpio::Pwm::Pin pwmpin);
*/

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
tap::motor::DjiMotor motor5(src::DoNotUse_getDrivers(), MOTOR_ID5, CAN_BUS, true, "cool motor");

tap::gpio::Pwm::Pin pwmPin = tap::gpio::Pwm::Pin::Z;
tap::gpio::Pwm::Pin pwmPin2 = tap::gpio::Pwm::Pin::Y;

tap::communication::sensors::imu::mpu6500::Mpu6500 MPU6500;

float heading, actualHeading, controllerHeading, headingPower, move, MotorA, MotorB, MotorC, MotorD;
float FWDJoy, StrafeJoy, TXJoy, TYJoy;
bool done=false;

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
        MPU6500.read();
        PROFILE(drivers->profiler, updateIo, (drivers));
        
        if (sendMotorTimeout.execute())
        {
            PROFILE(drivers->profiler, drivers->mpu6500.periodicIMUUpdate, ());
            PROFILE(drivers->profiler, drivers->commandScheduler.run, ());
            PROFILE(drivers->profiler, drivers->djiMotorTxHandler.encodeAndSendCanData, ());
            PROFILE(drivers->profiler, drivers->terminalSerial.update, ());

             if (motor.isMotorOnline() && !done) 
            {
                initializePwmSequence(drivers, pwmPin, pwmPin2);
                done = true;
            }

            FWDJoy = remote.getChannel(tap::communication::serial::Remote::Channel::LEFT_VERTICAL);
            StrafeJoy = remote.getChannel(tap::communication::serial::Remote::Channel::LEFT_HORIZONTAL);
            TXJoy = remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_HORIZONTAL);
            TYJoy = remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_VERTICAL);
            

            // heading reading from the sensor in radians
            actualHeading = (MPU6500.getYaw() * (M_PI/180));

            // the heading given by the controller *should already be in radinas*
            controllerHeading = atan2(FWDJoy, StrafeJoy);

            // power measurement from the center of the joystick to apply to sine wave
            headingPower = sqrt(pow(FWDJoy,2) + pow(StrafeJoy,2));

            // this is the heading plus pi later used to check if motor is greater than this (Right side of the circle)
            move = controllerHeading + M_PI; 

            // each motors position in the circle
            MotorA = (actualHeading );
            MotorB = (actualHeading + (M_PI/2));
            MotorC = (actualHeading + (M_PI));
            MotorD = (actualHeading + (3*M_PI/2));

            // this is where we check weather the motors position is on the right side of the circle
            if(MotorA > move)
            {

                // apply the sine wave function to move in the direction
                motor.setDesiredOutput(headingPower * sin(MotorA));
            }

            // else spin at this constant speed
            else 
            motor.setDesiredOutput(1000);

            if(MotorB > move)
            {
                motor2.setDesiredOutput(headingPower * sin(MotorB));
            }
            else 
            motor2.setDesiredOutput(1000);
            if(MotorC > move)
            {
                motor3.setDesiredOutput(headingPower * sin(MotorC));
            }
            else 
            motor3.setDesiredOutput(1000);
            if(MotorD > move)
            {
                motor4.setDesiredOutput(headingPower * sin(MotorD));
            }
            else 
            motor4.setDesiredOutput(1000);

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

void initializePwmSequence(src::Drivers *drivers, tap::gpio::Pwm::Pin pwmPin, tap::gpio::Pwm::Pin pwmPin2) {
    using namespace std::chrono_literals;  
    
    drivers->pwm.write(.9,pwmPin);
    drivers->pwm.write(.9,pwmPin2);

    modm::delay(2000ms);

    {
        drivers->pwm.write(.1,pwmPin2);
        drivers->pwm.write(.1,pwmPin);
    }

}
