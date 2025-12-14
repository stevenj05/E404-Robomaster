/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of taproot-template-project.
 *
 * taproot-template-project is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * taproot-template-project is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with taproot-template-project.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifdef PLATFORM_HOSTED
/* hosted environment (simulator) includes --------------------------------- */
#include <iostream>

#include "tap/communication/tcp-server/tcp_server.hpp"
#include "tap/motor/motorsim/dji_motor_sim_handler.hpp"
#endif

#include "tap/board/board.hpp"

#include "modm/architecture/interface/delay.hpp"

/* arch includes ------------------------------------------------------------*/
#include "tap/architecture/periodic_timer.hpp"
#include "tap/architecture/profiler.hpp"

/* communication includes ---------------------------------------------------*/
#include "drivers.hpp"
#include "drivers_singleton.hpp"

/* error handling includes --------------------------------------------------*/
#include "tap/errors/create_errors.hpp"

/* control includes ---------------------------------------------------------*/
#include "tap/architecture/clock.hpp"


#include "tap/algorithms/smooth_pid.hpp"

#include "UART_Communication\Receiver.hpp"
#include "UART_Communication\Sender.hpp"

/* define timers here -------------------------------------------------------*/
static constexpr float MAIN_LOOP_FREQUENCY = 500.0f;
tap::arch::PeriodicMilliTimer sendMotorTimeout(1000.0f / MAIN_LOOP_FREQUENCY);



static constexpr tap::motor::MotorId MOTOR_ID = tap::motor::MOTOR1;
static constexpr tap::can::CanBus CAN_BUS = tap::can::CanBus::CAN_BUS1;
static constexpr int DESIRED_RPM = 3000;



/*---------------------------MOTOR Config---------------------------------------------*/

tap::algorithms::SmoothPidConfig SmoothpidConfig1(20, 0, 0, 0, 8000, 1, 0, 1, 0);
tap::algorithms::SmoothPid pidController1(SmoothpidConfig1);
tap::motor::DjiMotor motor(src::DoNotUse_getDrivers(), MOTOR_ID, CAN_BUS, false, "cool motor");

/*------------------------------------------------------------------------------------*/

using tap::communication::serial::MotorReceiver;
static MotorReceiver motorReceiver(src::DoNotUse_getDrivers(), tap::communication::serial::Uart::UartPort::Uart1);

using tap::communication::serial::MotorSender;
static MotorSender motorSender(src::DoNotUse_getDrivers(), tap::communication::serial::Uart::UartPort::Uart1);

static  int16_t RPM_TO_SEND = 0;

// Place any sort of input/output initialization here. For example, place
// serial init stuff here.
static void initializeIo(src::Drivers *drivers);

// Anything that you would like to be called place here. It will be called
// very frequently. Use PeriodicMilliTimers if you don't want something to be
// called as frequently.
static void updateIo(src::Drivers *drivers);



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

    Board::initialize();

    motor.initialize();

    initializeIo(drivers);
    

#ifdef PLATFORM_HOSTED
    tap::motor::motorsim::DjiMotorSimHandler::getInstance()->resetMotorSims();
    // Blocking call, waits until Windows Simulator connects.
    tap::communication::TCPServer::MainServer()->getConnection();
#endif

    while (1)
    {
        // do this as fast as you can
        PROFILE(drivers->profiler, updateIo, (drivers));

        if (sendMotorTimeout.execute())
        {
            //PROFILE(drivers->profiler, drivers->mpu6500.periodicIMUUpdate, ());
            PROFILE(drivers->profiler, drivers->commandScheduler.run, ());
            PROFILE(drivers->profiler, drivers->djiMotorTxHandler.encodeAndSendCanData, ());
            PROFILE(drivers->profiler, drivers->terminalSerial.update, ());

            /*--------------------------------------------------------------------------------------*/
            /*---------------For Receiving Board (motor control)------------------------------------*/
            pidController1.runControllerDerivateError(
                ((motorReceiver.getLastRpm()) ) - (motor.getShaftRPM()), 1);

             motor.setDesiredOutput((static_cast<int32_t>(pidController1.getOutput())));
 
            /*--------------------------------------------------------------------------------------*/

             bool RampUp =true;
             
             if(RampUp)
             {
                // Ramping Up 
                RPM_TO_SEND += 1; // <- The RPM Step Size
             
             if (RPM_TO_SEND >= 1600) //<- The Max RPM Value
             {
                RPM_TO_SEND = 1600; //<- Set equal to Max RPM Value
                RampUp = false;
             }
            }
             else 
                {
                    RPM_TO_SEND -= 1;
                    if (RPM_TO_SEND <= -1600) //<- The Max RPM Value
                     {
                        RPM_TO_SEND = -1600; //<- Set equal to Max RPM Value
                        RampUp = true;
                     }
                }



             motorSender.sendRpmCommand(RPM_TO_SEND);
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
    //drivers->mpu6500.init(MAIN_LOOP_FREQUENCY, 0.1, 0);
    drivers->refSerial.initialize();
    drivers->terminalSerial.initialize();
    drivers->schedulerTerminalHandler.init();
    drivers->djiMotorTerminalSerialHandler.init();
    motorReceiver.initialize();
    motorSender.initialize();
}

static void updateIo(src::Drivers *drivers)
{
#ifdef PLATFORM_HOSTED
    tap::motor::motorsim::DjiMotorSimHandler::getInstance()->updateSims();
#endif

    drivers->canRxHandler.pollCanData();
    drivers->refSerial.updateSerial();
    drivers->remote.read();
    motorReceiver.update();
    //drivers->mpu6500.read();
}

