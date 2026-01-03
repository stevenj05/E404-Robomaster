/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruw-edu.
 *
 * aruw-edu is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruw-edu is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruw-edu.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "tap/architecture/clock.hpp"
#include "tap/architecture/periodic_timer.hpp"
#include "tap/architecture/profiler.hpp"
#include "tap/board/board.hpp"
#include "tap/communication/serial/remote.hpp"
#include "tap/algorithms/smooth_pid.hpp"
#include "controls/control_operator_interface.hpp"
#include "modm/architecture/interface/delay.hpp"
#include "drivers_singleton.hpp"
#include "standard/standard_constants.hpp"

static constexpr float IMU_SAMPLE_FREQUENCY = 500;
static constexpr float MAHONY_KP = 0.5f;
static constexpr float MAHONY_KI = 0;

tap::arch::PeriodicMilliTimer sendMotorTimeout(2);

// CAN bus constants
static constexpr tap::can::CanBus CAN_BUS = tap::can::CanBus::CAN_BUS1;

// Drive train motor ID constants
static constexpr tap::motor::MotorId DTRAIN_1 = tap::motor::MOTOR2;
static constexpr tap::motor::MotorId DTRAIN_2 = tap::motor::MOTOR3;
static constexpr tap::motor::MotorId DTRAIN_3 = tap::motor::MOTOR4;
static constexpr tap::motor::MotorId DTRAIN_4 = tap::motor::MOTOR1;

// Yaw motor and Pitch motor ID constants (6020 motors)
static constexpr tap::motor::MotorId YAWMOTOR = tap::motor::MOTOR5;
static constexpr tap::motor::MotorId PITCHMOTOR = tap::motor::MOTOR8;

// Agitator motor ID constant (2006 agitator)
static constexpr tap::motor::MotorId AGIMOTOR = tap::motor::MOTOR6;

// Drive train and flywheel motor max speeds
static constexpr float DTRAIN_MAX_RPM = control::chassis::ChassisSubsystem::MAX_WHEELSPEED_RPM;
static constexpr float MAX_FLY_RPM = 6000;

// Drive train PIDs and PID controllers
tap::algorithms::SmoothPidConfig SmoothpidConfigD1{100, 0, 0.5f, 0, 15000, 1, 0, 1, 0};
tap::algorithms::SmoothPidConfig SmoothpidConfigD2{10, 1, 1, 0, 8000, 1, 0, 1, 0};
tap::algorithms::SmoothPidConfig SmoothpidConfigD3{10, 1, 1, 0, 8000, 1, 0, 1, 0};
tap::algorithms::SmoothPidConfig SmoothpidConfigD4{10, 1, 1, 0, 8000, 1, 0, 1, 0};
tap::algorithms::SmoothPid pidControllerD1(SmoothpidConfigD1);
tap::algorithms::SmoothPid pidControllerD2(SmoothpidConfigD2);
tap::algorithms::SmoothPid pidControllerD3(SmoothpidConfigD3);
tap::algorithms::SmoothPid pidControllerD4(SmoothpidConfigD4);

// Flywheel PIDs and PID controllers
// Max output for C620 is 16384
tap::algorithms::SmoothPidConfig flywheel1PidConfig{20, 0, 0, 100, 16384, 1, 0, 1, 0};
tap::algorithms::SmoothPidConfig flywheel2PidConfig{20, 0, 0, 100, 16384, 1, 0, 1, 0};
tap::algorithms::SmoothPid flywheel1Pid(flywheel1PidConfig);
tap::algorithms::SmoothPid flywheel2Pid(flywheel2PidConfig);

//control::Robot robot(*DoNotUse_getDrivers());

// Place any sort of input/output initialization here. For example, place
// serial init stuff here.
static void initializeIo(Drivers *drivers);

// Anything that you would like to be called place here. It will be called
// very frequently. Use PeriodicMilliTimers if you don't want something to be
// called as frequently.
static void updateIo(Drivers *drivers);

int main()
{
    /*
     * NOTE: We are using DoNotUse_getDrivers here because in the main
     *      robot loop we must access the singleton drivers to update
     *      IO states and run the scheduler.
     */
    // Create the drivers pointer object and initialize all drivers, including the remote
    Drivers *drivers = DoNotUse_getDrivers();
    initializeIo(drivers);

    Board::initialize();

    //robot.initSubsystemCommands();

    // Instantiate and initialize all drive train motors with their starting data.
    // Each drive train motor object's variable name indicates where that motor is located 
    // (e.g., BR = "back right", FL = "front left") and the number of the drive train ID it's
    // associated with.
    tap::motor::DjiMotor dTrainFL1(drivers, DTRAIN_1, CAN_CHASSIS, false, "cool motor");
    tap::motor::DjiMotor dTrainFR2(drivers, DTRAIN_2, CAN_CHASSIS, false, "cool motor");
    tap::motor::DjiMotor dTrainBR3(drivers, DTRAIN_3, CAN_CHASSIS, false, "cool motor");
    tap::motor::DjiMotor dTrainBL4(drivers, DTRAIN_4, CAN_CHASSIS, false, "cool motor");
    dTrainFL1.initialize();
    dTrainFR2.initialize();
    dTrainBR3.initialize();
    dTrainBL4.initialize();

    // Instantiate and initialize the flywheel and agitator motors with their starting data
    tap::motor::DjiMotor flywheel1(drivers, DTRAIN_2, CAN_BUS, false, "cool motor");
    tap::motor::DjiMotor flywheel2(drivers, DTRAIN_1, CAN_BUS, true, "cool motor");
    flywheel1.initialize();
    flywheel2.initialize();

    // Stores the max clockwise and counterclockwise drivetrain motor RPM
    float maxCwRPM = -DTRAIN_MAX_RPM;
    float maxCcwRPM = DTRAIN_MAX_RPM;

    while (1)
    {
        // Updates I/O for all the drivers, including remote readings
        PROFILE(drivers->profiler, updateIo, (drivers));

        if (sendMotorTimeout.execute())
        {
            PROFILE(drivers->profiler, drivers->mpu6500.periodicIMUUpdate, ());
            PROFILE(drivers->profiler, drivers->commandScheduler.run, ());
            PROFILE(drivers->profiler, drivers->djiMotorTxHandler.encodeAndSendCanData, ());
            PROFILE(drivers->profiler, drivers->terminalSerial.update, ());

            // ===== Powers the drive train motors based on controller input ======

            // Stores the horizontal and vertical input values of the left joystick on the 
            // remote, respectively. -1 is maximum left or down, +1 is maximum up or right.
            float chassisXInput = drivers->controlOperatorInterface.getChassisTankXInput();
            float chassisYInput = drivers->controlOperatorInterface.getChassisTankYInput();

            // Set all motor outputs to zero to stop moving when no left joystick input is given
            if(chassisXInput == 0 && chassisYInput == 0)
            {
                pidControllerD1.runControllerDerivateError(-(dTrainFL1.getShaftRPM()), 1);
                dTrainFL1.setDesiredOutput((int32_t)(pidControllerD1.getOutput()));

                pidControllerD2.runControllerDerivateError(-(dTrainFR2.getShaftRPM()), 1);
                dTrainFR2.setDesiredOutput((int32_t)(pidControllerD2.getOutput()));

                pidControllerD3.runControllerDerivateError(-(dTrainBR3.getShaftRPM()), 1);
                dTrainBR3.setDesiredOutput((int32_t)(pidControllerD3.getOutput()));

                pidControllerD4.runControllerDerivateError(-(dTrainBL4.getShaftRPM()), 1);
                dTrainBL4.setDesiredOutput((int32_t)(pidControllerD4.getOutput()));
            }
            else
            {
                // Move forward if the left joystick is pushed upwards. Pushing farther
                // results in faster speed.
                if(chassisYInput > 0)
                {
                    pidControllerD2.runControllerDerivateError((chassisYInput*maxCwRPM) - (dTrainFR2.getShaftRPM()), 1);
                    dTrainFR2.setDesiredOutput((int32_t)(pidControllerD2.getOutput()));

                    pidControllerD4.runControllerDerivateError((chassisYInput*maxCcwRPM) - (dTrainBL4.getShaftRPM()), 1);
                    dTrainBL4.setDesiredOutput((int32_t)(pidControllerD4.getOutput()));

                    // Loose wheel
                    // dTrainBR3.setDesiredOutput((int32_t)(chassisYInput*maxCwRPM));

                    pidControllerD1.runControllerDerivateError((chassisYInput*maxCwRPM) - (dTrainFL1.getShaftRPM()), 1);
                    dTrainFL1.setDesiredOutput((int32_t)(pidControllerD1.getOutput()));
                }

                // Move backwards if the left joystick is pushed downwards
                else if(chassisYInput < 0)
                {
                    pidControllerD2.runControllerDerivateError((chassisYInput*maxCcwRPM) - (dTrainFR2.getShaftRPM()), 1);
                    dTrainFR2.setDesiredOutput((int32_t)(pidControllerD2.getOutput()));

                    pidControllerD4.runControllerDerivateError((chassisYInput*maxCcwRPM) - (dTrainBL4.getShaftRPM()), 1);
                    dTrainBL4.setDesiredOutput((int32_t)(pidControllerD4.getOutput()));

                    // Loose wheel
                    // dTrainBR3.setDesiredOutput((int32_t)(chassisYInput*maxCwRPM));

                    pidControllerD1.runControllerDerivateError((chassisYInput*maxCwRPM) - (dTrainFL1.getShaftRPM()), 1);
                    dTrainFL1.setDesiredOutput((int32_t)pidControllerD1.getOutput());
                }

            }
        }

        modm::delay_us(100);
    }

    return 0;
}

static void initializeIo(Drivers *drivers)
{
    drivers->analog.init();
    drivers->pwm.init();
    drivers->digital.init();
    drivers->leds.init();
    drivers->can.initialize();
    drivers->errorController.init();
    drivers->remote.initialize();
    drivers->mpu6500.init(IMU_SAMPLE_FREQUENCY, MAHONY_KP, MAHONY_KI);
    drivers->refSerial.initialize();
    drivers->terminalSerial.initialize();
    drivers->schedulerTerminalHandler.init();
    drivers->djiMotorTerminalSerialHandler.init();
}

static void updateIo(Drivers *drivers)
{
    drivers->canRxHandler.pollCanData();
    drivers->refSerial.updateSerial();
    drivers->remote.read();
    drivers->mpu6500.read();
}
