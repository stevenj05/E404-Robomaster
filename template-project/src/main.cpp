#include <stdio.h>

#include "drivers_singleton.hpp"

/* define timers here -------------------------------------------------------*/
static constexpr float MAIN_LOOP_FREQUENCY = 500.0f;

// CAN Bus Configuration for Type C Board
static constexpr tap::can::CanBus CAN_BUS = tap::can::CanBus::CAN_BUS1;

// Timer for motor updates
tap::arch::PeriodicMilliTimer sendMotorTimeout(1000.0f / MAIN_LOOP_FREQUENCY);

int main()
{
    // Get drivers singleton
    src::Drivers *drivers = src::DoNotUse_getDrivers();

    // Initialize remote control
    tap::communication::serial::Remote remote(drivers);

    // Initialize board
    Board::initialize();

    // Create motors for ALL possible IDs (1-8) to find your 2006
    tap::motor::DjiMotor motor1(drivers, tap::motor::MOTOR1, CAN_BUS, false, "Test ID 1");
    tap::motor::DjiMotor motor2(drivers, tap::motor::MOTOR2, CAN_BUS, false, "Test ID 2");
    tap::motor::DjiMotor motor3(drivers, tap::motor::MOTOR3, CAN_BUS, false, "Test ID 3");
    tap::motor::DjiMotor motor4(drivers, tap::motor::MOTOR4, CAN_BUS, false, "Test ID 4");
    tap::motor::DjiMotor motor5(drivers, tap::motor::MOTOR5, CAN_BUS, false, "Test ID 5");
    tap::motor::DjiMotor motor6(drivers, tap::motor::MOTOR6, CAN_BUS, false, "Test ID 6");
    tap::motor::DjiMotor motor7(drivers, tap::motor::MOTOR7, CAN_BUS, false, "Test ID 7");
    tap::motor::DjiMotor motor8(drivers, tap::motor::MOTOR8, CAN_BUS, false, "Test ID 8");

    // Initialize all motors
    motor1.initialize();
    motor2.initialize();
    motor3.initialize();
    motor4.initialize();
    motor5.initialize();
    motor6.initialize();
    motor7.initialize();
    motor8.initialize();

    // Initialize remote
    remote.initialize();

    // Initialize drivers
    drivers->analog.init();
    drivers->pwm.init();
    drivers->digital.init();
    drivers->leds.init();
    drivers->can.initialize();
    drivers->errorController.init();
    drivers->remote.initialize();

    // Main control loop
    while (1)
    {
        // Read remote control data
        remote.read();

        if (sendMotorTimeout.execute())
        {
            // Check LEFT_SWITCH to enable motor testing
            if (remote.getSwitch(tap::communication::serial::Remote::Switch::LEFT_SWITCH) ==
                tap::communication::serial::Remote::SwitchState::UP)
            {
                // TEST ALL MOTOR IDs - ONE OF THESE SHOULD BE YOUR 2006 MOTOR
                motor1.setDesiredOutput(-8000);  // Test Motor ID 1
                motor2.setDesiredOutput(-8000);  // Test Motor ID 2
                motor3.setDesiredOutput(-8000);  // Test Motor ID 3
                motor4.setDesiredOutput(-8000);  // Test Motor ID 4
                motor5.setDesiredOutput(-8000);  // Test Motor ID 5
                motor6.setDesiredOutput(-8000);  // Test Motor ID 6
                motor7.setDesiredOutput(-8000);  // Test Motor ID 7
                motor8.setDesiredOutput(-8000);  // Test Motor ID 8
            }
            else
            {
                // Stop all motors
                motor1.setDesiredOutput(0);
                motor2.setDesiredOutput(0);
                motor3.setDesiredOutput(0);
                motor4.setDesiredOutput(0);
                motor5.setDesiredOutput(0);
                motor6.setDesiredOutput(0);
                motor7.setDesiredOutput(0);
                motor8.setDesiredOutput(0);
            }

            // Send CAN data to motors
            drivers->djiMotorTxHandler.encodeAndSendCanData();
        }

        // Poll for incoming CAN data
        drivers->canRxHandler.pollCanData();

        // Small delay
        modm::delay_us(100);
    }
    
    return 0;
}
