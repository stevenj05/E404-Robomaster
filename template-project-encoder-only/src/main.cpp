#include "drivers_singleton.hpp"
#include "tap/architecture/periodic_timer.hpp"
#include "tap/board/board.hpp"
#include "subsystems/turret.hpp"
#include "subsystems/gimbal.hpp"
#include "subsystems/chassis.hpp"

static constexpr float MAIN_LOOP_FREQUENCY = 500.0f;

tap::arch::PeriodicMilliTimer sendMotorTimeout(1000.0f / MAIN_LOOP_FREQUENCY);

int main()
{
    src::Drivers *drivers = src::DoNotUse_getDrivers();
    Board::initialize();

    // Initialize subsystems
    Turret turret(drivers, &drivers->remote);
    Gimbal gimbal(drivers);
    Chassis chassis(drivers);
    chassis.setGimbal(&gimbal);

    turret.initialize();
    gimbal.initialize();
    chassis.initialize();

    drivers->remote.initialize();
    drivers->can.initialize();
    
    // Initialize BMI088 IMU
    // Arguments: sampleFrequency (Hz), Kp, Ki for Mahony filter
    drivers->bmi088.initialize(MAIN_LOOP_FREQUENCY, 2.0f, 0.0f);

    while (1)
    {
        // Must update IMU at 500Hz loop rate
        drivers->bmi088.periodicIMUUpdate();
        
        drivers->remote.read();
        drivers->canRxHandler.pollCanData();

        if (sendMotorTimeout.execute())
        {
            // Update subsystems - they handle their own logic
            turret.update();
            gimbal.update();
            chassis.update();

            // Send motor commands
            turret.sendMotorCommands();
            gimbal.sendMotorCommands();
            chassis.sendMotorCommands();
        }

        modm::delay_us(100);
    }
    return 0;
}