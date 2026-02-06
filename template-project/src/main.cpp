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

    turret.initialize();
    gimbal.initialize();
    chassis.initialize();

    drivers->remote.initialize();
    drivers->can.initialize();

    while (1)
    {
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