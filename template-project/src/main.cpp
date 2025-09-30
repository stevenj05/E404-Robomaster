#include "drivers.hpp"
#include "Board.hpp"
#include "modm/architecture/interface/clock.hpp"
#include "modm/architecture/interface/delay.hpp"
#include "tap/communication/serial/remote.hpp"

#include "drivetrain/Drivetrain.hpp"

#define EXIT_SUCCESS 0

int main()
{
    // 1) Get drivers
    src::Drivers* drivers = src::DoNotUse_getDrivers();
    Board::initialize();
    drivers->can.initialize();
    drivers->remote.initialize();

    // 2) Set up drivetrain
    robot::Drivetrain drive(drivers, robot::makeDefaultDrivetrainConfig());
    drive.init();

    // 3) Timing
    modm::PreciseClock clock{};
    auto last = modm::chrono::micro_clock::now();
    constexpr float LOOP_HZ = 500.0f;
    const uint32_t loopDelayUs = static_cast<uint32_t>(1'000'000.0f / LOOP_HZ);

    // 4) Main loop
    while (true)
    {
        drivers->remote.read();

        // Example: map joystick channels to vx, vy, w
        float vx = drivers->remote.getChannel(tap::communication::serial::Remote::Channel::LEFT_VERTICAL) * 1.0f;
        float vy = drivers->remote.getChannel(tap::communication::serial::Remote::Channel::LEFT_HORIZONTAL) * 1.0f;
        float w  = drivers->remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_HORIZONTAL) * 2.0f;

        // Compute dt
        auto now = modm::chrono::micro_clock::now();
        float dt = (now - last).count() / 1e6f;
        last = now;
        if (dt <= 0.0f) dt = 1.0f / LOOP_HZ;

        // Pass commands to drivetrain
        drive.setCommand(vx, vy, w);
        drive.update(dt);

        // Send motor outputs
        drivers->djiMotorTxHandler.encodeAndSendCanData();
        drivers->canRxHandler.pollCanData();

        modm::delay_us(loopDelayUs);
    }

    return EXIT_SUCCESS;
}
