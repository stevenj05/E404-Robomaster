// ========== main.cpp ==========
// Standard includes
#include "Constants.hpp"
#ifdef PLATFORM_HOSTED
#include "tap/communication/tcp-server/tcp_server.hpp"
#include "tap/motor/motorsim/dji_motor_sim_handler.hpp"
#endif


// Include referenceHead FIRST to establish base taproot environment
#include "subsystems/Drivetrain.hpp"
#include "subsystems/Flywheels.hpp"
#include "subsystems/Gimbal.hpp"

using namespace Constants;

// Timers
tap::arch::PeriodicMilliTimer sendMotorTimeout(1000.0f / MAIN_LOOP_FREQUENCY);
tap::arch::PeriodicMilliTimer updateImuTimeout(2);

// Clock / timing
modm::PreciseClock theClock{};
modm::chrono::micro_clock::time_point epoch;
using MicrosecondDuration = modm::PreciseClock::duration;

// -----------------------------------------------------------------------------
// Function declarations
// -----------------------------------------------------------------------------
static void initializeIo(src::Drivers* drivers);
static void updateIo(src::Drivers* drivers);

// -----------------------------------------------------------------------------
// Main
// -----------------------------------------------------------------------------
int main() {
#ifdef PLATFORM_HOSTED
    std::cout << "Simulation starting..." << std::endl;
#endif

    src::Drivers* drivers = src::DoNotUse_getDrivers();
    tap::communication::serial::Remote remote(drivers);

    Board::initialize();
    initializeIo(drivers);

    remote.initialize();
    drivers->bmi088.initialize(500.f, 0.1f, 0.0f);
    drivers->pwm.setTimerFrequency(tap::gpio::Pwm::Timer::TIMER8, PWM_FREQUENCY);

#ifdef PLATFORM_HOSTED
    tap::motor::motorsim::DjiMotorSimHandler::getInstance()->resetMotorSims();
    tap::communication::TCPServer::MainServer()->getConnection();
#endif

    // Subsystems
    Drivetrain driveTrain(remote);
    Gimbal gimbal(remote);
    Flywheels flywheels(remote);

    // Initialize motors + PID
    driveTrain.initialize();
    gimbal.initialize();
    flywheels.initialize();

    while (true) {
        remote.read();

        // Update IMU
        if (updateImuTimeout.execute()) {
            drivers->bmi088.periodicIMUUpdate();
        }

        // Update all subsystems
        driveTrain.update();
        gimbal.update();
        flywheels.update();

        // Send motor commands periodically
        if (sendMotorTimeout.execute()) {
            driveTrain.tick();  
            gimbal.tick();
            flywheels.tick();
        }

        updateIo(drivers);
        modm::delay_us(100);
    }

    return 0;
}


// -----------------------------------------------------------------------------
// I/O Initialization
// -----------------------------------------------------------------------------
static void initializeIo(src::Drivers* drivers)
{
    drivers->analog.init();
    drivers->pwm.init();
    drivers->digital.init();
    drivers->leds.init();
    drivers->can.initialize();
    drivers->errorController.init();
    drivers->remote.initialize();

    //Added initialization of bmi088 (only in TYPE-C board)
    drivers->bmi088.initialize(MAIN_LOOP_FREQUENCY, 0.1f, 0.0f);
    drivers->bmi088.requestRecalibration();

    drivers->refSerial.initialize();
    drivers->terminalSerial.initialize();
    drivers->schedulerTerminalHandler.init();
    drivers->djiMotorTerminalSerialHandler.init();
}

// -----------------------------------------------------------------------------
// I/O Update
// -----------------------------------------------------------------------------
static void updateIo(src::Drivers* drivers)
{
#ifdef PLATFORM_HOSTED
    tap::motor::motorsim::DjiMotorSimHandler::getInstance()->updateSims();
#endif
    drivers->canRxHandler.pollCanData();
    drivers->refSerial.updateSerial();
    drivers->remote.read();
    //there is no read for the bmi088
}
