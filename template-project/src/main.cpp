// ========== main.cpp ==========
// Standard includes
#include "Constants.hpp"

#ifdef PLATFORM_HOSTED
#include "tap/communication/tcp-server/tcp_server.hpp"
#include "tap/motor/motorsim/dji_motor_sim_handler.hpp"
#endif

#include "drivers_singleton.hpp"

#include "subsystems/Drivetrain.hpp"
#include "subsystems/Flywheels.hpp"
#include "subsystems/Gimbal.hpp"

using namespace Constants;

// Timers
tap::arch::PeriodicMilliTimer sendMotorTimeout(1000.0f / main_loop_freq_hz);
tap::arch::PeriodicMilliTimer updateImuTimeout(2);

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

    initializeIo(drivers);
    Board::initialize();

    drivers->mpu6500.init(500.f, 0.1f, 0.0f);
    drivers->pwm.setTimerFrequency(tap::gpio::Pwm::Timer::TIMER8, pwm_freq_hz);

#ifdef PLATFORM_HOSTED
    tap::motor::motorsim::DjiMotorSimHandler::getInstance()->resetMotorSims();
    tap::communication::TCPServer::MainServer()->getConnection();
#endif

    double yaw = 0;
    double pitch = 0;

    // Subsystems
    Drivetrain driveTrain(drivers, drivers->remote, yaw);
    Gimbal gimbal(drivers, drivers->remote, yaw, pitch);
    Flywheels flywheels(drivers, drivers->remote);

    // Initialize motors + PID
    driveTrain.initialize();
    gimbal.initialize();
    flywheels.initialize();

    while (true) {
        updateIo(drivers);

        // Update IMU
        if (updateImuTimeout.execute()) {
            drivers->mpu6500.periodicIMUUpdate();
            yaw = drivers->mpu6500.getYaw();
            pitch = drivers->mpu6500.getPitch();
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

        drivers->djiMotorTxHandler.encodeAndSendCanData();
    }

    return 0;
}


// -----------------------------------------------------------------------------
// I/O Initialization
// -----------------------------------------------------------------------------
static void initializeIo(src::Drivers* drivers) {
    drivers->analog.init();
    drivers->pwm.init();
    drivers->digital.init();
    drivers->leds.init();
    drivers->can.initialize();
    drivers->errorController.init();
    drivers->remote.initialize();
    drivers->mpu6500.init(main_loop_freq_hz, 0.1, 0);
    drivers->refSerial.initialize();
    drivers->terminalSerial.initialize();
    drivers->schedulerTerminalHandler.init();
    drivers->djiMotorTerminalSerialHandler.init();
}

// -----------------------------------------------------------------------------
// I/O Update
// -----------------------------------------------------------------------------
static void updateIo(src::Drivers* drivers) {
#ifdef PLATFORM_HOSTED
    tap::motor::motorsim::DjiMotorSimHandler::getInstance()->updateSims();
#endif
    drivers->canRxHandler.pollCanData();
    drivers->refSerial.updateSerial();
    drivers->remote.read();
    drivers->mpu6500.read();
}
