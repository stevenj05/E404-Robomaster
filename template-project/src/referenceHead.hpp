#pragma once

#include "tap/board/board.hpp"

#include "modm/architecture/interface/delay.hpp"
#include <iostream>
/* controller includes ------------------------------------------------------*/
#include "tap/communication/serial/remote.hpp"

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

#include "tap/communication/sensors/imu/bmi088/bmi088.hpp"

#include "tap/communication/sensors/imu/imu_terminal_serial_handler.hpp"

#include "tap/communication/serial/terminal_serial.hpp"
