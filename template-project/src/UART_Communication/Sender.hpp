#ifndef MOTOR_SENDER_HPP_
#define MOTOR_SENDER_HPP_

#include "tap/communication/serial/dji_serial.hpp"
#include "UART_Communication/MotorCommandProtocol.hpp"
// Need full path for Drivers in C++
#include "drivers.hpp" 

namespace tap::communication::serial
{

class MotorSender
{
public:
    MotorSender(Drivers *drivers, Uart::UartPort port);
    
    void initialize();

    /**
     * @brief Creates and sends a motor command packet.
     * @param rpm The target RPM (-1600 to 1600).
     */
    void sendRpmCommand(int16_t rpm);

private:
    Drivers *drivers;
    Uart::UartPort port;
    uint8_t seq_counter = 0;
};

} 

#endif 