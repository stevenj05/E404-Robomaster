

#include "UART_Communication/Sender.hpp"
#include "tap/drivers.hpp" // For accessing drivers->uart

namespace tap::communication::serial
{

MotorSender::MotorSender(Drivers *drivers, Uart::UartPort port)
    : drivers(drivers), port(port)
{
}

void MotorSender::initialize()
{
    // Initializes the hardware UART port (e.g., Uart1)
    // We manually initialize the Uart port using the same logic as DJISerial::initialize so you can do it in main easily
    switch (this->port)
    {
        case Uart::UartPort::Uart1:
            drivers->uart.init<Uart::UartPort::Uart1, 115200>();
            break;
        case Uart::UartPort::Uart3:
            drivers->uart.init<Uart::UartPort::Uart3, 115200>();
            break;
        case Uart::UartPort::Uart6:
            drivers->uart.init<Uart::UartPort::Uart6, 115200>();
            break;
        default:
            break;
    }
}

void MotorSender::sendRpmCommand(int16_t rpm)
{
    // 1. Create the message container and configure the header
    // The constructor calculates and sets the CRC8 and dataLength
    MotorCommandMessage msg(seq_counter++);

    // 2. Set the message type
    msg.messageType = MOTOR_RPM_COMMAND_ID;
    
    // 3. Populate the data payload
    MotorCommandPayload payload = { .targetRpm = rpm };

    // 4. Copy the payload into the message's generic data array
    std::memcpy(msg.data, &payload, sizeof(MotorCommandPayload));

    // 5. Calculate and set the final CRC16 value
    msg.setCRC16();

    // 6. Send the raw bytes of the entire struct
    const uint8_t *buffer = reinterpret_cast<const uint8_t *>(&msg);
    std::size_t size = sizeof(msg.header) + sizeof(msg.messageType) + msg.header.dataLength + sizeof(msg.CRC16);
    
    drivers->uart.write(this->port, buffer, size);
    drivers->uart.flushWriteBuffer(this->port);
}

} // namespace tap::communication::serial