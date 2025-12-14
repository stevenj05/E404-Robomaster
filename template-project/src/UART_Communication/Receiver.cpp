
#include "UART_Communication/Receiver.hpp"

namespace tap::communication::serial
{

MotorReceiver::MotorReceiver(Drivers *drivers, Uart::UartPort port)
    : DJISerial(drivers, port) // Call base class constructor
{
}

void MotorReceiver::update()
{
    // Inherited from DJISerial, this runs the s machine to receive bytes
    DJISerial::updateSerial();
}

void MotorReceiver::messageReceiveCallback(const ReceivedSerialMessage &completeMessage)
{
    // 1. Check if the message type matches the one we care about
    if (completeMessage.messageType == MOTOR_RPM_COMMAND_ID)
    {
        // 2. Verify the data length is correct (2 bytes for int16_t)
        if (completeMessage.header.dataLength == sizeof(MotorCommandPayload))
        {
            // 3. Cast the received data back into the struct type
            const MotorCommandPayload *payload = 
                reinterpret_cast<const MotorCommandPayload *>(completeMessage.data);
            
            // 4. Extract the value and store it
            lastReceivedRpm = payload->targetRpm;
        }
    }
   
}

} // namespace tap::communication::serial