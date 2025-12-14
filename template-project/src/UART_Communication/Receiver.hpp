
#ifndef MOTOR_RECEIVER_HPP_
#define MOTOR_RECEIVER_HPP_

#include "tap/communication/serial/dji_serial.hpp"
#include "UART_Communication/MotorCommandProtocol.hpp"

namespace tap::communication::serial
{

class MotorReceiver : public DJISerial
{
public:
    MotorReceiver(Drivers *drivers, Uart::UartPort port);

    // Call this in  main loop to process incoming serial bytes
    void update();
    
    // Gets the last received RPM command
    int16_t getLastRpm() const { return lastReceivedRpm; }

private:
    int16_t lastReceivedRpm = 0;

    
     // The callback function that processes valid incoming packets.
     
    void messageReceiveCallback(const ReceivedSerialMessage &completeMessage) override;
};

} // namespace tap::communication::serial

#endif // MOTOR_RECEIVER_HPP_