
#ifndef MOTOR_COMMAND_PROTOCOL_HPP_
#define MOTOR_COMMAND_PROTOCOL_HPP_

#include <cstdint>

namespace tap::communication::serial
{

// 1. Define the unique Message Type ID
// Use a value high enough to avoid conflicts with standard DJI messages.
static constexpr uint16_t MOTOR_RPM_COMMAND_ID = 0x1000;

// 2. Define the data payload structure
#pragma pack(push, 1)
struct MotorCommandPayload
{
    // Motor RPM from -1600 to 1600 fits perfectly in a signed 16-bit integer. I think these are the max RPM Values ...??.
    int16_t targetRpm;
};
#pragma pack(pop)

// 3. Define the specific Serial Message type for this payload size
using MotorCommandMessage = DJISerial::SerialMessage<sizeof(MotorCommandPayload)>;

} // namespace tap::communication::serial

#endif // MOTOR_COMMAND_PROTOCOL_HPP_