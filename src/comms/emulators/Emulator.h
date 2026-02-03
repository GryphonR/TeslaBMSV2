#pragma once

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <functional>

/**
 * @brief Base class for BMS emulators that output data to external systems.
 * 
 * Emulators periodically send CAN messages to make this BMS appear as 
 * a compatible device to external systems (e.g., Victron, inverters).
 */
class Emulator
{
public:
    using SendFunc = std::function<void(const CAN_message_t &)>;

    Emulator(SendFunc sendFunc) : _send(sendFunc) {}
    virtual ~Emulator() = default;

    /**
     * @brief Called periodically to send status messages.
     * Implement in derived classes to send protocol-specific messages.
     */
    virtual void update() = 0;

protected:
    void send(const CAN_message_t &msg) { _send(msg); }

private:
    SendFunc _send;
};
