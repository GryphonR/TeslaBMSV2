#pragma once

#include <FlexCAN_T4.h>

/**
 * @brief Interface for any device that wants to receive CAN messages.
 */
class CanListener
{
public:
    virtual ~CanListener() {}

    /**
     * @brief Called by the BusManager when a new message arrives.
     * @param msg The incoming CAN message.
     * @return true if the message was handled/consumed, false otherwise.
     */
    virtual bool onReceive(const CAN_message_t &msg) = 0;
};
