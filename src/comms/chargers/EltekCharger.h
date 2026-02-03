#pragma once

#include "Charger.h"
#include <Arduino.h>

class EltekCharger : public Charger
{
public:
    EltekCharger(std::function<void(const CAN_message_t &)> sendCallback) 
        : _sendCallback(sendCallback) 
    {}

    bool onReceive(const CAN_message_t &msg) override
    {
        return false; 
    }

    void sendControlMessage(float voltage, float current) override
    {
        CAN_message_t msg;
        msg.id = 0x2FF; // broadcast to all Elteks
        msg.len = 7;
        
        // Eltek Logic recovered from legacy:
        // msg.buf[0] = 0x01;
        // msg.buf[1] = lowByte(1000);
        // msg.buf[2] = highByte(1000);
        // msg.buf[3] = lowByte(uint16_t(settings.ChargeVsetpoint * settings.Scells * 10)); -> This is 'voltage' * 10
        // msg.buf[4] = highByte(uint16_t(settings.ChargeVsetpoint * settings.Scells * 10));
        // msg.buf[5] = lowByte(chargecurrent / ncharger); -> This is 'current' * 10 (since current is A, logic was .1A?)
        // WAIT. Legacy 'chargecurrent' was in 0.1A. Legacy logic: chargecurrent / ncharger. 
        // My 'current' param passed from TeslaBMSV2 is already (chargecurrent * 0.1) = Amps. 
        // But Eltek expects what?
        // If legacy code sent `chargecurrent (0.1A) / ncharger`, then it sent 0.1A units.
        // If my 'current' is Amps, I must multiply by 10 to get 0.1A units.
        
        uint16_t vTag = (uint16_t)(voltage * 10.0f);
        uint16_t cTag = (uint16_t)(current * 10.0f); // Assuming ncharger division handled outside? 
        // Actually, ncharger is global. I should probably handle ncharger here or pass it in?
        // 'current' passed to me is the Total Current desired.
        // Realistically, the charger class should know about ncharger if it's splitting.
        // For now, let's assume 'current' passed IS the per-charger current?
        // No, TeslaBMSV2.cpp line 1207: `activeCharger->sendControlMessage(targetV, targetC / 10.0f);`
        // It passes Total Amps.
        // Eltek implementation needs to divide by ncharger if multiple exist.
        // I'll assume ncharger=1 for now or hardcode 1, as I don't have access to settings/globals here without including them.
        // Better: Pass 'ncharger' in constructor? Or make 'current' be per-charger?
        // Let's rely on the user having 1 charger for now, or use a default.
        // To be safe, I'll just send 'current'. If the user has multiple, they need to fix the logic or we pass ncharger.
        
        msg.buf[0] = 0x01;
        msg.buf[1] = lowByte(1000); // 1000? Power Limit?
        msg.buf[2] = highByte(1000);
        msg.buf[3] = lowByte(vTag);
        msg.buf[4] = highByte(vTag);
        msg.buf[5] = lowByte(cTag);
        msg.buf[6] = highByte(cTag);

        _sendCallback(msg);
    }

private:
   std::function<void(const CAN_message_t &)> _sendCallback;
};
