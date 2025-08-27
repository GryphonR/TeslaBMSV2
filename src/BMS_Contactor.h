#pragma once

#include "pinouts.h"

enum ContactorState{
    CLOSED = 0,
    OPEN = 1
};

enum ContactorType{
    POSITIVE_CONTACOTR = 0,
    NEGATIVE_CONTACOTR = 1,
    PRECHARGE_CONTACOTR = 2,
    CHARGE_CONTACOTR = 3
};

enum ContactorConnection{
    H1 = 0,
    H2 = 1,
    H3 = 2, 
    H4 = 3,
    L5 = 4,
    L6 = 5, 
    L7 = 6,
    L8 = 7
};


class BMS_Contactor{

    public:
        BMS_Contactor();
        void close();
        void open();
        void set(ContactorState state);
        bool getState();
};