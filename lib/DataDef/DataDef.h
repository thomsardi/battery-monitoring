#ifndef DataDef_H
#define DataDef_H

#include <stdint.h>

struct CellBal
{
    uint8_t cellBalAddress;
    uint8_t databal;
};

struct ShortingPins
{
    //Refer to CELLBAL Register
    int pinList1[3] = {1,2,3}; //CB2, CB3, CB4
    int pinList2[2] = {2,3}; //CB3, CB4
};


#endif