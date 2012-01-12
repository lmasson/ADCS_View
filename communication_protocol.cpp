#include "communication_protocol.h"
#include "flags.h"
#include <stdio.h>


// Macro Definitions
#define HI_UINT16(a)    (((a) >> 8) & 0xFF)
#define LOW_UINT16(a)   ((a) & 0xFF)









void setMessage_simulationState(char* buffer, int state)
{
    // Empty previous buffer and set packet flag
    emptyBuffer(buffer);
    buffer[0] = SET_SIMULATION_STATE;

    // Set simulation state
    buffer[1] = state;
}



void setMessage_setQuaternions(char* buffer, double q0, double q1, double q2, double q3)
{
    int value;

    // Empty previous buffer and set packet flag
    emptyBuffer(buffer);
    buffer[0] = SET_QUATERNIONS;

    // Set first quaternion*1000 (high and low bytes)
    value = 10000*q0;
    buffer[1] = LOW_UINT16(value);
    buffer[2] = HI_UINT16(value);

    // Set second quaternion*1000 (high and low bytes)
    value = 10000*q1;
    buffer[3] = LOW_UINT16(value);
    buffer[4] = HI_UINT16(value);

    // Set third quaternion*1000 (high and low bytes)
    value = 10000*q2;
    buffer[5] = LOW_UINT16(value);
    buffer[6] = HI_UINT16(value);

    // Set fourth quaternion*1000 (high and low bytes)
    value = 10000*q3;
    buffer[7] = LOW_UINT16(value);
    buffer[8] = HI_UINT16(value);
}



void setMessage_setAlignmentPrecision(char* buffer, int precision)
{
    // Empty previous buffer and set packet flag
    emptyBuffer(buffer);
    buffer[0] = SET_ALIGNMENT_PRECISION;

    // Set alignment precision (fine or coarse)
    buffer[1] = precision;
}




void setMessage_motorDebug(char* buffer, int value)
{
    // Empty previous buffer and set packet flag
    emptyBuffer(buffer);
    buffer[0] = SET_MOTOR_DEBUG;

    // Set debug value
    buffer[1] = value;
}



void emptyBuffer(char* buffer)
{
    for(int i = 0; i < sizeof(buffer); i++)
        buffer[i] = 0;
}
