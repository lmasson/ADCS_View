#ifndef COMMUNICATION_PROTOCOL_H
#define COMMUNICATION_PROTOCOL_H






//static char sendBuffer[30];



void setMessage_simulationState(char* buffer, int state);
void setMessage_setQuaternions(char* buffer, double q0, double q1, double q2, double q3);
void setMessage_setAlignmentPrecision(char* buffer, int precision);
void setMessage_motorDebug(char* buffer, int value);

void emptyBuffer(char* buffer);

#endif // COMMUNICATION_PROTOCOL_H
