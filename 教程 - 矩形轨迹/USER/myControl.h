#ifndef __MYCONTROL_H
#define __MYCONTROL_H 

#include "sys.h"

void initiateCars(void);

void showADCData(void);
void setMove(float Vx, float Vy, float Vz);
void moveSquare(void);
void setTargetMotors(long int a,long int b, long int c, long int d);

#endif
