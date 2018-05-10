#ifndef __MYCONTROL_H
#define __MYCONTROL_H 

#include "sys.h"

void initiateCars(void);

void LxGetVal(void);
void IRGetVal(void);
void showADCData(void);
void updateState(void);
u8 isPrevState(int state);
int countStates(int recent, int state);

void rotateRight(void);
void rotateLeft(void) ;
void back(void);
void setTargetMotors(long int a,long int b, long int c, long int d);

#endif
