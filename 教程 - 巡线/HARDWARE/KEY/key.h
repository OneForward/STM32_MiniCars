#ifndef __KEY_H
#define __KEY_H	 
#include "sys.h"
#define KEY PBin(14)
#define MODE PBin(12)
void KEY_Init(void);          //������ʼ��
u8 click_N_Double (u8 time);  //��������ɨ���˫������ɨ��
u8 click(void);               //��������ɨ��
u8 Long_Press(void);

#endif 
