#ifndef __MOTOR_H
#define __MOTOR_H
#include <sys.h>	 
  /**************************************************************************
作者：平衡小车之家
我的淘宝小店：http://shop114407458.taobao.com/
**************************************************************************/

#define IND   PBout(1)  
#define INC   PCout(5)  
#define INB   PBout(0)  
#define INA   PCout(4)  
#define ST    PCout(3) 
void MiniBalance_PWM_Init(u16 arr,u16 psc);
void MiniBalance_Motor_Init(void);
void Set_PWM_Final(u16 arr1,u16 arr2,u16 arr3,u16 arr4);
#endif
