#include "myControl.h"

/**************************************************************************
函数功能：设定电机目标转速
          
**************************************************************************/
void setTargetMotors(long int a,long int b, long int c, long int d) {
  
  Target_A=a,Target_B=b,Target_C=c,Target_D=d; 
}

/**************************************************************************
函数功能：设定小车重心x, y, z目标速度

注：x为水平速度，y为纵向速度，z为自旋角速度          
**************************************************************************/
void setMove(float Vx, float Vy, float Vz)
{
  Target_A   = -Vx+Vy-Vz*0.1885;
  Target_B   = +Vx+Vy-Vz*0.1885;
  Target_C   = -Vx+Vy+Vz*0.1885;
  Target_D   = +Vx+Vy+Vz*0.1885;
}

/**************************************************************************
函数功能：矩形轨迹动作

**************************************************************************/
void moveSquare(void) {
  
  u32 cnt=0;
    float T1 = 1, T2 = T1 + 1, 
        T3 = T2 + 1, T4 = T3 + 1,
        T5 = T4 + 1;
  while(cnt < 20*T5){
      
      showADCData();
    
      if (cnt < 20*T1) {
        setMove(0, 800, 0);
      }
      
      else if (cnt < 20*T2){
        setMove(800, 0, 0);
      }
      else if (cnt < 20*T3){
        setMove(0, -800, 0);
      }
      else if (cnt < 20*T4) {
        setMove(-800, 0, 0);
      }
      else{
        setMove(0, 0, 0);
      }

      cnt++;

      delay_flag=1; 
      delay_50=0;
      while(delay_flag);         //通过MPU6050的INT中断实现的50ms精准延时 
  }
}


/**************************************************************************
函数功能：刷新显示当前相关传感器ADC数据与电机速度值

**************************************************************************/
void showADCData() {
  
  //=============显示3轴角度===============//	
    	OLED_ShowString(0,0,"X:");
		if(Pitch<0)		OLED_ShowNumber(15,0,Pitch+360,3,12);
		else					OLED_ShowNumber(15,0,Pitch,3,12);	
       
  	OLED_ShowString(40,0,"Y:");
		if(Roll<0)		OLED_ShowNumber(55,0,Roll+360,3,12);
		else					OLED_ShowNumber(55,0,Roll,3,12);	
	
	   OLED_ShowString(80,0,"Z:");
		if(Yaw<0)		  OLED_ShowNumber(95,0,Yaw+360,3,12);
		else					OLED_ShowNumber(95,0,Yaw,3,12);	
  
  OLED_Refresh_Gram();
}
/**************************************************************************
函数功能：初始化小车

**************************************************************************/
void initiateCars() {
  
    Stm32_Clock_Init(9);            //=====系统时钟设置
    delay_init(72);                 //=====延时初始化
    JTAG_Set(JTAG_SWD_DISABLE);     //=====关闭JTAG接口
    JTAG_Set(SWD_ENABLE);           //=====打开SWD接口 可以利用主板的SWD接口调试
    LED_Init();                     //=====初始化与 LED 连接的硬件接口
    KEY_Init();                     //=====按键初始化
    if(MODE==0)Run_Flag=1;          //=====启动的过程中，根据模式选择开关确定进入位置模式还是速度模式
    else Run_Flag=0;                //=====启动的过程中，根据模式选择开关确定进入位置模式还是速度模式
    OLED_Init();                    //=====OLED初始化
    uart_init(72,128000);           //=====串口1初始化
    uart2_init(36,9600);            //=====串口2初始化
    uart3_init(36,115200);          //=====串口3初始化 
    Adc_Init();                     //=====adc初始化
    IIC_Init();                     //=====IIC初始化
    MPU6050_initialize();           //=====MPU6050初始化 
    DMP_Init();                     //=====初始化DMP     
    if(KEY==0) Flash_Read();        //=====读取Flash里面的参数
    delay_ms(1000);                 //=====延时等待初始化稳定
    EXTI_Init();                    //=====MPU6050 5ms定时中断初始化
    CAN1_Mode_Init(1,2,3,6,0);      //=====CAN初始化
    MiniBalance_PWM_Init(7199,14);  //=====初始化PWM 用于驱动电机
    delay_ms(1000);
    
    /********************变量初始化*********************/
    Run_Flag=0;Flag_Left=0;Flag_Right=0;Flag_Direction=0;
    Motor_A=0,Motor_B=0,Motor_C=0,Motor_D=0;        //电机PWM变量
    Target_A=0,Target_B=0,Target_C=0,Target_D=0;     //电机目标值
}
/*********************************************************************
                        MY CONTROL END
*********************************************************************/
