#include "myControl.h"

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
