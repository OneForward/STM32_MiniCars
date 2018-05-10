#include "myControl.h"

#define Lx_CH_1 1
#define Lx_CH_2 2
#define Lx_CH_3 3
#define IR_CH_1 1
#define IR_CH_2 2
#define IR_CH_3 3

extern u16 L1, L2, L3;    // 光敏传感器数值
extern u16 IR1, IR2, IR3; // 红外传感器Digital Data
extern float dist1, dist2, dist3; // 红外传感器得到的距离
extern int STATE;                 // 判断小车或者迷宫状态
extern int STATE_ARR[20];         // FIFO队列，用于保存过去1s内的20个小车或者迷宫状态
extern u8 head_pt;                // 队列头指针
extern long int Target_A,Target_B,Target_C,Target_D; // 电机目标值 
extern int errL_over_cnt, errR_over_cnt;             // err偏左、偏右计数器
/*********************************************************************
                        MY CONTROL START
*********************************************************************/
enum STATE_FLAGS{           // 迷宫状态枚举类型
            STRAIGHT,       // 直线
            ROTATE_RIGHT,   // 右转
            ROTATE_LEFT,    // 左转
            LEFT_TMP,       // 左转-临时直线
            BACK            // 倒退
};

u8 t;

/**************************************************************************
函数功能：获取ADC引脚中巡线传感器的读数
**************************************************************************/
void LxGetVal() {
  
  L1 = Get_Adc(Lx_CH_1);
  L2 = Get_Adc(Lx_CH_2);
  L3 = Get_Adc(Lx_CH_3);
}

/**************************************************************************
函数功能：获取ADC引脚中巡线传感器的读数，以此更新距离dist1,dist2,
          以及对当前迷宫状态的判断
**************************************************************************/
void IRGetVal() {
  
    IR1 = Get_Adc(IR_CH_1);
    IR2 = Get_Adc(IR_CH_2);
    IR3 = Get_Adc(IR_CH_3);
    dist1 = 0.5/(0.00002*IR1 - 0.00045) - 2;
    dist2 = 0.5/(0.00002*IR2 - 0.00045) - 2;
    dist3 = 0.5/(0.00002*IR3 - 0.00045) - 2;
    if (dist1 > 45) dist1 = 45;
    if (dist1 < 3) dist1 = 3;
    if (dist2 > 45) dist2 = 45;
    if (dist2 < 3) dist2 = 3;
    if (dist3 > 45) dist3 = 45;
    if (dist3 < 3) dist3 = 3;
  
    updateState();
}

/**************************************************************************
函数功能：根据dist1, dist2, dist3判断当前迷宫状态

**************************************************************************/
void updateState() {
    
    if (dist1 >= 40)  
    {
      STATE=LEFT_TMP;
    }
    
    else if ((dist1 <= 17 && dist2 <= 20) || 
              dist1 <= 8 || 
            (dist1 <= 11 && dist3 >= 13 && dist3 <= 36)) 
    {
      STATE=ROTATE_RIGHT;
    }
    
    else if (dist1 <= 5 && dist2 <= 6)  
      STATE=BACK;
      
    else 
      STATE=STRAIGHT;
    
    if (STATE==LEFT_TMP && isPrevState(LEFT_TMP) &&
        isPrevState(ROTATE_RIGHT) != 1) STATE=ROTATE_LEFT;
    
    if (STATE==ROTATE_RIGHT && isPrevState(ROTATE_RIGHT) && dist1 > 8 && dist1 < 20 && 
        dist2 >= 14 && dist2 <= 35 && dist3 < 16) STATE=STRAIGHT;
}

/**************************************************************************
函数功能：判断先前250ms电机状态值是否为state
          是则返回1， 否则返回0
**************************************************************************/
u8 isPrevState(int state) {
  
  if (countStates(5, state) >= 2) return 1;
  else return 0;
}

/**************************************************************************
函数功能：统计在STATE_ARR[20]中最近recent个状态中值为state的状态数目
          
注： STATE_ARR[20]是一个FIFO队列，用于保存最近1s内的20个迷宫状态
**************************************************************************/
int countStates(int recent, int state) {
  
  int cnt=0;
  int pt=head_pt;
  for (t=0; t<recent; ++t) {
    pt+=20; pt--; pt %= 20;
    if (STATE_ARR[pt] == state)
      cnt++;
  }
  return cnt;
}

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
函数功能：倒退动作

**************************************************************************/
void back() {
  
  while(STATE==BACK){
      
      IRGetVal();
      showADCData();
      setTargetMotors(-200, -200, -200, -200);
    }
  
}

/**************************************************************************
函数功能：右转动作

注： 闭环控制，可能会超调
**************************************************************************/
void rotateRight() {
  
  while(STATE==ROTATE_RIGHT) {
    
    IRGetVal();
    showADCData();
    setTargetMotors(500, 500, -500, -500);
  }
  
}

/**************************************************************************
函数功能：180°左转动作

注： 开环控制，由于小车贴左边运动，故而包括前进、左转、前进、左转、前进五个子动作
**************************************************************************/
void rotateLeft() {
  
  u32 cnt=0;
  float T1 = 0.3, T2 = T1 + 0.5, 
        T3 = T2 + 0.7, T4 = T3 + 0.45,
        T5 = T4 + 0.5; 
  while(cnt < 20*T5){
      
      IRGetVal();
      showADCData();
      if (STATE == BACK) return;
    
      if (cnt < 20*T1) {
        setTargetMotors(800, 800, 800, 800);
      }
      
      else if (cnt < 20*T2){
        setTargetMotors(-800, -800, 800, 800);
      }
      else if (cnt < 20*T3){
        setTargetMotors(800, 800, 800, 800);
      }
      else if (cnt < 20*T4) {
        setTargetMotors(-800, -800, 800, 800);
      }
     
      else if (cnt < 20*T5) {
        setTargetMotors(800, 800, 800, 800);
      }
      else {
        setTargetMotors(0, 0, 0, 0);
      }
      cnt++;

      delay_flag=1;	
      delay_50=0;
      while(delay_flag);	       //通过MPU6050的INT中断实现的50ms精准延时	
  }
}

/**************************************************************************
函数功能：刷新显示当前相关传感器ADC数据与电机速度值

**************************************************************************/
void showADCData() {

  u16 testRxData;
  
  // 显示四个电机的转速
  OLED_ShowNumber(80, 0*12, Motor_A, 5, 12);
  OLED_ShowNumber(80, 1*12, Motor_B, 5, 12);
  OLED_ShowNumber(80, 2*12, Motor_C, 5, 12);
  OLED_ShowNumber(80, 3*12, Motor_D, 5, 12);

  // 显示巡线传感器读数
  testRxData = L1;OLED_ShowNumber(0, 0, testRxData, 5, 12);
  testRxData = L2;OLED_ShowNumber(0, 1*12, testRxData, 5, 12);
  testRxData = L3;OLED_ShowNumber(0, 2*12, testRxData, 5, 12);
  
//  for (t=0; t<16; ++t) { // 辅助判断PA0-15引脚的ADC数据是否正常
//    testRxData = Get_Adc(t);OLED_ShowNumber((t/4)*30, (t%4)*12, testRxData, 5, 12);
//  }
//  for (t=0; t<20; ++t) { // 显示过去1s内的20个状态值
//    testRxData = STATE_ARR[t];OLED_ShowNumber((t/3)*20, (t%3)*20, testRxData, 1, 16);
//  }

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
    L1=0;L2=0;L3=0;IR1=0;IR2=0;head_pt=0;
    errL_over_cnt=0;errR_over_cnt=0;
    for (t=0; t<20; ++t) {
      STATE_ARR[t] = STRAIGHT;
    }
  
}
/*********************************************************************
                        MY CONTROL END
*********************************************************************/
