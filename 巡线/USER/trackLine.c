#include "myControl.h"

u8 Flag_Left,Flag_Right,Flag_Direction=0;   //蓝牙遥控相关的变量
u8 Flag_Stop=1,Flag_Show=0;                 //停止标志位和 显示标志位 默认停止 显示打开
int Encoder_A,Encoder_B,Encoder_C,Encoder_D;          //速度
float Position_A,Position_B,Position_C,Position_D,Rate_A,Rate_B,Rate_C,Rate_D; //PID控制相关变量                      
long int Motor_A,Motor_B,Motor_C,Motor_D;        //电机PWM变量
long int Target_A,Target_B,Target_C,Target_D;     //电机目标值
int Voltage;                             //电池电压采样相关的变量
float Show_Data_Mb;                      //全局显示变量，用于显示需要查看的数据                         
u8 delay_50,delay_flag;                          //延时相关变量
u8 Run_Flag=0;  //蓝牙遥控相关变量和运行状态标志位
u8 rxbuf[8],Urxbuf[8],CAN_ON_Flag=0,Usart_ON_Flag=0,Usart_Flag,PID_Send,Flash_Send;  //CAN和串口控制相关变量
u8 txbuf[8],txbuf2[8],Turn_Flag;             //CAN发送相关变量
float Pitch,Roll,Yaw,Move_X,Move_Y,Move_Z;   //三轴角度和XYZ轴目标速度
u16 PID_Parameter[10],Flash_Parameter[10];  //Flash相关数组
float	Position_KP=20,Position_KI=0,Position_KD=20;  //位置控制PID参数
int RC_Velocity=800,Max_Velocity,RC_Position=5000;         //设置遥控的速度和位置值

/**************************************************************************
                              MY CONTROL START
**************************************************************************/
enum {  
  MID,        // 无偏离
  LEFT,       // 偏左
  RIGHT,      // 偏右
  WHITE       // 完全偏离
} DIRECTION;  // 小车偏向，枚举类型

u16 L1, L2, L3;    // 光敏传感器数值
u16 IR1, IR2, IR3; // 红外传感器Digital Data
float dist1, dist2, dist3; // 红外传感器得到的距离
int STATE;                 // 小车状态变量，每隔50ms会被推入队列STATE_ARR[]中
int STATE_ARR[20];         // FIFO队列，用于保存过去1s内的20个小车状态
u8 head_pt;                // 队列头指针

u16 L2_SET;                                   // L2的setpoint值
int err=0, err_diff=0, err_sum=0, err_prev=0; // err相关变量
float Kp1, Ki1, Kd1, Kp2, Ki2, Kd2;           // PID 相关变量
int du, du_left, du_right;                    // du操纵变量以及相关变量
int errL_over_cnt, errR_over_cnt;             // err偏左、偏右计数器

/**************************************************************************
函数功能： 巡线PID控制算法核心
**************************************************************************/
 void trackLineControl()
{

    // 设置PID参数
    Kp1=5;
    Kd1=30;

    // L2的setpoint值，与当时环境光线强度有关
    L2_SET = 2000; 
  
    // 判断偏离状态
    if (L1 < 1500 && L3 > L1) DIRECTION = RIGHT;
    else if (L3 < 1500 && L1 > L3) DIRECTION = LEFT;
    else if (L2 < L2_SET) DIRECTION = MID;
    else if (L1 > 1600 && L2 > 1800 && L3 > 1800) DIRECTION = WHITE;
    
    // 计算偏差err，PID计算操纵变量du
    err = L2 - L2_SET;
    err_diff = err - err_prev;    
    du = Kp1 * err + Kd1 * err_diff;
    if (du > 400) du = 400;
	
    // 没有偏向
    if (DIRECTION == MID) 
    {
      if (errL_over_cnt) setTargetMotors(800, 800, 400, 400);
      else if (errR_over_cnt) setTargetMotors(400, 400, 800, 800);
      else setTargetMotors(800, 800, 800, 800);
			errL_over_cnt=0;
			errR_over_cnt=0;
      du_left = du_right = 0;
    }
    
    // 偏右
    else if (DIRECTION == RIGHT)
    {
			errR_over_cnt++;
			errL_over_cnt=0;
      du_left = -du;
      du_right = du;
//      setTargetMotors(500-200*errR_over_cnt, 500-200*errR_over_cnt, 
//                      500+20*errR_over_cnt, 500+20*errR_over_cnt);
      setTargetMotors(800+du_left, 800+du_left, 800+du_right, 800+du_right);
    }
    
    // 偏左
    else if (DIRECTION == LEFT)
    {
      errL_over_cnt++;
      errR_over_cnt=0;
      du_left = du;
      du_right = -du;
//      setTargetMotors(800+20*errR_over_cnt, 500+20*errR_over_cnt,
//                      500-200*errR_over_cnt, 500-200*errR_over_cnt);
      setTargetMotors(800+du_left, 800+du_left, 800+du_right, 800+du_right);
    }
    
    
    // 更新记忆
    err_prev = err;
    STATE = DIRECTION;
//    if (STATE==WHITE && countStates(10, LEFT)>5) STATE=LEFT;
//    if (STATE==WHITE && countStates(10, RIGHT)>5) STATE=RIGHT;
    
    // 完全偏离
    if (errL_over_cnt>1 && STATE==WHITE)
    {
      setTargetMotors(800, 800, -300, -300);
    } 
    
    else if (errR_over_cnt>1 && STATE==WHITE)
    {
      setTargetMotors(-300, -300, 800, 800);
    } 
    if (errL_over_cnt>13) setTargetMotors(300, 300, -300, -300);
    if (errR_over_cnt>13) setTargetMotors(-300, -300, 300, 300);
    
    delay_flag=1;	
    delay_50=0;
    while(delay_flag);	       //通过MPU6050的INT中断实现的50ms精准延时	
}

/*********************************************************************
                        MAIN 函数入口
*********************************************************************/
int main(void)
{ 
	initiateCars();
	
	while (1) {
		
    LxGetVal();
    showADCData();
    //trackLineControl();
    
	}
}
/*********************************************************************
                        MAIN 函数结束
*********************************************************************/
