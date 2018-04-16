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

/*************************************************************************
                              MY CONTROL START
**************************************************************************/
enum STATE_FLAGS{           // 迷宫状态枚举类型
            STRAIGHT,       // 直线
            ROTATE_RIGHT,   // 右转
            ROTATE_LEFT,    // 左转
            LEFT_TMP,       // 左转-临时直线
            BACK            // 倒退
};

int STATE;                 // 判断迷宫状态
int STATE_ARR[20];         // FIFO队列，用于保存过去1s内的20个迷宫状态
u8 head_pt;                // 队列头指针

u16 IR1, IR2, IR3;                // 红外传感器Digital Data
u16 L1, L2, L3;                   // 光敏传感器数值
float dist1, dist2, dist3;        // 红外数据转化后的实际距离
int err, err_diff, err_sum=0, err_prev=0; // PID err
u16 L2_SET, DIST_SET;                     // PID set point
float Kp1, Ki1, Kd1, Kp2, Ki2, Kd2;       // PID 参数
int du, du_left, du_right;                // PID 操纵变量
int errL_over_cnt, errR_over_cnt;         // err偏左、偏右计数器

void followWalls(void);
void goStraight(void);

/*********************************************************************
                        MAIN 函数入口
*********************************************************************/
int main(void) 
{   
  
    initiateCars();
    while (1) {
      
        IRGetVal();
        showADCData();
        followWalls();
    }
}
/*********************************************************************
                        MAIN 函数结束
*********************************************************************/

/**************************************************************************
函数功能： 迷宫控制算法核心
**************************************************************************/
void followWalls() 
{
  
	if (STATE==ROTATE_RIGHT) 
      rotateRight();
	
	else if (STATE==ROTATE_LEFT) 
      rotateLeft();
	
  else if (STATE==BACK) 
      back();
  
  else
      goStraight();
  
}

/**************************************************************************
函数功能： 迷宫控制算法中直线贴墙动作的PID控制
**************************************************************************/
void goStraight() {

	// 设置PID参数
    Kp2=35;
    Kd2=10;
     
    DIST_SET = 11;
    
    err = dist1 - DIST_SET;
    err_diff = err - err_prev;
    du = Kp2 * err + Kd2 * err_diff;
    if (du > 200) du = 200;
    if (du < -200) du = -200;
  
    err_prev = err;  
    setTargetMotors(700-du, 700-du, 700+du, 700+du);
}
