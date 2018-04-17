#include "myControl.h"

u8 Flag_Left,Flag_Right,Flag_Direction=0;   //����ң����صı���
u8 Flag_Stop=1,Flag_Show=0;                 //ֹͣ��־λ�� ��ʾ��־λ Ĭ��ֹͣ ��ʾ��
int Encoder_A,Encoder_B,Encoder_C,Encoder_D;          //�ٶ�
float Position_A,Position_B,Position_C,Position_D,Rate_A,Rate_B,Rate_C,Rate_D; //PID������ر���                      
long int Motor_A,Motor_B,Motor_C,Motor_D;        //���PWM����
long int Target_A,Target_B,Target_C,Target_D;     //���Ŀ��ֵ
int Voltage;                             //��ص�ѹ������صı���
float Show_Data_Mb;                      //ȫ����ʾ������������ʾ��Ҫ�鿴������                         
u8 delay_50,delay_flag;                          //��ʱ��ر���
u8 Run_Flag=0;  //����ң����ر���������״̬��־λ
u8 rxbuf[8],Urxbuf[8],CAN_ON_Flag=0,Usart_ON_Flag=0,Usart_Flag,PID_Send,Flash_Send;  //CAN�ʹ��ڿ�����ر���
u8 txbuf[8],txbuf2[8],Turn_Flag;             //CAN������ر���
float Pitch,Roll,Yaw,Move_X,Move_Y,Move_Z;   //����ǶȺ�XYZ��Ŀ���ٶ�
u16 PID_Parameter[10],Flash_Parameter[10];  //Flash�������
float	Position_KP=20,Position_KI=0,Position_KD=20;  //λ�ÿ���PID����
int RC_Velocity=800,Max_Velocity,RC_Position=5000;         //����ң�ص��ٶȺ�λ��ֵ

/**************************************************************************
                              MY CONTROL START
**************************************************************************/
enum {  
  MID,        // ��ƫ��
  LEFT,       // ƫ��
  RIGHT,      // ƫ��
  WHITE       // ��ȫƫ��
} DIRECTION;  // С��ƫ��ö������

u16 L1, L2, L3;    // ������������ֵ
u16 IR1, IR2, IR3; // ���⴫����Digital Data
float dist1, dist2, dist3; // ���⴫�����õ��ľ���
int STATE;                 // С��״̬������ÿ��50ms�ᱻ�������STATE_ARR[]��
int STATE_ARR[20];         // FIFO���У����ڱ����ȥ1s�ڵ�20��С��״̬
u8 head_pt;                // ����ͷָ��

u16 L2_SET;                                   // L2��setpointֵ
int err=0, err_diff=0, err_sum=0, err_prev=0; // err��ر���
float Kp1, Ki1, Kd1, Kp2, Ki2, Kd2;           // PID ��ر���
int du, du_left, du_right;                    // du���ݱ����Լ���ر���
int errL_over_cnt, errR_over_cnt;             // errƫ��ƫ�Ҽ�����

/**************************************************************************
�������ܣ� Ѳ��PID�����㷨����
**************************************************************************/
 void trackLineControl()
{

    // ����PID����
    Kp1=5;
    Kd1=30;

    // L2��setpointֵ���뵱ʱ��������ǿ���й�
    L2_SET = 2000; 
  
    // �ж�ƫ��״̬
    if (L1 < 1500 && L3 > L1) DIRECTION = RIGHT;
    else if (L3 < 1500 && L1 > L3) DIRECTION = LEFT;
    else if (L2 < L2_SET) DIRECTION = MID;
    else if (L1 > 1600 && L2 > 1800 && L3 > 1800) DIRECTION = WHITE;
    
    // ����ƫ��err��PID������ݱ���du
    err = L2 - L2_SET;
    err_diff = err - err_prev;    
    du = Kp1 * err + Kd1 * err_diff;
    if (du > 400) du = 400;
	
    // û��ƫ��
    if (DIRECTION == MID) 
    {
      if (errL_over_cnt) setTargetMotors(800, 800, 400, 400);
      else if (errR_over_cnt) setTargetMotors(400, 400, 800, 800);
      else setTargetMotors(800, 800, 800, 800);
			errL_over_cnt=0;
			errR_over_cnt=0;
      du_left = du_right = 0;
    }
    
    // ƫ��
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
    
    // ƫ��
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
    
    
    // ���¼���
    err_prev = err;
    STATE = DIRECTION;
//    if (STATE==WHITE && countStates(10, LEFT)>5) STATE=LEFT;
//    if (STATE==WHITE && countStates(10, RIGHT)>5) STATE=RIGHT;
    
    // ��ȫƫ��
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
    while(delay_flag);	       //ͨ��MPU6050��INT�ж�ʵ�ֵ�50ms��׼��ʱ	
}

/*********************************************************************
                        MAIN �������
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
                        MAIN ��������
*********************************************************************/
