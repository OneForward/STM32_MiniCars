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

/*************************************************************************
                              MY CONTROL START
**************************************************************************/
enum STATE_FLAGS{           // �Թ�״̬ö������
            STRAIGHT,       // ֱ��
            ROTATE_RIGHT,   // ��ת
            ROTATE_LEFT,    // ��ת
            LEFT_TMP,       // ��ת-��ʱֱ��
            BACK            // ����
};

int STATE;                 // �ж��Թ�״̬
int STATE_ARR[20];         // FIFO���У����ڱ����ȥ1s�ڵ�20���Թ�״̬
u8 head_pt;                // ����ͷָ��

u16 IR1, IR2, IR3;                // ���⴫����Digital Data
u16 L1, L2, L3;                   // ������������ֵ
float dist1, dist2, dist3;        // ��������ת�����ʵ�ʾ���
int err, err_diff, err_sum=0, err_prev=0; // PID err
u16 L2_SET, DIST_SET;                     // PID set point
float Kp1, Ki1, Kd1, Kp2, Ki2, Kd2;       // PID ����
int du, du_left, du_right;                // PID ���ݱ���
int errL_over_cnt, errR_over_cnt;         // errƫ��ƫ�Ҽ�����

void followWalls(void);
void goStraight(void);

/*********************************************************************
                        MAIN �������
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
                        MAIN ��������
*********************************************************************/

/**************************************************************************
�������ܣ� �Թ������㷨����
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
�������ܣ� �Թ������㷨��ֱ����ǽ������PID����
**************************************************************************/
void goStraight() {

	// ����PID����
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
