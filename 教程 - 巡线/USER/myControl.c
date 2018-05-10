#include "myControl.h"

#define Lx_CH_1 1
#define Lx_CH_2 2
#define Lx_CH_3 3
#define IR_CH_1 1
#define IR_CH_2 2
#define IR_CH_3 3

extern u16 L1, L2, L3;    // ������������ֵ
extern u16 IR1, IR2, IR3; // ���⴫����Digital Data
extern float dist1, dist2, dist3; // ���⴫�����õ��ľ���
extern int STATE;                 // �ж�С�������Թ�״̬
extern int STATE_ARR[20];         // FIFO���У����ڱ����ȥ1s�ڵ�20��С�������Թ�״̬
extern u8 head_pt;                // ����ͷָ��
extern long int Target_A,Target_B,Target_C,Target_D; // ���Ŀ��ֵ 
extern int errL_over_cnt, errR_over_cnt;             // errƫ��ƫ�Ҽ�����
/*********************************************************************
                        MY CONTROL START
*********************************************************************/
enum STATE_FLAGS{           // �Թ�״̬ö������
            STRAIGHT,       // ֱ��
            ROTATE_RIGHT,   // ��ת
            ROTATE_LEFT,    // ��ת
            LEFT_TMP,       // ��ת-��ʱֱ��
            BACK            // ����
};

u8 t;

/**************************************************************************
�������ܣ���ȡADC������Ѳ�ߴ������Ķ���
**************************************************************************/
void LxGetVal() {
  
  L1 = Get_Adc(Lx_CH_1);
  L2 = Get_Adc(Lx_CH_2);
  L3 = Get_Adc(Lx_CH_3);
}

/**************************************************************************
�������ܣ���ȡADC������Ѳ�ߴ������Ķ������Դ˸��¾���dist1,dist2,
          �Լ��Ե�ǰ�Թ�״̬���ж�
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
�������ܣ�����dist1, dist2, dist3�жϵ�ǰ�Թ�״̬

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
�������ܣ��ж���ǰ250ms���״ֵ̬�Ƿ�Ϊstate
          ���򷵻�1�� ���򷵻�0
**************************************************************************/
u8 isPrevState(int state) {
  
  if (countStates(5, state) >= 2) return 1;
  else return 0;
}

/**************************************************************************
�������ܣ�ͳ����STATE_ARR[20]�����recent��״̬��ֵΪstate��״̬��Ŀ
          
ע�� STATE_ARR[20]��һ��FIFO���У����ڱ������1s�ڵ�20���Թ�״̬
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
�������ܣ��趨���Ŀ��ת��
          
**************************************************************************/
void setTargetMotors(long int a,long int b, long int c, long int d) {
  
  Target_A=a,Target_B=b,Target_C=c,Target_D=d; 
}

/**************************************************************************
�������ܣ��趨С������x, y, zĿ���ٶ�

ע��xΪˮƽ�ٶȣ�yΪ�����ٶȣ�zΪ�������ٶ�          
**************************************************************************/
void setMove(float Vx, float Vy, float Vz)
{
  Target_A   = -Vx+Vy-Vz*0.1885;
  Target_B   = +Vx+Vy-Vz*0.1885;
  Target_C   = -Vx+Vy+Vz*0.1885;
  Target_D   = +Vx+Vy+Vz*0.1885;
}

/**************************************************************************
�������ܣ����˶���

**************************************************************************/
void back() {
  
  while(STATE==BACK){
      
      IRGetVal();
      showADCData();
      setTargetMotors(-200, -200, -200, -200);
    }
  
}

/**************************************************************************
�������ܣ���ת����

ע�� �ջ����ƣ����ܻᳬ��
**************************************************************************/
void rotateRight() {
  
  while(STATE==ROTATE_RIGHT) {
    
    IRGetVal();
    showADCData();
    setTargetMotors(500, 500, -500, -500);
  }
  
}

/**************************************************************************
�������ܣ�180����ת����

ע�� �������ƣ�����С��������˶����ʶ�����ǰ������ת��ǰ������ת��ǰ������Ӷ���
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
      while(delay_flag);	       //ͨ��MPU6050��INT�ж�ʵ�ֵ�50ms��׼��ʱ	
  }
}

/**************************************************************************
�������ܣ�ˢ����ʾ��ǰ��ش�����ADC���������ٶ�ֵ

**************************************************************************/
void showADCData() {

  u16 testRxData;
  
  // ��ʾ�ĸ������ת��
  OLED_ShowNumber(80, 0*12, Motor_A, 5, 12);
  OLED_ShowNumber(80, 1*12, Motor_B, 5, 12);
  OLED_ShowNumber(80, 2*12, Motor_C, 5, 12);
  OLED_ShowNumber(80, 3*12, Motor_D, 5, 12);

  // ��ʾѲ�ߴ���������
  testRxData = L1;OLED_ShowNumber(0, 0, testRxData, 5, 12);
  testRxData = L2;OLED_ShowNumber(0, 1*12, testRxData, 5, 12);
  testRxData = L3;OLED_ShowNumber(0, 2*12, testRxData, 5, 12);
  
//  for (t=0; t<16; ++t) { // �����ж�PA0-15���ŵ�ADC�����Ƿ�����
//    testRxData = Get_Adc(t);OLED_ShowNumber((t/4)*30, (t%4)*12, testRxData, 5, 12);
//  }
//  for (t=0; t<20; ++t) { // ��ʾ��ȥ1s�ڵ�20��״ֵ̬
//    testRxData = STATE_ARR[t];OLED_ShowNumber((t/3)*20, (t%3)*20, testRxData, 1, 16);
//  }

  OLED_Refresh_Gram();
}

/**************************************************************************
�������ܣ���ʼ��С��

**************************************************************************/
void initiateCars() {
  
    Stm32_Clock_Init(9);            //=====ϵͳʱ������
  	delay_init(72);                 //=====��ʱ��ʼ��
  	JTAG_Set(JTAG_SWD_DISABLE);     //=====�ر�JTAG�ӿ�
  	JTAG_Set(SWD_ENABLE);           //=====��SWD�ӿ� �������������SWD�ӿڵ���
  	LED_Init();                     //=====��ʼ���� LED ���ӵ�Ӳ���ӿ�
  	KEY_Init();                     //=====������ʼ��
  	if(MODE==0)Run_Flag=1;          //=====�����Ĺ����У�����ģʽѡ�񿪹�ȷ������λ��ģʽ�����ٶ�ģʽ
  	else Run_Flag=0;                //=====�����Ĺ����У�����ģʽѡ�񿪹�ȷ������λ��ģʽ�����ٶ�ģʽ
  	OLED_Init();                    //=====OLED��ʼ��
  	uart_init(72,128000);           //=====����1��ʼ��
  	uart2_init(36,9600);            //=====����2��ʼ��
  	uart3_init(36,115200);          //=====����3��ʼ�� 
  	Adc_Init();                     //=====adc��ʼ��
  	IIC_Init();                     //=====IIC��ʼ��
  	MPU6050_initialize();           //=====MPU6050��ʼ��	
  	DMP_Init();                     //=====��ʼ��DMP     
  	if(KEY==0) Flash_Read();        //=====��ȡFlash����Ĳ���
  	delay_ms(1000);                 //=====��ʱ�ȴ���ʼ���ȶ�
  	EXTI_Init();                    //=====MPU6050 5ms��ʱ�жϳ�ʼ��
  	CAN1_Mode_Init(1,2,3,6,0);      //=====CAN��ʼ��
  	MiniBalance_PWM_Init(7199,14);  //=====��ʼ��PWM �����������
  	delay_ms(1000);
    
    /********************������ʼ��*********************/
    Run_Flag=0;Flag_Left=0;Flag_Right=0;Flag_Direction=0;
    Motor_A=0,Motor_B=0,Motor_C=0,Motor_D=0;        //���PWM����
    Target_A=0,Target_B=0,Target_C=0,Target_D=0;     //���Ŀ��ֵ
    L1=0;L2=0;L3=0;IR1=0;IR2=0;head_pt=0;
    errL_over_cnt=0;errR_over_cnt=0;
    for (t=0; t<20; ++t) {
      STATE_ARR[t] = STRAIGHT;
    }
  
}
/*********************************************************************
                        MY CONTROL END
*********************************************************************/
