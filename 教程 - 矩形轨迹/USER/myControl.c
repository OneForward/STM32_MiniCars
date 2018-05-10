#include "myControl.h"

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
�������ܣ����ι켣����

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
      while(delay_flag);         //ͨ��MPU6050��INT�ж�ʵ�ֵ�50ms��׼��ʱ 
  }
}


/**************************************************************************
�������ܣ�ˢ����ʾ��ǰ��ش�����ADC���������ٶ�ֵ

**************************************************************************/
void showADCData() {
  
  //=============��ʾ3��Ƕ�===============//	
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
}
/*********************************************************************
                        MY CONTROL END
*********************************************************************/
