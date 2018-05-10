#include "control.h"	
#include "filter.h"	

u8 Flag_Target,Flag_Change;                             //��ر�־λ
u8 temp1;                                               //��ʱ����
float Voltage_Count,Voltage_All;  //��ѹ������ر���
float Gyro_K=0.004;       //�����Ǳ���ϵ��
int j;
int Flag_Jiasu;
#define a_PARAMETER          (0.1025f)               
#define b_PARAMETER          (0.086)    

/*********************************************************************
                        MY CONTROL START
*********************************************************************/

/**************************************************************************
�������ܣ��������͵ķ���
          ����Ϊ1������Ϊ-1
**************************************************************************/
int sign(int x) {
  return (x > 0) - (x < 0);
}

/**************************************************************************
�������ܣ� �����ĸ������Ŀ���ٶ�

**************************************************************************/
void setMotors(long int Target_A,long int Target_B, long int Target_C, long int Target_D)
{
	Motor_A += 20*sign(Target_A-Motor_A);
	Motor_B += 20*sign(Target_B-Motor_B);
	Motor_C += 20*sign(Target_C-Motor_C);
	Motor_D += 20*sign(Target_D-Motor_D);
}

/**************************************************************************
�������ܣ����еĿ��ƴ��붼��������
         5ms��ʱ�ж���MPU6050��INT���Ŵ���
         �ϸ�֤���������ݴ����ʱ��ͬ��				 
**************************************************************************/
int EXTI15_10_IRQHandler(void) 
{    
  
	if(INT==0)		
	{     
		EXTI->PR=1<<15;                                                      //���LINE5�ϵ��жϱ�־λ  		
		Flag_Target=!Flag_Target;
		if(delay_flag==1)
		{
			// CHANGED
			if(++delay_50==10)	 delay_50=0,delay_flag=0;                     //���������ṩ50ms�ľ�׼��ʱ
		}
		if(Flag_Target==1)                                                  //5ms��ȡһ�������Ǻͼ��ٶȼƵ�ֵ
		{
			CAN1_SEND();                                                          //CAN����
			if(Usart_Flag==0&&Usart_ON_Flag==1)  memcpy(rxbuf,Urxbuf,8*sizeof(u8));	//��������˴��ڿ��Ʊ�־λ�����봮�ڿ���ģʽ
			Read_DMP();                                                           //===������̬		
			Key();//ɨ�谴���仯	
			return 0;	                                               
		}                            				//===10ms����һ��
		Read_DMP();                  				//===������̬	
		Encoder_A=Motor_A;            	 		//===ͨ��Ƶ�ʽ����ٶ�
		Encoder_B=Motor_B;           		 		//===ͨ��Ƶ�ʽ����ٶ�
		Encoder_C=Motor_C;            	 	  //===ͨ��Ƶ�ʽ����ٶ�
		Encoder_D=Motor_D;            	 	  //===ͨ��Ƶ�ʽ����ٶ�
		Position_A+=4.8*Encoder_A*0.01;     //===ͨ��Ƶ�ʻ��ּ���λ��
		Position_B+=4.8*Encoder_B*0.01;    	//===ͨ��Ƶ�ʻ��ּ���λ��
		Position_C+=4.8*Encoder_C*0.01;     //===ͨ��Ƶ�ʻ��ּ���λ��
		Position_D+=4.8*Encoder_D*0.01;     //===ͨ��Ƶ�ʻ��ּ���λ��
		Led_Flash(100);                     //===LED��˸;����ģʽ 1s�ı�һ��ָʾ�Ƶ�״̬	
		Voltage_All+=Get_battery_volt();     //��β����ۻ�
		if(++Voltage_Count==100) Voltage=Voltage_All/100,Voltage_All=0,Voltage_Count=0;//��ƽ��ֵ ��ȡ��ص�ѹ	       
		if(CAN_ON_Flag==1||Usart_ON_Flag==1) CAN_N_Usart_Control();       //�ӵ����ڻ���CANң�ؽ���ָ��֮��ʹ��CAN�ʹ��ڿ�������
		if(Turn_Off(Voltage)==0)         //===�����ص�ѹ�������쳣
		{ 			 
        setMotors(Target_A, Target_B, Target_C, Target_D); // ��ֵ�����
        Xianfu_Pwm(1000);  // PWMƵ���޷�
        Set_Pwm(Motor_A,Motor_B,Motor_C,Motor_D);    //��ֵ��PWM�Ĵ���
		}
	}
	return 0;	 
} 

/*********************************************************************
                        MY CONTROL END
*********************************************************************/

/**************************************************************************
�������ܣ�С���˶���ѧģ��
��ڲ�����X Y Z �����ٶȻ���λ��
����  ֵ����
**************************************************************************/
void Kinematic_Analysis(float Vx,float Vy,float Vz)
{
    Target_A   = -Vx+Vy-Vz*(a_PARAMETER+b_PARAMETER);
    Target_B   = +Vx+Vy-Vz*(a_PARAMETER+b_PARAMETER);
    Target_C   = -Vx+Vy+Vz*(a_PARAMETER+b_PARAMETER);
    Target_D   = +Vx+Vy+Vz*(a_PARAMETER+b_PARAMETER);
}
/**************************************************************************
�������ܣ���ȡλ�ÿ��ƹ����ٶ�ֵ
��ڲ�����X Y Z ����λ�ñ仯��
����  ֵ����
**************************************************************************/
void Kinematic_Analysis2(float Vx,float Vy,float Vz)
{
    Rate_A   = -Vx+Vy-Vz*(a_PARAMETER+b_PARAMETER);
    Rate_B   = +Vx+Vy-Vz*(a_PARAMETER+b_PARAMETER);
    Rate_C   = -Vx+Vy+Vz*(a_PARAMETER+b_PARAMETER);
    Rate_D   = +Vx+Vy+Vz*(a_PARAMETER+b_PARAMETER);
}


/**************************************************************************
�������ܣ���ֵ��PWM�Ĵ���
��ڲ�����PWM
����  ֵ����
**************************************************************************/
void Set_Pwm(int motor_a,int motor_b,int motor_c,int motor_d)
{
			int Final_Motor_A,Final_Motor_B,Final_Motor_C,Final_Motor_D;
	    static long int Last_Target_A,Last_Target_B,Last_Target_C,Last_Target_D;
	    static int flag,count;
	   	if(motor_a>0)			    INA=0;   //���A�������
			else 	             	  INA=1;
	   	if(motor_b>0)			    INB=0;   //���B�������
			else 	             	  INB=1;
			if(motor_c>0)			    INC=1;   //���C�������
			else 	                INC=0;
			if(motor_d>0)			    IND=1;   //���D�������
			else 	                IND=0;
	    if(Run_Flag==0)//�ٶ�ģʽ�µ��Զ�ʧ�ܵ�����
			{
					if(flag==0)
					{
							if(0==Target_A&&0==Target_B&&0==Target_C&&0==Target_D)  count++;
							else count=0;
							if(count>200)
							{
								flag=1;
								count=0;
							}
							ST	=1;	
					}
						if(flag==1)
						{
								ST	=0;
								if(Target_A!=0||Target_B!=0||Target_C!=0||Target_D!=0)
								flag=0;	
						}
			}
				else//λ��ģʽ�µ��Զ�ʧ�ܵ�����
			{
					if(flag==0)
					{								
								if(Motor_A==0&&Motor_B==0&&Motor_C==0&&Motor_D==0)   count++;
						  else count=0;
							if(count>200)
							{
								flag=1;
								count=0;
							}
							ST	=1;	
					}
					if(flag==1) //ͣ��״̬
					{
							ST	=0;
							if(myabs(Last_Target_A-Target_A)>5||myabs(Last_Target_B-Target_B)>5||myabs(Last_Target_C-Target_C)>5||myabs(Last_Target_D-Target_D)>5) //Ŀ��ֵ����������5��ʹ�ܵ��
							flag=0;	
					}
			}
      Last_Target_A=Target_A;  //������ʷĿ��ֵ��Ϣ
			Last_Target_B=Target_B;
			Last_Target_C=Target_C;
			Last_Target_D=Target_D;
	    Final_Motor_A=Linear_Conversion(motor_a);  //���Ի�
    	Final_Motor_B=Linear_Conversion(motor_b);
			Final_Motor_C=Linear_Conversion(motor_c);
			Final_Motor_D=Linear_Conversion(motor_d);
			Set_PWM_Final(Final_Motor_A,Final_Motor_B,Final_Motor_C,Final_Motor_D);  
}
/**************************************************************************
�������ܣ��Կ��������PWM���Ի�,���ڸ�ϵͳ�Ĵ�����ֵ
��ڲ�����PWM
����  ֵ�����Ի����PWM
**************************************************************************/
u16  Linear_Conversion(int motor)
{ 
	 u32 temp;
   u16 Linear_Moto;
   temp=1000000/myabs(motor);   //1000000�Ǿ���ֵ
	 if(temp>65535) Linear_Moto=65535;
	 else Linear_Moto=(u16)temp;
	 return Linear_Moto;
}	

/**************************************************************************
�������ܣ�����PWM��ֵ 
��ڲ�������ֵ
����  ֵ����
**************************************************************************/
void Xianfu_Pwm(int amplitude)
{	
    if(Motor_A<-amplitude) Motor_A=-amplitude;	
		if(Motor_A>amplitude)  Motor_A=amplitude;	
	  if(Motor_B<-amplitude) Motor_B=-amplitude;	
		if(Motor_B>amplitude)  Motor_B=amplitude;		
	  if(Motor_C<-amplitude) Motor_C=-amplitude;	
		if(Motor_C>amplitude)  Motor_C=amplitude;
	  if(Motor_D<-amplitude) Motor_D=-amplitude;	
		if(Motor_D>amplitude)  Motor_D=amplitude;		
}
/**************************************************************************
�������ܣ�λ��PID���ƹ������ٶȵ�����
��ڲ������ޡ���ֵ
����  ֵ����
**************************************************************************/
void Xianfu_Velocity(int amplitude_A,int amplitude_B,int amplitude_C,int amplitude_D)
{	
    if(Motor_A<-amplitude_A) Motor_A=-amplitude_A;	//λ�ÿ���ģʽ�У�A����������ٶ�
		if(Motor_A>amplitude_A)  Motor_A=amplitude_A;	  //λ�ÿ���ģʽ�У�A����������ٶ�
	  if(Motor_B<-amplitude_B) Motor_B=-amplitude_B;	//λ�ÿ���ģʽ�У�B����������ٶ�
		if(Motor_B>amplitude_B)  Motor_B=amplitude_B;		//λ�ÿ���ģʽ�У�B����������ٶ�
	  if(Motor_C<-amplitude_C) Motor_C=-amplitude_C;	//λ�ÿ���ģʽ�У�C����������ٶ�
		if(Motor_C>amplitude_C)  Motor_C=amplitude_C;		//λ�ÿ���ģʽ�У�C����������ٶ�
		if(Motor_D<-amplitude_D) Motor_D=-amplitude_D;	//λ�ÿ���ģʽ�У�D����������ٶ�
		if(Motor_D>amplitude_D)  Motor_D=amplitude_D;		//λ�ÿ���ģʽ�У�D����������ٶ�
}
/**************************************************************************
�������ܣ������޸�С������״̬ 
��ڲ�������
����  ֵ����
**************************************************************************/
void Key(void)
{	
	u8 tmp;
	tmp=click_N_Double(100);    
	if(tmp==2)Flag_Show=!Flag_Show;//˫��������ʾģʽ                  
}

/**************************************************************************
�������ܣ��쳣�رյ��
��ڲ�������ѹ
����  ֵ��1���쳣  0������
**************************************************************************/
u8 Turn_Off( int voltage)
{
	    u8 temp;
			if(voltage<1110)//��ص�ѹ����11.1V�رյ��
			{	                                                
      temp=1;      
      INA=0;
      INB=0;
      INC=0;
      IND=0;				
			ST=0;   //ʧ�ܵ��
      }
			else
      temp=0;
      return temp;			
}

/**************************************************************************
�������ܣ�����ֵ����
��ڲ�����long int
����  ֵ��unsigned int
**************************************************************************/
u32 myabs(long int a)
{ 		   
	  u32 temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}
/**************************************************************************
�������ܣ�λ��ʽPID������
��ڲ���������������λ����Ϣ��Ŀ��λ��
����  ֵ�����PWM
����λ��ʽ��ɢPID��ʽ 
pwm=Kp*e(k)+Ki*��e(k)+Kd[e��k��-e(k-1)]
e(k)������ƫ�� 
e(k-1)������һ�ε�ƫ��  
��e(k)����e(k)�Լ�֮ǰ��ƫ����ۻ���;����kΪ1,2,,k;
pwm�������
**************************************************************************/
int Position_PID_A (int Encoder,int Target)
{ 	
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=Encoder-Target;                                  //����ƫ��
	 Integral_bias+=Bias;	                                 //���ƫ��Ļ���
	 Pwm=Position_KP*Bias+Position_KI*Integral_bias+Position_KD*(Bias-Last_Bias);       //λ��ʽPID������
	 Last_Bias=Bias;                                       //������һ��ƫ�� 
	 return Pwm;                                           //�������
}
int Position_PID_B (int Encoder,int Target)
{ 	
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=Encoder-Target;                                  //����ƫ��
	 Integral_bias+=Bias;	                                 //���ƫ��Ļ���
	 Pwm=Position_KP*Bias+Position_KI*Integral_bias+Position_KD*(Bias-Last_Bias);       //λ��ʽPID������
	 Last_Bias=Bias;                                       //������һ��ƫ�� 
	 return Pwm;                                           //�������
}
int Position_PID_C (int Encoder,int Target)
{ 	
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=Encoder-Target;                                  //����ƫ��
	 Integral_bias+=Bias;	                                 //���ƫ��Ļ���
	 Pwm=Position_KP*Bias+Position_KI*Integral_bias+Position_KD*(Bias-Last_Bias);       //λ��ʽPID������
	 Last_Bias=Bias;                                       //������һ��ƫ�� 
	 return Pwm;                                           //�������
}
int Position_PID_D (int Encoder,int Target)
{ 	
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=Encoder-Target;                                  //����ƫ��
	 Integral_bias+=Bias;	                                 //���ƫ��Ļ���
	 Pwm=Position_KP*Bias+Position_KI*Integral_bias+Position_KD*(Bias-Last_Bias);       //λ��ʽPID������
	 Last_Bias=Bias;                                       //������һ��ƫ�� 
	 return Pwm;                                           //�������
}
/**************************************************************************
�������ܣ�ͨ������ָ���С������ң��
��ڲ���������ָ��
����  ֵ����
**************************************************************************/
void Get_RC(u8 mode)
{

	float step=10;  //�����ٶȿ��Ʋ���ֵ��
	u8 Flag_Move=1;
	  if(mode==0)//�ٶ�
		{	
				 switch(Flag_Direction)   //�������
				 {
				 case 1:  Move_X=0;           Move_Y+=step;  Flag_Move=1;               break;
				 case 2:  Move_X+=step;       Move_Y+=step;  Flag_Move=1;               break;
				 case 3:  Move_X+=step;       Move_Y=0;      Flag_Move=1;               break;
				 case 4:  Move_X+=step;       Move_Y-=step;  Flag_Move=1;               break;
				 case 5:  Move_X=0;           Move_Y-=step;  Flag_Move=1;               break;
				 case 6:  Move_X-=step;       Move_Y-=step;  Flag_Move=1;               break;
				 case 7:  Move_X-=step;       Move_Y=0;      Flag_Move=1;               break;
				 case 8:  Move_X-=step;       Move_Y+=step;  Flag_Move=1;               break; 
				 default: Flag_Move=0;    		Move_X=Move_X/1.1;	Move_Y=Move_Y/1.1;	      break;	 
			
			 }			 
			if(Flag_Move==0)		//����޷������ָ��	 �����ת�����״̬
			{	
				if(Flag_Left==1)        Move_Z-=step,Gyro_K=0;    //������   
				else if(Flag_Right==1) 	Move_Z+=step,Gyro_K=0;    //������		
				else 		                Move_Z=0,Gyro_K=0.004;    //ֹͣ
			}	
				if(Flag_Move==1)	Flag_Left=0,Flag_Right=0,Move_Z=0;
				if(Move_X<-RC_Velocity) Move_X=-RC_Velocity;	   //�ٶȿ����޷�
				if(Move_X>RC_Velocity)  Move_X=RC_Velocity;	     
				if(Move_Y<-RC_Velocity) Move_Y=-RC_Velocity;	
				if(Move_Y>RC_Velocity)  Move_Y=RC_Velocity;	 
				if(Move_Z<-RC_Velocity) Move_Z=-RC_Velocity;	
				if(Move_Z>RC_Velocity)  Move_Z=RC_Velocity;	 
			 
	 }
		 else if(mode==1)//λ��ģʽ
		{	
				 switch(Flag_Direction)   //�������
				 {
				 case 1:  Move_Y+=RC_Position; Flag_Change=1;break;
				 case 2:  Move_X+=RC_Position; Flag_Change=2; Move_Y+=RC_Position; break;
				 case 3:  Move_X+=RC_Position; Flag_Change=3;break;
				 case 4:  Move_X+=RC_Position; Flag_Change=4;Move_Y-=RC_Position;break;
				 case 5:  Move_Y-=RC_Position; Flag_Change=5;break;
				 case 6:  Move_X-=RC_Position; Flag_Change=6;Move_Y-=RC_Position; break;
				 case 7:  Move_X-=RC_Position; Flag_Change=7; break;
				 case 8:  Move_X-=RC_Position; Flag_Change=8;Move_Y+=RC_Position;break;			 
				 case 9:  Move_Z-=RC_Position; Flag_Change=9; break;
				 case 10: Move_Z+=RC_Position; Flag_Change=10;break;			 
				 default: break;	 
			 }
	 }
		 Kinematic_Analysis(Move_X,Move_Y,Move_Z);//�õ�����Ŀ��ֵ�������˶�ѧ����
}
/**************************************************************************
�������ܣ�ÿ�����λ�ÿ��ƹ����ٶȼ���
��ڲ�������
����  ֵ����
**************************************************************************/
void Count_Velocity(void)
{
	static double Last_Target_X,Last_Target_Y,Last_Target_Z,Divider;
	double Bias_X,Bias_Y,Bias_Z;
	Bias_X=(Move_X-Last_Target_X);  //��X��λ����
	Bias_Y=(Move_Y-Last_Target_Y);	//��Y��λ����
	Bias_Z=(Move_Z-Last_Target_Z);	//��Z��λ����
	if(Bias_X!=0||Bias_Y!=0||Bias_Z!=0)Divider=sqrt(Bias_X*Bias_X+Bias_Y*Bias_Y+Bias_Z*Bias_Z);
	if(Bias_X!=0||Bias_Y!=0||Bias_Z!=0) Kinematic_Analysis2(Bias_X,Bias_Y,Bias_Z);

	Xianfu_Velocity(Max_Velocity*myabs(Rate_A)/Divider,Max_Velocity*myabs(Rate_B)/Divider,Max_Velocity*myabs(Rate_C)/Divider,Max_Velocity*myabs(Rate_D)/Divider); 

	Last_Target_X=Move_X;   //����X����һ�ε�λ����Ϣ�����ڵ���
	Last_Target_Y=Move_Y;   //����Y����һ�ε�λ����Ϣ�����ڵ���
	Last_Target_Z=Move_Z;   //����Z����һ�ε�λ����Ϣ�����ڵ���
}
/**************************************************************************
�������ܣ�����CAN���ߴ��ڿ���ָ����д���
��ڲ�������
����  ֵ����
**************************************************************************/
void CAN_N_Usart_Control(void)
{
   int flag_X, flag_Y,flag_Z;
	 if((rxbuf[7]&0x04)==0)flag_X=1;  else flag_X=-1;  //�������λ
	 if((rxbuf[7]&0x02)==0)flag_Y=1;  else flag_Y=-1;  //�������λ
	 if((rxbuf[7]&0x01)==0)flag_Z=1;  else flag_Z=-1;  //�������λ
	 Move_X=flag_X*(rxbuf[1]*256+rxbuf[2]);
	 Move_Y=flag_Y*(rxbuf[3]*256+rxbuf[4]);	
	 Move_Z=flag_Z*(rxbuf[5]*256+rxbuf[6]);	
	
   if(rxbuf[0]==1)Kinematic_Analysis(Move_X,Move_Y,Move_Z),Gyro_K=0;    //�����˶�ѧ����
	// if(rxbuf[0]==2)Target_A=Move_X,Target_B=Move_Y,Target_C=Move_Z;      //������ÿ��������п���
}

