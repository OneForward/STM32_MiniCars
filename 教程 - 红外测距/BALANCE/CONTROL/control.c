#include "control.h"	
#include "filter.h"	

u8 Flag_Target,Flag_Change;                             //相关标志位
u8 temp1;                                               //临时变量
float Voltage_Count,Voltage_All;  //电压采样相关变量
float Gyro_K=0.004;       //陀螺仪比例系数
int j;
int Flag_Jiasu;
#define a_PARAMETER          (0.1025f)               
#define b_PARAMETER          (0.086)    

/*********************************************************************
                        MY CONTROL START
*********************************************************************/

/**************************************************************************
函数功能：返回整型的符号
          正数为1，负数为-1
**************************************************************************/
int sign(int x) {
  return (x > 0) - (x < 0);
}

/**************************************************************************
函数功能： 设置四个电机的目标速度

**************************************************************************/
void setMotors(long int Target_A,long int Target_B, long int Target_C, long int Target_D)
{
	Motor_A += 20*sign(Target_A-Motor_A);
	Motor_B += 20*sign(Target_B-Motor_B);
	Motor_C += 20*sign(Target_C-Motor_C);
	Motor_D += 20*sign(Target_D-Motor_D);
}

/**************************************************************************
函数功能：所有的控制代码都在这里面
         5ms定时中断由MPU6050的INT引脚触发
         严格保证采样和数据处理的时间同步				 
**************************************************************************/
int EXTI15_10_IRQHandler(void) 
{    
  
	if(INT==0)		
	{     
		EXTI->PR=1<<15;                                                      //清除LINE5上的中断标志位  		
		Flag_Target=!Flag_Target;
		if(delay_flag==1)
		{
			// CHANGED
			if(++delay_50==10)	 delay_50=0,delay_flag=0;                     //给主函数提供50ms的精准延时
		}
		if(Flag_Target==1)                                                  //5ms读取一次陀螺仪和加速度计的值
		{
			CAN1_SEND();                                                          //CAN发送
			if(Usart_Flag==0&&Usart_ON_Flag==1)  memcpy(rxbuf,Urxbuf,8*sizeof(u8));	//如果解锁了串口控制标志位，进入串口控制模式
			Read_DMP();                                                           //===更新姿态		
			Key();//扫描按键变化	
			return 0;	                                               
		}                            				//===10ms控制一次
		Read_DMP();                  				//===更新姿态	
		Encoder_A=Motor_A;            	 		//===通过频率近似速度
		Encoder_B=Motor_B;           		 		//===通过频率近似速度
		Encoder_C=Motor_C;            	 	  //===通过频率近似速度
		Encoder_D=Motor_D;            	 	  //===通过频率近似速度
		Position_A+=4.8*Encoder_A*0.01;     //===通过频率积分计算位移
		Position_B+=4.8*Encoder_B*0.01;    	//===通过频率积分计算位移
		Position_C+=4.8*Encoder_C*0.01;     //===通过频率积分计算位移
		Position_D+=4.8*Encoder_D*0.01;     //===通过频率积分计算位移
		Led_Flash(100);                     //===LED闪烁;常规模式 1s改变一次指示灯的状态	
		Voltage_All+=Get_battery_volt();     //多次采样累积
		if(++Voltage_Count==100) Voltage=Voltage_All/100,Voltage_All=0,Voltage_Count=0;//求平均值 获取电池电压	       
		if(CAN_ON_Flag==1||Usart_ON_Flag==1) CAN_N_Usart_Control();       //接到串口或者CAN遥控解锁指令之后，使能CAN和串口控制输入
		if(Turn_Off(Voltage)==0)         //===如果电池电压不存在异常
		{ 			 
        setMotors(Target_A, Target_B, Target_C, Target_D); // 幅值给电机
        Xianfu_Pwm(1000);  // PWM频率限幅
        Set_Pwm(Motor_A,Motor_B,Motor_C,Motor_D);    //赋值给PWM寄存器
		}
	}
	return 0;	 
} 

/*********************************************************************
                        MY CONTROL END
*********************************************************************/

/**************************************************************************
函数功能：小车运动数学模型
入口参数：X Y Z 三轴速度或者位置
返回  值：无
**************************************************************************/
void Kinematic_Analysis(float Vx,float Vy,float Vz)
{
    Target_A   = -Vx+Vy-Vz*(a_PARAMETER+b_PARAMETER);
    Target_B   = +Vx+Vy-Vz*(a_PARAMETER+b_PARAMETER);
    Target_C   = -Vx+Vy+Vz*(a_PARAMETER+b_PARAMETER);
    Target_D   = +Vx+Vy+Vz*(a_PARAMETER+b_PARAMETER);
}
/**************************************************************************
函数功能：获取位置控制过程速度值
入口参数：X Y Z 三轴位置变化量
返回  值：无
**************************************************************************/
void Kinematic_Analysis2(float Vx,float Vy,float Vz)
{
    Rate_A   = -Vx+Vy-Vz*(a_PARAMETER+b_PARAMETER);
    Rate_B   = +Vx+Vy-Vz*(a_PARAMETER+b_PARAMETER);
    Rate_C   = -Vx+Vy+Vz*(a_PARAMETER+b_PARAMETER);
    Rate_D   = +Vx+Vy+Vz*(a_PARAMETER+b_PARAMETER);
}


/**************************************************************************
函数功能：赋值给PWM寄存器
入口参数：PWM
返回  值：无
**************************************************************************/
void Set_Pwm(int motor_a,int motor_b,int motor_c,int motor_d)
{
			int Final_Motor_A,Final_Motor_B,Final_Motor_C,Final_Motor_D;
	    static long int Last_Target_A,Last_Target_B,Last_Target_C,Last_Target_D;
	    static int flag,count;
	   	if(motor_a>0)			    INA=0;   //电机A方向控制
			else 	             	  INA=1;
	   	if(motor_b>0)			    INB=0;   //电机B方向控制
			else 	             	  INB=1;
			if(motor_c>0)			    INC=1;   //电机C方向控制
			else 	                INC=0;
			if(motor_d>0)			    IND=1;   //电机D方向控制
			else 	                IND=0;
	    if(Run_Flag==0)//速度模式下的自动失能电机检测
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
				else//位置模式下的自动失能电机检测
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
					if(flag==1) //停机状态
					{
							ST	=0;
							if(myabs(Last_Target_A-Target_A)>5||myabs(Last_Target_B-Target_B)>5||myabs(Last_Target_C-Target_C)>5||myabs(Last_Target_D-Target_D)>5) //目标值增减量大于5，使能电机
							flag=0;	
					}
			}
      Last_Target_A=Target_A;  //保存历史目标值信息
			Last_Target_B=Target_B;
			Last_Target_C=Target_C;
			Last_Target_D=Target_D;
	    Final_Motor_A=Linear_Conversion(motor_a);  //线性化
    	Final_Motor_B=Linear_Conversion(motor_b);
			Final_Motor_C=Linear_Conversion(motor_c);
			Final_Motor_D=Linear_Conversion(motor_d);
			Set_PWM_Final(Final_Motor_A,Final_Motor_B,Final_Motor_C,Final_Motor_D);  
}
/**************************************************************************
函数功能：对控制输出的PWM线性化,便于给系统寄存器赋值
入口参数：PWM
返回  值：线性化后的PWM
**************************************************************************/
u16  Linear_Conversion(int motor)
{ 
	 u32 temp;
   u16 Linear_Moto;
   temp=1000000/myabs(motor);   //1000000是经验值
	 if(temp>65535) Linear_Moto=65535;
	 else Linear_Moto=(u16)temp;
	 return Linear_Moto;
}	

/**************************************************************************
函数功能：限制PWM赋值 
入口参数：幅值
返回  值：无
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
函数功能：位置PID控制过程中速度的设置
入口参数：无、幅值
返回  值：无
**************************************************************************/
void Xianfu_Velocity(int amplitude_A,int amplitude_B,int amplitude_C,int amplitude_D)
{	
    if(Motor_A<-amplitude_A) Motor_A=-amplitude_A;	//位置控制模式中，A电机的运行速度
		if(Motor_A>amplitude_A)  Motor_A=amplitude_A;	  //位置控制模式中，A电机的运行速度
	  if(Motor_B<-amplitude_B) Motor_B=-amplitude_B;	//位置控制模式中，B电机的运行速度
		if(Motor_B>amplitude_B)  Motor_B=amplitude_B;		//位置控制模式中，B电机的运行速度
	  if(Motor_C<-amplitude_C) Motor_C=-amplitude_C;	//位置控制模式中，C电机的运行速度
		if(Motor_C>amplitude_C)  Motor_C=amplitude_C;		//位置控制模式中，C电机的运行速度
		if(Motor_D<-amplitude_D) Motor_D=-amplitude_D;	//位置控制模式中，D电机的运行速度
		if(Motor_D>amplitude_D)  Motor_D=amplitude_D;		//位置控制模式中，D电机的运行速度
}
/**************************************************************************
函数功能：按键修改小车运行状态 
入口参数：无
返回  值：无
**************************************************************************/
void Key(void)
{	
	u8 tmp;
	tmp=click_N_Double(100);    
	if(tmp==2)Flag_Show=!Flag_Show;//双击控制显示模式                  
}

/**************************************************************************
函数功能：异常关闭电机
入口参数：电压
返回  值：1：异常  0：正常
**************************************************************************/
u8 Turn_Off( int voltage)
{
	    u8 temp;
			if(voltage<1110)//电池电压低于11.1V关闭电机
			{	                                                
      temp=1;      
      INA=0;
      INB=0;
      INC=0;
      IND=0;				
			ST=0;   //失能电机
      }
			else
      temp=0;
      return temp;			
}

/**************************************************************************
函数功能：绝对值函数
入口参数：long int
返回  值：unsigned int
**************************************************************************/
u32 myabs(long int a)
{ 		   
	  u32 temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}
/**************************************************************************
函数功能：位置式PID控制器
入口参数：编码器测量位置信息，目标位置
返回  值：电机PWM
根据位置式离散PID公式 
pwm=Kp*e(k)+Ki*∑e(k)+Kd[e（k）-e(k-1)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  
∑e(k)代表e(k)以及之前的偏差的累积和;其中k为1,2,,k;
pwm代表输出
**************************************************************************/
int Position_PID_A (int Encoder,int Target)
{ 	
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=Encoder-Target;                                  //计算偏差
	 Integral_bias+=Bias;	                                 //求出偏差的积分
	 Pwm=Position_KP*Bias+Position_KI*Integral_bias+Position_KD*(Bias-Last_Bias);       //位置式PID控制器
	 Last_Bias=Bias;                                       //保存上一次偏差 
	 return Pwm;                                           //增量输出
}
int Position_PID_B (int Encoder,int Target)
{ 	
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=Encoder-Target;                                  //计算偏差
	 Integral_bias+=Bias;	                                 //求出偏差的积分
	 Pwm=Position_KP*Bias+Position_KI*Integral_bias+Position_KD*(Bias-Last_Bias);       //位置式PID控制器
	 Last_Bias=Bias;                                       //保存上一次偏差 
	 return Pwm;                                           //增量输出
}
int Position_PID_C (int Encoder,int Target)
{ 	
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=Encoder-Target;                                  //计算偏差
	 Integral_bias+=Bias;	                                 //求出偏差的积分
	 Pwm=Position_KP*Bias+Position_KI*Integral_bias+Position_KD*(Bias-Last_Bias);       //位置式PID控制器
	 Last_Bias=Bias;                                       //保存上一次偏差 
	 return Pwm;                                           //增量输出
}
int Position_PID_D (int Encoder,int Target)
{ 	
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=Encoder-Target;                                  //计算偏差
	 Integral_bias+=Bias;	                                 //求出偏差的积分
	 Pwm=Position_KP*Bias+Position_KI*Integral_bias+Position_KD*(Bias-Last_Bias);       //位置式PID控制器
	 Last_Bias=Bias;                                       //保存上一次偏差 
	 return Pwm;                                           //增量输出
}
/**************************************************************************
函数功能：通过串口指令对小车进行遥控
入口参数：串口指令
返回  值：无
**************************************************************************/
void Get_RC(u8 mode)
{

	float step=10;  //设置速度控制步进值。
	u8 Flag_Move=1;
	  if(mode==0)//速度
		{	
				 switch(Flag_Direction)   //方向控制
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
			if(Flag_Move==0)		//如果无方向控制指令	 ，检查转向控制状态
			{	
				if(Flag_Left==1)        Move_Z-=step,Gyro_K=0;    //左自旋   
				else if(Flag_Right==1) 	Move_Z+=step,Gyro_K=0;    //右自旋		
				else 		                Move_Z=0,Gyro_K=0.004;    //停止
			}	
				if(Flag_Move==1)	Flag_Left=0,Flag_Right=0,Move_Z=0;
				if(Move_X<-RC_Velocity) Move_X=-RC_Velocity;	   //速度控制限幅
				if(Move_X>RC_Velocity)  Move_X=RC_Velocity;	     
				if(Move_Y<-RC_Velocity) Move_Y=-RC_Velocity;	
				if(Move_Y>RC_Velocity)  Move_Y=RC_Velocity;	 
				if(Move_Z<-RC_Velocity) Move_Z=-RC_Velocity;	
				if(Move_Z>RC_Velocity)  Move_Z=RC_Velocity;	 
			 
	 }
		 else if(mode==1)//位置模式
		{	
				 switch(Flag_Direction)   //方向控制
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
		 Kinematic_Analysis(Move_X,Move_Y,Move_Z);//得到控制目标值，进行运动学分析
}
/**************************************************************************
函数功能：每个电机位置控制过程速度计算
入口参数：无
返回  值：无
**************************************************************************/
void Count_Velocity(void)
{
	static double Last_Target_X,Last_Target_Y,Last_Target_Z,Divider;
	double Bias_X,Bias_Y,Bias_Z;
	Bias_X=(Move_X-Last_Target_X);  //求X轴位移量
	Bias_Y=(Move_Y-Last_Target_Y);	//求Y轴位移量
	Bias_Z=(Move_Z-Last_Target_Z);	//求Z轴位移量
	if(Bias_X!=0||Bias_Y!=0||Bias_Z!=0)Divider=sqrt(Bias_X*Bias_X+Bias_Y*Bias_Y+Bias_Z*Bias_Z);
	if(Bias_X!=0||Bias_Y!=0||Bias_Z!=0) Kinematic_Analysis2(Bias_X,Bias_Y,Bias_Z);

	Xianfu_Velocity(Max_Velocity*myabs(Rate_A)/Divider,Max_Velocity*myabs(Rate_B)/Divider,Max_Velocity*myabs(Rate_C)/Divider,Max_Velocity*myabs(Rate_D)/Divider); 

	Last_Target_X=Move_X;   //保存X轴上一次的位置信息，便于调用
	Last_Target_Y=Move_Y;   //保存Y轴上一次的位置信息，便于调用
	Last_Target_Z=Move_Z;   //保存Z轴上一次的位置信息，便于调用
}
/**************************************************************************
函数功能：接收CAN或者串口控制指令进行处理
入口参数：无
返回  值：无
**************************************************************************/
void CAN_N_Usart_Control(void)
{
   int flag_X, flag_Y,flag_Z;
	 if((rxbuf[7]&0x04)==0)flag_X=1;  else flag_X=-1;  //方向控制位
	 if((rxbuf[7]&0x02)==0)flag_Y=1;  else flag_Y=-1;  //方向控制位
	 if((rxbuf[7]&0x01)==0)flag_Z=1;  else flag_Z=-1;  //方向控制位
	 Move_X=flag_X*(rxbuf[1]*256+rxbuf[2]);
	 Move_Y=flag_Y*(rxbuf[3]*256+rxbuf[4]);	
	 Move_Z=flag_Z*(rxbuf[5]*256+rxbuf[6]);	
	
   if(rxbuf[0]==1)Kinematic_Analysis(Move_X,Move_Y,Move_Z),Gyro_K=0;    //进行运动学分析
	// if(rxbuf[0]==2)Target_A=Move_X,Target_B=Move_Y,Target_C=Move_Z;      //单独对每个电机进行控制
}

