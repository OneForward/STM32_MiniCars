#include "motor.h"
  /**************************************************************************
作者：平衡小车之家
我的淘宝小店：http://shop114407458.taobao.com/
**************************************************************************/
void MiniBalance_Motor_Init(void)
{
	RCC->APB2ENR|=1<<3;       //PORTB时钟使能   
	RCC->APB2ENR|=1<<4;       //PORTC时钟使能  
	GPIOB->CRL&=0XFFFFFF00;   //推挽输出
	GPIOB->CRL|=0X00000022;   //推挽输出
	GPIOB->ODR|=1<<0;  	
	
	GPIOC->CRL&=0XFF000FFF;   //推挽输出
	GPIOC->CRL|=0X00222000;   //推挽输出
	GPIOC->ODR|=0<<3; 
	GPIOC->ODR|=1<<4;  	
	GPIOC->ODR|=1<<5;  
}
void MiniBalance_PWM_Init(u16 arr,u16 psc)
{		 					 
	MiniBalance_Motor_Init();  //初始化电机控制所需IO
	RCC->APB2ENR|=1<<11;       //使能TIM1时钟    
	RCC->APB1ENR|=1<<0;    		 //TIM2时钟使能
	RCC->APB1ENR|=1<<1;     		//TIM3时钟使能
	RCC->APB1ENR|=1<<2;     		//TIM4时钟使能
	
	RCC->APB2ENR|=1<<2;        //PORTA时钟使能   
	RCC->APB2ENR|=1<<3;        //PORTB时钟使能  


	GPIOA->CRH&=0XFFFFFFF0;    //PORTA 8复用输出
	GPIOA->CRH|=0X0000000B;   
	
	GPIOA->CRL&=0XF0FFFFF0;    //PORTA0 6 复用输出
	GPIOA->CRL|=0X0B00000B;   
	
	GPIOB->CRL&=0XF0FFFFFF;    //PORTB6 复用输出
	GPIOB->CRL|=0X0B000000;   
	
	
	TIM1->ARR=arr;             //设定计数器自动重装值 
	TIM1->PSC=psc;             //预分频器不分频
	TIM1->CCMR1|=6<<4;         //CH1 PWM1模式	
	TIM1->CCMR1|=1<<3;         //CH1预装载使能	     
	TIM1->CCER|=1<<0;          //CH1输出使能	
	TIM1->BDTR |= 1<<15;       //TIM1必须要这句话才能输出PWM
	TIM1->CR1|=0x01;          //使能定时器 			
	TIM1->CCR1=3600;

	TIM2->ARR=arr;             //设定计数器自动重装值 
	TIM2->PSC=psc;             //预分频器不分频
	TIM2->CCMR1|=6<<4;         //CH1 PWM1模式	
	TIM2->CCMR1|=1<<3;         //CH1预装载使能	     
	TIM2->CCER|=1<<0;          //CH1输出使能	
	TIM2->CR1|=0x01;          //使能定时器 			
	TIM2->CCR1=3600;
	
	
	TIM3->ARR=arr;             //设定计数器自动重装值 
	TIM3->PSC=psc;             //预分频器不分频
	TIM3->CCMR1|=6<<4;         //CH1 PWM1模式	
	TIM3->CCMR1|=1<<3;         //CH1预装载使能	     
	TIM3->CCER|=1<<0;          //CH1输出使能	
	TIM3->CR1|=0x01;          //使能定时器 			
	TIM3->CCR1=3600;
	
	TIM4->ARR=arr;             //设定计数器自动重装值 
	TIM4->PSC=psc;             //预分频器不分频
	TIM4->CCMR1|=6<<4;         //CH1 PWM1模式	
	TIM4->CCMR1|=1<<3;         //CH1预装载使能	     
	TIM4->CCER|=1<<0;          //CH1输出使能	
	TIM4->CR1|=0x01;          //使能定时器 			
	TIM4->CCR1=3600;
} 

void Set_PWM_Final(u16 arr1,u16 arr2,u16 arr3,u16 arr4)
{		 					 
	TIM1->ARR=arr1;             //设定计数器自动重装值 
	TIM1->PSC=14;             //预分频器不分频
	TIM1->CR1|=0<<4;          //向上计数模式
	TIM1->CCMR1|=6<<4;         //CH1 PWM1模式	
	TIM1->CCER|=1<<0;          //CH1输出使能	
	TIM1->CCER|=0<<1;          //CH1输出极性高
	TIM1->CCR1=arr1>>1;
	
	TIM2->ARR=arr2;             //设定计数器自动重装值 
	TIM2->PSC=14;             //预分频器不分频
	TIM2->CR1|=0<<4;          //向上计数模式
	TIM2->CCMR1|=6<<4;         //CH1 PWM1模式	
	TIM2->CCER|=1<<0;          //CH1输出使能	
	TIM2->CCER|=0<<1;          //CH1输出极性高
	TIM2->CCR1=arr2>>1;
	
	TIM3->ARR=arr3;             //设定计数器自动重装值 
	TIM3->PSC=14;             //预分频器不分频
	TIM3->CR1|=0<<4;          //向上计数模式
	TIM3->CCMR1|=6<<4;         //CH1 PWM1模式	
	TIM3->CCER|=1<<0;          //CH1输出使能	
	TIM3->CCER|=0<<1;          //CH1输出极性高
	TIM3->CCR1=arr3>>1;
	
	TIM4->ARR=arr4;             //设定计数器自动重装值 
	TIM4->PSC=14;             //预分频器不分频
	TIM4->CR1|=0<<4;          //向上计数模式
	TIM4->CCMR1|=6<<4;         //CH1 PWM1模式	
	TIM4->CCER|=1<<0;          //CH1输出使能	
	TIM4->CCER|=0<<1;          //CH1输出极性高
	TIM4->CCR1=arr4>>1;
} 
