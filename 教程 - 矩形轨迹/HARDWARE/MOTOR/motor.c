#include "motor.h"
  /**************************************************************************
���ߣ�ƽ��С��֮��
�ҵ��Ա�С�꣺http://shop114407458.taobao.com/
**************************************************************************/
void MiniBalance_Motor_Init(void)
{
	RCC->APB2ENR|=1<<3;       //PORTBʱ��ʹ��   
	RCC->APB2ENR|=1<<4;       //PORTCʱ��ʹ��  
	GPIOB->CRL&=0XFFFFFF00;   //�������
	GPIOB->CRL|=0X00000022;   //�������
	GPIOB->ODR|=1<<0;  	
	
	GPIOC->CRL&=0XFF000FFF;   //�������
	GPIOC->CRL|=0X00222000;   //�������
	GPIOC->ODR|=0<<3; 
	GPIOC->ODR|=1<<4;  	
	GPIOC->ODR|=1<<5;  
}
void MiniBalance_PWM_Init(u16 arr,u16 psc)
{		 					 
	MiniBalance_Motor_Init();  //��ʼ�������������IO
	RCC->APB2ENR|=1<<11;       //ʹ��TIM1ʱ��    
	RCC->APB1ENR|=1<<0;    		 //TIM2ʱ��ʹ��
	RCC->APB1ENR|=1<<1;     		//TIM3ʱ��ʹ��
	RCC->APB1ENR|=1<<2;     		//TIM4ʱ��ʹ��
	
	RCC->APB2ENR|=1<<2;        //PORTAʱ��ʹ��   
	RCC->APB2ENR|=1<<3;        //PORTBʱ��ʹ��  


	GPIOA->CRH&=0XFFFFFFF0;    //PORTA 8�������
	GPIOA->CRH|=0X0000000B;   
	
	GPIOA->CRL&=0XF0FFFFF0;    //PORTA0 6 �������
	GPIOA->CRL|=0X0B00000B;   
	
	GPIOB->CRL&=0XF0FFFFFF;    //PORTB6 �������
	GPIOB->CRL|=0X0B000000;   
	
	
	TIM1->ARR=arr;             //�趨�������Զ���װֵ 
	TIM1->PSC=psc;             //Ԥ��Ƶ������Ƶ
	TIM1->CCMR1|=6<<4;         //CH1 PWM1ģʽ	
	TIM1->CCMR1|=1<<3;         //CH1Ԥװ��ʹ��	     
	TIM1->CCER|=1<<0;          //CH1���ʹ��	
	TIM1->BDTR |= 1<<15;       //TIM1����Ҫ��仰�������PWM
	TIM1->CR1|=0x01;          //ʹ�ܶ�ʱ�� 			
	TIM1->CCR1=3600;

	TIM2->ARR=arr;             //�趨�������Զ���װֵ 
	TIM2->PSC=psc;             //Ԥ��Ƶ������Ƶ
	TIM2->CCMR1|=6<<4;         //CH1 PWM1ģʽ	
	TIM2->CCMR1|=1<<3;         //CH1Ԥװ��ʹ��	     
	TIM2->CCER|=1<<0;          //CH1���ʹ��	
	TIM2->CR1|=0x01;          //ʹ�ܶ�ʱ�� 			
	TIM2->CCR1=3600;
	
	
	TIM3->ARR=arr;             //�趨�������Զ���װֵ 
	TIM3->PSC=psc;             //Ԥ��Ƶ������Ƶ
	TIM3->CCMR1|=6<<4;         //CH1 PWM1ģʽ	
	TIM3->CCMR1|=1<<3;         //CH1Ԥװ��ʹ��	     
	TIM3->CCER|=1<<0;          //CH1���ʹ��	
	TIM3->CR1|=0x01;          //ʹ�ܶ�ʱ�� 			
	TIM3->CCR1=3600;
	
	TIM4->ARR=arr;             //�趨�������Զ���װֵ 
	TIM4->PSC=psc;             //Ԥ��Ƶ������Ƶ
	TIM4->CCMR1|=6<<4;         //CH1 PWM1ģʽ	
	TIM4->CCMR1|=1<<3;         //CH1Ԥװ��ʹ��	     
	TIM4->CCER|=1<<0;          //CH1���ʹ��	
	TIM4->CR1|=0x01;          //ʹ�ܶ�ʱ�� 			
	TIM4->CCR1=3600;
} 

void Set_PWM_Final(u16 arr1,u16 arr2,u16 arr3,u16 arr4)
{		 					 
	TIM1->ARR=arr1;             //�趨�������Զ���װֵ 
	TIM1->PSC=14;             //Ԥ��Ƶ������Ƶ
	TIM1->CR1|=0<<4;          //���ϼ���ģʽ
	TIM1->CCMR1|=6<<4;         //CH1 PWM1ģʽ	
	TIM1->CCER|=1<<0;          //CH1���ʹ��	
	TIM1->CCER|=0<<1;          //CH1������Ը�
	TIM1->CCR1=arr1>>1;
	
	TIM2->ARR=arr2;             //�趨�������Զ���װֵ 
	TIM2->PSC=14;             //Ԥ��Ƶ������Ƶ
	TIM2->CR1|=0<<4;          //���ϼ���ģʽ
	TIM2->CCMR1|=6<<4;         //CH1 PWM1ģʽ	
	TIM2->CCER|=1<<0;          //CH1���ʹ��	
	TIM2->CCER|=0<<1;          //CH1������Ը�
	TIM2->CCR1=arr2>>1;
	
	TIM3->ARR=arr3;             //�趨�������Զ���װֵ 
	TIM3->PSC=14;             //Ԥ��Ƶ������Ƶ
	TIM3->CR1|=0<<4;          //���ϼ���ģʽ
	TIM3->CCMR1|=6<<4;         //CH1 PWM1ģʽ	
	TIM3->CCER|=1<<0;          //CH1���ʹ��	
	TIM3->CCER|=0<<1;          //CH1������Ը�
	TIM3->CCR1=arr3>>1;
	
	TIM4->ARR=arr4;             //�趨�������Զ���װֵ 
	TIM4->PSC=14;             //Ԥ��Ƶ������Ƶ
	TIM4->CR1|=0<<4;          //���ϼ���ģʽ
	TIM4->CCMR1|=6<<4;         //CH1 PWM1ģʽ	
	TIM4->CCER|=1<<0;          //CH1���ʹ��	
	TIM4->CCER|=0<<1;          //CH1������Ը�
	TIM4->CCR1=arr4>>1;
} 
