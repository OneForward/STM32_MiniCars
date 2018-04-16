#include "adc.h"
/**************************************************************************
函数功能：ACD初始化电池电压检测
入口参数：无
返回  值：无
作    者：平衡小车之家
**************************************************************************/
void  Adc_Init(void)
{    
  //先初始化IO口
 	RCC->APB2ENR|=1<<2;    //使能PORTA口时钟 
  //RCC->APB2ENR|=1<<4;    //使能PORTC口时钟 
	GPIOA->CRL&=0;//PA76543210 anolog输入 	
  GPIOA->CRH&=0;//PA76543210 anolog输入
//  GPIOC->CRL&=0;//PC76543210 anolog输入
//  GPIOC->CRH&=0;//PC76543210 anolog输入
	
	RCC->APB2ENR|=1<<9;    //ADC1时钟使能	  
	RCC->APB2RSTR|=1<<9;   //ADC1复位
	RCC->APB2RSTR&=~(1<<9);//复位结束	    
	RCC->CFGR&=~(3<<14);   //分频因子清零	
	//SYSCLK/DIV2=12M ADC时钟设置为12M,ADC最大时钟不能超过14M!
	//否则将导致ADC准确度下降! 
	RCC->CFGR|=2<<14;      	 
	ADC1->CR1&=0XF0FFFF;   //工作模式清零
	ADC1->CR1|=0<<16;      //独立工作模式  
	ADC1->CR1&=~(1<<8);    //非扫描模式	  
	ADC1->CR2&=~(1<<1);    //单次转换模式
	ADC1->CR2&=~(7<<17);	   
	ADC1->CR2|=7<<17;	   //软件控制转换  
	ADC1->CR2|=1<<20;      //使用用外部触发(SWSTART)!!!	必须使用一个事件来触发
	ADC1->CR2&=~(1<<11);   //右对齐	 
	ADC1->SQR1&=~(0XF<<20);
	ADC1->SQR1&=0<<20;     //1个转换在规则序列中 也就是只转换规则序列1 			   
	
	//revised!!!	
	//设置通道0~7的采样时间
	ADC1->SMPR2&=0;//通道0-9采样时间清空	
  ADC1->SMPR1&=0;//通道10-15采样时间清空	
  ADC1->SMPR1|=5<<15;      //通道15  239.5周期,提高采样时间可以提高精确度
  ADC1->SMPR1|=5<<12;      //通道14  239.5周期,提高采样时间可以提高精确度
	ADC1->SMPR1|=5<<9;      //通道13  239.5周期,提高采样时间可以提高精确度
	ADC1->SMPR1|=5<<6;      //通道12  239.5周期,提高采样时间可以提高精确度
	ADC1->SMPR1|=5<<3;      //通道11  239.5周期,提高采样时间可以提高精确度	 
	ADC1->SMPR1|=5<<0;      //通道10  239.5周期,提高采样时间可以提高精确度	 
	ADC1->SMPR2|=5<<27;      //通道9  239.5周期,提高采样时间可以提高精确度	 
	ADC1->SMPR2|=5<<24;      //通道8  239.5周期,提高采样时间可以提高精确度	   
  ADC1->SMPR2|=5<<21;      //通道7  239.5周期,提高采样时间可以提高精确度
  ADC1->SMPR2|=5<<18;      //通道6  239.5周期,提高采样时间可以提高精确度
	ADC1->SMPR2|=5<<15;      //通道5  239.5周期,提高采样时间可以提高精确度
	ADC1->SMPR2|=5<<12;      //通道4  239.5周期,提高采样时间可以提高精确度
	ADC1->SMPR2|=5<<9;      //通道3  239.5周期,提高采样时间可以提高精确度	 
	ADC1->SMPR2|=5<<6;      //通道2  239.5周期,提高采样时间可以提高精确度	 
	ADC1->SMPR2|=5<<3;      //通道1  239.5周期,提高采样时间可以提高精确度	 
	ADC1->SMPR2|=5<<0;      //通道0  239.5周期,提高采样时间可以提高精确度	 

	ADC1->CR2|=1<<0;	    //开启AD转换器	 
	ADC1->CR2|=1<<3;        //使能复位校准  
	while(ADC1->CR2&1<<3);  //等待校准结束 			 
    //该位由软件设置并由硬件清除。在校准寄存器被初始化后该位将被清除。 		 
	ADC1->CR2|=1<<2;        //开启AD校准	   
	while(ADC1->CR2&1<<2);  //等待校准结束 
}		

/**************************************************************************
函数功能：AD采样
入口参数：ADC1 的通道
返回  值：AD转换结果
**************************************************************************/
u16 Get_Adc(u8 ch)   
{
	//设置转换序列	  		 
	ADC1->SQR3&=0XFFFFFFE0;//规则序列1 通道ch:0-16
	ADC1->SQR3|=ch;		  			    
	ADC1->CR2|=1<<22;       //启动规则转换通道 
	while(!(ADC1->SR&1<<1));//等待转换结束	 	   
	return ADC1->DR;		//返回adc值	
}

/**************************************************************************
函数功能：读取电池电压 
入口参数：无
返回  值：电池电压 单位MV
**************************************************************************/
int Get_battery_volt(void)   
{  
	int Volt;//电池电压
	Volt=Get_Adc(Battery_Ch)*3.3*11.0*100/1.0/4096;	//电阻分压，具体根据原理图简单分析可以得到	
	return Volt;
}


