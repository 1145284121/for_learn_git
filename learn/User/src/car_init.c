/*********************************************************************************************************************
 * 
 * @file       		car_init.c
 *  		
 * @core			S9KEA128
 * @date       		2018
 ********************************************************************************************************************/


#include "includefile.h"
#include "KEA128_i2c.h"
//#include "SoftIIC.h"
#include "BMX055.h"
uint8 ErrLoop = 0;
char init_temp[8]="initok";

void car_init()
{
    //ADC初始化
    adc_init(AD1);   
    adc_init(AD2);   
    adc_init(AD3);   
    adc_init(AD4);   
    
    //OLED初始化
    LCD_Init();
    oled_display_16x8str(0,0,init_temp);
      
    while(BMX055_init() == false)
    {
        ErrLoop++;
        if(ErrLoop == 5)
            NVIC_SystemReset();                         //////如果超出5次失败则直接重启
    };           ////BMX055配置
    //IIC初始化
    i2c_init(i2c1, 400 * 1000);                       //////400K速率
    IIC_init();
    
    //参数初始化
    ParameterInit();
    
    //陀螺仪零飘初始化
    GyroOffset_init();    

    //flash初始化	
    //FLASH_Init();
    
    //红外线初始化
    ir_init();
    set_irq_priority(IRQ_IRQn,0);	//设置优先级,根据自己的需求设置可设置范围为 0 - 3
    enable_irq(IRQ_IRQn);
      
    //蜂鸣器初始化
    gpio_init(BUZZ,GPO,0);
	
	//LED初始化
    gpio_init(LED,GPO,0);
    
    //蓝牙串口初始化
    //uart_init(uart1, 115200);
    
    //拨码开关或备用接口初始化(复用为拨码开关或备用接口)
    gpio_init(K1,GPI,1);         //1	拨码开关，K1-4若用于备用接口则改为GPO;
    gpio_init(K2,GPI,1);         //2
    gpio_init(K3,GPI,1);         //3
    gpio_init(K4,GPI,1);         //4
    
    //电机初始化
    ftm_pwm_init(ftm2,ftm_ch2,14000,0);
    ftm_pwm_init(ftm2,ftm_ch3,14000,0);
    ftm_pwm_init(ftm2,ftm_ch4,14000,0);
    ftm_pwm_init(ftm2,ftm_ch5,14000,0);
    
    //左编码器初始化
    ftm_count_init(ftm0);
    gpio_init(Coder_dir_left,GPI,0);
    //右编码器初始化
    ftm_count_init(ftm1); 
    gpio_init(Coder_dir_right,GPI,0);

	//延时2s
    systick_delay_ms(2000);
	
    //定时器初始化
    pit_init_us(PIT0,5000);                //中断初始化(5ms)
    set_irq_priority(PIT_CH0_IRQn,1);      //中断优先级1
    enable_irq(PIT_CH0_IRQn);              //使能中断
    
}

