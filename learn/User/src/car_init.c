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
    //ADC��ʼ��
    adc_init(AD1);   
    adc_init(AD2);   
    adc_init(AD3);   
    adc_init(AD4);   
    
    //OLED��ʼ��
    LCD_Init();
    oled_display_16x8str(0,0,init_temp);
      
    while(BMX055_init() == false)
    {
        ErrLoop++;
        if(ErrLoop == 5)
            NVIC_SystemReset();                         //////�������5��ʧ����ֱ������
    };           ////BMX055����
    //IIC��ʼ��
    i2c_init(i2c1, 400 * 1000);                       //////400K����
    IIC_init();
    
    //������ʼ��
    ParameterInit();
    
    //��������Ʈ��ʼ��
    GyroOffset_init();    

    //flash��ʼ��	
    //FLASH_Init();
    
    //�����߳�ʼ��
    ir_init();
    set_irq_priority(IRQ_IRQn,0);	//�������ȼ�,�����Լ����������ÿ����÷�ΧΪ 0 - 3
    enable_irq(IRQ_IRQn);
      
    //��������ʼ��
    gpio_init(BUZZ,GPO,0);
	
	//LED��ʼ��
    gpio_init(LED,GPO,0);
    
    //�������ڳ�ʼ��
    //uart_init(uart1, 115200);
    
    //���뿪�ػ��ýӿڳ�ʼ��(����Ϊ���뿪�ػ��ýӿ�)
    gpio_init(K1,GPI,1);         //1	���뿪�أ�K1-4�����ڱ��ýӿ����ΪGPO;
    gpio_init(K2,GPI,1);         //2
    gpio_init(K3,GPI,1);         //3
    gpio_init(K4,GPI,1);         //4
    
    //�����ʼ��
    ftm_pwm_init(ftm2,ftm_ch2,14000,0);
    ftm_pwm_init(ftm2,ftm_ch3,14000,0);
    ftm_pwm_init(ftm2,ftm_ch4,14000,0);
    ftm_pwm_init(ftm2,ftm_ch5,14000,0);
    
    //���������ʼ��
    ftm_count_init(ftm0);
    gpio_init(Coder_dir_left,GPI,0);
    //�ұ�������ʼ��
    ftm_count_init(ftm1); 
    gpio_init(Coder_dir_right,GPI,0);

	//��ʱ2s
    systick_delay_ms(2000);
	
    //��ʱ����ʼ��
    pit_init_us(PIT0,5000);                //�жϳ�ʼ��(5ms)
    set_irq_priority(PIT_CH0_IRQn,1);      //�ж����ȼ�1
    enable_irq(PIT_CH0_IRQn);              //ʹ���ж�
    
}
