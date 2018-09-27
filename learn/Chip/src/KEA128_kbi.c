/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2017,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��179029047
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		KEA128_kbi
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ3184284598)
 * @version    		v2.0
 * @Software 		IAR 7.7 or MDK 5.23
 * @Target core		S9KEA128AMLK
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2017-11-6
 ********************************************************************************************************************/


#include "KEA128_kbi.h"


KBI_Type * kbi[2] = KBI_BASES;
uint8 KeyBox[4] = {5, 4, 7, 6};
Key_msg_t  key_msg[FifoSize];             //������ϢFIFO
volatile uint8      key_msg_front = 0, key_msg_rear = 0;    //����FIFO��ָ��
volatile uint8      key_msg_flag = 0;           //������ϢFIFO״̬


//-------------------------------------------------------------------------------------------------------------------
//  @brief      KBI�����жϳ�ʼ��
//  @param      chn             ѡ��kbiͨ��   ��ѡ��Χ �ο�KBI_CHnö��
//  @return     void
//  @since      v2.0
//  Sample usage:               kbi_init(KBI1_P0,IRQ_RISING);		            //ͨ��ѡ��ΪKBI1_P0�������ش���
//								set_irq_priority(KBI1_IRQn,1);					//�������ȼ�,�����Լ����������� �����÷�ΧΪ 0 - 3
//								enable_irq(KBI1_IRQn);							//��KBI1_IRQn���жϿ���
//								EnableInterrupts;								//���ܵ��жϿ���
//-------------------------------------------------------------------------------------------------------------------
void kbi_init(KBI_CHn chn, TRIG_CFG cfg)
{
    uint8 kbi_x,kbi_n;
    
    kbi_x = KBIx(chn);      //��ȡģ���
    kbi_n = KBIn(chn);      //��ȡ���ź�
    
    //����KBIʱ��
    if(0 == kbi_x)  SIM->SCGC |= SIM_SCGC_KBI0_MASK;
    else            SIM->SCGC |= SIM_SCGC_KBI1_MASK;
    
    //����KBI�ж�
    kbi[kbi_x]->SC &= ~(uint32)KBI_SC_KBIE_MASK;

    //���ô���ģʽ
    if(IRQ_FALLING == cfg)
        kbi[kbi_x]->ES &= ~((uint32)1<<kbi_n);
    else
        kbi[kbi_x]->ES |= ((uint32)1<<kbi_n);
    
    //��������
    port_pull((PTX_n)((uint8)(chn)));
    
    //ʹ��KBI����
    kbi[kbi_x]->PE |= (1<<kbi_n);
    
    if(0 == kbi_x)  CLEAN_KBI0_FLAG;                         //�����־λ
    else            CLEAN_KBI1_FLAG;                         //�����־λ
    
    kbi[kbi_x]->SC = (0
                     | KBI_SC_KBIE_MASK    //KBI�ж�ʹ��
                     | KBI_SC_RSTKBSP_MASK
                     //| KBI_SC_KBMOD_MASK   //���ش���ģʽ 0�����ش���   1����ƽ����
                     //| KBI_SC_KBSPEN_MASK  //ʹ��KBI_SP�Ĵ���
                     );
}


void BUZZSystem(void)
{
    if(gpio_get(K1)==0||gpio_get(K2)==0||gpio_get(K3)==0||gpio_get(K4)==0)
        gpio_set(BUZZ,1);
    else
        gpio_set(BUZZ,0);
}


uint8 key_get(uint8 key_num)
{
    switch(key_num)
    {
        case 1:    
            if(gpio_get(K1))
                return 0;
            else 
                return 1;
            break;
        case 2:    
            if(gpio_get(K2))
                return 0;
            else 
                return 1;
            break;
        case 3:    
            if(gpio_get(K3))
                return 0;
            else 
                return 1;
            break;
        case 4:    
            if(gpio_get(K4))
                return 0;
            else 
                return 1;
            break;
        default:  
            return 1;
            break;
    }
}

uint8 key_check(uint8 key_num)
{
    if(key_get(key_num) == 0)
    {
        systick_delay_ms(5);
        if( key_get(key_num) == 0)
        {
            return 0;
        }
    }
    return 1;
}

void SendToFifo(Key_msg_t keymsg)
{
    uint8 temp = 0;
    if(key_msg_flag == 1)
    {
        return ;
    }
    
    key_msg[key_msg_rear].key = keymsg.key;
    key_msg[key_msg_rear].status = keymsg.status;

    key_msg_rear++;

    if(key_msg_rear >= FifoSize)
    {
        key_msg_rear = 0;                       //��ͷ��ʼ
    }

    temp = key_msg_rear;
    if(temp == key_msg_front)                   //׷��ƨ���ˣ�����
    {
        key_msg_flag = 1;
    }
    else
    {
        key_msg_flag = 2;
    }
}


uint8 Get_KeyFifo(Key_msg_t *keymsg)
{
    uint8 temp;

    if(key_msg_flag == 0)               //������ϢFIFOΪ�գ�ֱ�ӷ���0
    {
        return 0;
    }

    keymsg->key = key_msg[key_msg_front].key;       //��FIFO�����л�ȡ����ֵ
    keymsg->status = key_msg[key_msg_front].status; //��FIFO�����л�ȡ��������

    key_msg_front++;                                //FIFO����ָ���1��ָ����һ����Ϣ

    if(key_msg_front >= FifoSize)          //FIFOָ�����������0��ʼ����
    {
        key_msg_front = 0;                          //��ͷ��ʼ������ѭ���������飩
    }

    temp = key_msg_rear;
    if(key_msg_front == temp)                        //�Ƚ϶��׺Ͷ�β�Ƿ�һ����һ�����ʾFIFO�ѿ���
    {
        key_msg_flag = 0;
    }
    else
    {
        key_msg_flag = 2;
    }

    return 1;
}



void key_scan(void)
{
  uint8 num;
  uint8 status = 0;
  Key_msg_t keymsg;
  static uint8 keytime[4];
  BUZZSystem();
  for(num = 0; num < 4; num++)   //���ɨ��
  {
    status = key_get(num);
    if(status == 0)        //״̬Ϊ����
    {
      keytime[num]++;
      if(keytime[num] < 2)     //����ʱ��
      {
        continue;      //δ������ʱ�����ɨ��
      }
      else if(keytime[num] == 2)   //�ﵽ��Ӧʱ��
      {
         keymsg.key = num;
         keymsg.status = 0;
         SendToFifo(keymsg);
      }
      else if(keytime[num] < 40)
      {
        continue;
      }
      else if(keytime[num] >= 40)
      {
         keymsg.key = num;
         keymsg.status = 1;        
         SendToFifo(keymsg);
         keytime[num] = 0;//��ʱ����
      }
    }
    else
    {
      if(keytime[num] >= 2)    //�Ƿ��¹��ð���
      {
        keymsg.key = num;
        keymsg.status = 2;        
        SendToFifo(keymsg);
        keytime[num] = 0;//��ʱ����
      }
    }
  }
}







