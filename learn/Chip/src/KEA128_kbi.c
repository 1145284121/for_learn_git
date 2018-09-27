/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2017,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：179029047
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		KEA128_kbi
 * @company	   		成都逐飞科技有限公司
 * @author     		逐飞科技(QQ3184284598)
 * @version    		v2.0
 * @Software 		IAR 7.7 or MDK 5.23
 * @Target core		S9KEA128AMLK
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2017-11-6
 ********************************************************************************************************************/


#include "KEA128_kbi.h"


KBI_Type * kbi[2] = KBI_BASES;
uint8 KeyBox[4] = {5, 4, 7, 6};
Key_msg_t  key_msg[FifoSize];             //按键消息FIFO
volatile uint8      key_msg_front = 0, key_msg_rear = 0;    //接收FIFO的指针
volatile uint8      key_msg_flag = 0;           //按键消息FIFO状态


//-------------------------------------------------------------------------------------------------------------------
//  @brief      KBI键盘中断初始化
//  @param      chn             选择kbi通道   可选范围 参看KBI_CHn枚举
//  @return     void
//  @since      v2.0
//  Sample usage:               kbi_init(KBI1_P0,IRQ_RISING);		            //通道选择为KBI1_P0，上升沿触发
//								set_irq_priority(KBI1_IRQn,1);					//设置优先级,根据自己的需求设置 可设置范围为 0 - 3
//								enable_irq(KBI1_IRQn);							//打开KBI1_IRQn的中断开关
//								EnableInterrupts;								//打开总的中断开关
//-------------------------------------------------------------------------------------------------------------------
void kbi_init(KBI_CHn chn, TRIG_CFG cfg)
{
    uint8 kbi_x,kbi_n;
    
    kbi_x = KBIx(chn);      //获取模块号
    kbi_n = KBIn(chn);      //获取引脚号
    
    //开启KBI时钟
    if(0 == kbi_x)  SIM->SCGC |= SIM_SCGC_KBI0_MASK;
    else            SIM->SCGC |= SIM_SCGC_KBI1_MASK;
    
    //屏蔽KBI中断
    kbi[kbi_x]->SC &= ~(uint32)KBI_SC_KBIE_MASK;

    //设置触发模式
    if(IRQ_FALLING == cfg)
        kbi[kbi_x]->ES &= ~((uint32)1<<kbi_n);
    else
        kbi[kbi_x]->ES |= ((uint32)1<<kbi_n);
    
    //设置上拉
    port_pull((PTX_n)((uint8)(chn)));
    
    //使能KBI引脚
    kbi[kbi_x]->PE |= (1<<kbi_n);
    
    if(0 == kbi_x)  CLEAN_KBI0_FLAG;                         //清除标志位
    else            CLEAN_KBI1_FLAG;                         //清除标志位
    
    kbi[kbi_x]->SC = (0
                     | KBI_SC_KBIE_MASK    //KBI中断使能
                     | KBI_SC_RSTKBSP_MASK
                     //| KBI_SC_KBMOD_MASK   //边沿触发模式 0：边沿触发   1：电平触发
                     //| KBI_SC_KBSPEN_MASK  //使能KBI_SP寄存器
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
        key_msg_rear = 0;                       //重头开始
    }

    temp = key_msg_rear;
    if(temp == key_msg_front)                   //追到屁股了，满了
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

    if(key_msg_flag == 0)               //按键消息FIFO为空，直接返回0
    {
        return 0;
    }

    keymsg->key = key_msg[key_msg_front].key;       //从FIFO队首中获取按键值
    keymsg->status = key_msg[key_msg_front].status; //从FIFO队首中获取按键类型

    key_msg_front++;                                //FIFO队首指针加1，指向下一个消息

    if(key_msg_front >= FifoSize)          //FIFO指针队首溢出则从0开始计数
    {
        key_msg_front = 0;                          //重头开始计数（循环利用数组）
    }

    temp = key_msg_rear;
    if(key_msg_front == temp)                        //比较队首和队尾是否一样，一样则表示FIFO已空了
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
  for(num = 0; num < 4; num++)   //逐个扫描
  {
    status = key_get(num);
    if(status == 0)        //状态为按下
    {
      keytime[num]++;
      if(keytime[num] < 2)     //消抖时间
      {
        continue;      //未到消抖时间继续扫描
      }
      else if(keytime[num] == 2)   //达到响应时间
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
         keytime[num] = 0;//计时清零
      }
    }
    else
    {
      if(keytime[num] >= 2)    //是否按下过该按键
      {
        keymsg.key = num;
        keymsg.status = 2;        
        SendToFifo(keymsg);
        keytime[num] = 0;//计时清零
      }
    }
  }
}







