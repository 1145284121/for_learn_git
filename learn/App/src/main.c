/*********************************************************************************************************************
 * 
 * @file       		main.c
 *  				主函数
 * @core			S9KEA128
 * @date       		2018
 ********************************************************************************************************************/

#include "includefile.h"
   
int main(void)
{
    get_clk();              //获取时钟频率
    DisableInterrupts;

    car_init();             
//  param_in();             //读取参数
    EnableInterrupts; 
    while(1)
    {
	
    }
}
