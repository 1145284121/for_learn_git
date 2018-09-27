
#ifndef _KEA128_i2c_h
#define _KEA128_i2c_h


#include "common.h"
//#include "KEA128_GPIO.h"


#include "KEA128_PIT.h"         /////使用到其中的延时函数
#include "KEA128_GPIO.h"
#include "SKEAZ1284.h"

#define IICDelay       8 
#define IIC_GPIO       GPIOC_BASE_PTR ////////A12  A13对应的是B4   B5，详细见技术手册，更改管脚请在IIC_init()里面一并更改
#define IIC_SCLpin     6
#define IIC_SDApin     7



#define IIC_SDA_OUT()   {IIC_GPIO->PIDR |= (1 << IIC_SDApin); IIC_GPIO->PDDR |= (1 << IIC_SDApin);}
#define IIC_SDA_IN()    {IIC_GPIO->PIDR &= ~(1 << IIC_SDApin); IIC_GPIO->PDDR &= ~(1 << IIC_SDApin);}
#define IIC_DATA        ((IIC_GPIO->PDIR >> IIC_SDApin) & 0x01)
#define IIC_SDA_H       (IIC_GPIO->PDOR |= (1 << IIC_SDApin))
#define IIC_SCL_H       (IIC_GPIO->PDOR |= (1 << IIC_SCLpin))
#define IIC_SDA_L       (IIC_GPIO->PDOR &= ~(1 << IIC_SDApin))
#define IIC_SCL_L       (IIC_GPIO->PDOR &= ~(1 << IIC_SCLpin))

typedef enum _bool{
false = 0,
true = 1 
}bool;


uint8 IIC_Read_Reg(uint8 addr, uint8 offset);
bool IIC_Write_Reg(uint8 addr, uint8 offset, uint8 data);
bool IIC_Read_Buff(uint8 addr, uint8 offset, uint8* buff, uint8 size);
void IIC_init(void);



typedef enum
{
    i2c0  = 0,
    i2c1  = 1
} I2Cn_e;

typedef enum MSmode
{
    MWSR =   0x00,  // 主机写模式  
    MRSW =   0x01   // 主机读模式  
} MSmode;

typedef struct{
  int core_mhz;
  int core_khz;
  int bus_khz;
  int ftm_khz;
}SysClocktypedef;


typedef struct
{
  float GYROXdata;
  float GYROYdata;
  float GYROZdata;
  float ACCXdata;
  float ACCYdata;
  float ACCZdata;
  float MAGXdata;
  float MAGYdata;
  float MAGZdata;
}BMX055Datatypedef;

extern SysClocktypedef SysClock;
//仅支持 I2C主机模式
extern uint32  i2c_init(I2Cn_e i2cn, uint32 baud);                               //初始化I2C
extern void    i2c_write_reg(I2Cn_e, uint8 SlaveID, uint8 reg, uint8 Data);      //写入数据到寄存器
extern uint8   i2c_read_reg (I2Cn_e, uint8 SlaveID, uint8 reg);                  //读取寄存器的数据
extern uint8   i2c_read_reg_bytes(I2Cn_e i2cn, uint8 SlaveID, uint8 reg, uint8 num, uint8 * addr);







#endif
