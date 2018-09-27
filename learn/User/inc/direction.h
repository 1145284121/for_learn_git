#ifndef __DIRECTION_H__
#define __DIRECTION_H__

#include  "common.h"


typedef struct{
  float AD_First;
  float AD_Second;
  float AD_Third;
  float AD_Forth;
  float AD_R;
  float AD_L;
  float AD_C1;
  float AD_C2;
}ADTesttypedef;


typedef struct{
  uint8  Quaternion_init;   ////��Ԫ����ʼ��
  uint8  Motor_ON;          ////���ʹ��
  
  float speed_R;          ///�ұ������ٶ�
  float speed_L;          //��������ٶ�
  float speed_M;          //�б������ٶ�
  float speed_Mtrue;     //����ʵ�ٶ�
  float speed_Rtrue;      //����ʵ�ٶ�
  float speed_Ltrue;       //����ʵ�ٶ�
  float speed_RtrueEncoder;  //����ʵ�������ٶ�
  float speed_LtrueEncoder;  //����ʵ�������ٶ�
  float TureSpeedGrowLimit;    //��ʵ�ٶ�����
  float TureSpeedReduceLimit;  //��ʵ�ٶȽ���
  float TrueSpeedMAXLimit;     //��ʵ�ٶ����ֵ
  float TrueSpeedYawRate;      //����ʵ�ٶȼ���Ľ��ٶ�
  float ExpectSpeed;           //�����ٶ�
  float ExpectCur;              
  float ExpectAngleGrow;   ///�����Ƕ���������
  float ExpectAngleReduce; ///�����ǶȽ�������
  
  
  float BalanceAngle;      ///ƽ��Ƕ�
  float AngleMax;          ///���Ƕ�
  float AngleMin;          ///��С�Ƕ�
  
  
  float pitchrate;   //�������ٶ�
  float pitch;       //������
  float yaw;         //ƫ����
  float yawrate;     //ƫ�����ٶ�
  float rollrate;    //�������ٶ�
  float roll;        //������
  float radius;       //�뾶
  float curvature;    //����
  
  float time_s;      //ʱ��
  
  float length;        //����
  int16  lengthcount;  //��������
  float lengthcarry;  //�����λ
  
  float BAT;         ///��ص�ѹ
  
  float NowSiteX;    ///��ǰ����X
  float NowSiteY;    ///��ǰ����Y
  
  float MinSpeed;     //��С�ٶ�
  float MinR;         //��С�뾶
  float MaxR;         //���뾶
  float MaxAcc;       //������ļ��ٶ�
  float MaxSpeed;     //����ٶ�
  float BalanceSpeed; //�����ٶ�
  float SteerRmax;    //�����ƫ���ֵ
  float SteerLmax;    //�����ƫ���ֵ
  float BodyRadius;   //��������뾶
  float RadiusAdjustSpeed;      //
  float StraightTest;          //
  int   StraightCount;         //
  
  float MagX;
  float MagY;
  float MagZ;
  
  
  uint8 RoadType;            //ֱ��Ϊ1������Ϊ0 
  uint8 IsStop;
}CarInfotypedef;


typedef struct{
  float P;
  float I;
  float D;
}PIDcontroltypedef;

/**********ȫ�ֱ����ⲿ����********/

extern CarInfotypedef         CarInfo;
extern ADTesttypedef          ADTest;
extern PIDcontroltypedef      SpeedRadius;
extern PIDcontroltypedef      yawPIDcontrol;
extern float    g_dirControl_P;
extern float    g_dirControl_D;
extern float    g_fDirectionError[2];
extern float    g_fDirectionError_dot[2];
extern float    g_fDirectionControlOut;
extern int16    g_ValueOfAD[4];
extern int16    g_ValueOfADFilter[4];




/***********��������***********/

void DirectionControl(void);
static void GetSensorData(void);
float CalcDiff(float h, float L, float M, float R);
void Read_ADC(void);
void ParameterInit(void);
void GyroOffset_init(void);

#endif