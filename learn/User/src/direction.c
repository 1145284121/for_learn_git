/*********************************************************************************************************************
 * 
 * @file       		direction.c
 *  				方向控制
 * @core			S9KEA128
 * @date       		2018
 ********************************************************************************************************************/
#include  "includefile.h"
#include  "math.h"
#include  "BMX055.h"
#include  "Attitude_Calculation.h"

#define PCconvert          (float)0.551010112
#define ADconvert          (float)0.000212960
#define Yaw_Control_P_Max  1000
   
/**********全局变量定义********/ 
//static uint32 ErrCount = 0;   //////错误帧数
//static bool IsRead = false;  /////读取状态   
static bool  IsGyroOffsetReset = false;          /////如果需要进行陀螺仪零飘矫正则将改变量置为   1
                                                  /////置1的方式可以通过 按键或者别的操作
CarInfotypedef    CarInfo;
ADTesttypedef          ADTest;
BMX055Datatypedef      BMX055_data;
PIDcontroltypedef      SpeedRadius={100,0,500};
PIDcontroltypedef      yawPIDcontrol={700,0,2200};
AttitudeDatatypedef    GyroOffset;           //
//float g_dirControl_P = 700;		//方向控制P
//float g_dirControl_D = 2200;	//方向控制D
//float g_fDirectionError[2];		//方向偏差（g_fDirectionError[0]为一对水平电感的差比和偏差）
								//		  （g_fDirectionError[1]为一对垂直电感的差比和偏差）
//float g_fDirectionError_dot[2];//方向偏差微分（g_fDirectionError_dot[0]为一对水平电感的差比和偏差微分）
								//			 （g_fDirectionError_dot[1]为一对垂直电感的差比和偏差微分）
float g_fDirectionControlOut;	//方向控制输出
int16 g_ValueOfAD[4]={0};	        //获取的电感值
int16 g_ValueOfADFilter[4]={0};	//阶梯滤波的电感值（未使用）
uint8 Flag_Round = OFF;			//进入环岛的标志（在环岛里为ON）
extern float g_fRealSpeed;				//真实速度
extern float g_fLeftRealSpeed;
extern float g_fRighRealSpeed;
float termp=8.88;

void ParameterInit(void)   ///参数初始化
{
        //SysTicks = 0;
    CarInfo.time_s = 0;
    CarInfo.yaw = 0;
    CarInfo.Motor_ON = 'F';
    CarInfo.Quaternion_init = 'F';
    CarInfo.TrueSpeedMAXLimit = 600;
    CarInfo.TureSpeedGrowLimit = 300;
    CarInfo.TureSpeedReduceLimit = 300;
    CarInfo.length = 0;
    CarInfo.lengthcarry = 0;
    CarInfo.lengthcount = 0;
    CarInfo.speed_Mtrue = 0;
    CarInfo.speed_Ltrue = 0;
    CarInfo.speed_LtrueEncoder = 0;
    CarInfo.speed_Rtrue = 0;
    CarInfo.speed_RtrueEncoder = 0;
    CarInfo.ExpectAngleGrow = 30;
    CarInfo.ExpectAngleReduce = 30;
    CarInfo.MinR = 40;
    CarInfo.MaxR = 50;
    CarInfo.AngleMax = 15;
    CarInfo.AngleMin = -10;
    CarInfo.BodyRadius = 8;
    CarInfo.ExpectCur = 280;
    CarInfo.MaxAcc = 18;
    CarInfo.IsStop = 'F';     
}


float AccZAngle = 0;

void GyroOffset_init(void)      /////////陀螺仪零飘初始化
{
    static uint16 Count = 0;
    if(Count == 0)
    {
        GyroOffset.Xdata = 0;
        GyroOffset.Ydata = 0;
        GyroOffset.Zdata = 0;
    }
  
    if(Count == 1000)
    {
        GyroOffset.Xdata /= 1000;
        GyroOffset.Ydata /= 1000;
        GyroOffset.Zdata /= 1000;
        IsGyroOffsetReset = false;
        Count = 0;
    }
    else
    {
        BMX055_DataRead(&BMX055_data, 0);
        GyroOffset.Xdata = BMX055_data.GYROXdata;
        GyroOffset.Ydata = BMX055_data.GYROYdata;
        GyroOffset.Zdata = BMX055_data.GYROZdata;
        Count++;
    }
}




void Get_Car_Angle(void)   //获取陀螺仪角度
{
    float AccZ = 0;
    float LastAngle = 0;
    float AccZAdjust = 0;
    static uint8 MAGWaitCount = 0;
    MAGWaitCount++;
    if(MAGWaitCount < 15)
        BMX055_DataRead(&BMX055_data, 0);
    else
    {
        BMX055_DataRead(&BMX055_data, 1);    //////每30ms读取一次磁力计
        MAGWaitCount = 0;
    }
    BMX055_data.GYROXdata = (BMX055_data.GYROXdata - Gyro.Xoffset) * 0.030517578;   
    BMX055_data.GYROYdata = (BMX055_data.GYROYdata - Gyro.Yoffset) * 0.030517578;
    BMX055_data.GYROZdata = (BMX055_data.GYROZdata - Gyro.Zoffset) * 0.030517578;
    ///////1000 / 32768     //////BMX055本身零飘几乎可以忽略不计，但是安全起见还是矫正一下
    BMX055_data.ACCXdata *= 0.001953125;    ///////4 / 2048
    BMX055_data.ACCYdata *= 0.001953125;
    BMX055_data.ACCZdata *= 0.001953125;
  

    Acc.Xdata = BMX055_data.ACCXdata;
    Acc.Ydata = BMX055_data.ACCYdata;
    Acc.Zdata = BMX055_data.ACCZdata;
    Gyro.Xdata = BMX055_data.GYROXdata;
    Gyro.Ydata = BMX055_data.GYROYdata;
    Gyro.Zdata = BMX055_data.GYROZdata;
    
    if(CarInfo.Quaternion_init == 'F')
    {
        Quaternion_init();               //四元数初始化必要,此时最好静止
        CarInfo.Quaternion_init = 'T';
    }
    else
    {
      //Attitude_UpdateGyro();////由陀螺仪解四元数          ///
      //Attitude_UpdateAcc();////由加速度计解四元数              ///
  
        CarInfo.pitchrate = -EulerAngleRate.Roll/PI*180;   //俯仰角速度单位°
        CarInfo.yawrate = EulerAngleRate.Yaw / PI * 180;
        CarInfo.pitch = -EulerAngle.Roll/PI*180;            //俯仰角单位°
        CarInfo.yaw   = EulerAngle.Yaw/PI*180;
      
        AccZ = -Acc.Zdata;
        if(AccZ > 1)
            AccZ = 1;
        if(AccZ < -1)
            AccZ = -1;            /////////加速度真实值限幅
        AccZAngle = asinf(AccZ) * 180 / PI;
        LastAngle = CarInfo.pitch;
        AccZAdjust = (AccZAngle - LastAngle) * 0.002;
        CarInfo.pitch += (-Gyro.Xdata * 0.002 + AccZAdjust);
        CarInfo.MagX = BMX055_data.MAGXdata;
        CarInfo.MagY = BMX055_data.MAGYdata;
        CarInfo.MagZ = BMX055_data.MAGZdata;
    }
    return;
}



static void GetSensorData(void)                              //操作了CarInfo中的ExpectCur，RoadType，使用了StraightTest，，另外，改变了ADTest
{
    static uint16 StraightCount = 0;
      //static float LastCur = 0;
      
    Read_ADC();
    ADTest.AD_First = g_ValueOfAD[0];
    ADTest.AD_Second = g_ValueOfAD[1];
    ADTest.AD_Third = g_ValueOfAD[2];
    ADTest.AD_Forth = g_ValueOfAD[3];
    termp = g_ValueOfAD[0];
    //for(uint8 i = 0; i < 20; i++)
    //GetSensorAD();
    // ADTest.AD_C2 = ADTest.AD_Second / 20;
    // ADTest.AD_L =  ADTest.AD_First / 20;
    //ADTest.AD_C1 = ADTest.AD_Forth / 20;
    // ADTest.AD_R = ADTest.AD_Third / 20
      

    if(ADTest.AD_C1 <= 1)
        ADTest.AD_C1 = 1;
      
    CarInfo.ExpectCur = 3000 * (ADTest.AD_R - ADTest.AD_L) / ((ADTest.AD_L + ADTest.AD_R + ADTest.AD_C1) * ADTest.AD_C1);
    if(CarInfo.ExpectCur > 0)
        CarInfo.ExpectCur = (CarInfo.ExpectCur) / (35 * CarInfo.ExpectCur + 30);
    else
        CarInfo.ExpectCur = -(CarInfo.ExpectCur) / (35 * CarInfo.ExpectCur - 30);
      
      //CarInfo.ExpectCur += SpeedRadius.D * (CarInfo.ExpectCur - LastCur);
      //LastCur = CarInfo.ExpectCur;
    if(fabs(CarInfo.ExpectCur) <= CarInfo.StraightTest)  ////具体数值视情况而定
    {
        StraightCount++;
        if(StraightCount >= CarInfo.StraightCount)   ////直道判定
            CarInfo.RoadType = 1;
    }
    else
    {
        StraightCount = 0;
        CarInfo.RoadType = 0;
    }
}



static float d = 0.125; // 左右电感与中心电感的距离
// 根据高度、左中右电感值计算导线相对位置
float CalcDiff(float h, float L, float M, float R) //根据ADTest 中的值计算，最后返回一个folat数
{
    // 中间计算结果
    float LR_m = sqrtf(4 * L * R * d * d - (L - R) * (L - R) * h * h);
    float LM_m = sqrtf(L * M * d * d - (L - M) * (L - M) * h * h);
    float RM_m = sqrtf(R * M * d * d - (R - M) * (R - M) * h * h);
    // 一共3*2=6个解
    float LR_solve[2];
    float LM_solve[2];
    float RM_solve[2];
    LR_solve[0] = ((L + R) * d - LR_m) / (R - L);
    LR_solve[1] = ((L + R) * d + LR_m) / (R - L);
    LM_solve[0] = (L * d - LM_m) / (M - L);
    LM_solve[1] = (L * d + LM_m) / (M  - L);
    RM_solve[0] = (R * d - RM_m) / (R - M);
    RM_solve[1] = (R * d + RM_m) / (R - M);
    // 选取其中三个相差最小的解作为正解
    float LR_true, LM_true, RM_true;
    float LR_A = fabs(LM_solve[0] - LR_solve[0]) < fabs(LM_solve[1] - LR_solve[0]) ? LM_solve[0] : LM_solve[1]
               + fabs(RM_solve[0] - LR_solve[0]) < fabs(RM_solve[1] - LR_solve[0]) ? RM_solve[0] : RM_solve[1];
    float LR_B = fabs(LM_solve[0] - LR_solve[1]) < fabs(LM_solve[1] - LR_solve[1]) ? LM_solve[0] : LM_solve[1]
               + fabs(RM_solve[0] - LR_solve[1]) < fabs(RM_solve[1] - LR_solve[1]) ? RM_solve[0] : RM_solve[1];
    // 选取LR的两个解中的一个作为正解
    if (LR_A < LR_B)
    {
        LR_true = LR_solve[0];
    }
    else
    {
        LR_true = LR_solve[1];
    }
    // 选取LM的两个解中的一个作为正解
    if (fabs(LM_solve[0] - LR_true) < fabs(LM_solve[1] - LR_true))
    {
        LM_true = LM_solve[0];
    }
    else
    {
        LM_true = LM_solve[1];
    }
    // 选取RM的两个解中的一个作为正解
    if (fabs(RM_solve[0] - LR_true) < fabs(RM_solve[1] - LR_true))
    {
        RM_true = RM_solve[0];
    }
    else
    {
        RM_true = RM_solve[1];
    }
    // 三个正解取均值
    float avg_solve = (LR_true + LM_true + RM_true) / 3;
    return avg_solve;
}


static float Set_Curvature(float Excurvature)/////半径闭环（完成）       使用到了CarInfo中的speed_Mtrue，MaxAcc，MinR，RadiusAdjustSpeed，yawrate
{                                                                               //一系列运算操作后，获得OUTPwm
    static float   ExYawRate = 0;
    static float   OutPwm = 0;
    float SpeedMinRadius = 0;
    //float PathMinRadius = 0;
    float MinRadius = 0;
    float MaxCurvature = 0;
    float SpeedGainCur = 1;

    if(g_fRealSpeed > CarInfo.MinSpeed)                     //中真值速度
    {
        SpeedMinRadius = g_fRealSpeed * g_fRealSpeed * 0.01 / CarInfo.MaxAcc;//最大离心加速度
        if(SpeedMinRadius < CarInfo.MinR)
            MinRadius = CarInfo.MinR;
    }
    else
        MinRadius = CarInfo.MinR;


    
    if(g_fRealSpeed > CarInfo.RadiusAdjustSpeed)
    {
        SpeedGainCur = ((g_fRealSpeed - CarInfo.RadiusAdjustSpeed) * SpeedRadius.P);
        if(SpeedGainCur < 0.1)
            SpeedGainCur = 0.1;
        else if(SpeedGainCur > 10)
            SpeedGainCur = 10;
    }
    SpeedGainCur = 1 / SpeedGainCur;
    if(Excurvature > 0)
        Excurvature = SpeedGainCur * Excurvature / (SpeedGainCur - Excurvature);
    else
        Excurvature = SpeedGainCur * Excurvature / (SpeedGainCur + Excurvature); 
    

    MaxCurvature = 1 / MinRadius;                  ///////最大曲率限幅

    
    if(Excurvature > MaxCurvature)Excurvature = MaxCurvature;
    else if(Excurvature < -MaxCurvature)Excurvature = -MaxCurvature; /////期望曲率限幅

    ExYawRate = Excurvature * CarInfo.speed_Mtrue * 180 / PI;  /////期望角速度

    float Yaw_Control_P = ExYawRate - CarInfo.yawrate;

    if(Yaw_Control_P > Yaw_Control_P_Max)
        Yaw_Control_P = Yaw_Control_P_Max;
    else if(Yaw_Control_P < -Yaw_Control_P_Max)
        Yaw_Control_P = -Yaw_Control_P_Max;///积分限幅

    float SetP, SetD;

    SetP = yawPIDcontrol.P;
    SetD = yawPIDcontrol.D;


    if(Yaw_Control_P * CarInfo.yawrate < 0)//////转向与角速度不同向，微分清零
        SetD = 0;

      
    OutPwm = (SetP * Yaw_Control_P + SetD * CarInfo.yawrate); 
    
    return (OutPwm);
    
}


/**
 * @file		方向控制
 *		一般情况下：用两水平电感的差比和作为偏差
 *		在环岛时中：用量垂直电感的差比和作为偏差
 *
 *				电感值对应变量
 *
 *				g_ValueOfAD[0]		g_ValueOfAD[1]
 *				(水平左电感)		（水平右电感）
 *				g_ValueOfAD[2]		g_ValueOfAD[3]
 *				（垂直左电感）		（垂直右电感）
 * @date		2018
 */

//预警：此函数完全被修改，上面的说明与实际不符
void DirectionControl(void)
{
   // Get_Car_Angle();                                       //将BMX055的值读入CarInfo中，以供处理器做半径闭环
    GetSensorData();
    g_fDirectionControlOut=Set_Curvature(CarInfo.ExpectCur);
}


/**
 * @file		电感信号采集和处理
 * @note      	
 * @date		2018
 */
void Read_ADC(void)
{
     int16  i,j,k,temp;
     int16  ad_valu[4][5],ad_valu1[4],ad_sum[4];
	 int16 ValueOfADOld[4],ValueOfADNew[4];

     for(i=0;i<5;i++)
     {
         ad_valu[0][i]=ad_ave(AD1, ADC_12bit, 5);  			// AD1水平左
         ad_valu[1][i]=ad_ave(AD2, ADC_12bit, 5);     		// AD2水平右
         ad_valu[2][i]=ad_ave(AD3, ADC_12bit, 5);  			// AD3垂直左
         ad_valu[3][i]=ad_ave(AD4, ADC_12bit, 5);     		// AD4垂直右		 
     }
/*=========================冒泡排序升序==========================*///舍弃最大值和最小值
    for(i=0;i<4;i++)
     {
        for(j=0;j<4;j++)
        {
           for(k=0;k<4-j;k++)
           {
              if(ad_valu[i][k] > ad_valu[i][k+1])        //前面的比后面的大  则进行交换
              {
                 temp = ad_valu[i][k+1];
                 ad_valu[i][k+1] = ad_valu[i][k];
                 ad_valu[i][k] = temp;
              }
           }
        }
     }
//*===========================中值滤波=================================
     for(i=0;i<4;i++)    //求中间三项的和
     {
        ad_sum[i] = ad_valu[i][1] + ad_valu[i][2] + ad_valu[i][3];
        ad_valu1[i] = ad_sum[i] / 3;
     }
      
	
	 for(i=0;i<4;i++)            //将数值中个位数除掉
	 {
	 	g_ValueOfAD[i] = (int16)(ad_valu1[i]/10*10);
                
		//采集梯度平滑，每次采集最大变化40
		ValueOfADOld[i] = g_ValueOfADFilter[i];
		ValueOfADNew[i] = g_ValueOfAD[i];
		
		if(ValueOfADNew[i]>=ValueOfADOld[i])
			g_ValueOfADFilter[i] = ((ValueOfADNew[i]-ValueOfADOld[i])>50?(ValueOfADOld[i]+50):ValueOfADNew[i]);
		else
			g_ValueOfADFilter[i] = ((ValueOfADNew[i]-ValueOfADOld[i])<-60?(ValueOfADOld[i]-60):ValueOfADNew[i]);
	 }
	 
}


