/*********************************************************************************************************************
 * 
 * @file       		direction.c
 *  				�������
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
   
/**********ȫ�ֱ�������********/ 
//static uint32 ErrCount = 0;   //////����֡��
//static bool IsRead = false;  /////��ȡ״̬   
static bool  IsGyroOffsetReset = false;          /////�����Ҫ������������Ʈ�����򽫸ı�����Ϊ   1
                                                  /////��1�ķ�ʽ����ͨ�� �������߱�Ĳ���
CarInfotypedef    CarInfo;
ADTesttypedef          ADTest;
BMX055Datatypedef      BMX055_data;
PIDcontroltypedef      SpeedRadius={100,0,500};
PIDcontroltypedef      yawPIDcontrol={700,0,2200};
AttitudeDatatypedef    GyroOffset;           //
//float g_dirControl_P = 700;		//�������P
//float g_dirControl_D = 2200;	//�������D
//float g_fDirectionError[2];		//����ƫ�g_fDirectionError[0]Ϊһ��ˮƽ��еĲ�Ⱥ�ƫ�
								//		  ��g_fDirectionError[1]Ϊһ�Դ�ֱ��еĲ�Ⱥ�ƫ�
//float g_fDirectionError_dot[2];//����ƫ��΢�֣�g_fDirectionError_dot[0]Ϊһ��ˮƽ��еĲ�Ⱥ�ƫ��΢�֣�
								//			 ��g_fDirectionError_dot[1]Ϊһ�Դ�ֱ��еĲ�Ⱥ�ƫ��΢�֣�
float g_fDirectionControlOut;	//����������
int16 g_ValueOfAD[4]={0};	        //��ȡ�ĵ��ֵ
int16 g_ValueOfADFilter[4]={0};	//�����˲��ĵ��ֵ��δʹ�ã�
uint8 Flag_Round = OFF;			//���뻷���ı�־���ڻ�����ΪON��
extern float g_fRealSpeed;				//��ʵ�ٶ�
extern float g_fLeftRealSpeed;
extern float g_fRighRealSpeed;
float termp=8.88;

void ParameterInit(void)   ///������ʼ��
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

void GyroOffset_init(void)      /////////��������Ʈ��ʼ��
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




void Get_Car_Angle(void)   //��ȡ�����ǽǶ�
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
        BMX055_DataRead(&BMX055_data, 1);    //////ÿ30ms��ȡһ�δ�����
        MAGWaitCount = 0;
    }
    BMX055_data.GYROXdata = (BMX055_data.GYROXdata - Gyro.Xoffset) * 0.030517578;   
    BMX055_data.GYROYdata = (BMX055_data.GYROYdata - Gyro.Yoffset) * 0.030517578;
    BMX055_data.GYROZdata = (BMX055_data.GYROZdata - Gyro.Zoffset) * 0.030517578;
    ///////1000 / 32768     //////BMX055������Ʈ�������Ժ��Բ��ƣ����ǰ�ȫ������ǽ���һ��
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
        Quaternion_init();               //��Ԫ����ʼ����Ҫ,��ʱ��þ�ֹ
        CarInfo.Quaternion_init = 'T';
    }
    else
    {
      //Attitude_UpdateGyro();////�������ǽ���Ԫ��          ///
      //Attitude_UpdateAcc();////�ɼ��ٶȼƽ���Ԫ��              ///
  
        CarInfo.pitchrate = -EulerAngleRate.Roll/PI*180;   //�������ٶȵ�λ��
        CarInfo.yawrate = EulerAngleRate.Yaw / PI * 180;
        CarInfo.pitch = -EulerAngle.Roll/PI*180;            //�����ǵ�λ��
        CarInfo.yaw   = EulerAngle.Yaw/PI*180;
      
        AccZ = -Acc.Zdata;
        if(AccZ > 1)
            AccZ = 1;
        if(AccZ < -1)
            AccZ = -1;            /////////���ٶ���ʵֵ�޷�
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



static void GetSensorData(void)                              //������CarInfo�е�ExpectCur��RoadType��ʹ����StraightTest�������⣬�ı���ADTest
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
    if(fabs(CarInfo.ExpectCur) <= CarInfo.StraightTest)  ////������ֵ���������
    {
        StraightCount++;
        if(StraightCount >= CarInfo.StraightCount)   ////ֱ���ж�
            CarInfo.RoadType = 1;
    }
    else
    {
        StraightCount = 0;
        CarInfo.RoadType = 0;
    }
}



static float d = 0.125; // ���ҵ�������ĵ�еľ���
// ���ݸ߶ȡ������ҵ��ֵ���㵼�����λ��
float CalcDiff(float h, float L, float M, float R) //����ADTest �е�ֵ���㣬��󷵻�һ��folat��
{
    // �м������
    float LR_m = sqrtf(4 * L * R * d * d - (L - R) * (L - R) * h * h);
    float LM_m = sqrtf(L * M * d * d - (L - M) * (L - M) * h * h);
    float RM_m = sqrtf(R * M * d * d - (R - M) * (R - M) * h * h);
    // һ��3*2=6����
    float LR_solve[2];
    float LM_solve[2];
    float RM_solve[2];
    LR_solve[0] = ((L + R) * d - LR_m) / (R - L);
    LR_solve[1] = ((L + R) * d + LR_m) / (R - L);
    LM_solve[0] = (L * d - LM_m) / (M - L);
    LM_solve[1] = (L * d + LM_m) / (M  - L);
    RM_solve[0] = (R * d - RM_m) / (R - M);
    RM_solve[1] = (R * d + RM_m) / (R - M);
    // ѡȡ�������������С�Ľ���Ϊ����
    float LR_true, LM_true, RM_true;
    float LR_A = fabs(LM_solve[0] - LR_solve[0]) < fabs(LM_solve[1] - LR_solve[0]) ? LM_solve[0] : LM_solve[1]
               + fabs(RM_solve[0] - LR_solve[0]) < fabs(RM_solve[1] - LR_solve[0]) ? RM_solve[0] : RM_solve[1];
    float LR_B = fabs(LM_solve[0] - LR_solve[1]) < fabs(LM_solve[1] - LR_solve[1]) ? LM_solve[0] : LM_solve[1]
               + fabs(RM_solve[0] - LR_solve[1]) < fabs(RM_solve[1] - LR_solve[1]) ? RM_solve[0] : RM_solve[1];
    // ѡȡLR���������е�һ����Ϊ����
    if (LR_A < LR_B)
    {
        LR_true = LR_solve[0];
    }
    else
    {
        LR_true = LR_solve[1];
    }
    // ѡȡLM���������е�һ����Ϊ����
    if (fabs(LM_solve[0] - LR_true) < fabs(LM_solve[1] - LR_true))
    {
        LM_true = LM_solve[0];
    }
    else
    {
        LM_true = LM_solve[1];
    }
    // ѡȡRM���������е�һ����Ϊ����
    if (fabs(RM_solve[0] - LR_true) < fabs(RM_solve[1] - LR_true))
    {
        RM_true = RM_solve[0];
    }
    else
    {
        RM_true = RM_solve[1];
    }
    // ��������ȡ��ֵ
    float avg_solve = (LR_true + LM_true + RM_true) / 3;
    return avg_solve;
}


static float Set_Curvature(float Excurvature)/////�뾶�ջ�����ɣ�       ʹ�õ���CarInfo�е�speed_Mtrue��MaxAcc��MinR��RadiusAdjustSpeed��yawrate
{                                                                               //һϵ����������󣬻��OUTPwm
    static float   ExYawRate = 0;
    static float   OutPwm = 0;
    float SpeedMinRadius = 0;
    //float PathMinRadius = 0;
    float MinRadius = 0;
    float MaxCurvature = 0;
    float SpeedGainCur = 1;

    if(g_fRealSpeed > CarInfo.MinSpeed)                     //����ֵ�ٶ�
    {
        SpeedMinRadius = g_fRealSpeed * g_fRealSpeed * 0.01 / CarInfo.MaxAcc;//������ļ��ٶ�
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
    

    MaxCurvature = 1 / MinRadius;                  ///////��������޷�

    
    if(Excurvature > MaxCurvature)Excurvature = MaxCurvature;
    else if(Excurvature < -MaxCurvature)Excurvature = -MaxCurvature; /////���������޷�

    ExYawRate = Excurvature * CarInfo.speed_Mtrue * 180 / PI;  /////�������ٶ�

    float Yaw_Control_P = ExYawRate - CarInfo.yawrate;

    if(Yaw_Control_P > Yaw_Control_P_Max)
        Yaw_Control_P = Yaw_Control_P_Max;
    else if(Yaw_Control_P < -Yaw_Control_P_Max)
        Yaw_Control_P = -Yaw_Control_P_Max;///�����޷�

    float SetP, SetD;

    SetP = yawPIDcontrol.P;
    SetD = yawPIDcontrol.D;


    if(Yaw_Control_P * CarInfo.yawrate < 0)//////ת������ٶȲ�ͬ��΢������
        SetD = 0;

      
    OutPwm = (SetP * Yaw_Control_P + SetD * CarInfo.yawrate); 
    
    return (OutPwm);
    
}


/**
 * @file		�������
 *		һ������£�����ˮƽ��еĲ�Ⱥ���Ϊƫ��
 *		�ڻ���ʱ�У�������ֱ��еĲ�Ⱥ���Ϊƫ��
 *
 *				���ֵ��Ӧ����
 *
 *				g_ValueOfAD[0]		g_ValueOfAD[1]
 *				(ˮƽ����)		��ˮƽ�ҵ�У�
 *				g_ValueOfAD[2]		g_ValueOfAD[3]
 *				����ֱ���У�		����ֱ�ҵ�У�
 * @date		2018
 */

//Ԥ�����˺�����ȫ���޸ģ������˵����ʵ�ʲ���
void DirectionControl(void)
{
   // Get_Car_Angle();                                       //��BMX055��ֵ����CarInfo�У��Թ����������뾶�ջ�
    GetSensorData();
    g_fDirectionControlOut=Set_Curvature(CarInfo.ExpectCur);
}


/**
 * @file		����źŲɼ��ʹ���
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
         ad_valu[0][i]=ad_ave(AD1, ADC_12bit, 5);  			// AD1ˮƽ��
         ad_valu[1][i]=ad_ave(AD2, ADC_12bit, 5);     		// AD2ˮƽ��
         ad_valu[2][i]=ad_ave(AD3, ADC_12bit, 5);  			// AD3��ֱ��
         ad_valu[3][i]=ad_ave(AD4, ADC_12bit, 5);     		// AD4��ֱ��		 
     }
/*=========================ð����������==========================*///�������ֵ����Сֵ
    for(i=0;i<4;i++)
     {
        for(j=0;j<4;j++)
        {
           for(k=0;k<4-j;k++)
           {
              if(ad_valu[i][k] > ad_valu[i][k+1])        //ǰ��ıȺ���Ĵ�  ����н���
              {
                 temp = ad_valu[i][k+1];
                 ad_valu[i][k+1] = ad_valu[i][k];
                 ad_valu[i][k] = temp;
              }
           }
        }
     }
//*===========================��ֵ�˲�=================================
     for(i=0;i<4;i++)    //���м�����ĺ�
     {
        ad_sum[i] = ad_valu[i][1] + ad_valu[i][2] + ad_valu[i][3];
        ad_valu1[i] = ad_sum[i] / 3;
     }
      
	
	 for(i=0;i<4;i++)            //����ֵ�и�λ������
	 {
	 	g_ValueOfAD[i] = (int16)(ad_valu1[i]/10*10);
                
		//�ɼ��ݶ�ƽ����ÿ�βɼ����仯40
		ValueOfADOld[i] = g_ValueOfADFilter[i];
		ValueOfADNew[i] = g_ValueOfAD[i];
		
		if(ValueOfADNew[i]>=ValueOfADOld[i])
			g_ValueOfADFilter[i] = ((ValueOfADNew[i]-ValueOfADOld[i])>50?(ValueOfADOld[i]+50):ValueOfADNew[i]);
		else
			g_ValueOfADFilter[i] = ((ValueOfADNew[i]-ValueOfADOld[i])<-60?(ValueOfADOld[i]-60):ValueOfADNew[i]);
	 }
	 
}


