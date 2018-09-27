//#define  PI            3.1415926f
#define  PERIODHZ      (float)500       /////采样频率
#define  PERIODS        0.002           ////采样周期

typedef struct{
  float W;
  float X;
  float Y;
  float Z;
}QuaternionTypedef;

typedef struct{ 
  float Pitch;  //俯仰角
  float Yaw;    //偏航角
  float Roll;   //翻滚角
}EulerAngleTypedef;

/*
typedef struct{
  float Xdata;
  float Ydata;
  float Zdata;
}AttitudeDatatypedef;
*/
typedef struct{
  float Xdata;
  float Ydata;
  float Zdata;
  float Xoffset;
  float Yoffset;
  float Zoffset;
  uint8 GyrooffsetOK;
}AttitudeDatatypedef;


extern QuaternionTypedef    Quaternion;   //四元数
extern EulerAngleTypedef    EulerAngle;   //欧拉角
extern QuaternionTypedef    AxisAngle;    //轴角
extern EulerAngleTypedef    EulerAngleRate;//当前欧拉角速度

extern QuaternionTypedef    MeaQuaternion;
extern EulerAngleTypedef    MeaEulerAngle;
extern QuaternionTypedef    MeaAxisAngle;

extern QuaternionTypedef    ErrQuaternion;
extern EulerAngleTypedef    ErrEulerAngle;
extern QuaternionTypedef    ErrAxisAngle;
extern AttitudeDatatypedef         Acc;
extern AttitudeDatatypedef         Gyro;


extern void Quaternion_init();

extern void Attitude_UpdateGyro(void);

extern void Attitude_UpdateAcc(void);