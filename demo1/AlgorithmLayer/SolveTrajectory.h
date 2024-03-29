#ifndef __SOLVETRAJECTORY_H__
#define __SOLVETRAJECTORY_H__

#ifndef PI
#define PI 3.1415926535f
#endif

#define GRAVITY 9.78

struct SolveTrajectory
{
    float current_v;      //当前弹速
    float _k;             //弹道系数
    float current_pitch;  //当前pitch
    float current_yaw;    //当前yaw
};


extern void GimbalControlInit(float pitch,float yaw, float v, float k);
extern float GimbalControlBulletModel(float x, float v, float angle);
extern float GimbalControlGetPitch(float x, float y, float v);

extern void GimbalControlTransform(float xw, float yw, float zw,
                                float vxw, float vyw, float vzw,
                                int timestamp_start, float *pitch, float *yaw);

#endif /*__SOLVETRAJECTORY_H__*/
