/**
 ******************************************************************************
 * @file    ins_task.h
 * @author  Wang Hongxi
 * @version V2.0.0
 * @date    2022/2/23
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef __INS_TASK_H
#define __INS_TASK_H

#include "stdint.h"
#include "BMI088driver.h"
#include "QuaternionEKF.h"

#define X 0
#define Y 1
#define Z 2

#define INS_TASK_PERIOD 1

typedef struct
{
    float q[4]; // Quaternion values 

    float Gyro[3];  // Angular velocity
    float Accel[3]; // Acceleration
    float MotionAccel_b[3]; // Linear acceleration in body frame
    float MotionAccel_n[3]; // Linear acceleration in navigation frame

    float AccelLPF; // Acceleration low-pass filter coefficient

    // Acceleration coordinate system unit vectors
    float xn[3];
    float yn[3];
    float zn[3];

    float atanxz;
    float atanyz;

    // Euler angles.
    float Roll;
    float Pitch;
    float Yaw;
    float YawTotalAngle;
		float YawAngleLast;
		float YawRoundCount;
		
		float v_n;// Linear velocity in navigation frame
		float x_n;// Linear position in navigation frame
		
		uint8_t ins_flag;
} INS_t;


/**
 * @brief Inertial navigation parameter structure, demo does not use
 * 
 */
typedef struct
{
    uint8_t flag;

    float scale[3];

    float Yaw;
    float Pitch;
    float Roll;
} IMU_Param_t;

extern INS_t INS;

extern void INS_Init(void);
extern void INS_task(void);

void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q);
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q);
float rad_to_deg(float rad);

#endif


