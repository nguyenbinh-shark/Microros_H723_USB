#ifndef _MAHONY_FILTER_H
#define _MAHONY_FILTER_H

#include <math.h>
#include <stdlib.h>
#include "stm32h7xx.h"
#include "arm_math.h"

/*************************************
Create time: 2023-09-02
Description: Mahony attitude filter implementation and wrapper
Author ID: Lvxiaohe (Zhihu/Bilibili)
***************************************/

#define DEG2RAD 0.0174533f
#define RAD2DEG 57.295671f

typedef struct Axis3f_t
{
  float x;
  float y;
  float z;
}Axis3f;


// MAHONY_FILTER_t: packs Mahony filter data and function pointers
struct MAHONY_FILTER_t
{
    // Gains and inputs
    float Kp, Ki;          // proportional and integral gains
    float dt;              // sample period
    Axis3f  gyro, acc;     // gyro and accel measurements

    // Internal state
    float exInt, eyInt, ezInt;      // integrated error
    float q0, q1, q2, q3;           // quaternion
    float rMat[3][3];               // rotation matrix

    // Output
    float pitch, roll, yaw;         // Euler angles (rad)

    // function pointers
    void (*mahony_init)(struct MAHONY_FILTER_t *mahony_filter, float Kp, float Ki, float dt);
    void (*mahony_input)(struct MAHONY_FILTER_t *mahony_filter, Axis3f gyro, Axis3f acc);
    void (*mahony_update)(struct MAHONY_FILTER_t *mahony_filter);
    void (*mahony_output)(struct MAHONY_FILTER_t *mahony_filter);
    void (*RotationMatrix_update)(struct MAHONY_FILTER_t *mahony_filter);
};

// Functions
void mahony_init(struct MAHONY_FILTER_t *mahony_filter, float Kp, float Ki, float dt);          // initialize filter
void mahony_input(struct MAHONY_FILTER_t *mahony_filter, Axis3f gyro, Axis3f acc);              // feed measurements
void mahony_update(struct MAHONY_FILTER_t *mahony_filter);                                      // run filter update
void mahony_output(struct MAHONY_FILTER_t *mahony_filter);                                      // compute Euler output
void RotationMatrix_update(struct MAHONY_FILTER_t *mahony_filter);                              // update rotation matrix

#endif

