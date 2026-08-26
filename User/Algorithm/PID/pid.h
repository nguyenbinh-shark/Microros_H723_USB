/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       pid.c/h
  * @brief      PID implementation and API: init and compute routines
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. initial version
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#ifndef PID_H
#define PID_H
#include "main.h"
enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

typedef struct
{
    uint8_t mode;
    // PID gains
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;  // Output limit
    fp32 max_iout; // Integral output limit

    fp32 set;
    fp32 fdb;

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];  // Derivative buffer: 0 current, 1 last, 2 prev
    fp32 error[3]; // Error buffer: 0 current, 1 last, 2 prev

} PidTypeDef;
extern void PID_init(PidTypeDef *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout);
extern fp32 PID_Calc(PidTypeDef *pid, fp32 ref, fp32 set);
extern void PID_clear(PidTypeDef *pid);
#endif
