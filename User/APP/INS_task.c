/**
  *********************************************************************
  * @file      ins_task.c/h
	* @brief     Use Mahony filter to estimate attitude and linear motion acceleration.
  * @note       
  * @history
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  *********************************************************************
  */
	
#include "ins_task.h"
#include "QuaternionEKF.h"
#include "bsp_PWM.h"
#include "mahony_filter.h"
#include "can_bsp.h"
#include "bsp_dwt.h"
#include "cmsis_os.h"
#include <stdio.h>

INS_t INS;

struct MAHONY_FILTER_t mahony;
Axis3f Gyro,Accel;
float gravity[3] = {0, 0, 9.81f};

// Calibration offsets (in radians)
#define PITCH_OFFSET_DEG (-2.0f)  // Negative to compensate forward tilt
#define PITCH_OFFSET_RAD (PITCH_OFFSET_DEG * 0.0174533f)  // Convert to radians

uint32_t INS_DWT_Count = 0;
float ins_dt = 0.0f;
float ins_time;
int stop_time;
float Pitch_deg;
float Roll_deg;
float Yaw_deg;
void INS_Init(void)
{
	 mahony_init(&mahony, 1.0f, 0.001f, 0.001f);  /* Ki=0.001 to cancel residual gyro bias */
   INS.AccelLPF = 0.0089f;
}

void INS_task(void)
{
	 printf("[INS] INS_task started (Mahony AHRS filter active).\r\n");
	 INS_Init();
	 DWT_GetDeltaT(&INS_DWT_Count);  /* Reset counter to "now" — avoids giant dt on first loop */

	 uint8_t first_data_logged = 0;
	 uint32_t next_wake = osKernelGetTickCount();

	 while(1)
	 {
		/* Hold a true 1 kHz cadence; osDelay() drifted by the SPI read time.
		   osDelayUntil() returns immediately once the target tick has passed,
		   so after an overrun the deadline is re-anchored to now — otherwise
		   this task would spin at full speed to "catch up", and it runs at the
		   highest priority in the system. */
		next_wake += 1U;
		{
			uint32_t now_tick = osKernelGetTickCount();
			if ((int32_t)(next_wake - now_tick) <= 0) next_wake = now_tick + 1U;
		}
		osDelayUntil(next_wake);

		ins_dt = DWT_GetDeltaT(&INS_DWT_Count);
		if (ins_dt > 0.01f) ins_dt = 0.001f;  /* Cap: ignore any stale dt > 10ms */
    
		mahony.dt = ins_dt;

    BMI088_Read(&BMI088);

    INS.Accel[X] = BMI088.Accel[X];
    INS.Accel[Y] = BMI088.Accel[Y];
    INS.Accel[Z] = BMI088.Accel[Z];
	  Accel.x=BMI088.Accel[0];
	  Accel.y=BMI088.Accel[1];
		Accel.z=BMI088.Accel[2];
    INS.Gyro[X] = BMI088.Gyro[X];
    INS.Gyro[Y] = BMI088.Gyro[Y];
    INS.Gyro[Z] = BMI088.Gyro[Z];
  	Gyro.x=BMI088.Gyro[0];
		Gyro.y=BMI088.Gyro[1];
		Gyro.z=BMI088.Gyro[2];

		/* Log first IMU data to verify sensor is alive */
		if (!first_data_logged && ins_time > 10.0f) {
			printf("[INS] BMI088 Raw: Ax=%.3f Ay=%.3f Az=%.3f Gx=%.4f Gy=%.4f Gz=%.4f\r\n",
				BMI088.Accel[0], BMI088.Accel[1], BMI088.Accel[2],
				BMI088.Gyro[0], BMI088.Gyro[1], BMI088.Gyro[2]);
			first_data_logged = 1;
		}

		mahony_input(&mahony,Gyro,Accel);
		mahony_update(&mahony);
		mahony_output(&mahony);
	  RotationMatrix_update(&mahony);
				
		INS.q[0]=mahony.q0;
		INS.q[1]=mahony.q1;
		INS.q[2]=mahony.q2;
		INS.q[3]=mahony.q3;
       
		// Transform gravity from navigation frame (n) to body frame (b), then estimate motion acceleration.
		float gravity_b[3];
    EarthFrameToBodyFrame(gravity, gravity_b, INS.q);
    for (uint8_t i = 0; i < 3; i++) // Apply first-order low-pass filtering.
    {
      INS.MotionAccel_b[i] = (INS.Accel[i] - gravity_b[i]) * ins_dt / (INS.AccelLPF + ins_dt) 
														+ INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + ins_dt); 
//			INS.MotionAccel_b[i] = (INS.Accel[i] ) * dt / (INS.AccelLPF + dt) 
//														+ INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + dt);			
		}
		BodyFrameToEarthFrame(INS.MotionAccel_b, INS.MotionAccel_n, INS.q); // Convert back to navigation frame (n).
		
		// Suppress tiny acceleration noise.
		if(fabsf(INS.MotionAccel_n[0])<0.02f)
		{
		  INS.MotionAccel_n[0]=0.0f;	// x-axis
		}
		if(fabsf(INS.MotionAccel_n[1])<0.02f)
		{
		  INS.MotionAccel_n[1]=0.0f;	// y-axis
		}
		if(fabsf(INS.MotionAccel_n[2])<0.04f)
		{
		  INS.MotionAccel_n[2]=0.0f;// z-axis
		}
 
		if(ins_time>3000.0f)
		{
			if (!INS.ins_flag) {
				printf("[INS] Warmup complete! ins_flag=1, Mahony AHRS ready.\r\n");
				printf("[INS] Pitch=%.2f Roll=%.2f Yaw=%.2f deg\r\n",
					rad_to_deg(mahony.roll), rad_to_deg(mahony.pitch), rad_to_deg(mahony.yaw));
			}
			INS.ins_flag=1; // Quaternion and motion acceleration are ready; control can start.
			// Read attitude angles.
      INS.Pitch=mahony.roll ;//+ PITCH_OFFSET_RAD;  // Apply pitch calibration offset
		INS.Roll=mahony.pitch;
		INS.Yaw=mahony.yaw;
		Pitch_deg= rad_to_deg(INS.Pitch);
		Roll_deg= rad_to_deg(INS.Roll);
		Yaw_deg= rad_to_deg(INS.Yaw);
			
		//INS.YawTotalAngle=INS.YawTotalAngle+INS.Gyro[2]*0.001f;
			
			if (INS.Yaw - INS.YawAngleLast > 3.1415926f)
			{
					INS.YawRoundCount--;
			}
			else if (INS.Yaw - INS.YawAngleLast < -3.1415926f)
			{
					INS.YawRoundCount++;
			}
			INS.YawTotalAngle = 6.283f* INS.YawRoundCount + INS.Yaw;
			INS.YawAngleLast = INS.Yaw;
		}
		else
		{
		 ins_time++;
		}
	}
}
/**
 * @brief          Transform 3dvector from BodyFrame to EarthFrame
 * @param[1]       vector in BodyFrame
 * @param[2]       vector in EarthFrame
 * @param[3]       quaternion
 */
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q)
{
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                       (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                       (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
                       (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

/**
 * @brief          Transform 3dvector from EarthFrame to BodyFrame
 * @param[1]       vector in EarthFrame
 * @param[2]       vector in BodyFrame
 * @param[3]       quaternion
 */
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q)
{
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                       (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                       (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
                       (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}

/**
 * @brief          Convert radians to degrees
 * @param[1]       angle in radians
 * @retval         angle in degrees
 */
float rad_to_deg(float rad)
{
    return rad * 57.2957795f;  // 180/π
}



