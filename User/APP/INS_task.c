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
	
#include "INS_task.h"
#include "QuaternionEKF.h"
#include "bsp_PWM.h"
#include "mahony_filter.h"
#include "can_bsp.h"
#include "bsp_dwt.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <string.h>

INS_t INS;

struct MAHONY_FILTER_t mahony;
Axis3f Gyro,Accel;
float gravity[3] = {0, 0, 9.81f};

/* Startup calibration only.  Keep the board still: a six-axis IMU cannot
 * distinguish an extremely slow constant yaw rotation from gyro bias.  Once
 * locked, the bias never adapts, so real slow rotations remain measurable. */
#define INS_WARMUP_MS          1000U
#define INS_CALIBRATION_MS     3000U
#define INS_CALIBRATION_MIN_SAMPLES 2000U
#define INS_STILL_GYRO_MAX     0.01f   /* rad/s, after configured driver offsets */
#define INS_STILL_GYRO_SPAN    0.01f   /* rad/s over the entire window */
#define INS_STILL_GYRO_STD     0.0025f /* rad/s */
#define INS_STILL_ACCEL_SPAN   0.25f   /* m/s^2 per axis */
#define INS_STILL_GRAVITY_ERR  0.4f    /* m/s^2 */

typedef struct
{
    uint32_t count;
    uint32_t first_tick_ms;
    float mean[3];
    float m2[3];
    float gyro_min[3], gyro_max[3];
    float accel_min[3], accel_max[3];
} INS_Calibration_t;

static INS_Calibration_t calibration;
static INS_Snapshot_t snapshot;
static float gyro_bias[3];
static uint8_t calibrated;
static uint8_t attitude_seeded;
static uint32_t sequence;

uint32_t INS_DWT_Count = 0;
float ins_dt = 0.0f;
float ins_time;
int stop_time;
float Pitch_deg;
float Roll_deg;
float Yaw_deg;

uint8_t INS_GetSnapshot(INS_Snapshot_t *out)
{
    if (out == NULL) return 0;
    taskENTER_CRITICAL();
    *out = snapshot;
    taskEXIT_CRITICAL();
    return out->valid;
}

static void INS_StoreSnapshot(uint32_t sample_tick_ms, uint8_t valid)
{
    INS_Snapshot_t next = {0};
    memcpy(next.q, INS.q, sizeof(next.q));
    memcpy(next.gyro, INS.Gyro, sizeof(next.gyro));
    memcpy(next.accel, INS.Accel, sizeof(next.accel));
    memcpy(next.gyro_bias, gyro_bias, sizeof(next.gyro_bias));
    next.sample_tick_ms = sample_tick_ms;
    next.sequence = ++sequence;
    next.calibrated = calibrated;
    next.valid = valid;
    taskENTER_CRITICAL();
    snapshot = next;
    taskEXIT_CRITICAL();
}

static void INS_ResetCalibrationWindow(void)
{
    memset(&calibration, 0, sizeof(calibration));
}

static void INS_SeedAttitudeFromAccel(void)
{
    float roll = atan2f(BMI088.Accel[1], BMI088.Accel[2]);
    float pitch = atan2f(-BMI088.Accel[0],
                        sqrtf(BMI088.Accel[1] * BMI088.Accel[1] +
                              BMI088.Accel[2] * BMI088.Accel[2]));
    float cr = cosf(roll * 0.5f), sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
    mahony_init(&mahony, 1.0f, 0.001f, ins_dt);
    mahony.q0 = cr * cp;
    mahony.q1 = sr * cp;
    mahony.q2 = cr * sp;
    mahony.q3 = -sr * sp;
    RotationMatrix_update(&mahony);
    memset(INS.MotionAccel_b, 0, sizeof(INS.MotionAccel_b));
    memset(INS.MotionAccel_n, 0, sizeof(INS.MotionAccel_n));
    attitude_seeded = 1;
}

static void INS_UpdateCalibration(uint32_t now_ms)
{
    if (calibrated) return;

    float accel_norm = sqrtf(BMI088.Accel[0] * BMI088.Accel[0] +
                             BMI088.Accel[1] * BMI088.Accel[1] +
                             BMI088.Accel[2] * BMI088.Accel[2]);
    if (fabsf(accel_norm - 9.81f) > INS_STILL_GRAVITY_ERR) {
        INS_ResetCalibrationWindow();
        return;
    }
    for (uint8_t i = 0; i < 3; ++i) {
        if (fabsf(BMI088.Gyro[i]) > INS_STILL_GYRO_MAX) {
            INS_ResetCalibrationWindow();
            return;
        }
    }

    if (calibration.count == 0U) {
        calibration.first_tick_ms = now_ms;
        for (uint8_t i = 0; i < 3; ++i) {
            calibration.gyro_min[i] = calibration.gyro_max[i] = BMI088.Gyro[i];
            calibration.accel_min[i] = calibration.accel_max[i] = BMI088.Accel[i];
        }
    }
    ++calibration.count;
    for (uint8_t i = 0; i < 3; ++i) {
        float g = BMI088.Gyro[i], a = BMI088.Accel[i];
        float delta = g - calibration.mean[i];
        calibration.mean[i] += delta / (float)calibration.count;
        calibration.m2[i] += delta * (g - calibration.mean[i]);
        if (g < calibration.gyro_min[i]) calibration.gyro_min[i] = g;
        if (g > calibration.gyro_max[i]) calibration.gyro_max[i] = g;
        if (a < calibration.accel_min[i]) calibration.accel_min[i] = a;
        if (a > calibration.accel_max[i]) calibration.accel_max[i] = a;
        if (calibration.gyro_max[i] - calibration.gyro_min[i] > INS_STILL_GYRO_SPAN ||
            calibration.accel_max[i] - calibration.accel_min[i] > INS_STILL_ACCEL_SPAN) {
            INS_ResetCalibrationWindow();
            return;
        }
    }

    if (now_ms - calibration.first_tick_ms < INS_CALIBRATION_MS ||
        calibration.count < INS_CALIBRATION_MIN_SAMPLES) return;
    for (uint8_t i = 0; i < 3; ++i) {
        float variance = calibration.m2[i] / (float)(calibration.count - 1U);
        if (variance > INS_STILL_GYRO_STD * INS_STILL_GYRO_STD) {
            INS_ResetCalibrationWindow();
            return;
        }
    }
    memcpy(gyro_bias, calibration.mean, sizeof(gyro_bias));
    calibrated = 1;
    /* Reanchor relative yaw and tilt at the completed stationary startup. */
    INS_SeedAttitudeFromAccel();
    printf("[INS] Stationary gyro calibration locked: %.6f %.6f %.6f rad/s\r\n",
           gyro_bias[0], gyro_bias[1], gyro_bias[2]);
}

static uint8_t INS_SensorSampleValid(void)
{
    for (uint8_t i = 0; i < 3; ++i) {
        if (!isfinite(BMI088.Gyro[i]) || !isfinite(BMI088.Accel[i]) ||
            fabsf(BMI088.Gyro[i]) > 34.8f || fabsf(BMI088.Accel[i]) > 58.8f)
            return 0;
    }
    float accel_norm2 = BMI088.Accel[0] * BMI088.Accel[0] +
                        BMI088.Accel[1] * BMI088.Accel[1] +
                        BMI088.Accel[2] * BMI088.Accel[2];
    return accel_norm2 > 0.01f;
}

void INS_Init(void)
{
    memset(&INS, 0, sizeof(INS));
    memset(&snapshot, 0, sizeof(snapshot));
    memset(gyro_bias, 0, sizeof(gyro_bias));
    INS_ResetCalibrationWindow();
    calibrated = 0;
    attitude_seeded = 0;
    sequence = 0;
    ins_time = 0.0f;
    mahony_init(&mahony, 1.0f, 0.001f, 0.001f);
    INS.q[0] = 1.0f;
    INS.AccelLPF = 0.0089f;
}

void INS_task(void)
{
	 printf("[INS] INS_task started (Mahony AHRS filter active).\r\n");
	 INS_Init();
	 DWT_GetDeltaT(&INS_DWT_Count);  /* Reset counter to "now" — avoids giant dt on first loop */

	 uint8_t first_data_logged = 0;
	 uint32_t startup_tick_ms = HAL_GetTick();
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

        BMI088_Read(&BMI088);
        uint32_t sample_tick_ms = HAL_GetTick();
        if (!isfinite(ins_dt) || ins_dt <= 0.0f || ins_dt > 0.01f ||
            !BMI088_ReadIsValid() || !INS_SensorSampleValid()) {
            if (!calibrated) INS_ResetCalibrationWindow();
            INS.ins_flag = 0;
            INS_StoreSnapshot(sample_tick_ms, 0);
            continue;
        }
        mahony.dt = ins_dt;
        if (!attitude_seeded) INS_SeedAttitudeFromAccel();
        if (sample_tick_ms - startup_tick_ms >= INS_WARMUP_MS)
            INS_UpdateCalibration(sample_tick_ms);

    INS.Accel[X] = BMI088.Accel[X];
    INS.Accel[Y] = BMI088.Accel[Y];
    INS.Accel[Z] = BMI088.Accel[Z];
	  Accel.x=BMI088.Accel[0];
	  Accel.y=BMI088.Accel[1];
		Accel.z=BMI088.Accel[2];
    INS.Gyro[X] = BMI088.Gyro[X] - gyro_bias[X];
    INS.Gyro[Y] = BMI088.Gyro[Y] - gyro_bias[Y];
    INS.Gyro[Z] = BMI088.Gyro[Z] - gyro_bias[Z];
    Gyro.x=INS.Gyro[X];
    Gyro.y=INS.Gyro[Y];
    Gyro.z=INS.Gyro[Z];

		/* Log first IMU data to verify sensor is alive */
		if (!first_data_logged) {
			printf("[INS] BMI088 Raw: Ax=%.3f Ay=%.3f Az=%.3f Gx=%.4f Gy=%.4f Gz=%.4f\r\n",
				BMI088.Accel[0], BMI088.Accel[1], BMI088.Accel[2],
				BMI088.Gyro[0], BMI088.Gyro[1], BMI088.Gyro[2]);
			first_data_logged = 1;
		}

		mahony_input(&mahony,Gyro,Accel);
		mahony_update(&mahony);
        if (!mahony.valid) {
            attitude_seeded = 0;
            INS.ins_flag = 0;
            if (!calibrated) INS_ResetCalibrationWindow();
            INS_StoreSnapshot(sample_tick_ms, 0);
            continue;
        }
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
 
		INS.ins_flag = calibrated;
		if(calibrated)
		{
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
		ins_time = (float)(sample_tick_ms - startup_tick_ms);
		INS_StoreSnapshot(sample_tick_ms, 1);
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



