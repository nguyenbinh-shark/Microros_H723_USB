#include "mahony_filter.h"

/*************************************
Create time: 2023-09-02
Description: Mahony attitude filter implementation and wrapper
Author ID: Lvxiaohe (Zhihu/Bilibili)
***************************************/

struct MAHONY_FILTER_t mahony_filter;

/* Update rotation matrix R (body to world) from current quaternion */
void RotationMatrix_update(struct MAHONY_FILTER_t *mahony_filter)
{
    float q1q1 = mahony_filter->q1 * mahony_filter->q1;
    float q2q2 = mahony_filter->q2 * mahony_filter->q2;
    float q3q3 = mahony_filter->q3 * mahony_filter->q3;

    float q0q1 = mahony_filter->q0 * mahony_filter->q1;
    float q0q2 = mahony_filter->q0 * mahony_filter->q2;
    float q0q3 = mahony_filter->q0 * mahony_filter->q3;
    float q1q2 = mahony_filter->q1 * mahony_filter->q2;
    float q1q3 = mahony_filter->q1 * mahony_filter->q3;
    float q2q3 = mahony_filter->q2 * mahony_filter->q3;

    mahony_filter->rMat[0][0] = 1.0f - 2.0f * q2q2 - 2.0f * q3q3;
    mahony_filter->rMat[0][1] = 2.0f * (q1q2  -q0q3);
    mahony_filter->rMat[0][2] = 2.0f * (q1q3 +q0q2);

    mahony_filter->rMat[1][0] = 2.0f * (q1q2 +q0q3);
    mahony_filter->rMat[1][1] = 1.0f - 2.0f * q1q1 - 2.0f * q3q3;
    mahony_filter->rMat[1][2] = 2.0f * (q2q3  -q0q1);

    mahony_filter->rMat[2][0] = 2.0f * (q1q3  -q0q2);
    mahony_filter->rMat[2][1] = 2.0f * (q2q3  +q0q1);
    mahony_filter->rMat[2][2] = 1.0f - 2.0f * q1q1 - 2.0f * q2q2;
}

// Feed gyro and accelerometer measurements into filter state
void mahony_input(struct MAHONY_FILTER_t *mahony_filter,Axis3f gyro,Axis3f acc)
{
    mahony_filter->gyro = gyro;
    mahony_filter->acc = acc;
}

// Update filter state with Mahony algorithm
void mahony_update(struct MAHONY_FILTER_t *mahony_filter)
{
    float normalise;
    float ex,ey,ez;
    
    /* Gyro: assumed already in rad/s (conversion commented out) */
//    mahony_filter->gyro.x *= DEG2RAD;        /* normalize quaternion */
//    mahony_filter->gyro.y *= DEG2RAD;
//    mahony_filter->gyro.z *= DEG2RAD;;
       
    /* Normalize accelerometer measurement */
    normalise =sqrt(mahony_filter->acc.x * mahony_filter->acc.x 
										+mahony_filter->acc.y * mahony_filter->acc.y 
										+mahony_filter->acc.z * mahony_filter->acc.z);
	
    mahony_filter->acc.x =mahony_filter->acc.x/ normalise;
	  mahony_filter->acc.y =mahony_filter->acc.y/normalise;   
    mahony_filter->acc.z =mahony_filter->acc.z/normalise;

    /* Compute acceleration reference error using current rotation matrix */
	//vx=mahony_filter->rMat[2][0]=2.0f * (q1q3 + -q0q2)
	//vy=mahony_filter->rMat[2][1]=2.0f * (q2q3 - -q0q1)
	//vz=mahony_filter->rMat[2][2]=1.0f - 2.0f * q1q1 - 2.0f * q2q2
    ex = (mahony_filter->acc.y * mahony_filter->rMat[2][2] - mahony_filter->acc.z * mahony_filter->rMat[2][1]);
    ey = (mahony_filter->acc.z * mahony_filter->rMat[2][0] - mahony_filter->acc.x * mahony_filter->rMat[2][2]);
    ez = (mahony_filter->acc.x * mahony_filter->rMat[2][1] - mahony_filter->acc.y * mahony_filter->rMat[2][0]);
    
    /* Integral term */
    mahony_filter->exInt += mahony_filter->Ki * ex * mahony_filter->dt ;  
    mahony_filter->eyInt += mahony_filter->Ki * ey * mahony_filter->dt ;
    mahony_filter->ezInt += mahony_filter->Ki * ez * mahony_filter->dt ;
    
    /* PI feedback to correct gyro bias/attitude error */
    mahony_filter->gyro.x += mahony_filter->Kp * ex + mahony_filter->exInt;
    mahony_filter->gyro.y += mahony_filter->Kp * ey + mahony_filter->eyInt;
    mahony_filter->gyro.z += mahony_filter->Kp * ez + mahony_filter->ezInt;
    
    /* Integrate quaternion kinematics (first-order discrete) */
    float q0Last = mahony_filter->q0;
    float q1Last = mahony_filter->q1;
    float q2Last = mahony_filter->q2;
    float q3Last = mahony_filter->q3;
    float halfT = mahony_filter->dt * 0.5f;
    mahony_filter->q0 += (-q1Last * mahony_filter->gyro.x - q2Last * mahony_filter->gyro.y - q3Last * mahony_filter->gyro.z) * halfT;
    mahony_filter->q1 += ( q0Last * mahony_filter->gyro.x + q2Last * mahony_filter->gyro.z - q3Last * mahony_filter->gyro.y) * halfT;
    mahony_filter->q2 += ( q0Last * mahony_filter->gyro.y - q1Last * mahony_filter->gyro.z + q3Last * mahony_filter->gyro.x) * halfT;
    mahony_filter->q3 += ( q0Last * mahony_filter->gyro.z + q1Last * mahony_filter->gyro.y - q2Last * mahony_filter->gyro.x) * halfT;
    
    /* Normalize quaternion */
    normalise = sqrt(mahony_filter->q0 * mahony_filter->q0 
										+mahony_filter->q1 * mahony_filter->q1 
										+mahony_filter->q2 * mahony_filter->q2 
										+mahony_filter->q3 * mahony_filter->q3);
												
    mahony_filter->q0 = mahony_filter->q0/normalise;
    mahony_filter->q1 = mahony_filter->q1/normalise;
    mahony_filter->q2 = mahony_filter->q2/normalise;
    mahony_filter->q3 = mahony_filter->q3/normalise;
    
    /* Update rotation matrix */
    mahony_filter->RotationMatrix_update(mahony_filter);
}

// Compute Euler angles from rotation matrix
void mahony_output(struct MAHONY_FILTER_t *mahony_filter)
{
    /* Fill roll pitch yaw (Euler angles) */
    mahony_filter->pitch = -asinf(mahony_filter->rMat[2][0]); 
    mahony_filter->roll = atan2f(mahony_filter->rMat[2][1], mahony_filter->rMat[2][2]) ;
    mahony_filter->yaw = atan2f(mahony_filter->rMat[1][0], mahony_filter->rMat[0][0]) ;
}

// Initialize Mahony filter parameters and function pointers
void mahony_init(struct MAHONY_FILTER_t *mahony_filter,float Kp,float Ki,float dt)
{
    mahony_filter->Kp = Kp;
    mahony_filter->Ki = Ki;
    mahony_filter->dt = dt;
    mahony_filter->q0 = 1;
    mahony_filter->mahony_input = mahony_input;
    mahony_filter->mahony_update = mahony_update;
    mahony_filter->mahony_output = mahony_output;    
    mahony_filter->RotationMatrix_update = RotationMatrix_update;
}

