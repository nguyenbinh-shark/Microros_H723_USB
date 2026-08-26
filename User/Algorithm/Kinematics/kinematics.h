#ifndef __KINEMATICS_H
#define __KINEMATICS_H

#include <stdint.h>

/**
 * @brief Robot movement command structure
 */
typedef struct {
    float   velocity;   /* m/s   — linear forward/backward */
    float   yaw_rate;   /* rad/s — angular turn rate       */
    uint8_t enable;     /* 0 = disable, 1 = enable motors  */
} RobotCmd_t;

/**
 * @brief  Calculate target wheel speeds based on robot command
 * @param  cmd: Pointer to the input robot command (linear/angular velocities)
 * @param  target_left_vel: Pointer to output left wheel target speed (rad/s at motor shaft)
 * @param  target_right_vel: Pointer to output right wheel target speed (rad/s at motor shaft)
 */
void robot_control_algo(RobotCmd_t *cmd, float *target_left_vel, float *target_right_vel);

#endif /* __KINEMATICS_H */
