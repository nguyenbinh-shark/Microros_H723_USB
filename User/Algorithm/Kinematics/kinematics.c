#include "kinematics.h"
#include "robot_config.h"

/* ── Thuật toán điều khiển (User Algorithm) ───────────────────────── */
/* Hàm này nhận tín hiệu mong muốn (từ cmd_vel) và tính toán ra vận tốc mục tiêu 
 * (target velocity) cho từng động cơ. Bạn có thể sửa thuật toán trong này sau.
 */
void robot_control_algo(RobotCmd_t *cmd, float *target_left_vel, float *target_right_vel)
{
    /* 1. Tính toán động học (Kinematics) 
     * Robot (v [m/s], ω [rad/s]) -> vận tốc góc mỗi bánh (rad/s tại trục motor):
     *   v_left  = (v − ω·L/2) / R
     *   v_right = (v + ω·L/2) / R
     */
    float v_left_wheel  = (cmd->velocity - cmd->yaw_rate * WHEEL_BASE * 0.5f) / WHEEL_RADIUS;
    float v_right_wheel = (cmd->velocity + cmd->yaw_rate * WHEEL_BASE * 0.5f) / WHEEL_RADIUS;

    /* 2. Hiệu chỉnh chiều lắp đặt (Sign correction) 
     * Nhân sign lắp đặt để hai motor cùng đẩy tiến (do một bên bị lắp ngược).
     */
    *target_left_vel  = v_left_wheel  * MOTOR_LEFT_SIGN;
    *target_right_vel = v_right_wheel * MOTOR_RIGHT_SIGN;
}
