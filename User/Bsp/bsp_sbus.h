#ifndef __BSP_SBUS_H__
#define __BSP_SBUS_H__

#include <stdint.h>
#include <stdbool.h>
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SBUS_FRAME_LEN     25                     /* 1 SBUS frame = 25 bytes         */
#define SBUS_HEADER        0x0F                   /* SBUS Header byte                */
#define SBUS_FOOTER        0x00                   /* SBUS Footer byte                */
#define SBUS_RX_BUFSIZE    (SBUS_FRAME_LEN * 2)   /* DMA circular double buffer (50) */

#define SBUS_CHANNEL_MIN   364
#define SBUS_CHANNEL_MID   1024
#define SBUS_CHANNEL_MAX   1684
#define SBUS_CHANNEL_RANGE 660                    /* MID..MAX half-span             */
#define SBUS_DEADBAND      35                     /* Ignore |raw| below this        */

/* ── Stick → motion mapping ─────────────────────────────────────────────────
 * Channel order is not the same on every transmitter. Change the two indices
 * below if the sticks come out swapped (horizontal stick drives forward), and
 * flip the SIGN macros if an axis moves the right stick the wrong way.
 *   RC_CH_VX = channel of the VERTICAL stick axis  (forward / backward)
 *   RC_CH_WZ = channel of the HORIZONTAL stick axis (steering)
 */
#define RC_CH_VX           0                      /* CH1 = vertical / throttle      */
#define RC_CH_WZ           1                      /* CH2 = horizontal / steering     */
/* Both signs were flipped on 2026-10-17 together with MOTOR_LEFT/RIGHT_SIGN in
 * robot_config.h (the wheel frame now matches the URDF, LiDAR end = +x), so the
 * sticks keep driving RPBot exactly as before: stick-up toward the end without
 * the LiDAR, stick-left turns the robot left as seen driving that way. */
#define RC_VX_SIGN         (+1)                   /* radio sends stick-up low      */
#define RC_WZ_SIGN         (-1)                   /* radio sends stick-left low    */

/* ── Motor arm button ───────────────────────────────────────────────────────
 * A channel on the transmitter that enables / disables both wheels, mapped to
 * the same path as the /motor_enable topic.
 *   RC_ARM_TOGGLE 1 = momentary button: every press flips the armed state.
 *   RC_ARM_TOGGLE 0 = 2-position switch: HIGH = armed, LOW = disarmed.
 * The two thresholds leave a dead zone in the middle so a noisy channel cannot
 * chatter between the two states.
 */
#define RC_CH_ARM          5                      /* CH6                            */
#define RC_ARM_HIGH        1350                   /* above = pressed / switch high  */
#define RC_ARM_LOW         700                    /* below = released / switch low  */
#define RC_ARM_TOGGLE      1

typedef enum {
    RC_MODE_AUTO_ROS2 = 0,   /* Autonomous Nav2 / micro-ROS /cmd_vel mode */
    RC_MODE_MANUAL_RC = 1,   /* Manual Remote Controller teleop mode     */
    RC_MODE_EMERGENCY_STOP = 2 /* Emergency Stop / E-Stop                */
} RC_ControlMode_t;

typedef struct {
    uint16_t ch[16];         /* 16 Analog channels (0..2047, mid ~1024)   */
    uint8_t  ch17;           /* Digital Channel 17 (0 / 1)                */
    uint8_t  ch18;           /* Digital Channel 18 (0 / 1)                */
    uint8_t  frame_lost;     /* 1 = Frame lost flag                       */
    uint8_t  failsafe;       /* 1 = Failsafe active (transmitter off/out) */
    uint8_t  online;         /* 1 = Active signal receiving, 0 = Offline  */
    uint32_t last_rx_tick;   /* Timestamp of last valid frame             */
} sbus_t;

extern sbus_t sbus;
extern UART_HandleTypeDef huart5;

/**
 * @brief Initialize UART5 (PD2 = RX, DMA1_Stream4) for SBUS / DBUS receiver
 */
void sbus_bsp_init(void);

/**
 * @brief Check if SBUS receiver is online and receiving valid packets
 */
bool sbus_is_online(void);

/**
 * @brief Get current control mode from RC switch (e.g. Channel 5 / Switch A)
 */
RC_ControlMode_t sbus_get_control_mode(void);

/**
 * @brief Compute normalized linear velocity (m/s) and angular velocity (rad/s) from RC sticks
 * @param max_vx Max linear velocity in m/s (e.g. 1.0 m/s)
 * @param max_wz Max angular velocity in rad/s (e.g. 2.5 rad/s)
 * @param out_vx Pointer to output linear velocity (m/s)
 * @param out_wz Pointer to output angular velocity (rad/s)
 */
void sbus_get_motion_cmd(float max_vx, float max_wz, float *out_vx, float *out_wz);

/**
 * @brief Edge-process the arm button. Call once per control cycle, from ONE
 *        task only — each press must be consumed exactly once.
 */
void sbus_arm_update(void);

/**
 * @brief Latched armed state. Pure read, safe from any task.
 */
bool sbus_is_armed(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_SBUS_H__ */

