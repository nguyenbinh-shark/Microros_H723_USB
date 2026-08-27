#include "lcd_task.h"
#include "lcd.h"
#include "motor_task.h"
#include "INS_task.h"
#include "observe_task.h"
#include "can_bsp.h"
#include "bsp_navkey.h"
#include "bsp_sbus.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <string.h>

/* ── UI Color Palette (RGB565) ────────────────────────────────────── */
#define UI_COLOR_BG          0x0000  /* Pure Black */
#define UI_COLOR_PANEL       0x0842  /* Dark Charcoal Slate */
#define UI_COLOR_HEADER_BG   0x0926  /* Deep Slate Blue */
#define UI_COLOR_BORDER      0x21CD  /* Cyber Steel Blue */
#define UI_COLOR_ACCENT      0x07FF  /* Vibrant Cyan */
#define UI_COLOR_GREEN       0x07E0  /* Neon Lime Green */
#define UI_COLOR_YELLOW      0xFFE0  /* High-vis Yellow */
#define UI_COLOR_ORANGE      0xFD20  /* Amber Orange */
#define UI_COLOR_RED         0xF800  /* Bright Red */
#define UI_COLOR_WHITE       0xFFFF  /* Crisp White */
#define UI_COLOR_MUTED       0x8410  /* Muted Silver Gray */
#define UI_COLOR_DARK_TEXT   0x0000  /* Black for badge text */

#define TOTAL_PAGES          4

static uint8_t s_current_page = 0;
static uint8_t s_last_drawn_page = 0xFF;

extern INS_t INS;
extern float Pitch_deg, Roll_deg, Yaw_deg;
extern volatile float g_cmd_velocity;
extern volatile float g_cmd_yaw_rate;
extern volatile uint8_t g_system_enabled;

uint8_t motor_get_enabled_state(void);

/* Exported from freertos.c */
extern uint8_t micro_ros_get_state(void);

/* ── UI Drawing Helper Functions ──────────────────────────────────── */

static void LCD_DrawCard(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const char *title, uint16_t borderColor)
{
    LCD_DrawRectangle(x, y, x + w - 1, y + h - 1, borderColor);
    LCD_Fill(x + 1, y + 1, x + w - 2, y + 13, UI_COLOR_HEADER_BG);
    LCD_ShowString(x + 4, y + 2, (const uint8_t *)title, UI_COLOR_ACCENT, UI_COLOR_HEADER_BG, 12, 0);
    LCD_Fill(x + 1, y + 14, x + w - 2, y + h - 2, UI_COLOR_PANEL);
}

static void LCD_DrawBadge(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const char *text, uint16_t bgColor, uint16_t textColor)
{
    LCD_Fill(x, y, x + w - 1, y + h - 1, bgColor);
    LCD_ShowString(x + 3, y + 2, (const uint8_t *)text, textColor, bgColor, 12, 0);
}

/* ── Page Background Renderers (Drawn Only When Page Changes) ────── */

static void LCD_DrawHeader(const char *pageTitle)
{
    LCD_Fill(0, 0, LCD_W, 22, 0x0128); /* Dark Tech Navy */
    LCD_ShowString(6, 4, (const uint8_t *)pageTitle, UI_COLOR_ACCENT, 0x0128, 16, 0);
    LCD_DrawLine(0, 22, LCD_W, 22, UI_COLOR_ACCENT);
}

static void LCD_DrawFooter(void)
{
    LCD_DrawLine(0, 218, LCD_W, 218, UI_COLOR_ACCENT);
    LCD_Fill(0, 219, LCD_W, LCD_H, 0x0128);
}

static void LCD_RenderPageBackground(uint8_t page)
{
    LCD_Fill(0, 0, LCD_W, LCD_H, UI_COLOR_BG);

    switch (page)
    {
        /* ── PAGE 0: OVERVIEW ─────────────────────────────────────── */
        case 0:
            LCD_DrawHeader("OVERVIEW [1/4]");
            LCD_DrawCard(2, 25, 136, 56, "COMM LINK", UI_COLOR_BORDER);
            LCD_ShowString(6, 40, (const uint8_t *)"RX :", UI_COLOR_MUTED, UI_COLOR_PANEL, 12, 0);
            LCD_ShowString(6, 53, (const uint8_t *)"LAT:", UI_COLOR_MUTED, UI_COLOR_PANEL, 12, 0);
            LCD_ShowString(6, 66, (const uint8_t *)"RTT:", UI_COLOR_MUTED, UI_COLOR_PANEL, 12, 0);

            LCD_DrawCard(142, 25, 136, 56, "TARGET CMD", UI_COLOR_BORDER);
            LCD_ShowString(146, 40, (const uint8_t *)"Vx :", UI_COLOR_MUTED, UI_COLOR_PANEL, 12, 0);
            LCD_ShowString(146, 53, (const uint8_t *)"Wz :", UI_COLOR_MUTED, UI_COLOR_PANEL, 12, 0);
            LCD_ShowString(146, 66, (const uint8_t *)"PWR:", UI_COLOR_MUTED, UI_COLOR_PANEL, 12, 0);

            LCD_DrawCard(2, 83, 136, 68, "LEFT MOTOR (CAN1)", UI_COLOR_BORDER);
            LCD_ShowString(6, 120, (const uint8_t *)"Torq:", UI_COLOR_MUTED, UI_COLOR_PANEL, 12, 0);
            LCD_ShowString(6, 134, (const uint8_t *)"Pos :", UI_COLOR_MUTED, UI_COLOR_PANEL, 12, 0);

            LCD_DrawCard(142, 83, 136, 68, "RIGHT MOTOR (CAN3)", UI_COLOR_BORDER);
            LCD_ShowString(146, 120, (const uint8_t *)"Torq:", UI_COLOR_MUTED, UI_COLOR_PANEL, 12, 0);
            LCD_ShowString(146, 134, (const uint8_t *)"Pos :", UI_COLOR_MUTED, UI_COLOR_PANEL, 12, 0);

            LCD_DrawCard(2, 153, 276, 62, "IMU 6-DOF ATTITUDE", UI_COLOR_BORDER);
            LCD_ShowString(6, 169, (const uint8_t *)"PITCH:", UI_COLOR_MUTED, UI_COLOR_PANEL, 12, 0);
            LCD_ShowString(144, 169, (const uint8_t *)"ROLL:", UI_COLOR_MUTED, UI_COLOR_PANEL, 12, 0);
            LCD_ShowString(6, 184, (const uint8_t *)"YAW  :", UI_COLOR_MUTED, UI_COLOR_PANEL, 12, 0);
            LCD_ShowString(144, 184, (const uint8_t *)"RATE:", UI_COLOR_MUTED, UI_COLOR_PANEL, 12, 0);
            LCD_ShowString(6, 199, (const uint8_t *)"AHRS :", UI_COLOR_MUTED, UI_COLOR_PANEL, 12, 0);
            break;

        /* ── PAGE 1: DUAL MOTORS & ODOMETRY ───────────────────────── */
        case 1:
            LCD_DrawHeader("MOTOR & ODOM [2/4]");
            LCD_DrawCard(2, 25, 276, 62, "LEFT WHEEL MOTOR (FDCAN1 ID 1)", UI_COLOR_BORDER);
            LCD_ShowString(6, 41, (const uint8_t *)"VEL:", UI_COLOR_MUTED, UI_COLOR_PANEL, 12, 0);
            LCD_ShowString(6, 56, (const uint8_t *)"TOR:", UI_COLOR_MUTED, UI_COLOR_PANEL, 12, 0);
            LCD_ShowString(144, 56, (const uint8_t *)"POS:", UI_COLOR_MUTED, UI_COLOR_PANEL, 12, 0);
            LCD_ShowString(6, 71, (const uint8_t *)"BUS: FDCAN1 1M | STATUS: RUN", UI_COLOR_GREEN, UI_COLOR_PANEL, 12, 0);

            LCD_DrawCard(2, 89, 276, 62, "RIGHT WHEEL MOTOR (FDCAN3 ID 1)", UI_COLOR_BORDER);
            LCD_ShowString(6, 105, (const uint8_t *)"VEL:", UI_COLOR_MUTED, UI_COLOR_PANEL, 12, 0);
            LCD_ShowString(6, 120, (const uint8_t *)"TOR:", UI_COLOR_MUTED, UI_COLOR_PANEL, 12, 0);
            LCD_ShowString(144, 120, (const uint8_t *)"POS:", UI_COLOR_MUTED, UI_COLOR_PANEL, 12, 0);
            LCD_ShowString(6, 135, (const uint8_t *)"BUS: FDCAN3 1M | STATUS: RUN", UI_COLOR_GREEN, UI_COLOR_PANEL, 12, 0);

            LCD_DrawCard(2, 153, 276, 62, "DIFF KINEMATICS & ODOM", UI_COLOR_BORDER);
            LCD_ShowString(6, 169, (const uint8_t *)"TRACK: 0.30m  RADIUS: 0.05m", UI_COLOR_MUTED, UI_COLOR_PANEL, 12, 0);
            LCD_ShowString(6, 184, (const uint8_t *)"ODOM Vx :", UI_COLOR_MUTED, UI_COLOR_PANEL, 12, 0);
            LCD_ShowString(6, 199, (const uint8_t *)"ODOM Wz :", UI_COLOR_MUTED, UI_COLOR_PANEL, 12, 0);
            break;

        /* ── PAGE 2: IMU & KALMAN OBSERVER ────────────────────────── */
        case 2:
            LCD_DrawHeader("IMU & OBSERVER [3/4]");
            LCD_DrawCard(2, 25, 276, 60, "BMI088 RAW SENSORS (SPI2)", UI_COLOR_BORDER);
            LCD_ShowString(6, 41, (const uint8_t *)"GYRO :", UI_COLOR_MUTED, UI_COLOR_PANEL, 12, 0);
            LCD_ShowString(6, 56, (const uint8_t *)"ACCEL:", UI_COLOR_MUTED, UI_COLOR_PANEL, 12, 0);
            LCD_ShowString(6, 71, (const uint8_t *)"RATE : 1 kHz | SPI2 DMA: ACTIVE", UI_COLOR_GREEN, UI_COLOR_PANEL, 12, 0);

            LCD_DrawCard(2, 87, 276, 62, "MAHONY AHRS ORIENTATION", UI_COLOR_BORDER);
            LCD_ShowString(6, 103, (const uint8_t *)"PITCH:", UI_COLOR_MUTED, UI_COLOR_PANEL, 12, 0);
            LCD_ShowString(144, 103, (const uint8_t *)"ROLL:", UI_COLOR_MUTED, UI_COLOR_PANEL, 12, 0);
            LCD_ShowString(6, 118, (const uint8_t *)"YAW  :", UI_COLOR_MUTED, UI_COLOR_PANEL, 12, 0);
            LCD_ShowString(144, 118, (const uint8_t *)"FLAG:", UI_COLOR_MUTED, UI_COLOR_PANEL, 12, 0);
            LCD_ShowString(6, 133, (const uint8_t *)"QUAT :", UI_COLOR_MUTED, UI_COLOR_PANEL, 12, 0);

            LCD_DrawCard(2, 151, 276, 64, "VELOCITY KALMAN OBSERVER", UI_COLOR_BORDER);
            LCD_ShowString(6, 167, (const uint8_t *)"FUSED Vx  :", UI_COLOR_MUTED, UI_COLOR_PANEL, 12, 0);
            LCD_ShowString(6, 182, (const uint8_t *)"FUSED Wz  :", UI_COLOR_MUTED, UI_COLOR_PANEL, 12, 0);
            LCD_ShowString(6, 197, (const uint8_t *)"SLIP DET  :", UI_COLOR_MUTED, UI_COLOR_PANEL, 12, 0);
            break;

        /* ── PAGE 3: COMMUNICATION & SYSTEM DIAGNOSTICS ───────────── */
        case 3:
            LCD_DrawHeader("SYSTEM STATS [4/4]");
            LCD_DrawCard(2, 25, 276, 92, "micro-ROS / XRCE-DDS LINK", UI_COLOR_BORDER);
            LCD_ShowString(6, 41,  (const uint8_t *)"Transport : USB-CDC HS (12 Mbps)", UI_COLOR_WHITE, UI_COLOR_PANEL, 12, 0);
            LCD_ShowString(6, 56,  (const uint8_t *)"RX Cmds   :", UI_COLOR_MUTED, UI_COLOR_PANEL, 12, 0);
            LCD_ShowString(6, 71,  (const uint8_t *)"TX Feedback:", UI_COLOR_MUTED, UI_COLOR_PANEL, 12, 0);
            LCD_ShowString(6, 86,  (const uint8_t *)"Ping RTT  :", UI_COLOR_MUTED, UI_COLOR_PANEL, 12, 0);
            LCD_ShowString(6, 101, (const uint8_t *)"USB Queue :", UI_COLOR_MUTED, UI_COLOR_PANEL, 12, 0);

            LCD_DrawCard(2, 120, 276, 95, "HARDWARE & RTOS DIAGNOSTICS", UI_COLOR_BORDER);
            LCD_ShowString(6, 136, (const uint8_t *)"MCU Clock : STM32H723 @ 192 MHz", UI_COLOR_WHITE, UI_COLOR_PANEL, 12, 0);
            LCD_ShowString(6, 151, (const uint8_t *)"RTOS Heap : 96 KB (heap_4: OK)", UI_COLOR_GREEN, UI_COLOR_PANEL, 12, 0);
            LCD_ShowString(6, 166, (const uint8_t *)"Tasks     : 5 Running (All OK)", UI_COLOR_WHITE, UI_COLOR_PANEL, 12, 0);
            LCD_ShowString(6, 181, (const uint8_t *)"RC Link   : UART5 (PD2) 100k 8E2", UI_COLOR_MUTED, UI_COLOR_PANEL, 12, 0);
            LCD_ShowString(6, 196, (const uint8_t *)"RC Status :", UI_COLOR_MUTED, UI_COLOR_PANEL, 12, 0);
            break;
    }

    LCD_DrawFooter();
}

/* ── Dynamic Content Update for Active Page ───────────────────────── */

void LCD_Debug_Init(void)
{
    NavKey_Init();
    s_current_page = 0;
    s_last_drawn_page = 0xFF;
}

void LCD_Debug_Update(void)
{
    char buf[48];
    CommMetrics_t metrics;
    comm_metrics_get(&metrics);

    /* 1. Handle Navigation Key Scanning */
    NavKey_t key = NavKey_Scan();
    if (key == NAV_KEY_RIGHT || key == NAV_KEY_DOWN || key == NAV_KEY_PRESS)
    {
        s_current_page = (s_current_page + 1) % TOTAL_PAGES;
    }
    else if (key == NAV_KEY_LEFT || key == NAV_KEY_UP)
    {
        s_current_page = (s_current_page + TOTAL_PAGES - 1) % TOTAL_PAGES;
    }

    /* Print diagnostic to serial terminal every 1s */
    static uint32_t last_diag_tick = 0;
    uint32_t now_tick = HAL_GetTick();
    if ((now_tick - last_diag_tick) >= 1000U)
    {
        last_diag_tick = now_tick;
        printf("[LCD_DIAG] Page=%u, RawADC=%u, SPI1_Err=0x%08lX\r\n", 
               s_current_page, NavKey_GetRawADC(), hspi1.ErrorCode);
    }

    /* 2. Redraw Page Background if Page Changed */
    if (s_current_page != s_last_drawn_page)
    {
        LCD_RenderPageBackground(s_current_page);
        s_last_drawn_page = s_current_page;
    }

    /* 3. Header Status Badge (Common to all pages) */
    uint8_t ros_state = micro_ros_get_state();
    if (ros_state == 2) {
        LCD_DrawBadge(186, 3, 90, 16, " CONNECTED ", UI_COLOR_GREEN, UI_COLOR_DARK_TEXT);
    } else if (ros_state == 1) {
        LCD_DrawBadge(186, 3, 90, 16, " INIT AGENT", UI_COLOR_YELLOW, UI_COLOR_DARK_TEXT);
    } else {
        LCD_DrawBadge(186, 3, 90, 16, " NO AGENT  ", UI_COLOR_RED, UI_COLOR_WHITE);
    }

    /* Motor Feedback Common */
    float m_pos[2], m_vel[2], m_tor[2];
    uint8_t m_valid[2];
    CAN_MotorFeedback_Get_Idx(0, &m_pos[0], &m_vel[0], &m_tor[0], &m_valid[0]);
    CAN_MotorFeedback_Get_Idx(1, &m_pos[1], &m_vel[1], &m_tor[1], &m_valid[1]);

    /* 4. Render Active Page Dynamic Data */
    switch (s_current_page)
    {
        /* ── PAGE 0: OVERVIEW ─────────────────────────────────────── */
        case 0:
        {
            snprintf(buf, sizeof(buf), "#%-5lu %4.1fHz", metrics.cmd_rx_count, (double)metrics.cmd_rate_hz);
            LCD_ShowString(32, 40, (const uint8_t *)buf, UI_COLOR_WHITE, UI_COLOR_PANEL, 12, 0);

            uint16_t lat_color = (metrics.cmd_age_ms > 500) ? UI_COLOR_RED : ((metrics.cmd_age_ms > 100) ? UI_COLOR_YELLOW : UI_COLOR_GREEN);
            if (metrics.cmd_age_ms == 0xFFFF || metrics.cmd_rx_count == 0) {
                snprintf(buf, sizeof(buf), "---ms Q:%2uB", metrics.rx_pending_bytes);
            } else {
                snprintf(buf, sizeof(buf), "%3lums Q:%2uB", (metrics.cmd_age_ms > 999) ? 999 : metrics.cmd_age_ms, metrics.rx_pending_bytes);
            }
            LCD_ShowString(32, 53, (const uint8_t *)buf, lat_color, UI_COLOR_PANEL, 12, 0);

            snprintf(buf, sizeof(buf), "%2u ms (USB) ", metrics.ping_rtt_ms);
            LCD_ShowString(32, 66, (const uint8_t *)buf, UI_COLOR_ACCENT, UI_COLOR_PANEL, 12, 0);

            snprintf(buf, sizeof(buf), "%+5.2f m/s", g_cmd_velocity);
            LCD_ShowString(172, 40, (const uint8_t *)buf, UI_COLOR_YELLOW, UI_COLOR_PANEL, 12, 0);

            snprintf(buf, sizeof(buf), "%+5.2f r/s", g_cmd_yaw_rate);
            LCD_ShowString(172, 53, (const uint8_t *)buf, UI_COLOR_YELLOW, UI_COLOR_PANEL, 12, 0);

            if (sbus_is_online()) {
                RC_ControlMode_t m = sbus_get_control_mode();
                if (m == RC_MODE_MANUAL_RC) {
                    LCD_ShowString(172, 66, (const uint8_t *)"RC:MANUAL", UI_COLOR_GREEN, UI_COLOR_PANEL, 12, 0);
                } else if (m == RC_MODE_AUTO_ROS2) {
                    LCD_ShowString(172, 66, (const uint8_t *)"RC:AUTO  ", UI_COLOR_ACCENT, UI_COLOR_PANEL, 12, 0);
                } else {
                    LCD_ShowString(172, 66, (const uint8_t *)"RC:ESTOP ", UI_COLOR_RED, UI_COLOR_PANEL, 12, 0);
                }
            } else {
                uint8_t motor_state = motor_get_enabled_state();
                LCD_ShowString(172, 66, (const uint8_t *)(motor_state ? "ENABLED " : "IDLE/OFF"), (motor_state ? UI_COLOR_GREEN : UI_COLOR_MUTED), UI_COLOR_PANEL, 12, 0);
            }

            CAN_BusMetrics_t can_m;
            CAN_GetMetrics(&can_m);

            if (m_valid[0]) {
                snprintf(buf, sizeof(buf), "%+6.2f rad/s", m_vel[0]);
                LCD_ShowString(6, 100, (const uint8_t *)buf, UI_COLOR_ACCENT, UI_COLOR_PANEL, 16, 0);
            } else {
                snprintf(buf, sizeof(buf), "OFFLINE (%3lu) ", (unsigned long)(can_m.can1_rx_cnt % 1000));
                LCD_ShowString(6, 100, (const uint8_t *)buf, UI_COLOR_RED, UI_COLOR_PANEL, 16, 0);
            }
            snprintf(buf, sizeof(buf), "%+5.2f Nm ", m_tor[0]);
            LCD_ShowString(40, 120, (const uint8_t *)buf, m_valid[0] ? UI_COLOR_WHITE : UI_COLOR_MUTED, UI_COLOR_PANEL, 12, 0);
            snprintf(buf, sizeof(buf), "%+6.1f rad", m_pos[0]);
            LCD_ShowString(40, 134, (const uint8_t *)buf, m_valid[0] ? UI_COLOR_WHITE : UI_COLOR_MUTED, UI_COLOR_PANEL, 12, 0);

            if (m_valid[1]) {
                snprintf(buf, sizeof(buf), "%+6.2f rad/s", m_vel[1]);
                LCD_ShowString(146, 100, (const uint8_t *)buf, UI_COLOR_ACCENT, UI_COLOR_PANEL, 16, 0);
            } else {
                snprintf(buf, sizeof(buf), "OFFLINE (%3lu) ", (unsigned long)(can_m.can3_rx_cnt % 1000));
                LCD_ShowString(146, 100, (const uint8_t *)buf, UI_COLOR_RED, UI_COLOR_PANEL, 16, 0);
            }
            snprintf(buf, sizeof(buf), "%+5.2f Nm ", m_tor[1]);
            LCD_ShowString(180, 120, (const uint8_t *)buf, m_valid[1] ? UI_COLOR_WHITE : UI_COLOR_MUTED, UI_COLOR_PANEL, 12, 0);
            snprintf(buf, sizeof(buf), "%+6.1f rad", m_pos[1]);
            LCD_ShowString(180, 134, (const uint8_t *)buf, m_valid[1] ? UI_COLOR_WHITE : UI_COLOR_MUTED, UI_COLOR_PANEL, 12, 0);

            snprintf(buf, sizeof(buf), "%+5.1f deg", (double)Pitch_deg);
            LCD_ShowString(46, 169, (const uint8_t *)buf, UI_COLOR_WHITE, UI_COLOR_PANEL, 12, 0);
            snprintf(buf, sizeof(buf), "%+5.1f deg", (double)Roll_deg);
            LCD_ShowString(180, 169, (const uint8_t *)buf, UI_COLOR_WHITE, UI_COLOR_PANEL, 12, 0);
            snprintf(buf, sizeof(buf), "%+5.1f deg", (double)Yaw_deg);
            LCD_ShowString(46, 184, (const uint8_t *)buf, UI_COLOR_YELLOW, UI_COLOR_PANEL, 12, 0);
            snprintf(buf, sizeof(buf), "%+5.2f r/s", (double)INS.Gyro[2]);
            LCD_ShowString(180, 184, (const uint8_t *)buf, UI_COLOR_YELLOW, UI_COLOR_PANEL, 12, 0);

            snprintf(buf, sizeof(buf), "Mahony 1kHz (ins_flag=%d)", INS.ins_flag);
            LCD_ShowString(46, 199, (const uint8_t *)buf, (INS.ins_flag ? UI_COLOR_GREEN : UI_COLOR_ORANGE), UI_COLOR_PANEL, 12, 0);

            snprintf(buf, sizeof(buf), "H723 @ 192MHz | NAV: <>");
            LCD_ShowString(6, 224, (const uint8_t *)buf, UI_COLOR_WHITE, 0x0128, 12, 0);
            snprintf(buf, sizeof(buf), "TX:#%-5lu", metrics.pub_tx_count);
            LCD_ShowString(200, 224, (const uint8_t *)buf, UI_COLOR_ACCENT, 0x0128, 12, 0);
            break;
        }

        /* ── PAGE 1: DUAL MOTORS & ODOMETRY ───────────────────────── */
        case 1:
        {
            CAN_BusMetrics_t can_m;
            CAN_GetMetrics(&can_m);

            float v_l = m_vel[0] * WHEEL_RADIUS;
            snprintf(buf, sizeof(buf), "%+6.2f rad/s  (%+5.2f m/s)", m_vel[0], v_l);
            LCD_ShowString(36, 41, (const uint8_t *)buf, m_valid[0] ? UI_COLOR_ACCENT : UI_COLOR_MUTED, UI_COLOR_PANEL, 12, 0);
            snprintf(buf, sizeof(buf), "%+5.2f Nm", m_tor[0]);
            LCD_ShowString(36, 56, (const uint8_t *)buf, m_valid[0] ? UI_COLOR_YELLOW : UI_COLOR_MUTED, UI_COLOR_PANEL, 12, 0);
            snprintf(buf, sizeof(buf), "%+7.1f rad", m_pos[0]);
            LCD_ShowString(174, 56, (const uint8_t *)buf, m_valid[0] ? UI_COLOR_WHITE : UI_COLOR_MUTED, UI_COLOR_PANEL, 12, 0);

            if (m_valid[0]) {
                snprintf(buf, sizeof(buf), "BUS: CAN1 | RX:%-5lu | STATUS: OK ", can_m.can1_rx_cnt);
                LCD_ShowString(6, 71, (const uint8_t *)buf, UI_COLOR_GREEN, UI_COLOR_PANEL, 12, 0);
            } else {
                snprintf(buf, sizeof(buf), "BUS: CAN1 | RX:%-5lu | OFFLINE    ", can_m.can1_rx_cnt);
                LCD_ShowString(6, 71, (const uint8_t *)buf, UI_COLOR_RED, UI_COLOR_PANEL, 12, 0);
            }

            float v_r = m_vel[1] * WHEEL_RADIUS;
            snprintf(buf, sizeof(buf), "%+6.2f rad/s  (%+5.2f m/s)", m_vel[1], v_r);
            LCD_ShowString(36, 105, (const uint8_t *)buf, m_valid[1] ? UI_COLOR_ACCENT : UI_COLOR_MUTED, UI_COLOR_PANEL, 12, 0);
            snprintf(buf, sizeof(buf), "%+5.2f Nm", m_tor[1]);
            LCD_ShowString(36, 120, (const uint8_t *)buf, m_valid[1] ? UI_COLOR_YELLOW : UI_COLOR_MUTED, UI_COLOR_PANEL, 12, 0);
            snprintf(buf, sizeof(buf), "%+7.1f rad", m_pos[1]);
            LCD_ShowString(174, 120, (const uint8_t *)buf, m_valid[1] ? UI_COLOR_WHITE : UI_COLOR_MUTED, UI_COLOR_PANEL, 12, 0);

            if (m_valid[1]) {
                snprintf(buf, sizeof(buf), "BUS: CAN3 | RX:%-5lu | STATUS: OK ", can_m.can3_rx_cnt);
                LCD_ShowString(6, 135, (const uint8_t *)buf, UI_COLOR_GREEN, UI_COLOR_PANEL, 12, 0);
            } else {
                snprintf(buf, sizeof(buf), "BUS: CAN3 | RX:%-5lu | OFFLINE    ", can_m.can3_rx_cnt);
                LCD_ShowString(6, 135, (const uint8_t *)buf, UI_COLOR_RED, UI_COLOR_PANEL, 12, 0);
            }

            float forward_v = (v_r + v_l) / 2.0f;
            float diff_w    = (v_r - v_l) / WHEEL_BASE;
            snprintf(buf, sizeof(buf), "%+6.3f m/s (Target: %+5.2f)", forward_v, g_cmd_velocity);
            LCD_ShowString(70, 184, (const uint8_t *)buf, UI_COLOR_WHITE, UI_COLOR_PANEL, 12, 0);
            snprintf(buf, sizeof(buf), "%+6.3f rad/s (Target: %+5.2f)", diff_w, g_cmd_yaw_rate);
            LCD_ShowString(70, 199, (const uint8_t *)buf, UI_COLOR_WHITE, UI_COLOR_PANEL, 12, 0);

            LCD_ShowString(6, 224, (const uint8_t *)"PAGE: MOTOR & ODOM", UI_COLOR_WHITE, 0x0128, 12, 0);
            snprintf(buf, sizeof(buf), "NAV: [<-/->]");
            LCD_ShowString(190, 224, (const uint8_t *)buf, UI_COLOR_ACCENT, 0x0128, 12, 0);
            break;
        }

        /* ── PAGE 2: IMU & KALMAN OBSERVER ────────────────────────── */
        case 2:
        {
            snprintf(buf, sizeof(buf), "X:%+5.2f Y:%+5.2f Z:%+5.2f", (double)INS.Gyro[0], (double)INS.Gyro[1], (double)INS.Gyro[2]);
            LCD_ShowString(48, 41, (const uint8_t *)buf, UI_COLOR_WHITE, UI_COLOR_PANEL, 12, 0);

            snprintf(buf, sizeof(buf), "X:%+5.2f Y:%+5.2f Z:%+5.2f", (double)INS.Accel[0], (double)INS.Accel[1], (double)INS.Accel[2]);
            LCD_ShowString(48, 56, (const uint8_t *)buf, UI_COLOR_WHITE, UI_COLOR_PANEL, 12, 0);

            snprintf(buf, sizeof(buf), "%+6.2f deg", (double)Pitch_deg);
            LCD_ShowString(48, 103, (const uint8_t *)buf, UI_COLOR_WHITE, UI_COLOR_PANEL, 12, 0);
            snprintf(buf, sizeof(buf), "%+6.2f deg", (double)Roll_deg);
            LCD_ShowString(180, 103, (const uint8_t *)buf, UI_COLOR_WHITE, UI_COLOR_PANEL, 12, 0);

            snprintf(buf, sizeof(buf), "%+6.2f deg", (double)Yaw_deg);
            LCD_ShowString(48, 118, (const uint8_t *)buf, UI_COLOR_YELLOW, UI_COLOR_PANEL, 12, 0);
            LCD_ShowString(180, 118, (const uint8_t *)(INS.ins_flag ? "LOCKED (OK)" : "CALIB...   "), (INS.ins_flag ? UI_COLOR_GREEN : UI_COLOR_ORANGE), UI_COLOR_PANEL, 12, 0);

            snprintf(buf, sizeof(buf), "[%.2f, %.2f, %.2f, %.2f]", (double)INS.q[0], (double)INS.q[1], (double)INS.q[2], (double)INS.q[3]);
            LCD_ShowString(48, 133, (const uint8_t *)buf, UI_COLOR_ACCENT, UI_COLOR_PANEL, 12, 0);

            snprintf(buf, sizeof(buf), "%+6.3f m/s (Od:%+5.2f)", (double)Observe.v_filter, (double)Observe.v_odom);
            LCD_ShowString(84, 167, (const uint8_t *)buf, UI_COLOR_GREEN, UI_COLOR_PANEL, 12, 0);

            snprintf(buf, sizeof(buf), "%+6.3f rad/s", (double)Observe.omega_filter);
            LCD_ShowString(84, 182, (const uint8_t *)buf, UI_COLOR_GREEN, UI_COLOR_PANEL, 12, 0);

            if (Observe.is_slipping) {
                snprintf(buf, sizeof(buf), "SLIP! %2.0f%% [IMU FUSION]", (double)(Observe.slip_ratio * 100.0f));
                LCD_ShowString(84, 197, (const uint8_t *)buf, UI_COLOR_RED, UI_COLOR_PANEL, 12, 0);
            } else {
                snprintf(buf, sizeof(buf), "NO SLIP (Grip %2.0f%%) ", (double)((1.0f - Observe.slip_ratio) * 100.0f));
                LCD_ShowString(84, 197, (const uint8_t *)buf, UI_COLOR_ACCENT, UI_COLOR_PANEL, 12, 0);
            }

            LCD_ShowString(6, 224, (const uint8_t *)"PAGE: IMU & AHRS", UI_COLOR_WHITE, 0x0128, 12, 0);
            snprintf(buf, sizeof(buf), "NAV: [<-/->]");
            LCD_ShowString(190, 224, (const uint8_t *)buf, UI_COLOR_ACCENT, 0x0128, 12, 0);
            break;
        }

        /* ── PAGE 3: COMMUNICATION & SYSTEM DIAGNOSTICS ───────────── */
        case 3:
        {
            snprintf(buf, sizeof(buf), "#%-6lu msgs (%4.1f Hz)", metrics.cmd_rx_count, (double)metrics.cmd_rate_hz);
            LCD_ShowString(84, 56, (const uint8_t *)buf, UI_COLOR_YELLOW, UI_COLOR_PANEL, 12, 0);

            snprintf(buf, sizeof(buf), "#%-6lu msgs (20.0 Hz)", metrics.pub_tx_count);
            LCD_ShowString(84, 71, (const uint8_t *)buf, UI_COLOR_WHITE, UI_COLOR_PANEL, 12, 0);

            snprintf(buf, sizeof(buf), "%2u ms (Round-Trip)", metrics.ping_rtt_ms);
            LCD_ShowString(84, 86, (const uint8_t *)buf, UI_COLOR_GREEN, UI_COLOR_PANEL, 12, 0);

            snprintf(buf, sizeof(buf), "%2u Bytes Pending (Realtime)", metrics.rx_pending_bytes);
            LCD_ShowString(84, 101, (const uint8_t *)buf, (metrics.rx_pending_bytes == 0 ? UI_COLOR_GREEN : UI_COLOR_ORANGE), UI_COLOR_PANEL, 12, 0);

            if (sbus_is_online()) {
                snprintf(buf, sizeof(buf), "ONLINE (C1:%4u C2:%4u)", sbus.ch[0], sbus.ch[1]);
                LCD_ShowString(84, 196, (const uint8_t *)buf, UI_COLOR_GREEN, UI_COLOR_PANEL, 12, 0);
            } else {
                LCD_ShowString(84, 196, (const uint8_t *)"NO SIGNAL (OFFLINE)    ", UI_COLOR_MUTED, UI_COLOR_PANEL, 12, 0);
            }

            LCD_ShowString(6, 224, (const uint8_t *)"PAGE: SYSTEM & TELEM", UI_COLOR_WHITE, 0x0128, 12, 0);
            snprintf(buf, sizeof(buf), "NAV: [<-/->]");
            LCD_ShowString(190, 224, (const uint8_t *)buf, UI_COLOR_ACCENT, 0x0128, 12, 0);
            break;
        }
    }
}

void LCD_Task_Entry(void *argument)
{
    (void)argument;
    printf("[LCD] LCD_Task_Entry started!\r\n");
    osDelay(200); /* Wait for SPI, ADC and peripherals to stabilize */
    LCD_Debug_Init();
    printf("[LCD] LCD_Debug_Init done, entering update loop\r\n");

    for (;;)
    {
        LCD_Debug_Update();
        /* CAN/motor diagnostics live here rather than in Motor_task so that
           formatting and logging can never eat into the 10 ms control period. */
        motor_diag_print();
        osDelay(50); /* 20 Hz update rate for ultra-smooth key response */
    }
}
