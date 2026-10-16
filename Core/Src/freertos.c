/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/joint_state.h>
#include <nav_msgs/msg/odometry.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/u_int8.h>
#include <std_msgs/msg/float64_multi_array.h>
#include <string.h>
#include <math.h>
#include "INS_task.h"
#include "observe_task.h"
#include "motor_task.h"
#include "can_bsp.h"
#include "BMI088driver.h"
#include "bsp_dwt.h"
#include "lcd_task.h"
#include "spi.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/* ── micro-ROS State Machine Definitions ─────────────────────────── */
typedef enum {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} agent_state_t;

static agent_state_t g_agent_state = WAITING_AGENT;
static CommMetrics_t g_comm_metrics = {0};
static uint32_t      g_cmd_rx_window_count = 0;
static uint32_t      g_last_hz_calc_tick = 0;

uint8_t micro_ros_get_state(void)
{
  return (uint8_t)g_agent_state;
}

extern size_t cubemx_transport_get_pending(void);

/* micro-ROS entities */
static rcl_publisher_t    pub_motor_fb;
static rcl_publisher_t    pub_motor_status;
static rcl_publisher_t    pub_imu;
static rcl_publisher_t    pub_imu_debug;
static rcl_subscription_t sub_cmd_vel;
static rcl_subscription_t sub_motor_enable;
static rcl_subscription_t sub_cmd_mit;
static rcl_subscription_t sub_cmd_torque;
static rclc_support_t     support;
static rcl_allocator_t    allocator;
static rcl_node_t         node;
static rclc_executor_t    executor;

static sensor_msgs__msg__JointState motor_fb_msg;
static std_msgs__msg__UInt8         motor_status_msg;
static sensor_msgs__msg__Imu        imu_msg;
static std_msgs__msg__Float64MultiArray imu_debug_msg;
static double imu_debug_data[22];
static geometry_msgs__msg__Twist    cmd_vel_msg;
static std_msgs__msg__Bool          motor_enable_msg;
static std_msgs__msg__Float64MultiArray cmd_mit_msg;
static std_msgs__msg__Float64MultiArray cmd_torque_msg;

/* Static buffers for incoming /cmd_mit. micro-ROS deserialises sequences into
   caller-owned memory and drops any message that does not fit, so layout.dim
   gets room too: a publisher that fills it in must not lose every command. */
#define CMD_MIT_DIM_MAX    2U
#define CMD_MIT_LABEL_MAX  16U
static double                             cmd_mit_data[CMD_MIT_LEN];
static std_msgs__msg__MultiArrayDimension cmd_mit_dim[CMD_MIT_DIM_MAX];
static char                               cmd_mit_label[CMD_MIT_DIM_MAX][CMD_MIT_LABEL_MAX];

/* /cmd_torque gets its own buffers: the executor takes every ready message
   before running any callback, so shared buffers would let one topic
   overwrite the other's data before it is read. */
static double                             cmd_torque_data[CMD_TORQUE_LEN];
static std_msgs__msg__MultiArrayDimension cmd_torque_dim[CMD_MIT_DIM_MAX];
static char                               cmd_torque_label[CMD_MIT_DIM_MAX][CMD_MIT_LABEL_MAX];

/* Static buffers for JointState (zero dynamic heap allocation in control loop) */
static double                     js_pos[2];
static double                     js_vel[2];
static double                     js_eff[2];
static rosidl_runtime_c__String   js_name_item[2];
static char                       js_name_str0[] = "left_wheel";
static char                       js_name_str1[] = "right_wheel";
static char                       js_frame_id[]  = "base_link";
static char                       imu_frame_id[] = "imu_link";

/* Track successful initializations so every failure/reconnect releases only
 * live entities, in reverse dependency order. */
enum {
  ENT_SUPPORT = 1U << 0, ENT_NODE = 1U << 1,
  ENT_MOTOR_FB = 1U << 2, ENT_MOTOR_STATUS = 1U << 3, ENT_IMU = 1U << 4,
  ENT_CMD_VEL = 1U << 5, ENT_MOTOR_ENABLE = 1U << 6, ENT_CMD_MIT = 1U << 7,
  ENT_EXECUTOR = 1U << 8, ENT_CMD_TORQUE = 1U << 9,
  ENT_IMU_DEBUG = 1U << 10
};
static uint32_t g_entities;
static bool g_time_synced;
static uint32_t g_last_imu_sequence;
static int64_t g_last_imu_stamp_ns;
extern size_t xPortGetFreeHeapSizeMicroROS(void);
extern size_t xPortGetMinimumEverFreeHeapSizeMicroROS(void);

/* /motor_status bit meanings — lets a ROS node tell "wheel stopped" apart from
   "no CAN feedback", which a JointState full of zeros cannot express. */
#define MSTAT_LEFT_FRESH    (1U << 0)
#define MSTAT_RIGHT_FRESH   (1U << 1)
#define MSTAT_CAN1_BUSOFF   (1U << 2)
#define MSTAT_CAN3_BUSOFF   (1U << 3)
#define MSTAT_MOTORS_ENABLED (1U << 4)
#define MSTAT_CMD_TIMEOUT   (1U << 5)
#define MSTAT_MODE_MIT      (1U << 6)   /* 0 = Velocity (/cmd_vel), 1 = MIT (/cmd_mit, /cmd_torque) */
extern UART_HandleTypeDef huart7;

/* App task handles/attributes live INSIDE this USER CODE block so CubeMX
   regeneration cannot delete them (it wiped them once already). */
osThreadId_t insTaskHandle;
const osThreadAttr_t insTask_attributes = {
  .name = "insTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};

osThreadId_t observeTaskHandle;
const osThreadAttr_t observeTask_attributes = {
  .name = "observeTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

osThreadId_t motorTaskHandle;
const osThreadAttr_t motorTask_attributes = {
  .name = "motorTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

osThreadId_t lcdTaskHandle;
const osThreadAttr_t lcdTask_attributes = {
  .name = "lcdTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 3000 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* micro-ROS Entities Helper Functions */
static bool create_entities(void);
static void destroy_entities(void);
static void ros_report_error(const char *stage, rcl_ret_t rc);
static bool ros_stamp_from_tick(uint32_t sample_tick,
                               builtin_interfaces__msg__Time *stamp);
static void publish_imu(void);
void INS_Task_Entry(void *argument);
void Observe_Task_Entry(void *argument);
void Motor_Task_Entry(void *argument);
void LCD_Task_Entry(void *argument);

static void cmd_vel_callback(const void *msg_in);
static void motor_enable_callback(const void *msg_in);
static void cmd_mit_callback(const void *msg_in);
static void cmd_torque_callback(const void *msg_in);

bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);

static bool create_entities(void);
static void destroy_entities(void);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  insTaskHandle     = osThreadNew(INS_Task_Entry,     NULL, &insTask_attributes);
  observeTaskHandle = osThreadNew(Observe_Task_Entry, NULL, &observeTask_attributes);
  motorTaskHandle   = osThreadNew(Motor_Task_Entry,   NULL, &motorTask_attributes);
  lcdTaskHandle     = osThreadNew(LCD_Task_Entry,     NULL, &lcdTask_attributes);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  printf("[ROS] StartDefaultTask running. Init USB custom transport...\r\n");

  rmw_uros_set_custom_transport(
    true,
    (void *) NULL,
    cubemx_transport_open,
    cubemx_transport_close,
    cubemx_transport_write,
    cubemx_transport_read);

  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
  freeRTOS_allocator.allocate = microros_allocate;
  freeRTOS_allocator.deallocate = microros_deallocate;
  freeRTOS_allocator.reallocate = microros_reallocate;
  freeRTOS_allocator.zero_allocate = microros_zero_allocate;

  if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
    printf("[ROS_ERR] Failed to set default allocator!\r\n");
    for(;;) osDelay(10);
  }

  uint32_t last_pub_ms = 0U;
  uint32_t last_ping_ms = 0U;
  uint32_t last_imu_ms = 0U;
  uint32_t last_sync_ms = 0U;
  uint8_t ping_misses = 0U;

  for (;;)
  {
    switch (g_agent_state)
    {
      case WAITING_AGENT:
        if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK)
        {
          printf("[ROS] micro-ROS Agent Detected! Initializing entities...\r\n");
          g_agent_state = AGENT_AVAILABLE;
        }
        else
        {
          osDelay(100);
        }
        break;

      case AGENT_AVAILABLE:
        if (create_entities())
        {
          g_agent_state = AGENT_CONNECTED;
          last_pub_ms = HAL_GetTick();
          last_ping_ms = HAL_GetTick();
          last_imu_ms = HAL_GetTick();
          last_sync_ms = HAL_GetTick();
          ping_misses = 0U;
        }
        else
        {
          g_agent_state = WAITING_AGENT;
          osDelay(500);
        }
        break;

      case AGENT_CONNECTED:
      {
        /* 1. Spin executor with ultra-short 1ms timeout */
        rcl_ret_t spin_rc = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
        if (spin_rc != RCL_RET_OK && spin_rc != RCL_RET_TIMEOUT)
        {
          ros_report_error("executor spin", spin_rc);
          g_agent_state = AGENT_DISCONNECTED;
          break;
        }

        uint32_t now = HAL_GetTick();

        /* Publish a coherent, calibrated IMU sample at 50 Hz. Acquisition time
         * is retained even if USB/executor work delays this loop. */
        if ((now - last_imu_ms) >= 20U)
        {
          last_imu_ms = now;
          publish_imu();
        }

        /* 2. Periodic Publish: /motor_fb at 50 Hz (every 20ms) */
        if ((now - last_pub_ms) >= 20U)
        {
          last_pub_ms = now;

          float   m_pos[2], m_vel[2], m_tor[2];
          uint8_t m_valid[2];
          CAN_MotorFeedback_Get_Idx(0, &m_pos[0], &m_vel[0], &m_tor[0], &m_valid[0]);
          CAN_MotorFeedback_Get_Idx(1, &m_pos[1], &m_vel[1], &m_tor[1], &m_valid[1]);
          /* Wheel frame: positive = robot forward on both sides, so the host
             odometry needs no knowledge of how the motors are mounted. */
          js_pos[0] = (double)(m_pos[0] * MOTOR_LEFT_SIGN);
          js_vel[0] = (double)(m_vel[0] * MOTOR_LEFT_SIGN);
          js_eff[0] = (double)(m_tor[0] * MOTOR_LEFT_SIGN);
          js_pos[1] = (double)(m_pos[1] * MOTOR_RIGHT_SIGN);
          js_vel[1] = (double)(m_vel[1] * MOTOR_RIGHT_SIGN);
          js_eff[1] = (double)(m_tor[1] * MOTOR_RIGHT_SIGN);

          /* Never send uptime as ROS epoch time when synchronization failed. */
          if (ros_stamp_from_tick(now, &motor_fb_msg.header.stamp))
          {
            rcl_ret_t pub_ret = rcl_publish(&pub_motor_fb, &motor_fb_msg, NULL);
            if (pub_ret == RCL_RET_OK) g_comm_metrics.pub_tx_count++;
            else rcl_reset_error(); /* Health ping decides when to reconnect. */
          }

          /* Link health, at 1/10 of the feedback rate. Without this a consumer
             cannot distinguish "wheels are stopped" from "CAN is dead". */
          static uint8_t status_divider = 0U;
          if (++status_divider >= 10U)
          {
            status_divider = 0U;
            CAN_BusMetrics_t bm;
            CAN_GetMetrics(&bm);

            uint8_t st = 0U;
            if (m_valid[0])            st |= MSTAT_LEFT_FRESH;
            if (m_valid[1])            st |= MSTAT_RIGHT_FRESH;
            if (bm.can1_proto.bus_off) st |= MSTAT_CAN1_BUSOFF;
            if (bm.can3_proto.bus_off) st |= MSTAT_CAN3_BUSOFF;
            if (g_system_enabled)      st |= MSTAT_MOTORS_ENABLED;
            if (motor_cmd_timed_out()) st |= MSTAT_CMD_TIMEOUT;
            if (motor_get_control_mode() == CTRL_MODE_MIT) st |= MSTAT_MODE_MIT;

            motor_status_msg.data = st;
            rcl_ret_t st_ret = rcl_publish(&pub_motor_status, &motor_status_msg, NULL);
            if (st_ret != RCL_RET_OK) rcl_reset_error();

            /* Diagnostic telemetry remains available when valid IMU output is
             * deliberately suppressed. It is never an estimator input. */
            INS_DebugState_t dbg;
            INS_GetDebugState(&dbg);
            imu_debug_data[0] = BMI088_GetInitStatus();
            imu_debug_data[1] = dbg.read_valid;
            imu_debug_data[2] = dbg.sample.valid;
            imu_debug_data[3] = dbg.sample.calibrated;
            imu_debug_data[4] = dbg.calibration_count;
            imu_debug_data[5] = dbg.sample.sequence;
            /* INS can preempt us after `now` was captured; use a fresh tick
             * after copying the sample to avoid unsigned age underflow. */
            imu_debug_data[6] = (uint32_t)(HAL_GetTick() - dbg.sample.sample_tick_ms);
            for (uint8_t i = 0; i < 3; ++i) {
              imu_debug_data[7 + i] = dbg.accel[i];
              imu_debug_data[10 + i] = dbg.gyro[i];
            }
            imu_debug_data[13] = dbg.accel_scale;
            imu_debug_data[14] = BMI088.gNorm;
            imu_debug_data[15] = dbg.dt;
            imu_debug_data[16] = hspi2.ErrorCode;
            imu_debug_data[17] = g_time_synced;
            imu_debug_data[18] = SYSCFG->PMCR;
            imu_debug_data[19] = SPI2->SR;
            imu_debug_data[20] = BMI088_GetChipId(0);
            imu_debug_data[21] = BMI088_GetChipId(1);
            rcl_ret_t dbg_ret = rcl_publish(&pub_imu_debug, &imu_debug_msg, NULL);
            if (dbg_ret != RCL_RET_OK) rcl_reset_error();
          }
        }

        /* 3. Calculate Command Frequency every 1 second */
        if ((now - g_last_hz_calc_tick) >= 1000U)
        {
          uint32_t dt = now - g_last_hz_calc_tick;
          g_last_hz_calc_tick = now;
          g_comm_metrics.cmd_rate_hz = (float)g_cmd_rx_window_count * 1000.0f / (float)dt;
          g_cmd_rx_window_count = 0;
        }

        /* 4. Periodic Health Check & Ping RTT measurement.
         * A single 50 ms ping every 2 s used to be enough to declare the agent
         * lost: one slow USB turnaround tore down every entity and the topics
         * vanished and reappeared. Ping less often, wait longer, and require
         * several misses in a row before reconnecting. */
        if ((now - last_ping_ms) >= 3000U)
        {
          last_ping_ms = now;
          uint32_t t0 = HAL_GetTick();
          if (rmw_uros_ping_agent(150, 1) == RMW_RET_OK)
          {
            g_comm_metrics.ping_rtt_ms = (uint16_t)(HAL_GetTick() - t0);
            ping_misses = 0U;
          }
          else if (++ping_misses >= 3U)
          {
            printf("[ROS_WARN] Agent unreachable for 3 pings. Reconnecting...\r\n");
            ping_misses = 0U;
            g_agent_state = AGENT_DISCONNECTED;
            break;
          }
        }

        /* 5. Periodic Time Synchronization (every 60s) to prevent clock drift */
        uint32_t sync_interval_ms = g_time_synced ? 60000U : 1000U;
        if ((now - last_sync_ms) >= sync_interval_ms)
        {
          last_sync_ms = now;
          g_time_synced = (rmw_uros_sync_session(200) == RMW_RET_OK);
          if (g_time_synced)
          {
            int64_t epoch_ms = rmw_uros_epoch_millis();
            printf("[ROS] Periodic time re-sync OK: epoch=%lld ms\r\n", (long long)epoch_ms);
          }
          else
          {
            printf("[ROS_WARN] Periodic time re-sync missed.\r\n");
          }
        }

        osDelay(1);
        break;
      }

      case AGENT_DISCONNECTED:
        destroy_entities();
        /* Stop motors safely when agent disconnects */
        robot_cmd_set(0.0f, 0.0f, 0U);
        g_agent_state = WAITING_AGENT;
        osDelay(200);
        break;
    }
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* ── Create & Destroy Entities Helper Functions ───────────────────── */
static bool create_entities(void)
{
  static uint32_t attempts;
  const char *stage = "support";
  rcl_ret_t rc;
  allocator = rcl_get_default_allocator();
  memset(&support, 0, sizeof(support));
  node = rcl_get_zero_initialized_node();
  pub_motor_fb = rcl_get_zero_initialized_publisher();
  pub_motor_status = rcl_get_zero_initialized_publisher();
  pub_imu = rcl_get_zero_initialized_publisher();
  pub_imu_debug = rcl_get_zero_initialized_publisher();
  sub_cmd_vel = rcl_get_zero_initialized_subscription();
  sub_motor_enable = rcl_get_zero_initialized_subscription();
  sub_cmd_mit = rcl_get_zero_initialized_subscription();
  sub_cmd_torque = rcl_get_zero_initialized_subscription();
  executor = rclc_executor_get_zero_initialized_executor();
  g_entities = 0U;
  g_time_synced = false;
  g_last_imu_sequence = 0U;
  g_last_imu_stamp_ns = 0;
  rcl_reset_error();
  printf("[ROS] Creating entities, attempt %lu\r\n", (unsigned long)++attempts);

  rc = rclc_support_init(&support, 0, NULL, &allocator);
  if (rc != RCL_RET_OK)
  {
    /* A clock-init failure can leave a live context behind. */
    if (rcl_context_is_valid(&support.context)) g_entities |= ENT_SUPPORT;
    goto fail;
  }
  g_entities |= ENT_SUPPORT;

  stage = "node";
  rc = rclc_node_init_default(&node, "stm32h7_node", "", &support);
  if (rc != RCL_RET_OK) goto fail;
  g_entities |= ENT_NODE;

  stage = "publisher /motor_fb";
  rc = rclc_publisher_init_best_effort(&pub_motor_fb, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), "motor_fb");
  if (rc != RCL_RET_OK) goto fail;
  g_entities |= ENT_MOTOR_FB;

  stage = "publisher /motor_status";
  rc = rclc_publisher_init_default(&pub_motor_status, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8), "motor_status");
  if (rc != RCL_RET_OK) goto fail;
  g_entities |= ENT_MOTOR_STATUS;
  /* Bound a reliable status publication so a missing ACK cannot stall sensor
   * publication for the library's default one-second timeout. */
  (void)rmw_uros_set_publisher_session_timeout(
    rcl_publisher_get_rmw_handle(&pub_motor_status), 20);

  stage = "publisher /imu/data";
  rc = rclc_publisher_init_best_effort(&pub_imu, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu/data");
  if (rc != RCL_RET_OK) goto fail;
  g_entities |= ENT_IMU;

  stage = "publisher /imu/debug";
  rc = rclc_publisher_init_best_effort(&pub_imu_debug, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray), "imu/debug");
  if (rc != RCL_RET_OK) goto fail;
  g_entities |= ENT_IMU_DEBUG;
  memset(&imu_debug_msg, 0, sizeof(imu_debug_msg));
  imu_debug_msg.data.data = imu_debug_data;
  imu_debug_msg.data.size = 22;
  imu_debug_msg.data.capacity = 22;

  stage = "subscription /cmd_vel";
  rc = rclc_subscription_init_best_effort(&sub_cmd_vel, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel");
  if (rc != RCL_RET_OK) goto fail;
  g_entities |= ENT_CMD_VEL;

  stage = "subscription /motor_enable";
  rc = rclc_subscription_init_default(&sub_motor_enable, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "motor_enable");
  if (rc != RCL_RET_OK) goto fail;
  g_entities |= ENT_MOTOR_ENABLE;

  stage = "subscription /cmd_mit";
  rc = rclc_subscription_init_best_effort(&sub_cmd_mit, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray), "cmd_mit");
  if (rc != RCL_RET_OK) goto fail;
  g_entities |= ENT_CMD_MIT;

  stage = "subscription /cmd_torque";
  rc = rclc_subscription_init_best_effort(&sub_cmd_torque, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray), "cmd_torque");
  if (rc != RCL_RET_OK) goto fail;
  g_entities |= ENT_CMD_TORQUE;

  memset(&cmd_vel_msg, 0, sizeof(cmd_vel_msg));
  memset(&motor_enable_msg, 0, sizeof(motor_enable_msg));
  memset(&cmd_mit_msg, 0, sizeof(cmd_mit_msg));
  for (uint32_t i = 0; i < CMD_MIT_DIM_MAX; i++)
  {
    cmd_mit_dim[i].label.data     = cmd_mit_label[i];
    cmd_mit_dim[i].label.size     = 0U;
    cmd_mit_dim[i].label.capacity = CMD_MIT_LABEL_MAX;
  }
  cmd_mit_msg.layout.dim.data     = cmd_mit_dim;
  cmd_mit_msg.layout.dim.size     = 0U;
  cmd_mit_msg.layout.dim.capacity = CMD_MIT_DIM_MAX;
  cmd_mit_msg.data.data           = cmd_mit_data;
  cmd_mit_msg.data.size           = 0U;
  cmd_mit_msg.data.capacity       = CMD_MIT_LEN;

  memset(&cmd_torque_msg, 0, sizeof(cmd_torque_msg));
  for (uint32_t i = 0; i < CMD_MIT_DIM_MAX; i++)
  {
    cmd_torque_dim[i].label.data     = cmd_torque_label[i];
    cmd_torque_dim[i].label.size     = 0U;
    cmd_torque_dim[i].label.capacity = CMD_MIT_LABEL_MAX;
  }
  cmd_torque_msg.layout.dim.data     = cmd_torque_dim;
  cmd_torque_msg.layout.dim.size     = 0U;
  cmd_torque_msg.layout.dim.capacity = CMD_MIT_DIM_MAX;
  cmd_torque_msg.data.data           = cmd_torque_data;
  cmd_torque_msg.data.size           = 0U;
  cmd_torque_msg.data.capacity       = CMD_TORQUE_LEN;

  stage = "executor";
  rc = rclc_executor_init(&executor, &support.context, 4, &allocator);
  if (rc != RCL_RET_OK) goto fail;
  g_entities |= ENT_EXECUTOR;

  stage = "executor /cmd_vel";
  rc = rclc_executor_add_subscription(&executor, &sub_cmd_vel, &cmd_vel_msg,
    &cmd_vel_callback, ON_NEW_DATA);
  if (rc != RCL_RET_OK) goto fail;
  stage = "executor /motor_enable";
  rc = rclc_executor_add_subscription(&executor, &sub_motor_enable, &motor_enable_msg,
    &motor_enable_callback, ON_NEW_DATA);
  if (rc != RCL_RET_OK) goto fail;
  stage = "executor /cmd_mit";
  rc = rclc_executor_add_subscription(&executor, &sub_cmd_mit, &cmd_mit_msg,
    &cmd_mit_callback, ON_NEW_DATA);
  if (rc != RCL_RET_OK) goto fail;
  stage = "executor /cmd_torque";
  rc = rclc_executor_add_subscription(&executor, &sub_cmd_torque, &cmd_torque_msg,
    &cmd_torque_callback, ON_NEW_DATA);
  if (rc != RCL_RET_OK) goto fail;

  /* 6. Prepare Static JointState Message */
  memset(&motor_fb_msg, 0, sizeof(motor_fb_msg));
  memset(&motor_status_msg, 0, sizeof(motor_status_msg));

  /* A zeroed header leaves frame_id.data == NULL, which serialises as a
     zero-length CDR string, and stamp == 0, which makes every time-aware
     consumer (rqt_plot, PlotJuggler, robot_localization) discard the sample. */
  motor_fb_msg.header.frame_id.data     = js_frame_id;
  motor_fb_msg.header.frame_id.size     = sizeof(js_frame_id) - 1U;
  motor_fb_msg.header.frame_id.capacity = sizeof(js_frame_id);

  js_name_item[0].data     = js_name_str0;
  js_name_item[0].size     = sizeof(js_name_str0) - 1U;
  js_name_item[0].capacity = sizeof(js_name_str0);
  js_name_item[1].data     = js_name_str1;
  js_name_item[1].size     = sizeof(js_name_str1) - 1U;
  js_name_item[1].capacity = sizeof(js_name_str1);

  motor_fb_msg.name.data     = js_name_item;
  motor_fb_msg.name.size     = 2U;
  motor_fb_msg.name.capacity = 2U;
  motor_fb_msg.position.data = js_pos;
  motor_fb_msg.position.size = 2U;
  motor_fb_msg.position.capacity = 2U;
  motor_fb_msg.velocity.data = js_vel;
  motor_fb_msg.velocity.size = 2U;
  motor_fb_msg.velocity.capacity = 2U;
  motor_fb_msg.effort.data   = js_eff;
  motor_fb_msg.effort.size   = 2U;
  motor_fb_msg.effort.capacity = 2U;

  memset(&imu_msg, 0, sizeof(imu_msg));
  imu_msg.header.frame_id.data = imu_frame_id;
  imu_msg.header.frame_id.size = sizeof(imu_frame_id) - 1U;
  imu_msg.header.frame_id.capacity = sizeof(imu_frame_id);
  /* Diagonal covariances (row-major 3x3). Left at zero, robot_localization
   * would read them as perfect measurements and over-trust the IMU.
   * Quaternion yaw is relative to startup: there is no magnetometer heading. */
  imu_msg.orientation_covariance[0]         = IMU_ORIENT_RP_VARIANCE;
  imu_msg.orientation_covariance[4]         = IMU_ORIENT_RP_VARIANCE;
  imu_msg.orientation_covariance[8]         = IMU_ORIENT_YAW_VARIANCE;
  imu_msg.angular_velocity_covariance[0]    = IMU_GYRO_VARIANCE;
  imu_msg.angular_velocity_covariance[4]    = IMU_GYRO_VARIANCE;
  imu_msg.angular_velocity_covariance[8]    = IMU_GYRO_VARIANCE;
  imu_msg.linear_acceleration_covariance[0] = IMU_ACCEL_VARIANCE;
  imu_msg.linear_acceleration_covariance[4] = IMU_ACCEL_VARIANCE;
  imu_msg.linear_acceleration_covariance[8] = IMU_ACCEL_VARIANCE;

  for (uint8_t retry = 0; retry < 3U; ++retry)
  {
    g_time_synced = (rmw_uros_sync_session(1000) == RMW_RET_OK);
    if (g_time_synced) break;
    osDelay(100U);
  }
  printf("[ROS] Time sync %s; sensor publication requires synchronized time.\r\n",
         g_time_synced ? "OK" : "pending");
  printf("[ROS] Node [stm32h7_node] Ready | Pub: [/motor_fb, /motor_status, /imu/data] | Sub: [/cmd_vel, /cmd_mit, /cmd_torque, /motor_enable]\r\n");
  return true;

fail:
  ros_report_error(stage, rc);
  destroy_entities();
  return false;
}

static void destroy_entities(void)
{
  rcl_ret_t rc = RCL_RET_OK;
  /* A disconnected agent cannot acknowledge DELETEs. Local cleanup still
   * releases every pool entry; a new session recreates the remote entities. */
  if (g_entities & ENT_SUPPORT)
    (void)rmw_uros_set_context_entity_destroy_session_timeout(
      rcl_context_get_rmw_context(&support.context), 0);
  if (g_entities & ENT_EXECUTOR) rc = rclc_executor_fini(&executor);
  if (g_entities & ENT_CMD_TORQUE) rc = rcl_subscription_fini(&sub_cmd_torque, &node);
  if (g_entities & ENT_CMD_MIT) rc = rcl_subscription_fini(&sub_cmd_mit, &node);
  if (g_entities & ENT_MOTOR_ENABLE) rc = rcl_subscription_fini(&sub_motor_enable, &node);
  if (g_entities & ENT_CMD_VEL) rc = rcl_subscription_fini(&sub_cmd_vel, &node);
  if (g_entities & ENT_IMU_DEBUG) rc = rcl_publisher_fini(&pub_imu_debug, &node);
  if (g_entities & ENT_IMU) rc = rcl_publisher_fini(&pub_imu, &node);
  if (g_entities & ENT_MOTOR_STATUS) rc = rcl_publisher_fini(&pub_motor_status, &node);
  if (g_entities & ENT_MOTOR_FB) rc = rcl_publisher_fini(&pub_motor_fb, &node);
  if (g_entities & ENT_NODE) rc = rcl_node_fini(&node);
  if (g_entities & ENT_SUPPORT) rc = rclc_support_fini(&support);
  (void)rc;
  g_entities = 0U;
  g_time_synced = false;
  rcl_reset_error();
}

static void ros_report_error(const char *stage, rcl_ret_t rc)
{
  printf("[ROS_ERR] %s failed rc=%d: %s | ROS heap=%lu min=%lu RTOS heap=%lu\r\n",
    stage, (int)rc, rcl_get_error_string().str,
    (unsigned long)xPortGetFreeHeapSizeMicroROS(),
    (unsigned long)xPortGetMinimumEverFreeHeapSizeMicroROS(),
    (unsigned long)xPortGetFreeHeapSize());
  rcl_reset_error();
}

static bool ros_stamp_from_tick(uint32_t sample_tick,
                               builtin_interfaces__msg__Time *stamp)
{
  if (!g_time_synced || !rmw_uros_epoch_synchronized()) return false;
  uint32_t epoch_tick = HAL_GetTick();
  int64_t epoch_ns = rmw_uros_epoch_nanos();
  int64_t sample_ns = epoch_ns - (int64_t)(uint32_t)(epoch_tick - sample_tick) * 1000000LL;
  if (sample_ns <= 0 || sample_ns / 1000000000LL > INT32_MAX) return false;
  stamp->sec = (int32_t)(sample_ns / 1000000000LL);
  stamp->nanosec = (uint32_t)(sample_ns % 1000000000LL);
  return true;
}

static void publish_imu(void)
{
  INS_Snapshot_t sample;
  if (!INS_GetSnapshot(&sample) || !sample.calibrated ||
      (uint32_t)(HAL_GetTick() - sample.sample_tick_ms) > 50U ||
      sample.sequence == g_last_imu_sequence) return;
  if (!ros_stamp_from_tick(sample.sample_tick_ms, &imu_msg.header.stamp)) return;

  int64_t stamp_ns = (int64_t)imu_msg.header.stamp.sec * 1000000000LL +
                    imu_msg.header.stamp.nanosec;
  if (stamp_ns <= g_last_imu_stamp_ns) return;
  /* INS is in the physical sensor axes, not the legacy swapped Euler labels.
   * Host TF must describe the measured imu_link -> base_link mounting. */
  imu_msg.orientation.w = sample.q[0];
  imu_msg.orientation.x = sample.q[1];
  imu_msg.orientation.y = sample.q[2];
  imu_msg.orientation.z = sample.q[3];
  imu_msg.angular_velocity.x = sample.gyro[0];
  imu_msg.angular_velocity.y = sample.gyro[1];
  imu_msg.angular_velocity.z = sample.gyro[2];
  imu_msg.linear_acceleration.x = sample.accel[0];
  imu_msg.linear_acceleration.y = sample.accel[1];
  imu_msg.linear_acceleration.z = sample.accel[2];
  rcl_ret_t rc = rcl_publish(&pub_imu, &imu_msg, NULL);
  if (rc == RCL_RET_OK)
  {
    g_last_imu_sequence = sample.sequence;
    g_last_imu_stamp_ns = stamp_ns;
  }
  else rcl_reset_error();
}


/* Callback: /cmd_vel → robot_cmd_set(), selects Velocity mode */
static void cmd_vel_callback(const void *msg_in)
{
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msg_in;
  /* A NaN passes every clamp (all comparisons are false) and would reach
     SPD_REF as-is. Drop it; the watchdog stops the wheels if nothing valid
     follows. */
  if (msg != NULL && (!isfinite(msg->linear.x) || !isfinite(msg->angular.z)))
  {
    printf("[CMD_VEL] Ignored: non-finite value\r\n");
    return;
  }
  if (msg != NULL)
  {
    uint32_t now = HAL_GetTick();
    g_comm_metrics.cmd_last_tick = now;
    g_comm_metrics.cmd_rx_count++;
    g_cmd_rx_window_count++;

    /* No per-message logging here: this runs inside the executor, and printing
       every command adds latency to the path that matters most. The rate and
       age are already tracked in g_comm_metrics and shown on the LCD. */
    robot_cmd_set((float)msg->linear.x, (float)msg->angular.z, 1U);
  }
}

/* Callback: /cmd_mit → motor_mit_cmd_set(), selects MIT mode
   data = [pos, vel, kp, kd, tau] for the left wheel, then the same five for
   the right, wheel frame (positive = robot forward), the same frame /motor_fb
   reports in. MOTOR_*_SIGN converts pos/vel/tau to the motor-shaft frame;
   the gains are sign-free. */
static void cmd_mit_callback(const void *msg_in)
{
  const std_msgs__msg__Float64MultiArray *msg = (const std_msgs__msg__Float64MultiArray *)msg_in;
  if (msg == NULL) return;

  /* Reject rather than guess: a short array or a NaN gain reaching the motor
     is a torque command nobody asked for. The previous setpoint stays, and the
     watchdog stops the wheels if no valid one follows. */
  if (msg->data.size != CMD_MIT_LEN)
  {
    printf("[CMD_MIT] Ignored: expected %u values, got %u\r\n",
           (unsigned)CMD_MIT_LEN, (unsigned)msg->data.size);
    return;
  }
  const double *d = msg->data.data;
  for (uint32_t i = 0; i < CMD_MIT_LEN; i++)
  {
    if (!isfinite(d[i]))
    {
      printf("[CMD_MIT] Ignored: non-finite value at [%lu]\r\n", (unsigned long)i);
      return;
    }
  }

  MotorMitCmd_t left  = { (float)d[0] * MOTOR_LEFT_SIGN,  (float)d[1] * MOTOR_LEFT_SIGN,
                          (float)d[2], (float)d[3], (float)d[4] * MOTOR_LEFT_SIGN };
  MotorMitCmd_t right = { (float)d[5] * MOTOR_RIGHT_SIGN, (float)d[6] * MOTOR_RIGHT_SIGN,
                          (float)d[7], (float)d[8], (float)d[9] * MOTOR_RIGHT_SIGN };

  g_comm_metrics.cmd_last_tick = HAL_GetTick();
  g_comm_metrics.cmd_rx_count++;
  g_cmd_rx_window_count++;

  motor_mit_cmd_set(&left, &right);
}

/* Callback: /cmd_torque → motor_mit_cmd_set(), selects MIT mode
   data = [tau_left, tau_right] in Nm, wheel frame (positive = robot forward).
   Pure torque: sent as an MIT setpoint with p = v = Kp = Kd = 0, so the
   actuator outputs tau and nothing else. Same message as ros2_control's
   effort_controllers/JointGroupEffortController. */
static void cmd_torque_callback(const void *msg_in)
{
  const std_msgs__msg__Float64MultiArray *msg = (const std_msgs__msg__Float64MultiArray *)msg_in;
  if (msg == NULL) return;

  if (msg->data.size != CMD_TORQUE_LEN)
  {
    printf("[CMD_TORQUE] Ignored: expected %u values, got %u\r\n",
           (unsigned)CMD_TORQUE_LEN, (unsigned)msg->data.size);
    return;
  }
  const double *d = msg->data.data;
  if (!isfinite(d[0]) || !isfinite(d[1]))
  {
    printf("[CMD_TORQUE] Ignored: non-finite value\r\n");
    return;
  }

  /* No speed loop in this mode, so torque is the only thing limiting how
     hard the wheels accelerate. Clamp in the wheel frame before the sign. */
  float tau_l = fminf(fmaxf((float)d[0], -CMD_TORQUE_MAX_NM), CMD_TORQUE_MAX_NM);
  float tau_r = fminf(fmaxf((float)d[1], -CMD_TORQUE_MAX_NM), CMD_TORQUE_MAX_NM);

  MotorMitCmd_t left  = { 0.0f, 0.0f, 0.0f, 0.0f, tau_l * MOTOR_LEFT_SIGN  };
  MotorMitCmd_t right = { 0.0f, 0.0f, 0.0f, 0.0f, tau_r * MOTOR_RIGHT_SIGN };

  g_comm_metrics.cmd_last_tick = HAL_GetTick();
  g_comm_metrics.cmd_rx_count++;
  g_cmd_rx_window_count++;

  motor_mit_cmd_set(&left, &right);
}

/* Callback: /motor_enable → motor_enable_set() */
static void motor_enable_callback(const void *msg_in)
{
  const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *)msg_in;
  if (msg != NULL)
  {
    printf("[MOTOR_EN] State=%d\r\n", msg->data ? 1 : 0);
    motor_enable_set(msg->data ? 1U : 0U);
  }
}

void comm_metrics_get(CommMetrics_t *metrics)
{
  if (metrics != NULL)
  {
    *metrics = g_comm_metrics;
    uint32_t now = HAL_GetTick();
    if (metrics->cmd_last_tick > 0)
    {
      metrics->cmd_age_ms = now - metrics->cmd_last_tick;
    }
    else
    {
      metrics->cmd_age_ms = 0xFFFF;
    }
    metrics->rx_pending_bytes = (uint16_t)cubemx_transport_get_pending();
  }
}

void INS_Task_Entry(void *argument)
{
  INS_task(); /* BMI088_Init already called in main() before RTOS start */
}

void Observe_Task_Entry(void *argument)
{
  for (;;) { Observe_task(); }
}

void Motor_Task_Entry(void *argument)
{
  Motor_task();
}

/* ── FreeRTOS Error Hooks ─────────────────────────────────────────── */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  (void)xTask;
  printf("\r\n[FATAL] Stack Overflow in Task: [%s]!\r\n", pcTaskName ? pcTaskName : "Unknown");
  for (;;) { osDelay(100); }
}

void vApplicationMallocFailedHook(void)
{
  printf("\r\n[FATAL] FreeRTOS Heap Allocation Failed (Out of Memory)!\r\n");
  for (;;) { osDelay(100); }
}
/* USER CODE END Application */

