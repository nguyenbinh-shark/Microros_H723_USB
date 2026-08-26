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
#include <string.h>
#include <math.h>
#include "INS_task.h"
#include "observe_task.h"
#include "motor_task.h"
#include "can_bsp.h"
#include "BMI088driver.h"
#include "bsp_dwt.h"
#include "lcd_task.h"
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
extern UART_HandleTypeDef huart7;
/* USER CODE END Variables */

/* Definitions for tasks */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 3000 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

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
  .priority = (osPriority_t) osPriorityLow,
};

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
static rcl_subscription_t sub_cmd_vel;
static rcl_subscription_t sub_motor_enable;
static rclc_support_t     support;
static rcl_allocator_t    allocator;
static rcl_node_t         node;
static rclc_executor_t    executor;

static sensor_msgs__msg__JointState motor_fb_msg;
static geometry_msgs__msg__Twist    cmd_vel_msg;
static std_msgs__msg__Bool          motor_enable_msg;

/* Static buffers for JointState (zero dynamic heap allocation in control loop) */
static double                     js_pos[2];
static double                     js_vel[2];
static double                     js_eff[2];
static rosidl_runtime_c__String   js_name_item[2];
static char                       js_name_str0[] = "left_wheel";
static char                       js_name_str1[] = "right_wheel";

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void INS_Task_Entry(void *argument);
void Observe_Task_Entry(void *argument);
void Motor_Task_Entry(void *argument);

static void cmd_vel_callback(const void *msg_in);
static void motor_enable_callback(const void *msg_in);

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
  insTaskHandle = osThreadNew(INS_Task_Entry, NULL, &insTask_attributes);
  observeTaskHandle = osThreadNew(Observe_Task_Entry, NULL, &observeTask_attributes);
  motorTaskHandle = osThreadNew(Motor_Task_Entry, NULL, &motorTask_attributes);
  lcdTaskHandle = osThreadNew(LCD_Task_Entry, NULL, &lcdTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* ── Create & Destroy Entities Helper Functions ───────────────────── */
static bool create_entities(void)
{
  allocator = rcl_get_default_allocator();

  /* 1. Init Support & Sync session */
  rcl_ret_t rc = rclc_support_init(&support, 0, NULL, &allocator);
  if (rc != RCL_RET_OK) {
    printf("[ROS_ERR] rclc_support_init failed (rc=%d)\r\n", (int)rc);
    return false;
  }
  rmw_uros_sync_session(1000);

  /* 2. Init Node */
  rc = rclc_node_init_default(&node, "stm32h7_node", "", &support);
  if (rc != RCL_RET_OK) {
    printf("[ROS_ERR] rclc_node_init_default failed (rc=%d)\r\n", (int)rc);
    rclc_support_fini(&support);
    return false;
  }

  /* 3. Init Publishers (Best Effort for high throughput & zero buffering stalls) */
  rc = rclc_publisher_init_best_effort(
    &pub_motor_fb,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "motor_fb");
  if (rc != RCL_RET_OK) {
    printf("[ROS_ERR] rclc_publisher_init_best_effort [/motor_fb] failed\r\n");
    rc = rcl_node_fini(&node);
    rc = rclc_support_fini(&support);
    (void)rc;
    return false;
  }

  /* 4. Init Subscribers (Best Effort for cmd_vel to prevent latency build-up) */
  rc = rclc_subscription_init_best_effort(
    &sub_cmd_vel,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel");
  if (rc != RCL_RET_OK) {
    printf("[ROS_ERR] rclc_subscription_init_best_effort [/cmd_vel] failed\r\n");
    rc = rcl_publisher_fini(&pub_motor_fb, &node);
    rc = rcl_node_fini(&node);
    rc = rclc_support_fini(&support);
    (void)rc;
    return false;
  }

  rc = rclc_subscription_init_default(
    &sub_motor_enable,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "motor_enable");
  if (rc != RCL_RET_OK) {
    printf("[ROS_ERR] rclc_subscription_init_default [/motor_enable] failed\r\n");
    rc = rcl_subscription_fini(&sub_cmd_vel, &node);
    rc = rcl_publisher_fini(&pub_motor_fb, &node);
    rc = rcl_node_fini(&node);
    rc = rclc_support_fini(&support);
    (void)rc;
    return false;
  }

  /* 5. Init Executor: 2 handles (cmd_vel + motor_enable) */
  memset(&cmd_vel_msg, 0, sizeof(cmd_vel_msg));
  memset(&motor_enable_msg, 0, sizeof(motor_enable_msg));
  rc = rclc_executor_init(&executor, &support.context, 2, &allocator);
  if (rc != RCL_RET_OK) {
    printf("[ROS_ERR] rclc_executor_init failed\r\n");
    rc = rcl_subscription_fini(&sub_motor_enable, &node);
    rc = rcl_subscription_fini(&sub_cmd_vel, &node);
    rc = rcl_publisher_fini(&pub_motor_fb, &node);
    rc = rcl_node_fini(&node);
    rc = rclc_support_fini(&support);
    (void)rc;
    return false;
  }

  rclc_executor_add_subscription(&executor, &sub_cmd_vel, &cmd_vel_msg,
    &cmd_vel_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &sub_motor_enable, &motor_enable_msg,
    &motor_enable_callback, ON_NEW_DATA);

  /* 6. Prepare Static JointState Message */
  memset(&motor_fb_msg, 0, sizeof(motor_fb_msg));
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

  printf("[ROS] Node [stm32h7_node] Ready | Pub: [/motor_fb] | Sub: [/cmd_vel, /motor_enable]\r\n");
  return true;
}

static void destroy_entities(void)
{
  printf("[ROS] Cleaning up micro-ROS entities for reconnection...\r\n");
  rcl_ret_t rc;
  rc = rcl_publisher_fini(&pub_motor_fb, &node);
  rc = rcl_subscription_fini(&sub_cmd_vel, &node);
  rc = rcl_subscription_fini(&sub_motor_enable, &node);
  rc = rclc_executor_fini(&executor);
  rc = rcl_node_fini(&node);
  rc = rclc_support_fini(&support);
  (void)rc;
  printf("[ROS] Cleanup complete. Waiting for Agent...\r\n");
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
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));

        uint32_t now = HAL_GetTick();

        /* 2. Periodic Publish: /motor_fb at 20 Hz (every 50ms) */
        if ((now - last_pub_ms) >= 50U)
        {
          last_pub_ms = now;

          float   m_pos[2], m_vel[2], m_tor[2];
          uint8_t m_valid[2];
          CAN_MotorFeedback_Get_Idx(0, &m_pos[0], &m_vel[0], &m_tor[0], &m_valid[0]);
          CAN_MotorFeedback_Get_Idx(1, &m_pos[1], &m_vel[1], &m_tor[1], &m_valid[1]);
          js_pos[0] = (double)m_pos[0];
          js_vel[0] = (double)m_vel[0];
          js_eff[0] = (double)m_tor[0];
          js_pos[1] = (double)m_pos[1];
          js_vel[1] = (double)m_vel[1];
          js_eff[1] = (double)m_tor[1];

          rcl_ret_t pub_ret = rcl_publish(&pub_motor_fb, &motor_fb_msg, NULL);
          if (pub_ret != RCL_RET_OK)
          {
            printf("[ROS_WARN] Publish failed (rc=%d). Agent may be lost.\r\n", (int)pub_ret);
            g_agent_state = AGENT_DISCONNECTED;
            break;
          }
          g_comm_metrics.pub_tx_count++;
        }

        /* 3. Calculate Command Frequency every 1 second */
        if ((now - g_last_hz_calc_tick) >= 1000U)
        {
          uint32_t dt = now - g_last_hz_calc_tick;
          g_last_hz_calc_tick = now;
          g_comm_metrics.cmd_rate_hz = (float)g_cmd_rx_window_count * 1000.0f / (float)dt;
          g_cmd_rx_window_count = 0;
        }

        /* 4. Periodic Health Check & Ping RTT measurement (every 2000ms) */
        if ((now - last_ping_ms) >= 2000U)
        {
          last_ping_ms = now;
          uint32_t t0 = HAL_GetTick();
          if (rmw_uros_ping_agent(50, 1) == RMW_RET_OK)
          {
            g_comm_metrics.ping_rtt_ms = (uint16_t)(HAL_GetTick() - t0);
          }
          else
          {
            printf("[ROS_WARN] Agent Ping Lost! Triggering Reconnect...\r\n");
            g_agent_state = AGENT_DISCONNECTED;
            break;
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

/* Callback: /cmd_vel → robot_cmd_set() */
static void cmd_vel_callback(const void *msg_in)
{
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msg_in;
  if (msg != NULL)
  {
    uint32_t now = HAL_GetTick();
    g_comm_metrics.cmd_last_tick = now;
    g_comm_metrics.cmd_rx_count++;
    g_cmd_rx_window_count++;

    printf("[CMD_VEL] vx=%.2f m/s, wz=%.2f rad/s\r\n", (float)msg->linear.x, (float)msg->angular.z);
    robot_cmd_set((float)msg->linear.x, (float)msg->angular.z, 1U);
  }
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

