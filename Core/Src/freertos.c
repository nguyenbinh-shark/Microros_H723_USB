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

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 3000 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

/* Definitions for INS_TASK */
osThreadId_t INS_TaskHandle;
const osThreadAttr_t INS_Task_attributes = {
  .name = "INS_TASK",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};

/* Definitions for OBSERVE_TASK */
osThreadId_t Observe_TaskHandle;
const osThreadAttr_t Observe_Task_attributes = {
  .name = "OBSERVE_TASK",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

/* Definitions for MOTOR_TASK */
osThreadId_t Motor_TaskHandle;
const osThreadAttr_t Motor_Task_attributes = {
  .name = "MOTOR_TASK",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

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
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

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
  /* Temporarily commented out to isolate reset issue */
  // INS_TaskHandle     = osThreadNew(INS_Task_Entry,     NULL, &INS_Task_attributes);

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
  /* USER CODE BEGIN StartDefaultTask */
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
    for(;;) osDelay(10);
  }

  /* micro-ROS entities */
  rcl_publisher_t    pub_imu;
  rcl_publisher_t    pub_euler;
  rcl_publisher_t    pub_motor_fb;
  rcl_publisher_t    pub_odom;
  rcl_subscription_t sub_cmd_vel;
  rcl_subscription_t sub_motor_enable;
  rclc_support_t     support;
  rcl_allocator_t    allocator;
  rcl_node_t         node;
  rclc_executor_t    executor;

  sensor_msgs__msg__Imu        imu_msg;
  geometry_msgs__msg__Vector3  euler_msg;
  sensor_msgs__msg__JointState motor_fb_msg;
  nav_msgs__msg__Odometry      odom_msg;
  geometry_msgs__msg__Twist    cmd_vel_msg;
  std_msgs__msg__Bool          motor_enable_msg;

  /* Static buffers for JointState (no heap allocation for message data) */
  static double                     js_pos[2];
  static double                     js_vel[2];
  static double                     js_eff[2];
  static rosidl_runtime_c__String   js_name_item[2];
  static char                       js_name_str0[]     = "left_wheel";
  static char                       js_name_str1[]     = "right_wheel";
  static char                       imu_frame_id[]     = "imu_link";
  static char                       odom_frame_id[]    = "odom";
  static char                       odom_child_id[]    = "base_link";

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rmw_uros_sync_session(1000);
  rclc_node_init_default(&node, "stm32h7_node", "", &support);

  /* --- Publishers (imu/euler/odom disabled to save UART bandwidth) --- */
  // rclc_publisher_init_default(
  //   &pub_imu, &node,
  //   ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
  //   "imu");

  // rclc_publisher_init_default(
  //   &pub_euler, &node,
  //   ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
  //   "euler");

  rclc_publisher_init_default(
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "motor_fb");

  // rclc_publisher_init_default(
  //   &pub_odom, &node,
  //   ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
  //   "odom");

  /* --- Subscribers --- */
  rclc_subscription_init_default(
    &sub_cmd_vel, &node,
    "cmd_vel");

  rclc_subscription_init_default(
    &sub_motor_enable, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "motor_enable");

  /* Executor: 2 handles (cmd_vel + motor_enable) */
  memset(&cmd_vel_msg, 0, sizeof(cmd_vel_msg));
  memset(&motor_enable_msg, 0, sizeof(motor_enable_msg));
  rclc_executor_init(&executor, &support.context, 2, &allocator);
  rclc_executor_add_subscription(&executor, &sub_cmd_vel, &cmd_vel_msg,
    &cmd_vel_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &sub_motor_enable, &motor_enable_msg,
    &motor_enable_callback, ON_NEW_DATA);

  /* Init IMU message (fixed-size fields, use static frame_id) */
  memset(&imu_msg, 0, sizeof(imu_msg));
  imu_msg.header.frame_id.data     = imu_frame_id;
  imu_msg.header.frame_id.size     = sizeof(imu_frame_id) - 1U;
  imu_msg.header.frame_id.capacity = sizeof(imu_frame_id);
  imu_msg.orientation_covariance[0]         = -1.0;  /* unknown */
  imu_msg.angular_velocity_covariance[0]    = -1.0;
  imu_msg.linear_acceleration_covariance[0] = -1.0;

  /* Init JointState message with static buffers (avoid heap) */
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

  /* Init /odom message */
  memset(&odom_msg, 0, sizeof(odom_msg));
  odom_msg.header.frame_id.data     = odom_frame_id;
  odom_msg.header.frame_id.size     = sizeof(odom_frame_id) - 1U;
  odom_msg.header.frame_id.capacity = sizeof(odom_frame_id);
  odom_msg.child_frame_id.data      = odom_child_id;
  odom_msg.child_frame_id.size      = sizeof(odom_child_id) - 1U;
  odom_msg.child_frame_id.capacity  = sizeof(odom_child_id);
  /* Pose covariance diagonal: (x, y, z, roll, pitch, yaw) */
  odom_msg.pose.covariance[0]  = 0.01;   /* x */
  odom_msg.pose.covariance[7]  = 0.01;   /* y */
  odom_msg.pose.covariance[35] = 0.05;   /* yaw */
  /* Twist covariance diagonal: (vx, vy, vz, wx, wy, wz) */
  odom_msg.twist.covariance[0]  = 0.01;  /* linear.x */
  odom_msg.twist.covariance[35] = 0.05;  /* angular.z */

  extern INS_t INS;
  extern float Pitch_deg, Roll_deg, Yaw_deg;
  static float odom_x = 0.0f;
  static float odom_y = 0.0f;
  uint32_t last_pub_fast_ms = 0U;
  uint32_t last_pub_slow_ms = 0U;

  for (;;)
  {
    /* Process /cmd_vel callbacks (non-blocking, DMA transport) */
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

    uint32_t now = HAL_GetTick();

    /* Fast publish: /imu and /euler DISABLED to save UART bandwidth */
    // if ((now - last_pub_fast_ms) >= 20U)
    // {
    //   last_pub_fast_ms = now;
    //   imu_msg.orientation.w         = (double)INS.q[0];
    //   imu_msg.orientation.x         = (double)INS.q[1];
    //   imu_msg.orientation.y         = (double)INS.q[2];
    //   imu_msg.orientation.z         = (double)INS.q[3];
    //   imu_msg.angular_velocity.x    = (double)INS.Gyro[X];
    //   imu_msg.angular_velocity.y    = (double)INS.Gyro[Y];
    //   imu_msg.angular_velocity.z    = (double)INS.Gyro[Z];
    //   imu_msg.linear_acceleration.x = (double)INS.Accel[X];
    //   imu_msg.linear_acceleration.y = (double)INS.Accel[Y];
    //   imu_msg.linear_acceleration.z = (double)INS.Accel[Z];
    //   rcl_publish(&pub_imu, &imu_msg, NULL);
    //   euler_msg.x = (double)Roll_deg;
    //   euler_msg.y = (double)Pitch_deg;
    //   euler_msg.z = (double)Yaw_deg;
    //   rcl_publish(&pub_euler, &euler_msg, NULL);
    // }

    /* Slow publish: /motor_fb at 10 Hz (every 100ms), /odom DISABLED */
    if ((now - last_pub_slow_ms) >= 100U)
    {
      last_pub_slow_ms = now;

      /* /motor_fb */
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
      rcl_publish(&pub_motor_fb, &motor_fb_msg, NULL);
    }

    osDelay(1);
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
    robot_cmd_set((float)msg->linear.x, (float)msg->angular.z, 1U);
  }
}

/* Callback: /motor_enable → motor_enable_set() */
static void motor_enable_callback(const void *msg_in)
{
  const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *)msg_in;
  motor_enable_set(msg->data ? 1U : 0U);
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
/* USER CODE END Application */
