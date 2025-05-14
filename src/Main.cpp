#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/float64_multi_array.h>
#include <std_msgs/msg/int32.h>

// Global variables
rcl_publisher_t joint_state_pub;
rcl_publisher_t trial_status_pub;
rcl_subscription_t torque_sub;
sensor_msgs__msg__JointState joint_state_msg;
std_msgs__msg__Float64MultiArray torque_msg;
std_msgs__msg__Int32 trial_status_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop() {
  while (1) {
    delay(100);
  }
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    // Fixed joint states (can be made dynamic)
    joint_state_msg.position.data[0] = 0.0;  // q1
    joint_state_msg.position.data[1] = 0.0;  // q2
    joint_state_msg.velocity.data[0] = 0.0;  // dq1
    joint_state_msg.velocity.data[1] = 0.0;  // dq2
    RCSOFTCHECK(rcl_publish(&joint_state_pub, &joint_state_msg, NULL));
  }
}

void subscription_callback(const void *msgin) {
  const std_msgs__msg__Float64MultiArray *msg = (const std_msgs__msg__Float64MultiArray *)msgin;
  if (msg->data.size == 2) {
    Serial.print("Received torques: ");
    Serial.print(msg->data.data[0]);
    Serial.print(", ");
    Serial.println(msg->data.data[1]);
  }
}

void setup() {
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "micro_ros_node", "", &support));

  // Publishers
  RCCHECK(rclc_publisher_init_default(
    &joint_state_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "/joint_states"));
  RCCHECK(rclc_publisher_init_default(
    &trial_status_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "/trial_status"));

  // Subscriber
  RCCHECK(rclc_subscription_init_default(
    &torque_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
    "/torque_commands"));

  // Initialize JointState message
    sensor_msgs__msg__JointState__init(&joint_state_msg);
    joint_state_msg.name.size = 2;
    joint_state_msg.name.capacity = 2;
    joint_state_msg.name.data = (rosidl_runtime_c__String*)malloc(2 * sizeof(rosidl_runtime_c__String));
    const char* joint_names[] = {"joint1", "joint2"};
    for (size_t i = 0; i < 2; i++) {
        size_t len = strlen(joint_names[i]);
        joint_state_msg.name.data[i].data = (char*)malloc(len + 1);
        strcpy(joint_state_msg.name.data[i].data, joint_names[i]);
        joint_state_msg.name.data[i].size = len;
        joint_state_msg.name.data[i].capacity = len + 1;
    }

  // Initialize trial status
  trial_status_msg.data = 1;  // Execute trial

  // Timer for joint states (100 Hz)
  const unsigned int timer_timeout = 10;  // 10 ms
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback));

  // Executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  
  // Add subscription with executor
  std_msgs__msg__Float64MultiArray torque_msg;
  std_msgs__msg__Float64MultiArray__init(&torque_msg); // Optional but recommended
  RCCHECK(rclc_executor_add_subscription(&executor, &torque_sub, &torque_msg, &subscription_callback, ON_NEW_DATA));

  // Publish initial joint states
  RCSOFTCHECK(rcl_publish(&trial_status_pub, &trial_status_msg, NULL));

  // Publish initial trial status
  RCSOFTCHECK(rcl_publish(&joint_state_pub, &joint_status_msg, NULL));
}

void loop() {
  delay(100);

  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}