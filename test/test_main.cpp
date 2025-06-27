#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <goemetry_msgs/msg/twist.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

rcl_subscriber_t subscriber;
geometry_msgs__msg__Twist msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

//MOTOR
const int MOTOR_A_PWM_PIN = 13;
const int MOTOR_B_PWM_PIN = 32;

const int MOTOR_A_IN1_PIN = 4;
const int MOTOR_A_IN2_PIN = 5;

const int MOTOR_B_IN3_PIN = 18;
const int MOTOR_B_IN4_PIN = 19;

const int PWM_CHANNEL_A = 0;
const int PWM_CHANNEL_B = 1;
const int PWM_FREQUENCY = 5000;
int PWM_RESOLUTION = 8;


// Error handle loop
void error_loop() {
  while(1) {
    delay(100);
  }
}

void cmdVelCallback(const void * msgin) {
  const geometry_msgs__msg__Twist * twist = (const geometry_msgs__msg__Twist *)msgin;

  float linear = twist->linear.x;
  float angular = twist->angular.z;

  if(linear > 0.1){
    move_forward();
  }else if(linear < -0.1){
    move_backward();
  }else if(angular > 0.1){
    turn_left();
  }else if(angular < -0.1){
    turn_right();
  }else{
    
  }

}

void setup() {
  // Configure serial transport
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

  // create publisher
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/cmd_vel"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &subscriber, &msg, &cmdVelCallback, ON_NEW_DATA));

}

void loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}