#include <Arduino.h>
#include <Wire.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// ROS2 Message
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32.h>

#include "EncoderSensor.h"
#include "MotorControl.h"
#include "MPU9250.h"

#define SDA_PIN 21
#define SCL_PIN 22
MPU9250 IMU(Wire, 0x68);

// Encoder and Motor
MotorControl motorcontrol;
EncoderSensor encoder_left(15, 2);
EncoderSensor encoder_right(13, 12);

// Parameters
#define speed 190

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;

rcl_publisher_t imu_pub;
rcl_publisher_t enc_left_pub;
rcl_publisher_t enc_right_pub;
rcl_subscription_t cmd_vel_sub;


sensor_msgs__msg__Imu imu_msg;
std_msgs__msg__Int32 enc_left_msg;
std_msgs__msg__Int32 enc_right_msg;
geometry_msgs__msg__Twist cmd_vel_msg;


void cmd_vel_callback(const void *msg_in){
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msg_in;

  float linear_x = msg->linear.x;
  float angular_z = msg->angular.z;

  if(linear_x > 0.0f){
    motorcontrol.forward(speed);
  }else if(linear_x < 0.0f){
    motorcontrol.backward(speed);
  }else if(angular_z > 0.0f){
    motorcontrol.turn_left(speed);
  }else if(angular_z < 0.0f){
    motorcontrol.turn_right(speed);
  }else{
    motorcontrol.stop();
  }
}


void setup(){
  Serial.begin(115200);
  delay(2000);  
  set_microros_serial_transports(Serial);

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);

  rcl_ret_t ret = rclc_node_init_default(&node, "robot_interface_node", "", &support);
  if (ret != RCL_RET_OK) {
    Serial.println("Failed to create node!");
  }

  // Publisher
  rclc_publisher_init_default(&imu_pub, &node, 
  ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "/imu/data");

  rclc_publisher_init_default(&enc_left_pub, &node, 
  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/encoder_left");

  rclc_publisher_init_default(&enc_right_pub, &node, 
  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/encoder_right");

  // Subscriber
    rclc_subscription_init_default(&cmd_vel_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel");


  // Executor
      rclc_executor_init(&executor, &support.context, 1, &allocator);

      rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg,
        &cmd_vel_callback, ON_NEW_DATA);


  Wire.begin(SDA_PIN, SCL_PIN);
  IMU.begin();
  motorcontrol.begin();
  encoder_left.begin();
  encoder_right.begin();

  Serial.println("Robot Starting...");
  delay(3000);

}


void loop(){
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

  IMU.readSensor();

  imu_msg.orientation.x = 0.0;
  imu_msg.orientation.y = 0.0;
  imu_msg.orientation.z = 0.0;
  imu_msg.orientation.w = 0.0;

  imu_msg.angular_velocity.x = IMU.getGyroX_rads();
  imu_msg.angular_velocity.y = IMU.getGyroY_rads();
  imu_msg.angular_velocity.z = IMU.getGyroZ_rads();

  imu_msg.linear_acceleration.x = IMU.getAccelX_mss();
  imu_msg.linear_acceleration.y = IMU.getAccelY_mss();
  imu_msg.linear_acceleration.z = IMU.getAccelZ_mss();

  rcl_publish(&imu_pub, &imu_msg, NULL);

  long left_count = encoder_left.get_count();
  long right_count = encoder_right.get_count();

  enc_left_msg.data = (int32_t) left_count;
  enc_right_msg.data = (int32_t) right_count;

  rcl_publish(&enc_left_pub, &enc_left_msg, NULL);
  rcl_publish(&enc_right_pub, &enc_right_msg, NULL);

  delay(50);  
}