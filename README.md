#Micro-ROS Robot Interface (ESP32 + MPU9250 + Encoders)

This project provides a micro-ROS-enabled interface for a mobile robot built on the ESP32 platform, using:
- **MPU9250** for IMU data
- **Quadrature encoders** for wheel odometry
- **Motor driver control**
- **MICRO ROS** communication over serial transport

##Hardware Used
- ESP32 Dev Board
- MPU9250 IMU (I2C: SDA - 21, SCL - 22)
- Two quadrature encoders (connected to GPIOs)
- Motor driver (controlled via MotorControl class)
- micro-ROS via serial transport

##ROS 2 Topics
### Subscribed

| Topic      | Type                     | Description            |
|------------|--------------------------|------------------------|
| `/cmd_vel` | `geometry_msgs/Twist`    | Robot velocity command |

### Published

| Topic           | Type                  | Description                  |
|------------------|------------------------|------------------------------|
| `/imu/data`      | `sensor_msgs/Imu`      | IMU readings (gyro & accel) |
| `/encoder_left`  | `std_msgs/Int32`       | Left encoder tick count     |
| `/encoder_right` | `std_msgs/Int32`       | Right encoder tick count    |
