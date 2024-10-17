# IMU Calibration GUI

This is a simple GUI that helps you calibrate your IMU and visualize the results.

![screenshot](.media/screenshot.png)

## ROS Topics

This applications receives data from the following ROS topics and follows the conventions described in their respective message definitions:
* `/imu` of type [Imu](https://docs.ros2.org/foxy/api/sensor_msgs/msg/Imu.html)
* `/mag` of type [MagneticField](https://docs.ros2.org/foxy/api/sensor_msgs/msg/MagneticField.html)

## How to run?

This does require a working ROS2 setup (Foxy or more recent).
Make sure your ROS install is sourced, then run:

```
git clone https://github.com/krepa098/imu_cal_gui.git
cd imu_cal_gui
cargo r --release
```
