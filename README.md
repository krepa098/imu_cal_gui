# IMU Calibration GUI

This is a simple GUI that helps you calibrate your IMU (accelerometer, gyroscope and magnetometer) and visualize the measurements before and after calibration.

![screenshot](.media/screenshot.png)

## Features

* Gyro offset calibration
* Accelerometer offset and scale calibration
* Magnetometer soft- and hard-iron calibration
* Save and load measurements [*]
* ROS2 integration to receive sensor messages

[*] Some example measurements are provided in the `test files` folder.

## ROS Topics

This applications receives data from the following ROS topics and follows the conventions described in their respective message definitions:
* `/imu` of type [sensor_msgs/msg/Imu](https://docs.ros2.org/foxy/api/sensor_msgs/msg/Imu.html)
* `/mag` of type [sensor_msgs/msg/MagneticField](https://docs.ros2.org/foxy/api/sensor_msgs/msg/MagneticField.html)

## How to run?

This does require a working ROS2 setup (Foxy or more recent).
Make sure your ROS install is sourced, then run:

```
git clone https://github.com/krepa098/imu_cal_gui.git
cd imu_cal_gui
cargo r --release
```


## Acknowledgements

* The code is in part based on `nliaudat`'s Python implementation found [here](https://github.com/nliaudat/magnetometer_calibration).
* `TESLABS` magnetometer calibration explanations found [here](https://teslabs.com/articles/magnetometer-calibration/)

## License

MIT