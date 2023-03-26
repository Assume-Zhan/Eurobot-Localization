# Eurobot-Localization
> Simulation on PC

## Setup

### ROS setup
```bash
roscore &
```

### Prepare simulation data
> Odometry raw data to topic [/Toposition]
> IMU raw data to topic [/imu/data_raw]

- Publish odometry data
```bash
rostopic pub -r 50 /Toposition geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```
- Publish imu data
```bash
rostopic pub -r 50 /imu/data sensor_msgs/Imu "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
orientation_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
angular_velocity: {x: 0.0, y: 0.0, z: 0.0}
angular_velocity_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
linear_acceleration: {x: 0.0, y: 0.0, z: 0.0}
linear_acceleration_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
```
- Or just run simulation ( but need to kill by other step )
```bash
./simulation.sh
```
