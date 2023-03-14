# Local filter 

- Local filter ( with odometry + imu )

## With all firmware

```xml
roslaunch local_filter local_filter.launch
``` 

## Without firmware

```xml
roslaunch local_filter local_filter_no_firmware.launch
```

## Only without rosserial communication

```xml
roslaunch local_filter local_filter_no_comm.launch
```
