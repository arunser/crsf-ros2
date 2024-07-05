<h1>Implementation of CRSF protocol on ROS2 Humble</h1>

CRSF is a telemetry protocol that can be used for both RC control and to get telemetry information from the vehicle/flight controller on a compatible RC transmitter.

<h2>Prerequisites</h2>

- Ubuntu 22.04 LTS
- ROS2 Humble

<h2>Installation Instructions</h2>

```
mkdir ~/crsf_ws
cd crsf_ws
git clone https://github.com/arunser/crsf-ros2.git src
```
```
cd ~/crsf_ws
colcon build
source install/setup.bash
```

To run the CRSF ROS2 node, use the following command;

```
ros2 run crsf_ros2 crsf_ros
```