Here's a clean and organized `README.md` file based on the information you provided:

---

```markdown
# ArduPilot + Gazebo + MAVROS (ROS2) Simulation Setup

This guide walks you through setting up a simulation environment using ArduPilot SITL, Gazebo, and MAVROS on ROS2. It covers installation steps and basic command usage to simulate and control a drone.

---

## 📦 Installation Guide

### 1. ArduPilot SITL
Follow the official documentation to download and set up ArduPilot:
🔗 [Downloading the Code / Using Git — ArduPilot Dev Docs](https://ardupilot.org/dev/docs/where-to-get-the-code.html)

### 2. ArduPilot Gazebo Plugin
Refer to the official plugin documentation:
🔗 [ardupilot_gazebo GitHub README](https://github.com/ArduPilot/ardupilot_gazebo/blob/685e3e3d10c0ba33324df2b81474f9ed5c01289f/README.md)

### 3. Gazebo Installation
Install Gazebo with ROS2 integration:
🔗 [Installing Gazebo with ROS — Gazebo Docs](https://gazebosim.org/docs/latest/ros_installation/)

### 4. MAVROS Installation
Set up MAVROS for ROS2 following this guide:
🔗 [Installing MAVROS — ArduPilot Dev Docs](https://ardupilot.org/dev/docs/ros-install.html#installing-mavros)

---

## 🚀 Running the Full Simulation

### 1. Launch Gazebo with Drone World
```bash
gz sim -v4 -r iris_runway.sdf
```

### 2. Start ArduPilot SITL
Navigate to the ArduPilot folder and run:
```bash
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console
```

### 3. Echo Drone State (in another terminal)
```bash
ros2 topic echo /mavros/state
```

### 4. Connect MAVROS (in another terminal)
```bash
ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://:14550@localhost:14550
```

---

## ✈️ Basic MAVROS Control

### Set Mode to GUIDED
```bash
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: 'GUIDED'}"
```

### Arm the Drone
```bash
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
```

### Takeoff (Altitude: 5m)
```bash
ros2 service call /mavros/cmd/takeoff mavros_msgs/srv/CommandTOL "{altitude: 5.0}"
```

---

## 🕹️ Movement Control

### Move Forward Continuously (1 m/s at 10 Hz)
```bash
ros2 topic pub -r 10 /mavros/setpoint_velocity/cmd_vel geometry_msgs/msg/TwistStamped \
"{twist: {linear: {x: 1.0, y: 0.0, z: 0.0}}}"
```

### Stop the Vehicle
```bash
ros2 topic pub /mavros/setpoint_velocity/cmd_vel geometry_msgs/msg/TwistStamped \
"{twist: {linear: {x: 0.0, y: 0.0, z: 0.0}}}"
```

---

## 📷 Viewing the Camera Feed

### Bridge Gazebo Camera Topic to ROS
```bash
ros2 run ros_gz_bridge parameter_bridge \
/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image
```

### View Using RQT Image Viewer
```bash
ros2 run rqt_image_view rqt_image_view
```

### View Using RViz2

1. Launch RViz:
```bash
rviz2
```
2. Click **"Add"**
3. Choose **"Image"**
4. Set topic to:
```
/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image
```
You should now see the live camera feed from the drone.

---

## ✅ Summary

This setup enables you to:
- Simulate a drone with ArduPilot SITL and Gazebo
- Connect and control it using MAVROS on ROS2
- Visualize state and sensor data including camera feed

Make sure to open all required terminals for proper communication between components.
```

---

Let me know if you'd like to add sections for troubleshooting, system requirements, or dependencies.
