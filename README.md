#  Drone Simulation with ArduPilot, MAVProxy, and Gazebo Harmonic

This repository contains the files and documentation for setting up a drone simulation environment using **ArduPilot SITL**, **MAVProxy**, **Gazebo Harmonic**, and **QGroundControl**. The simulation includes basic drone flight and waypoint navigation using QGroundControl.

##  1. What This Repo Is About

This project demonstrates:
- Installing and running **ArduPilot SITL (Software-In-The-Loop)** for drone simulation
- Bridging with **MAVProxy** and **Gazebo Harmonic** for visual simulation
- Connecting to **QGroundControl** for mission planning and control
- Running basic waypoint navigation using QGroundControl + ArduPilot SITL

## 2. How to Install QGroundControl
```bash
# Download AppImage (64-bit Linux)
wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage

# Make it executable
chmod +x QGroundControl.AppImage

# Run it
./QGroundControl.AppImage
```

You can also download the latest version from the official site:
https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html

## 3. Installing the required Softwares
To installing Ardupilot SITL:

[Downloading the Code / Using Git — Dev  documentation](https://ardupilot.org/dev/docs/where-to-get-the-code.html)

For Ardupilot gazebo plugin:

https://github.com/ArduPilot/ardupilot_gazebo/blob/685e3e3d10c0ba33324df2b81474f9ed5c01289f/README.md

For installing gazebo:

[Installing Gazebo with ROS — Gazebo ionic documentation](https://gazebosim.org/docs/latest/ros_installation/)

Installing MAVROS:

[Installing ROS — Dev  documentation](https://ardupilot.org/dev/docs/ros-install.html#installing-mavros)

## 4. How to Run QGroundControl with SITL using ArduPilot
1. Run SITL with MAVProxy:
```bash
cd ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map

```
2. Launch QGroundControl:
```bash
./QGroundControl.AppImage
```
note: this must be used from directory where it is downloaded usually `/home/(name)/Downloads/`

3. QGroundControl will automatically connect via UDP on port 14550. You should see the simulated drone appear on the map. You can now use QGroundControl to send commands, upload waypoints, and monitor the drone state in real-time.

working demo 
https://www.youtube.com/watch?v=09Z5kr5EHag


## 5. ArduPilot + Gazebo + MAVROS (ROS 2) Quick Start
This guide walks you through running ArduCopter (Iris) simulation in Gazebo, connecting with MAVROS (ROS 2), and sending basic commands.

1. Run Gazebo

```bash
gz sim -v4 -r iris_runway.sdf
```

2. Start ArduPilot SITL

Navigate to your ArduPilot directory and run:
```bash

sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console
```

3. Check MAVROS State (in another terminal)

```bash
ros2 topic echo /mavros/state
```


4. Connect MAVROS (in another terminal)
```bash
ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://:14550@localhost:14550
```


5. Set Mode, Arm, and Takeoff (in another terminal)

Set flight mode to GUIDED:
```bash
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: 'GUIDED'}"
```

Arm the vehicle:
```bash
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
```


Take off to 5 meters altitude:
```bash
ros2 service call /mavros/cmd/takeoff mavros_msgs/srv/CommandTOL "{altitude: 5.0}"
```


6. Move Forward Continuously

To move forward at 1 m/s, publish at 10 Hz:
```bash
ros2 topic pub -r 10 /mavros/setpoint_velocity/cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 1.0, y: 0.0, z: 0.0}}}"
```


7. Stop the Vehicle

To stop, publish zeros:
```bash
ros2 topic pub /mavros/setpoint_velocity/cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.0, y: 0.0, z: 0.0}}}"
```

### Notes

- Use a new terminal for each major step as indicated.
- Ensure all dependencies (Gazebo, ArduPilot, MAVROS, ROS 2) are installed and sourced.
- Adjust file/model names as needed for your setup.

## 6. Robot Vizualization using RViz

This part contains the final component of the ROS2 drone simulation project focused on **robot visualization using RViz**. The goal of this module is to reflect the drone's movements in Gazebo within RViz by broadcasting pose data and publishing the robot model for visualization.

### Directory structure
```bash
drone_ws/
├── src/
│   └── drone/
|       ├── drone/
│       │   └── __init__.py
│       │   └── pose_to_tf_broadcaster.py
│       ├── launch/
│       │   └── robot_launch.py
│       ├── urdf/
│       │   └── my_robot.urdf
│       └── ...
```


### Building the Workspace
Open a terminal and run the following commands:

```bash
cd ~/drone_ws
colcon build
source ~/drone_ws/install/setup.bash
```

### Launch the Drone
Run the following command to launch the drone:

```bash
ros2 launch drone robot_launch.py
```

### Visualize in RViz

1. Open a new terminal and run:

   ```bash
   rviz2
   ```

2. In the RViz window:
   - Click Add.
   - Select RobotModel.
   - In the *Description Topic*, select /robot_description.
You should see a red box appear in RViz, which mimics the drone in Gazebo.
---

## 7. Drone Camera topic
To see image from camera, run the following command 
```bash
ros2 run ros_gz_bridge parameter_bridge \
/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image
```

then we can use `rqt_image_viewer` to view the topic
```bash
ros2 run rqt_image_view rqt_image_view
```

We can also view this in Rviz. To open Rviz follow the below command
```bash
rviz2
```
Inside Rviz click on `Add` then Select image and then select the topic
```bash  
/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image
```

from the drop down menu 
you will be able to see the image from the cam
