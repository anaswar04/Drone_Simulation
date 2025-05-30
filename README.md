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

## 3. How to Run QGroundControl with SITL using ArduPilot
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


## 4. ArduPilot + Gazebo + MAVROS (ROS 2) Quick Start

This guide walks you through running ArduCopter (Iris) simulation in Gazebo, connecting with MAVROS (ROS 2), and sending basic commands.

1. Run Gazebo

```bash
gz sim -v4 -r iris_runway.sdf
```

2. Start ArduPilot SITL

  _Navigate to your ArduPilot directory and run:_
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

  _Set flight mode to GUIDED:_
```bash
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: 'GUIDED'}"
```

  _Arm the vehicle:_
```bash
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
```


  _Take off to 5 meters altitude:_
```bash
ros2 service call /mavros/cmd/takeoff mavros_msgs/srv/CommandTOL "{altitude: 5.0}"
```


6. Move Forward Continuously

  _To move forward at 1 m/s, publish at 10 Hz:_
```bash
ros2 topic pub -r 10 /mavros/setpoint_velocity/cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 1.0, y: 0.0, z: 0.0}}}"
```


7. Stop the Vehicle

  _To stop, publish zeros:_
```bash
ros2 topic pub /mavros/setpoint_velocity/cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.0, y: 0.0, z: 0.0}}}"
```

### Notes

- Use a new terminal for each major step as indicated.
- Ensure all dependencies (Gazebo, ArduPilot, MAVROS, ROS 2) are installed and sourced.
- Adjust file/model names as needed for your setup.

## 5. Robot Vizualization using RViz

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
### How to Run

1. Launch ArduPilot
```bash
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console
```
2. Start Gazebo Simulation
```bash
gz sim -v4 -r iris_runway.sdf
```

3. Source workspace:
```bash
cd ~/drone_ws
source install/setup.bash
```

4. Start MAVROS :
```bash
ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://:14550@localhost:14550
```

5. Launch the RViz integration:
```bash
ros2 launch my_drone_pkg drone_rviz.launch.py
```
The launch file starts `pose_to_tf_broadcaster` custom node that reads from `/mavros/local_position/pose` and broadcasts TF from map → base_link.
The `robot\_state\_publisher` node is responsible for publishing the robot model's transform data to the TF tree based on the `robot\_description` parameter, which typically references a URDF file. 

This setup allows RViz2 to correctly visualize the robot's structure and movement within the defined coordinate frames. To reflect the actual drone motion simulated via MAVROS, a custom ROS 2 node subscribes to the `/mavros/local\_position/pose` topic, which provides the drone’s pose in the form of `geometry\_msgs/PoseStamped` messages. The node then converts this pose data into a TF transform and broadcasts it from the map frame to the `base\_link` frame using `tf2\_ros`.TransformBroadcaster. 

Importantly, the node is configured with a QoS profile set to SensorData (BestEffort + Volatile), ensuring compatibility with MAVROS’s publisher settings. This mechanism is essential for bridging the drone’s position output from MAVROS into the TF system so that RViz can interpret and display the drone’s real-time movement accurately.This is essential to bridge the position output from MAVROS into TF for RViz to interpret.

6. Runnning a robot_state_publisher
```bash
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat {location-to-the-urdf-file}urdf/my_robot.urdf)"
```

The `robot\_state\_publisher` node is used to publish the kinematic transform tree derived from the drone’s URDF model. This model is loaded by passing the contents of the URDF file to the `robot\_description` parameter, typically using a command like `ros2 run robot\_state\_publisher robot\_state\_publisher --ros-args -p robot\_description:="\$(cat urdf/my\_robot.urdf)"`. The node establishes the `base\_link` frame as the root of the model, which aligns with the TF transform being broadcast by the pose-to-TF node. This alignment ensures that RViz can correctly anchor and render the drone's model in the visual space.

In RViz, the fixed frame is set to map, which acts as the global reference frame. Visual elements such as the TF tree and the RobotModel can be added to the scene. With this setup, the drone model appears in RViz and follows the real-time motion data being streamed through TF, allowing accurate visualization of the drone’s movements as simulated by MAVROS.

### Verification

Check TF broadcast:
```bash
ros2 topic echo /tf
```

You should see transforms like:

```bash
header:
  |-frame_id: map
child_frame_id: base_link
transform:
  |-translation: {x: ..., y: ..., z: ...}
  |-rotation: {x: ..., y: ..., z: ..., w: ...}
```
