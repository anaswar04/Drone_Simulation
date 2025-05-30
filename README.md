#  Drone Simulation with ArduPilot, MAVProxy, and Gazebo Harmonic

This repository contains the files and documentation for setting up a drone simulation environment using **ArduPilot SITL**, **MAVProxy**, **Gazebo Harmonic**, and **QGroundControl**. The simulation includes basic drone flight and waypoint navigation using QGroundControl.

##  1. What This Repo Is About

This project demonstrates:
- Installing and running **ArduPilot SITL (Software-In-The-Loop)** for drone simulation
- Bridging with **MAVProxy** and **Gazebo Harmonic** for visual simulation
- Connecting to **QGroundControl** for mission planning and control
- Running basic waypoint navigation using QGroundControl + ArduPilot SITL

---
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

