# 🛸 Drone Simulation with ArduPilot, MAVProxy, and Gazebo Harmonic

This repository contains the files and documentation for setting up a drone simulation environment using **ArduPilot SITL**, **MAVProxy**, **Gazebo Harmonic**, and **QGroundControl**. The simulation includes basic drone flight and waypoint navigation using QGroundControl.

## 📖 What This Repo Is About

This project demonstrates:
- Installing and running **ArduPilot SITL (Software-In-The-Loop)** for drone simulation
- Bridging with **MAVProxy** and **Gazebo Harmonic** for visual simulation
- Connecting to **QGroundControl** for mission planning and control
- Running basic waypoint navigation using QGroundControl + ArduPilot SITL

---
## How to Install QGroundControl
```bash
# Download AppImage (64-bit Linux)
wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage

# Make it executable
chmod +x QGroundControl.AppImage

# Run it
./QGroundControl.AppImage
```

## How to Run QGroundControl with SITL using ArduPilot
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

## License

This project is open-source and licensed under the MIT License.
