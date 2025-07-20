use the following for installing Ardupilot SITL:

[Downloading the Code / Using Git — Dev  documentation](https://ardupilot.org/dev/docs/where-to-get-the-code.html)

Ardupilot gazebo plugin:

https://github.com/ArduPilot/ardupilot_gazebo/blob/685e3e3d10c0ba33324df2b81474f9ed5c01289f/README.md

installing gazebo:

[Installing Gazebo with ROS — Gazebo ionic documentation](https://gazebosim.org/docs/latest/ros_installation/)

installing MAVROS:

[Installing ROS — Dev  documentation](https://ardupilot.org/dev/docs/ros-install.html#installing-mavros)

Full process:

```bash
Run Gazebo

gz sim -v4 -r iris_runway.sdf

inside ardupilot folder run this command 
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console

To check state (in another terminal)

ros2 topic echo /mavros/state

to connect mavros ( in another terminal )

ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://:14550@localhost:14550

in another terminal

ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: 'GUIDED'}"

ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"

ros2 service call /mavros/cmd/takeoff mavros_msgs/srv/CommandTOL "{altitude: 5.0}"

Example: Move Forward Continuously

To move forward at 1 m/s, publish at 10 Hz:

ros2 topic pub -r 10 /mavros/setpoint_velocity/cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 1.0, y: 0.0, z: 0.0}}}"

Stop the Vehicle

To stop, publish zeros:

ros2 topic pub /mavros/setpoint_velocity/cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.0, y: 0.0, z: 0.0}}}"

To see image from camera 

ros2 run ros_gz_bridge parameter_bridge \
/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image
ros2 run rqt_image_view rqt_image_view

then see it in 

ros2 run rqt_image_view rqt_image_view

or in another terminal 

rviz2

click on Add 
Select image 

select the topic  
/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image

from the drop down menu 
you will be able to see the image from the cam
```
