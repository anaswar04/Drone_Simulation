## Drone Camera topic

Following is the code to get ros2 camera topic:
```bash
gz topic -i -t /world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image
ros2 run ros_gz_bridge parameter_bridge /world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image@gz.msgs.Image
ros2 run rqt_image_view rqt_image_view
```
