# Main Camera ROS 2 publisher

`MainCameraCompressedImagePublisher` starts automatically after a scene loads and publishes a UI-free JPEG image from the currently active `MainCamera`.

## Unity configuration

1. Open the project and wait for Package Manager to install ROS-TCP-Connector.
2. Open **Robotics > ROS Settings**.
3. Set **Protocol** to `ROS2`.
4. The publisher connects to `192.168.50.188:10000` by default.
5. Keep `CameraImageDelay` disabled.

Default output:

- Topic: `/rov/camera/image/compressed`
- Type: `sensor_msgs/msg/CompressedImage`
- Size: `1280 x 720`
- Rate: `20 Hz`
- JPEG quality: `85`
- Publisher queue: `1`
- Frame ID: `main_camera_optical_frame`
- Vertical flip: disabled

The publisher follows the camera carrying the `MainCamera` tag, including camera changes made by `MainCameraSwitcher`.

## ROS 2 verification

Start the ROS-TCP-Endpoint on the ROS 2 machine, then run:

```bash
ros2 topic list
ros2 topic hz /rov/camera/image/compressed
ros2 run rqt_image_view rqt_image_view
```

Select `/rov/camera/image/compressed` in `rqt_image_view`.
