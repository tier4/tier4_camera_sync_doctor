# tier4_camera_sync_doctor

This ROS package provides a diagnostic node to monitor and analyze the synchronization between camera data (image timestamps) and trigger signals.  It's designed for applications requiring precise timing between camera capture and external events (like LiDAR scans or control signals), such as autonomous driving or robotics.

## Overview

The `Tier4CameraSyncDoctor` node subscribes to camera info messages (`sensor_msgs/msg/CameraInfo`) and trigger messages (`builtin_interfaces/msg/Time`). It calculates timing differences, reports synchronization status, and provides diagnostic information to help identify and resolve synchronization issues.

## Installation

1. **Clone:** Clone the package into the ROS workspace

   ```bash
   mkdir src
   git clone https://github.com/tier4/tier4_camera_sync_doctor.git src/tier4_camera_sync_doctor
   ```

2.  **Build:** Build the package with:

    ```bash
    colcon build --symlink-install -cmake-args -DCMAKE_BUILD_TYPE=Release
    ```

3.  **Source:** Source the setup file:

    ```bash
    source install/setup.bash
    ```

## Usage

1.  **Launch:** Syntax looks as follows.

    ```bash
    ros2 run tier4_camera_sync_doctor tier4_camera_sync_doctor_node --ros-args\
      -p v4l2_param_path:=<V4L2_PARAM_FILE_PATH> \
      -p trigger_param_path:=<TRIGGER_PARAM_FILE_PATH> \
      -p readout_delay_param_path:=<READOUT_DELAY_PARAM_FILE_PATH> \
      -r /tier4_camera_sync_doctor_node/input/camera_info:=/camera0/camera_info \
      -r /tier4_camera_sync_doctor_node/input/trigger_time:=/camera0/trigger_time
    ```

2.  **Observe:** Observe the diagnostic messages:

    ```bash
    ros2 topic echo /diagnostics
    # or
    ros2 run rqt_runtime_monitor rqt_runtime_monitor
    ```

## Node Parameters

| Parameter Name              | Units   | Default Value      | Description                                                                                                                                    |
|-----------------------------|---------|--------------------|------------------------------------------------------------------------------------------------------------------------------------------------|
| `v4l2_param_path`           | string  | `""`               | File path for camera behavior configuration. The following parameters are read from this file: <ul><li>`timestamp_offset` (optional)</li></ul> |
| `trigger_param_path`        | string  | `""`               | File path for trigger timing configuration. The following parameters are read from this file: <ul><li>`frame_rate`</li><li>`phase`</li></ul>   |
| `readout_delay_param_path`  | string  | `""`               | File path for readout delay configuration. The following parameters are read from this file: <ul><li>`delay_ms`</li></ul>                      |
| `trigger_time_tolerance_ns` | int64_t | `1000000` (1 ms)   | Maximum allowed time difference (nanoseconds) between observed and ideal trigger time.                                                         |
| `camera_time_tolerance_ns`  | int64_t | `10000000` (10 ms) | Maximum allowed time difference (nanoseconds) between observed trigger time and camera timestamp.                                              |
| `hardware_id`               | string  | `""`               | Hardware ID to use in diagnostic messages.                                                                                                     |




## Contributing

Contributions are welcome! Please submit pull requests with clear explanations of your changes.
