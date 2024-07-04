# April_tag_part_detection

# ROS Package for Object Detection Using AprilTags and YOLO v8

This ROS package utilizes AprilTags and YOLO v8 for robust object detection and TF (Transform) publishing. It is designed to work seamlessly within the ROS ecosystem, providing accurate detection and localization of objects in a robotic environment.

## Features

- **AprilTags Detection**: Detect and identify AprilTags in the environment.
- **YOLO v8 Object Detection**: Leverage the power of YOLO v8 for detecting an specific screw in a part.
- **TF Publishing**: Publish transformations for detected objects, enabling easy integration with other ROS components.

## Prerequisites

Before using this package, ensure you have the following dependencies installed:

- ROS (Robot Operating System) - Tested on ROS Noetic
- OpenCV
- Pytorch (for YOLO v8)
- ROS packages: `tf`, `tf2_ros`, `sensor_msgs, `cv_bridge`
- git clone https://github.com/AprilRobotics/apriltag.git
- git clone https://github.com/AprilRobotics/apriltag_ros.git
- git clone https://github.com/UoS-EEE-Automation/fixture_tracker.git
- git clone https://github.com/MShields1986/realsense_launch.git

## Installation

1. Clone the repository into your ROS workspace:

    ```bash
    cd ~/catkin_ws/src
    git clone https://github.com/SMAminNabipour/April_tag_part_detection.git
    ```

2. Install the required dependencies:

    ```bash
    cd ~/catkin_ws
    rosdep install --from-paths src --ignore-src -r -y
    ```

3. Build the package:

    ```bash
    catkin_make
    source devel/setup.bash
    ```

## Usage

    Launch the part_detection node:

    ```bash
    roslaunch part_detection part_tracker.launch
    ```

## Nodes

### apriltags_detection_node

**Subscribed Topics**:
- `/camera/image_raw` (sensor_msgs/Image): Input image stream from a camera.

**Published Topics**:
- `/apriltags/detections` (apriltags_ros/AprilTagDetectionArray): Detected AprilTags.

### yolo_v8_detection_node

**Subscribed Topics**:
- `/camera/image_raw` (sensor_msgs/Image): Input image stream from a camera.

**Published Topics**:
- `/yolo/detections` (your_package_name/Detections): Detected objects.

### tf_publisher_node

**Subscribed Topics**:
- `/apriltags/detections` (apriltags_ros/AprilTagDetectionArray): Detected AprilTags.
- `/yolo/detections` (your_package_name/Detections): Detected objects.

**Published TFs**:
- `/tf` (tf/tfMessage): Transforms for detected objects.

## Configuration

Configuration files are located in the `config` directory. You can adjust parameters such as camera calibration, detection thresholds, and TF settings in these files.

## Contributing

Contributions are welcome! Please fork the repository and submit a pull request with your changes.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgements

- [AprilTags](https://april.eecs.umich.edu/software/apriltag.html)
- [YOLO v8](https://github.com/ultralytics/yolov8)
- [ROS](https://www.ros.org/)

## Contact

For questions or support, please open an issue or contact [Amin Nabipour](mailto:seyedmohammadamin.nabi-pour@strath.ac.uk).
