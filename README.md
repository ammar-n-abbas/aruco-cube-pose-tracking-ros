### Title:
Cube Pose Tracking using ArUco Board and ROS

### README.md:

#### Overview
This ROS (Robot Operating System) node implements ArUco marker detection and pose estimation for a cube in a camera's field of view. It subscribes to camera information and image topics, performs marker detection using OpenCV's ArUco library, and estimates the pose of a cube with ArUco markers. The resulting pose information is published as a `PoseStamped` message and visualized using RViz markers.

#### Prerequisites
- ROS installed
- OpenCV installed (`cv2`, `cv2.aruco`)
- `tf` library installed
- `numpy` library installed

#### Usage
1. Clone this repository into your ROS workspace.
   ```bash
   git clone https://github.com/your-username/aruco-cube-pose-tracking-ros.git
   ```

2. Build the ROS workspace.
   ```bash
   cd path/to/your/ros/workspace
   catkin_make
   ```

3. Run the ROS node.
   ```bash
   roslaunch your_package_name aruco_cube_pose.launch
   ```

#### ROS Topics
- Subscribed Topics:
  - `/camera/color/camera_info` (CameraInfo): Camera information topic.
  - `/camera/color/image_raw` (Image): Raw camera image topic.

- Published Topics:
  - `/aruco_cube/pose` (PoseStamped): Pose of the detected cube.
  - `/visualization_marker_real_cube` (Marker): RViz visualization marker for the cube.

#### Parameters
- Marker Size: The size of the ArUco marker used for detection.
- Cube Size: The size of the cube.

#### Notes
- The node assumes a specific marker dictionary (`DICT_6X6_1000`). Please ensure that this matches your setup.
- Marker visualization in RViz is represented by a cube.
