# ros2_object_analytics
Object Analytics (OA) is ROS2 wrapper for realtime object 3D localization.
These packages aim to provide real-time object analyses over RGB-D camera inputs, enabling ROS developer to easily create amazing robotics advanced features, like intelligent collision avoidance, people follow and semantic SLAM. It consumes [sensor_msgs::PointClould2](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html) data delivered by RGB-D camera, subscribing topic on [object detection](https://github.com/intel/ros2_object_msgs) by [ros2_intel_movidius_ncs](https://github.com/intel/ros2_intel_movidius_ncs), publishing topics on [object localization](https://github.com/intel/ros2_object_analytics/object_analytics_msgs) in 3D camera coordination system.

![OA_Architecture](https://github.com/intel/ros2_object_analytics/blob/master/images/oa_architecture.png "OA Architecture")

OA keeps integrating with various "state-of-the-art" algorithms.
* Object detection offload to VPU, Intel Movidius NCS, with MobileNet SSD model and Caffe framework.

## System Requirements
We support Ubuntu Linux Bionic Beaver 18.04 on 64-bit. We not support Mac OS X and Windows.

## Hardware Requirements

* Intel NUC (CPU: Intel i7-6700HQ @2.60GHz, Mem:16G)
* Intel Movidius Neural Compute Stick
* Intel RealSense D435/D415

## Dependencies
  Install ROS2 packages [ros-bouncy-desktop](https://github.com/ros2/ros2/wiki/Linux-Install-Debians)
  * ament_cmake
  * std_msgs
  * sensor_msgs
  * geometry_msgs
  * rclcpp
  * rosidl_default_generators
  * rosidl_interface_packages
  * launch
  * ros2run
  * class_loader
  * pcl_conversions
  * [cv_bridge](https://github.com/ros-perception/vision_opencv/tree/ros2/cv_bridge)
  * [object_msgs](https://github.com/intel/ros2_object_msgs)
  * [ros2_message_filters](https://github.com/ros2/message_filters)
  * [ros2_intel_realsense](https://github.com/intel/ros2_intel_realsense) (The only supported RGB-D camera by now is Intel RealSense)
  * [ros2_intel_movidius_ncs](https://github.com/intel/ros2_intel_movidius_ncs) (Movidius NCS is the only supported detection backend)

  Other non-ROS debian packages
  * libpcl-dev
  * python3-numpy

## Get Code
  ```bash
  mkdir ~/ros2_ws/src -p
  cd ~/ros2_ws/src

  git clone https://github.com/ros-perception/vision_opencv.git -b ros2
  git clone https://github.com/intel/ros2_object_msgs.git
  git clone https://github.com/ros2/message_filters.git
  git clone https://github.com/intel/ros2_intel_realsense.git
  git clone https://github.com/intel/ros2_intel_movidius_ncs.git
  git clone https://github.com/intel/ros2_object_analytics.git
  ```
  
## Build
  ```bash
  cd ~/ros2_ws/src
  source /opt/ros/bouncy/local_setup.bash
  cd ${ros2_ws} # "ros2_ws" is the workspace root directory where this project is placed in
  colcon build --symlink-install
  ```

## Run
#### Configure NCS default.yaml
  ```
  source /opt/ros/bouncy/local_setup.bash
  source ~/ros2_ws/local_setup.bash
  echo -e "param_file: mobilenetssd.yaml\ninput_topic: /object_analytics/rgb" > `ros2 pkg prefix movidius_ncs_launch`/share/movidius_ncs_launch/config/default.yaml
  ```
#### Start OA Demo
  ```
  source /opt/ros/bouncy/local_setup.bash
  source ~/ros2_ws/local_setup.bash
  ros2 launch object_analytics_node object_analytics.launch.py
  ```
![OA_demo_video](https://github.com/intel/ros2_object_analytics/blob/master/images/oa_demo.gif "OA demo video")

## Subscribed topics
  /movidius_ncs_stream/detected_objects ([object_msgs::msg::ObjectsInBoxes](https://github.com/intel/ros2_object_msgs/blob/master/msg/ObjectsInBoxes.msg))

## Published topics
  /object_analytics/rgb ([sensor_msgs::msg::Image](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Image.msg))

  /object_analytics/pointcloud ([sensor_msgs::msg::PointCloud2](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/PointCloud2.msg))

  /object_analytics/localization ([object_analytics_msgs::msg::ObjectsInBoxes3D](https://github.com/intel/ros2_object_analytics/blob/master/object_analytics_msgs/msg/ObjectsInBoxes3D.msg))


###### *Any security issue should be reported using process at https://01.org/security*
