# ros2_object_analytics
Object Analytics (OA) is ROS2 wrapper for realtime object tracking and 3D localization.
These packages aim to provide real-time object analyses over RGB-D camera inputs, enabling ROS developer to easily create amazing robotics advanced features, like intelligent collision avoidance, people follow and semantic SLAM. It consumes [sensor_msgs::PointClould2](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html) data delivered by RGB-D camera, subscribing topic on [object detection](https://github.com/intel/ros2_object_msgs) by [ros2_intel_movidius_ncs](https://github.com/intel/ros2_intel_movidius_ncs), publishing topics on [object tracking](https://github.com/intel/ros2_object_analytics/tree/master/object_analytics_msgs) in 2D RGB image and [object localization](https://github.com/intel/ros2_object_analytics/object_analytics_msgs) in 3D camera coordination system.

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
  * OpenCV3 & opencv-contrib 3.3 (OA depends on tracking feature from OpenCV Contrib 3.3. OpenCV 3.3 is not integrated in ROS2 Bouncy release, need to build and install Opencv3 with contrib from source to apply tracking feature)
  ```
  # Build and Install OpenCV3 with opencv-contrib
  mkdir ${HOME}/opencv
  cd ${HOME}/opencv
  git clone https://github.com/opencv/opencv.git -b 3.3.0
  git clone https://github.com/opencv/opencv_contrib.git
  mkdir opencv/build -p
  cd opencv/build
  cmake -DOPENCV_EXTRA_MODULES_PATH=${HOME}/opencv/opencv_contrib/modules \ 
        -DCMAKE_INSTALL_PREFIX=/usr/local -DBUILD_opencv_cnn_3dobj=OFF ..
  make -j8
  sudo make install
  sudo ldconfig
  ```

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
#### Realsense
  ```
  # Terminal 1:
  source /opt/ros/bouncy/local_setup.bash
  source ~/ros2_ws/local_setup.bash
  realsense_ros2_camera
  ```
#### NCS
  ```
  # Terminal 2
  source /opt/ros/bouncy/local_setup.bash
  source ~/ros2_ws/local_setup.bash
  echo -e "param_file: mobilenetssd.yaml\ninput_topic: /object_analytics/rgb" > `ros2 pkg prefix movidius_ncs_launch`/share/movidius_ncs_launch/config/default.yaml
  ros2 run composition api_composition &
  launch `ros2 pkg prefix movidius_ncs_launch`/share/movidius_ncs_launch/launch/ncs_stream_launch.py
  ```
#### OA
  ```
  # Terminal 3
  source /opt/ros/bouncy/local_setup.bash
  source ~/ros2_ws/local_setup.bash
  launch `ros2 pkg prefix object_analytics_launch`/share/object_analytics_launch/launch/analytics_object.py
  ```
#### OA Rviz
  ```
  # Terminal 4
  source /opt/ros/bouncy/local_setup.bash
  source ~/ros2_ws/local_setup.bash
  launch `ros2 pkg prefix object_analytics_launch`/share/object_analytics_launch/launch/object_rviz.py
  ```

![OA_demo](https://github.com/intel/ros2_object_analytics/blob/yechun1/images/oa_demo.jpg "OA demo")

![OA_demo_video](https://github.com/intel/ros2_object_analytics/blob/yechun1/images/oa_demo.gif "OA demo video")

## Subscribed topics
  /movidius_ncs_stream/detected_objects ([object_msgs::msg::ObjectsInBoxes](https://github.com/intel/ros2_object_msgs/blob/master/msg/ObjectsInBoxes.msg))

## Published topics
  /object_analytics/rgb ([sensor_msgs::msg::Image](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Image.msg))

  /object_analytics/pointcloud ([sensor_msgs::msg::PointCloud2](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/PointCloud2.msg))

  /object_analytics/localization ([object_analytics_msgs::msg::ObjectsInBoxes3D](https://github.com/intel/ros2_object_analytics/blob/master/object_analytics_msgs/msg/ObjectsInBoxes3D.msg))

  /object_analytics/tracking ([object_analytics_msgs::msg::TrackedObjects](https://github.com/intel/ros2_object_analytics/blob/master/object_analytics_msgs/msg/TrackedObjects.msg))


## Customize launch
  By default, object analytics will launch both tracking and localization features, but either tracking or localization or both can be dropped. Detailed please refer comments embedded in launch file.

## Tools
To ensure the algorims in OA components to archive best performance in ROS2, we have below tools used to examine design/development performance/accuracy/precision..., more tools are in developing progress and will publish later.

### 1. tracker_regression
The tools is used to feed tracking node with raw images from datasets within fixed time interval(33ms), also simulate detector send groundtruth as detections to tracking node for rectification, then receive tracking results for precision and recall stastics. It support multiple algorithms(dynamic configure to tracking node when start).

#### * Tools usages
    # ros2 run object_analytics_node tracker_regression --options
           options: [-a algorithm] [-p dataset_path] [-t dataset_type] [-n dataset_name] [-h];
           -h : Print this help function.
           -a algorithm_name : Specify the tracking algorithm in the tracker.
              supported algorithms: KCF,TLD,BOOSTING,MEDIAN_FLOW,MIL,GOTURN.
           -p dataset_path : Specify the tracking datasets location.
           -t dataset_type : Specify the dataset type: video,image.
           -n dataset_name : Specify the dataset name
#### * Example:

    Video dataset with tracking algorithm("MEDIAN_FLOW"):
    # ros2 run object_analytics_node tracker_regression -p /your/video/datasets/root/path -t video -n dudek -a MEDIAN_FLOW

    Image dataset with default algorithm("MEDIAN_FLOW"):
    # ros2 run object_analytics_node tracker_regression -p /your/image/datasets/root/path -t image -n Biker -a MEDIAN_FLOW

#### * Dataset:

 Support both video and image dataset, but you may need to translate into below formats.

 Video dataset: ([Refer to opencv_extra tracking dataset](https://github.com/opencv/opencv_extra/tree/master/testdata/cv/tracking))

     track_vid/    (/your/video/datasets/root/path)
           ├── david
           │   ├── data
           │   │   └── david.webm
           │   ├── david.yml
           │   ├── gt.txt
           │   └── initOmit
           │       └── david.txt
           ├── dudek
           │   ├── data
           │   │   └── dudek.webm
           │   ├── dudek.yml
           │   ├── gt.txt
           │   └── initOmit
           │       └── dudek.txt
           ├── faceocc2
           │   ├── data
           │   │   └── faceocc2.webm
           │   ├── faceocc2.yml
           │   ├── gt.txt
           │   └── initOmit
           │       └── faceocc2.txt
           ├── list.txt (Note: this is mannually added, list the dataset names which will be used)
           └── README.md

  Image dataset: ([Refer to database from Computer Vision Lab@HYU](http://cvlab.hanyang.ac.kr/tracker_benchmark/datasets.html))

     track_img/    (/your/video/datasets/root/path)
           ├── Biker
           ├── Bird1
           ├── Bird2
           ├── list.txt (Note: this is mannually added, list the dataset names which will be used)
           ├── Man
           ├── Matrix
           └── Woman


###### *Any security issue should be reported using process at https://01.org/security*
