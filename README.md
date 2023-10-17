# realsense_aliengo

dog moves forward and backward according to tag

/home/kangni/realsense_ros_ws/src/realsense-ros/realsense2_camera/scripts

/home/kangni/unitree/unitree_legged_sdk-Aliengo/example_py

https://www.yuque.com/ironfatty/nly1un/htq6286xdtir8fag 配unitree python环境

https://github.com/unitreerobotics/unitree_legged_sdk/releases/tag/v3.8.4 用的是aliengo的sdk

我电脑

export ROS_HOSTNAME=192.168.123.162

export ROS_MASTER_URI=http://192.168.123.162:11311

roscore

我电脑
cd realsense_ros_ws/src/realsense-ros/realsense2_camera/scripts/

python3 aliengo_detect_apriltag.py


sudo kill -9 $(sudo lsof -t -i:8081-8082)

aliengo上

ssh unitree@192.168.123.12

export ROS_HOSTNAME=192.168.123.12

export ROS_MASTER_URI=http://192.168.123.162:11311

cd realsense_ros_ws/src/realsense-ros/realsense2_camera/launch/

roslaunch realsense2_camera rs_aligned_depth.launch 



我电脑

/home/kangni/unitree/unitree_legged_sdk-Aliengo/example_py

python3 example_walk_apriltag_forward_backward.py
