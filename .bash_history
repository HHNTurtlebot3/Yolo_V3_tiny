roscore
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl # if you haven't already installed curl
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
apt list --upgradable
sudo apt install ros-melodic-desktop-full
apt search ros-melodic
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo apt install python-rosdep
sudo apt autoremove
sudo apt install python-rosdep
sudo rosdep init
rosdep update
clear
mkdir -p ~/catkin_ws/src
cd catkin_ws/
catkin_make
source devel/setup.bash
echo $ROS_PACKAGE_PATH
cd src/
catkin_create_pkg depth_recognition std_msgs rospy roscpp img_msgs cv_bridge
cd
roscore
sudo apt-get install ros-melodic-rqt-common-plugins
rosrun rqt_graph rqt_graph
touch depth_recognition.py
open depth_recognition.py
import cv2
dpkg -l | grep libopencv
pkg-config --modversion opencv
pkg-config --cflags opencv
pkg-config --libs opencv
dpkg -l | grep libcv2
clear
python3
python
echo $PYTHONPATH
ls
chmod +rwx cv_bridgeConfig.cmake
sudo chmod +rwx cv_bridgeConfig.cmake
ls
sudo chmod -rwx cv_bridgeConfig.cmake
ls
sudo chmod -w cv_bridgeConfig.cmake
sudo chmod +w cv_bridgeConfig.cmake
sudo chmod +rwx cv_bridgeConfig.cmake
ls
sudo chmod -rwx cv_bridgeConfig.cmake
ls
sudo chmod -rw cv_bridgeConfig.cmake
ls
sudo chmod +rw cv_bridgeConfig.cmake
git clone https://github.com/IntelligentRoboticsLabs/gb_visual_detection_3d.git
git clone --single-branch --branch melodic https://github.com/IntelligentRoboticsLabs/gb_visual_detection_3d.git
git clone --single-branch --branch melodic https://github.com/IntelligentRoboticsLabs/gb_visual_detection_3d_msgs/tree/master
git clone --single-branch --branch melodic https://github.com/IntelligentRoboticsLabs/gb_visual_detection_3d_msgs
catkin_make
cd ..
catkin_make
cd catkin_ws/
catkin_make
cd catkin_ws/
catkin_make
cd
ls
clear
mkdir -p ~/catkin_ws/src
cd catkin_ws/
catkin_make
source devel/setup.bash
echo $ROS_PACKAGE_PATH
cd
nvidia-smi
nvcc --version
cat /usr/local/cuda/version.txt
git clone https://github.com/IntelRealSense/realsense-ros.git
cd realsense-ros/
git checkout `git tag | sort -V | grep -P "^2.\d+\.\d+" | tail -1`
cd ..
cd --
catkin_make
cd catkin_ws/
catkin_make
chmod +wr cv_bridgeConfig.cmake
sudo chmod +wr cv_bridgeConfig.cmake
ls
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
sudo apt-get install librealsense2-utils
sudo apt-get install librealsense2-dev
pck-config --modversion opencv
dpkg -l | grep libopencv
cmake --version
git clone -b melodic --single-branch https://github.com/ros-perception/vision_opencv
cd ..
catkin_make
python
nvcc --version
cat /usr/local/cuda/version.txt
which g++
gcc --version
g++ --version
cmake --version
catkin clean
catkin_clean
catkin_make clean
cd src/
git clone https://github.com/pal-robotics/ddynamic_reconfigure
cd ..
catkin_make clean
catkin_make
catkin_make clean
catkin_make
catkin_make clean
catkin_make
sudo vim cv_bridgeConfig.cmake
roslaunch rs_camera.launch
catkin_make clean
cd ..
catkin_make clean
clear
roslaunch darknet_ros yolo_v3_tiny.launch
roscore
catkin_make
source devel/setup.bash
echo $ROS_PACKAGE_PATH
source ~/.bashrc
cd
roslaunch darknet_ros yolo_v3_tiny.launch
source ~/.bashrc
cd catkin_ws/
source devel/setup.bash
cd
roslaunch darknet_ros yolo_v3_tiny.launch
roslaunch realsense2_camera rs_camera.launch
roslaunch yolo_v3_tiny.launch
sudo vim point_cloud.h
git clone -b melodic --single-branch https://github.com/IntelligentRoboticsLabs/gb_visual_detection_3d
git clone -b melodic --single-branch https://github.com/IntelligentRoboticsLabs/gb_visual_detection_3d_msgs
cd ..
catkin_make
catkin_make -j1
catkin_make 
roslaunch realsense2_camera rs_camera.launch
modinfo uvcvideo | grep "version:"
cmake --version
modinfo uvcvideo | grep "version:" should include realsense string
modinfo uvcvideo | grep "version:"
roslaunch rs_camera.launch
realsense-viewer
roslaunch rs_camera.launch
cd
git clone https://github.com/Microsoft/vcpkg.git
cd vcpkg
./bootstrap-vcpkg.sh
./vcpkg integrate install
./bootstrap-vcpkg.sh
sudo apt-get install librealsense2-dkms
sudo apt-get update
sudo apt-get upgrade
./bootstrap-vcpkg.sh
sudo apt-get update && sudo apt-get upgrade && sudo apt-get dist-upgrade
roslaunch realsense2_camera rs_camera.launch
sudo apt-get update
sudo apt-get upgrade
realsense-viewer
roscore
sudo apt-get install ros-melodic-joy ros-melodic-teleop-twist-joy   ros-melodic-teleop-twist-keyboard ros-melodic-laser-proc   ros-melodic-rgbd-launch ros-melodic-depthimage-to-laserscan   ros-melodic-rosserial-arduino ros-melodic-rosserial-python   ros-melodic-rosserial-server ros-melodic-rosserial-client   ros-melodic-rosserial-msgs ros-melodic-amcl ros-melodic-map-server   ros-melodic-move-base ros-melodic-urdf ros-melodic-xacro   ros-melodic-compressed-image-transport ros-melodic-rqt*   ros-melodic-gmapping ros-melodic-navigation ros-melodic-interactive-markers
sudo apt-get install ros-melodic-dynamixel-sdk
sudo apt-get install ros-melodic-turtlebot3-msgs
sudo apt-get install ros-melodic-turtlebot3
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
ifconfig
nano ~/.bashrc
sudo apt install nano
nano ~/.bashrc
ifconfig
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
sudo apt install curl
sudo apt install ros-melodic-desktop-full
apt search ros-melodic
source ~/.bashrc
sudo apt install python-rosdep
sudo rosdep init
rosdep update
nano ~/.bashrc
source ~/.bashrc
roscore
nano ~/.bashrc
source ~/.bashrc
clear
roscore
sudo dpkg --add-architecture armhf
sudo apt-get update
sudo apt-get install libc6:armhf
export OPENCR_PORT=/dev/ttyACM0
export OPENCR_MODEL=burger
rm -rf ./opencr_update.tar.bz2
wget https://github.com/ROBOTIS-GIT/OpenCR-Binaries/raw/master/turtlebot3/ROS1/latest/opencr_update.tar.bz2
tar -xvf opencr_update.tar.bz2
cd ./opencr_update
exit
cd ./opencr_update
./update.sh $OPENCR_PORT $OPENCR_MODEL.opencr
cd
clear
sudo dpkg --add-architecture armhf
sudo apt-get update
sudo apt-get install libc6:armhf
export OPENCR_PORT=/dev/ttyACM0
export OPENCR_MODEL=burger
rm -rf ./opencr_update.tar.bz2
wget https://github.com/ROBOTIS-GIT/OpenCR-Binaries/raw/master/turtlebot3/ROS1/latest/opencr_update.tar.bz2
tar -xvf opencr_update.tar.bz2
cd ./opencr_update
./update.sh $OPENCR_PORT $OPENCR_MODEL.opencr
cd
clear
sudo dpkg --add-architecture armhf
sudo apt-get update
export OPENCR_PORT=/dev/ttyACM0
export OPENCR_MODEL=burger
tar -xvf opencr_update.tar.bz2
cd ./opencr_update
./update.sh $OPENCR_PORT $OPENCR_MODEL.opencr
cd
clear
roslaunch turtlebot3_bringup robot.launch
roslaunch turtlebot3_bringup turtlebot3_robot.launch
sudo chmod a+rw /dev/ttyACM0
roslaunch turtlebot3_bringup turtlebot3_robot.launch
exit
sudo apt-get upgrade
sudo apt install ros-melodic-rosserial-python ros-melodic-tf
cd catkin_ws/srx
cd catkin_ws/src
sudo apt install ros-melodic-hls-lfcd-lds-drive
sudo apt install ros-melodic-hls-lfcd-lds-driver
sudo apt install ros-melodic-turtlebot3-msgs
sudo apt install ros-melodic-dynamixel-sdk
git clone -b melodic-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
cd ~/catkin_ws/src/turtlebot3
nano ~/.bashrc
cd ..
catkin_make
cd src/
git clone https://github.com/ROBOTIS-GIT/turtlebot3_applications.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_applications_msgs.git
cd ..
catkin_make
clear
cd
clear
cd catkin_ws/
catkin_make
roslaunch turtlebot_follower follower.launch
cd
roslaunch turtlebot3_follower turtlebot3_follower.launch
exit
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_follow_filter turtlebot3_follow_filter.launch
roslaunch turtlebot3_bringup turtlebot3_robot.launch
exit
roslaunch turtlebot3_follower.launch 
pip install sklearn
pip2 install sklearn
pip3 install sklearn
exit
cd catkin_ws/src
git clone https://github.com/ros-perception/laser_filters.git
cd ..
catkin_make
cd
roslaunch turtlebot3_follow_filter turtlebot3_follow_filter.launch
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-melodic-desktop-full
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt install python-rosdep
sudo rosdep init
rosdep update
roslaunch turtlebot3_follower turtlebot3_follower.launch
cd catkin_ws/
catkin_make
cd ..
roslaunch turtlebot3_follow_filter turtlebot3_follow_filter.launch
roslaunch turtlebot3_follow_filter.launch 
roslaunch turtlebot3_follower.launch 
cd
clear
exit
roslaunch turtlebot3_bringup turtlebot3_robot.launch
sudo pip install scikit-learn
pip3 install scikit-learn
python2
pip install sklearn
pip install numpy
sudo apt-get install python-pip
sudo pip install -U scikit-learn numpy scipy
sudo pip install -U scikit-learn
sudo pip install --upgrade pip
roscore
cd catkin_ws/src
git clone https://github.com/ROBOTIS-GIT/turtlebot3_applications.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_applications_msgs.git
cd ..
catkin_make -j4
sudo apt-get install ros-melodic-ar-track-alvar
catkin_make -j4
catkin_make
cd
clear
sudo apt-get install ros-melodic-ar-track-alvar-msgs
sudo apt-get install ros-melodic-ar-track-alvar
clear
sudo apt-get install python-pip
sudo pip install --upgrade pip
roscore
roscore#
roscore
exit
clear
roslaunch turtlebot3_bringup turtlebot3_robot.launch
sudo chmod 666 /dev/ttyACM0  
roslaunch turtlebot3_bringup turtlebot3_robot.launch
roslaunch realsense2_camera rs_camera.launch
exit
roslaunch turtlebot3_bringup turtlebot3_robot.launch
exit
roscore
sudo reboot -h o
cd catkin_ws/src
git clone https://github.com/catkin/catkin_simple.git
git clone https://github.com/ethz-asl/glog_catkin.git
git clone https://github.com/ethz-asl/ethzasl_msf.git
catkin build
cd ..
catkin_make
cd
roslaunch 
clear
roslaunch rs_camera.launch 
cd catkin_ws/src
git clone https://github.com/JetsonHacksNano/installLibrealsense.git
cd ..
cd src/
cd installLibrealsense/
./installLibrealsense.sh
cd .
cd ..
git clone https://github.com/JetsonHacksNano/installRealSenseROS.git
cd installRealSenseROS/
./installRealSenseROS.sh catkin_ws
cd
clear
sudo apt-get install ros-melodic-xacro
exit
roslaunch darknet_ros darknet_ros.launch
rviz
exit
source ~/.bashrc
roslaunch realsense2_camera rs_camera.launch
exit
roslaunch realsense2_camera rs_camera.launch
roslaunch darknet_ros yolo_v3_tiny.launch
sudo apt-get update
sudo apt-get upgrade
clkear
clear
sudo apt-get update
sudo apt-get upgrade
/usr/bin/python3 /home/nvidia/Downloads/Mauszeiger.py
usr/bin/python3 /home/nvidia/Downloads/objTracking.py
/usr/bin/python3 /home/nvidia/Downloads/Mauszeiger.py
git clone https://github.com/JetsonHacksNano/installVSCode.git
cd installVSCode
./installVSCode.sh
sudo apt install libcanberra-gtk-module libcanberra-gtk3-module
pip install numpy
pip install matplotlib
sudo apt install python3-pip
pip3 --version
cd ..
pip --version
pip3 --version
pip3 install numpy
sudo apt-get install python3.6-dev
sudo apt install python3-pip
pip3 install numpy
pip3 install cython
pip3 install numpy
python3
exit
sudo apt-get update
realsense-viewer
sudo apt-get upgrade
pip3 install matplotlib
sudo apt-get install python3-pip
sudo pip3 install -U pip testresources setuptools==49.6.0 
sudo pip3 install -U --no-deps numpy==1.19.4 future==0.18.2 mock==3.0.5 keras_preprocessing==1.1.2 keras_applications==1.0.8 gast==0.4.0 protobuf pybind11 cython pkgconfig
udo env H5PY_SETUP_REQUIRES=0 pip3 install -U h5py==3.1.0
sudo env H5PY_SETUP_REQUIRES=0 pip3 install -U h5py==3.1.0
sudo pip3 install --pre --extra-index-url https://developer.download.nvidia.com/compute/redist/jp/v46 tensorflow
exit
/usr/bin/python3 /home/nvidia/Downloads/Mauszeiger.py
/usr/bin/python3 /home/nvidia/Downloads/objTracking.py
/usr/bin/python3 /home/nvidia/Downloads/camera.py
/usr/bin/python3 /home/nvidia/Downloads/objTracking.py
/usr/bin/python3 /home/nvidia/Downloads/Detector.py
/usr/bin/python3 /home/nvidia/Downloads/KalmanFilter.py
/usr/bin/python3 /home/nvidia/Downloads/objTracking.py
/usr/bin/python3 /home/nvidia/Downloads/camera.py
pip --version
pip3 --version
roslaunch realsense2_camera rs_aligned_depth.launch
roslaunch darknet_ros yolo_v3_tiny.launch
/usr/bin/python3 /home/nvidia/Downloads/objTracking.py
/usr/bin/python3 /home/nvidia/depth_recognition.py
/usr/bin/python /home/nvidia/depth_recognition.py
/usr/bin/python3 /home/nvidia/depth_recognition.py
/usr/bin/python /home/nvidia/depth_recognition.py
/usr/bin/python /home/nvidia/depth_recognition.py/usr/bin/python /home/nvidia/depth_recognition.py
/usr/bin/python /home/nvidia/depth_recognition.py
/usr/bin/python /home/nvidia/depth_recognition.py/usr/bin/python /home/nvidia/depth_recognition.py
/usr/bin/python /home/nvidia/depth_recognition.py
roslaunch darknet_ros yolo_v3_tiny.launch
roslaunch realsense2_camera rs_aligned_depth.launch
roscore
clear
roslaunch realsense2_camera rs_aligned_depth.launch
rostopic list
rostopic echo /darknet_ros/bounding_boxes 
/usr/bin/python /home/nvidia/Downloads/fps_check.py
/usr/bin/python /home/nvidia/Downloads/camera.py
/usr/bin/python /home/nvidia/depth_recognition.py
/usr/bin/python /home/nvidia/BB.py
/usr/bin/python /home/nvidia/BB_Kevin.py
/usr/bin/python /home/nvidia/darknet.py
git clone https://github.com/IntelligentRoboticsLabs/gb_visual_detection_3d_msgs.git
git clone https://github.com/IntelligentRoboticsLabs/gb_visual_detection_3d.git
cd ..
catkin_make
roslaunch darknet_ros yolo_v3_tiny.launch 
rostopic list
roslaunch darknet_ros yolo_v3_tiny.launch 
exit
rostopic info /camera/depth/image_rect_raw
rostopic echo /darknet_ros/bounding_boxes
rostopic list
rostopic info /distance/distance_BB 
rostopic info /tf
rostopic echo /distance/distance_BB 
rostopic info /distance/distance_BB 
rostopic echo /distance/distance_BB 
/usr/bin/python /home/nvidia/BB_Kevin.py
clear
/usr/bin/python /home/nvidia/BB_Kevin.py
roslaunch realsense2_camera rs_aligned_depth.launch 
realsense-viewer
roslaunch realsense2_camera rs_aligned_depth.launch 
realsense-viewer
sudo apt-get install librealsense2-dkms
(ds5-options.cpp:88) Asic Temperature value is not valid!
sudo apt-get install librealsense2-dkms
sudo apt-get install librealsense2-utils
sudo apt-get install librealsense2-dev
modinfo uvcvideo | grep "version:"
sudo apt-get update
sudo apt-get upgrade
roslaunch darknet_ros yolo_v3_tiny.launch
/usr/bin/python /home/nvidia/depth_recognition.py
roslaunch realsense2_camera rs_aligned_depth.launch 
rostopic info /camera/aligned_depth_to_color/image_raw
rostopic echo /darknet_ros/bounding_boxes
sudo apt-get update
sudo apt-get upgrade
sudo apt autoremove
/usr/bin/python /home/nvidia/depth_recognition.py
/usr/bin/python /home/nvidia/BB_Kevin.py
/usr/bin/python /home/nvidia/depth_recognition.py
rviz
roslaunch darknet_ros yolo_v3_tiny.launch
roslaunch realsense2_camera rs_aligned_depth.launch 
realsense-viewer
roslaunch realsense2_camera rs_aligned_depth.launch 
cmake --version
cd catkin_ws/
cd src/
git clone https://github.com/yasenh/sort-cpp
cd ..
catkin_make
realsense-viewer
exit
sudo apt-get update
sudo apt-get upgrade
sudo apt-get update
sudo apt-get upgrade
cd catkin_ws/src
ls
cd ..
catkin_make
catkin_make -j4
cd src/
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
cd src/
cd ..
catkin_make
cd
roslaunch turtlebot3_follow
roslaunch turtlebot3_follow_filter turtlebot3_follow_filter.launch model model 
roslaunch turtlebot3_follow_filter turtlebot3_follow_filter.launch
cd catkin_ws/src/
git clone https://github.com/ros-perception/laser_filters.git
cd ..
catkin_make
sudo shuthown -h 0
sudo shutdown -h 0
git clone https://github.com/JetsonHacksNano/installRealSenseROS.git
cd installRealSenseROS/
./installRealSenseROS.sh 
cd
cd catkin_ws/
catkin_make
apt-get update
sudo apt-get update
apt-get ros-melodic-ddynamic-reconfigure
sudo apt-get ros-melodic-ddynamic-reconfigure
catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
clear
exit
sudo apt-get install librealsense2-dkms
sudo apt-get install librealsense2-utils
sudo apt-get install librealsense2-dev
sudo apt-get install librealsense2-dbg
sudo find / -name librealsense2.so*
sudo apt-get install librealsense2-dkms
sudo apt-get install librealsense2-dev
clear
cd catkin_ws/
catkin_make
clear
cd
cd catkin_ws/src/
git clone https://github.com/ros-perception/depthimage_to_laserscan.git
cd ..
catkin_make
roslaunch turtlebot3_follow_filter turtlebot3_follow_filter.launch 
roslaunch turtlebot3_follower turtlebot3_follower.launch 
git clone https://github.com/ros-perception/laser_filters.git
cd ..
catkin_make
catkin_make#
catkin_make
dpkg -l | grep "realsense" | cut -d " " -f 3 | xargs sudo dpkg --purge
clear
cd ..
cd installRealSenseROS/
./installRealSenseROS.sh 
cd ..
clear
cd catkin_ws/
catkin_make
cd ..
realsense-viewer
sudo apt-get update
sudo apt-get upgrade
sudo apt autoremove
sudo apt-get update
sudo apt-get upgrade
clear
roslaunch turtlebot3_follow_filter 
roslaunch turtlebot3_follow_filter turtlebot3_follow_filter.launch model 
roslaunch turtlebot3_follow_filter turtlebot3_follow_filter.launch
roslaunch turtlebot3_navigation turtlebot3_navigation.launch 
roslaunch turtlebot3_follow_filter turtlebot3_follow_filter.launch
source ~/.bashrc
realsense-viewer
~/.bashrc
sudo
sudo ~/.bashrc
~/.bashrc
source ~/.bashrc
clear
roslaunch turtlebot3_navigation turtlebot3_navigation.launch
roslaunch turtlebot3_slam turtlebot3_slam.launch 
roslaunch turtlebot3_bringup turtlebot3_robot.launch 
sudo apt-get update
sudo apt-get upgrade
clear
roscore
cd catkin_ws/
catkin_make
sudo find / -name librealsense2.so*
realsense-viewer
git clone https://github.com/JetsonHacksNano/installLibrealsense.git
cd installLibrealsense/
./installLibrealsense.sh 
realsense-viewer
clear
cd
clear
roslaunch realsense2_camera rs_camera.launch 
realsense-viewer
lsusb
exit
rviz
gedit
cc usbreset.c -o usbreset
lsusb
chmod +x usbreset
sudo ./usbreset /dev/bus/usb/002/004  
realsense-viewer
roslaunch realsense2_camera rs_camera.launch
exit
apt-get install ros-melodic-realsense2-camera
sudo apt-get install ros-melodic-realsense2-camera
realsense-viewer
lsusb
cd
cd ..
cd
clear
cd Downloads/
ls
/home/nvidia/Desktop/usbreset
cc usbreset.c -o usbreset
lsusb
chmod +x usbreset
sudo ./usbreset /dev/bus/usb/002/003
sudo ./usbreset /dev/bus/usb/001/001
sudo ./usbreset /dev/bus/usb/001/002
sudo ./usbreset /dev/bus/usb/001/004
sudo ./usbreset /dev/bus/usb/001/005
sudo ./usbreset /dev/bus/usb/001/006
sudo ./usbreset /dev/bus/usb/001/002
lsusb
sudo ./usbreset /dev/bus/usb/002/005
sudo ./usbreset /dev/bus/usb/002/002
sudo ./usbreset /dev/bus/usb/002/001
sudo ./usbreset /dev/bus/usb/001/001
sudo ./usbreset /dev/bus/usb/001/002
sudo ./usbreset /dev/bus/usb/001/003
sudo ./usbreset /dev/bus/usb/001/004
sudo ./usbreset /dev/bus/usb/001/005
sudo ./usbreset /dev/bus/usb/001/006
sudo ./usbreset /dev/bus/usb/001/007
sudo ./usbreset /dev/bus/usb/001/008
sudo ./usbreset /dev/bus/usb/001/009
sudo ./usbreset /dev/bus/usb/001/01
sudo ./usbreset /dev/bus/usb/001/010
sudo ./usbreset /dev/bus/usb/001/011
sudo ./usbreset /dev/bus/usb/001/012
sudo ./usbreset /dev/bus/usb/001/013
clear
sudo reboot -h 0
rm ~/.local/share/keyrings/login.keyring
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch 
roslaunch turtlebot3_slam turtlebot3_slam.launch
exit
rviz
roslaunch turtlebot3_bringup turtlebot3_robot.launch
exit
realsense-viewer
rs-enumerate-devices
clear
roslaunch realsense2_camera rs_camera.launch
exit
realsense-viewer
sudo apt-get purge librealsense2-dkms
sudo apt-get update --fix-missing && sudo apt-get install -f && sudo apt-get dist-upgrade -y
realsense-viewer
exit
roscore
roslaunch darknet_ros yolo_v3_tiny.launch
roslaunch realsense2_camera rs_camera.launch#
roslaunch realsense2_camera rs_camera.launch
sudo apt-get install terminator
clear
exit
roslaunch turtlebot3_bringup turtlebot3_robot.launch
exit
/usr/bin/python /home/nvidia/BB_Kevin.py
exit
/usr/bin/python /home/nvidia/BB_Kevin.py
clear
/usr/bin/python /home/nvidia/BB_Kevin.py
chmod 777 faceDetection.py 
python2 
ls
python2 BB_Kevin.py
python2 depth_recognition.py 
python2 20211130_KB_depth_recognition_kevin.py 
python2 BB_Kevin.py
python depth_recognition.py 
python2 BB_Kevin.py
clear
'/home/nvidia/BB_Kevin.py'
python BB_Kevin.py 
clear
python BB_Kevin.py 
'/home/nvidia/Downloads/faceDetection.py' 
chmod 777 yolov2-tiny.cfg
rosrun darknet_ros darknet_ros ObjectDetection.cpp 
rosrun darknet_ros darknet_ros object_detection.test 
rosrun darknet_ros darknet_ros object_detection.test yolov2.yaml 
exit
exit
roslaunch darknet_ros yolo_v3_tiny.launch
clear
exit
roslaunch turtlebot3_bringup turtlebot3_robot.launch
clear
rostopic echo list
rostopic list
rostopic echo /distance/distance_BB 
clear
exit
roslaunch realsense_camera rs_camera.launch
roslaunch realsense2_camera rs_camera.launch
clear
roslaunch realsense2_camera rs_camera.launch
exit
roslaunch realsense2_camera rs_camera.launch
rostopic list
rostopic echo /camera/color/image_raw
rostopic list
rostopic info /camera/color/image_raw
rostopic info /camera/color/image_raw/compressed
rostopic list /camera/color/image_raw
rostopic echo /camera/color/image_raw
rostopic info /camera/color/image_raw
rostopic echo /camera/color/image_raw
rostopic list
rostopic pub /txt_msg std_msgs/String "data: 'Test message'" -1
rostopic pub /txt_msg std_msgs/String "data: 'WE DID ITge'" -1
python -m SimpleHTTPServer
ipconfig
ifconfig
python -m SimpleHTTPServer
python -m SimpleHTTPServer
python -m SimpleHTTPServer 123.123.123.1:8000
localhost
echo localhost
python -m SimpleHTTPServer 
sudo /sbin/iptables -A INPUT -p tcp --dport 80 -j ACCEPT
python -m SimpleHTTPServer 
sudo python -m SimpleHTTPServer 
sudo python -m SimpleHTTPServer 127.222.222.1:8000
sudo python -m SimpleHTTPServer 127.222.222.1
sudo python -m SimpleHTTPServer 127.3.2.1
roslaunch realsense2_camera rs_camera.launch
ifconfig
mkdir -p ~/oliaref/src
cd oliaref
cd src
catkin create pkg robot_gui_bridge --catkin-deps rosbridge_server
catkin_create_pkg robot_gui_bridge rosbridge_server
+cd ..
cd ..
rosdep install --from-paths src --ignore-src -r -y
mkdir src/robot_gui_bridge/launch
touch src/robot_gui_bridge/launch/websocket.launch
catkin build
catkin_make
source devel/setup.bash
roslaunch robot_gui_bridge websocket.launch
mkdir src/robot_gui_bridge/gui/gui.html
mkdir src/robot_gui_bridge/gui/
touch src/robot_gui_bridge/gui/gui.html
roslaunch robot_gui_bridge websocket.launch
ls
rosrun main.cpp
chmod +x main.cpp
rosrun main.cpp
ls
chmod -x main.cpp
cd ..
git clone --recursive https://github.com/pushkalkatara/darknet_ros.git
catkin_make
cd src/
git clone --recursive https://github.com/pushkalkatara/darknet_ros.git
catkin_make
cd ..
catkin_make
which python
cd
python -V
python -c "import sys; print '\n'.join(sys.path)"
python -c "import sys; print(sys.path)"
echo $PYTHONPATH
catkin_make
python -V
cd catkin_ws/
catkin_make
/usr/bin/python /home/nvidia/BB_Kevin.py
/usr/bin/python /home/nvidia/20211203_BoundingBox.py
clear
rostopic list
rostopic echo /distance/distance_BB 
roslaunch realsense2_camera rs_camera.launch
clear
roslaunch darknet_ros yolo_v3_tiny.launch
sudo apt-get update
sudo apt-get upgrade
clear
roslaunch turtlebot3_bringup turtlebot3_robot.launch
'/home/nvidia/Desktop/download/WEBSOCKETS_CLIENT.py' 
sudo '/home/nvidia/Desktop/download/WEBSOCKETS_CLIENT.py' 
'/home/nvidia/Desktop/download/WEBSOCKETS_CLIENT.py' 
pip3 install websockets
'/home/nvidia/Desktop/download/WEBSOCKETS_CLIENT.py' 
rosmaster
roslaunch realsense2_camera rs_camera.launch
roslaunch darknet_ros yolo_v3_tiny.launch
scp /home/nvidia/20211207_SetWaypoints.py
/usr/bin/python /home/nvidia/20211203_BoundingBox.py
rosnode list
ros
rosnode list
rostopic echo /myNode_13028_1638889761666
rosnode list
cd home
cd /home
python 20211103_BoundingBox.py
python
python 202112
python 20211203_BoundingBox.py
python
clear
rosnode list
sudo apt-get update
sudo apt-get upgrade
sudo apt-get update
sudo apt-get upgrade
realsense-viewer
sudo apt-get install ros-melodic-follow-waypoints
cd catkin_ws/src
git clone https://github.com/yujinrobot/yujin_ocs.git
cd ..
catkin_make
cd
clear
sudo apt-get install ros-melodic-follow-waypoints
cd catkin_ws/src
git clone https://github.com/ArghyaChatterjee/waypoint-based-2D-navigation-in-ros.git
cd ..
catkin_make
clear
cd
~/.bashrc
sudo ~/.bashrc
cd catkin_ws/src
git clone https://github.com/khayliang/person_tracking_ros.git
cd person_tracking_ros
cd src/weights
/usr/bin/python /home/nvidia/20211207_SetWaypoint.py
rviz
clear
pip install torch
pip install torchvision
pip install pillow
pip install vizer
pip install edict
pip3 install numpy
pip3 install torch
pip3 install torchvision
pip3 install pillow
pip3 install vizer
pip3 install edict
pip3 install pyyaml
pip3 install easydict
clear
roslaunch turtlebot3_bringup turtlebot3_realsense.launch 
roslaunch realsense2_camera rs_camera.launch
roslaunch turtlebot3_bringup turtlebot3_robot.launch
wget https://pjreddie.com/media/files/yolov3.weights
cd ..
cd detector/YOLOv3/nms
sh build.sh
cd person_tracking_ros
cd src/weights
cd ..
ls
cd detector/
las
ls
cd YOLOv3/
ls
cd nms/
sh build.sh
sudo sh build.sh
exit
cd person_follow_bot/
git clone https://github.com/IvLabs/person_following_bot.git
python3 follow.py
'/home/nvidia/person_follow_bot/person_following_bot/follow.py' 
chmod 777 follow.py
cd person_following_bot/
chmod 777 follow.py
python3 follow.py
sudo apt-get install python-rospkg
python2 follow.py
pip install imutils
python2 follow.py
python2
pip install imutils
pip2 install imutils
python2 follow.py
pip2 install pyrealsense2
python3 follow.py
cd
cd catkin_ws/
catkin_make
roslaunch person_tracking_ros person_tracker.launch
pip3 install utils
rostopic list
roslaunch person_tracker.launch 
roslaunch realsense2_camera rs_camera.launch
roslaunch turtlebot3_bringup turtlebot3_robot.launch
/usr/bin/python3 /home/nvidia/20211208_Person_tracking.py
/usr/bin/python3 /home/nvidia/camera_node.py
pip3 install realsense2-utils
pip install pyrealsense2
/usr/bin/python3 /home/nvidia/camera_node.py
pip3 install pyrealsense-dkms
pip3 install pyrealsense2-dkms
python3
pip3 install pyrealsense2-dkms
pip install pyrealsense2-dkms
pip install pyrealsense2
pip3 install pyrealsense2
pip3 install pyrealsense
python3
git clone https://github.com/IntelRealSense/librealsense
mkdir build & cd build
mkdir buil
cd build
cmake ../ -DBUILD_PYTHON_BINDINGS=bool:true
cmake ../-DBUILD_PYTHON_BINDINGS=bool:true
cd
python
python3
mkdir build
clear
cd buil
ls
cd
ls
sudo make install
export PYTHONPATH=$PYTHONPATH:/home/jetson/Desktop/librealsense-2.50.0/build/wrappers/python
sudo nano .bashrc
python3
source ~/.bashrc
python3
exit
python3
source ~/.bashrc
clear
python3
source ~/.bashrc
python3
source ~/.bashrc
python3
source ~/.bashrc
python3
cmake ../ -DFORCE_RSUSB_BACKEND=ON -DBUILD_PYTHON_BINDINGS:bool=true -DPYTHON_EXECUTABLE=/usr/bin/python3
make -j4
sudo make install
realsense-viewer
clear
sudo apt-get install python3-rospkg-modules
sudo apt-get install python3-catkin-pkg-modules
exit
/usr/bin/python3 /home/nvidia/camera_node.py
rostopic echo /output/image_raw/compressed_rgb 
rviz
rostopic list
rviz
rostopic list
rviz
'/home/nvidia/20211208_Person_tracking.py' 
ls
chmod 777 20211208_Person_tracking.py 
'/home/nvidia/20211208_Person_tracking.py' 
python3 20211208_Person_tracking.py 
chmod 777 camera_node.py
python3 camera_node.py 
rosmaster
roscore
scp /home/nvdia/Downloads/camera_node.py turtlebot@141.7.25.107:/home/turtlebot/Downloads
scp /home/nvdia/Downloads/camera_node.py turtlebot@141.7.25.108:/home/turtlebot/Downloads
scp /home/nvdia/Downloads/camera_node.py turtlebot@141.7.159.186:/home/turtlebot/Downloads
scp /home/nvdia/Downloads/camera_node.py turtlebot@141.7.25.108:/home/turtlebot/Downloads
scp /home/nvdia/Downloads/camera_node.py turtlebot@141.7.25.107:/home/turtlebot/Downloads
ssh turtlebot@141.7.25.107
scp /home/nvdia/Downloads/camera_node.py turtlebot@141.7.25.107:/home/turtlebot/Downloads
scp /home/nvidia/Downloads/camera_node.py turtlebot@141.7.25.107:/home/turtlebot/Downloads
exit
cd catkin_ws/
catkin_make
person_tracker.launch
roslaunch person_tracker.launch
'/home/nvidia/catkin_ws/src/person_tracking_ros/launch/person_tracker.launch' 
roslaunch person_tracking_ros person_tracker.launch
roslaunch darknet_ros yolo_v3_tiny.launch
roslaunch realsense2_camera rs_aligned_depth.launch
pip2
pip3
pip
pip install filterpy==1.4.1
pip2 install filterpy==1.4.1
pip2 install scipy==1.2.3
pip2 install filterpy==1.4.1
pip2 install llvmlite==0.32.0
pip2 install llvmlite==0.25.0
cd catkin_ws/
catkin_make
rostopic list
rostopic echo /person_tracking/bbox_center /
rostopic echo /person_tracking/bbox_center
rostopic echo /darknet_ros/bounding_boxes
rostopic echo /person_tracking/bbox_center
roslaunch darknet_ros yolo_v3_tiny.launch
clear
roslaunch darknet_ros yolo_v3_tiny.launch
sudo apt-get update
sudo apt-get upgrade
clear
roslaunch realsense2_camera rs_camera.launch
realsense-viewer
clear
roslaunch realsense2_camera rs_camera.launch
python 20211210_Drive.py 
rostopic list
python 20211210_Drive.py 
/usr/bin/python /home/nvidia/20211203_BoundingBox.py
/usr/bin/python /home/nvidia/20211210_Drive.py
roslaunch turtlebot3_bringup turtlebot3_robot.launch
roslaunch turtlebot3_navigation turtlebot3_navigation.launch 
clear
roslaunch realsense2_camera rs_camera.launch
roslaunch darknet_ros yolo_v3_tiny.launch
cd Downloads/
ls
python 20211203_BoundingBox.py 
cd ..
ls
python 20211203_BoundingBox.py 
/usr/bin/python /home/nvidia/20211210_Drive.py
exit
roslaunch realsense2_camera rs_camera.launch
exit
roslaunch darknet_ros yolo_v3_tiny.launch
exit
python 20211203_BoundingBox.py 
roslaunch darknet_ros yolo_v3_tiny.launch
roslaunch darknet_ros yolo_v3_tiny.launch
rosshutdown
roscore
roslaunch turtlebot3_bringup turtlebot3_robot.launch
roslaunch realsense2_camera rs_camera.launch
ls
python 20211203_BoundingBox.py 
ls
python 20211210_Drive.py 
scp /home/nvidia/Downloads/camera_node.py turtlebot@141.7.25.107:/home/turtlebot/Downloads
scp /home/nvidia/Downloads/camera_node.py turtlebot@141.7.25.107:/home/turtlebot/Downloads
scp /home/nvidia/20211210_Drive.py turtlebot@141.7.25.107:/home/turtlebot/Downloads
scp /home/nvidia/20211203_BoundingBox.py turtlebot@141.7.25.107:/home/turtlebot/Downloads
cd catkin_ws/
catkin_make
roslaunch person_tracker.launch
ls -al $HOME/.ros
roslaunch person_tracker.launch
git clone https://github.com/engcang/ros-yolo-sort.git
cd ..
catkin_make
cd
python ros-sort.py
cd catkin_ws/
cd src/
cd ros-yolo-sort/
ls
cd SORT_and_ROS_ver/
l
ls
cd python_original/
ls
python sort.py
python3 sort.py
python sort.py
python3 sort.py
python2 sort.py
python sort.py
python3 sort.py
python3 ros-sort.py
roslaunch darknet_ros yolo_v3_tiny.launch
pip2 install scikit-image==0.14.0
pip2 install -U scikit-image==0.14.0
python3 -m pip install -U scikit-image
roscore
ls
python depth_recognition.py
cd catkin_ws/
ls
cd src/
ls
roslaunch realsense2_camera rs_aligned_depth.launch
python3 ros-sort.py
python3 ros-sort.py --display
rostopic list
rostopic echo /tracked_boxes
rostopic echo /tracked_image 
rostopic list
rostopic echo /imx477/image_raw 
chmod +x ros-sort.py
rosrun package_name ros-sort.py _/display:=True 
rosrun ros-sort.py _/display:=True 
ls
python3 ros-sort.py _/display:=True 
ls
chmod -x ros-sort.py
ls
python3 ros-sort.py --display
roslaunch realsense2_camera rs_camera.launch
python3 ros-sort.py
cd ..
ls
cd SORT_ROS_cpp/
ls
cd src/
ls
cd ..
ls
cd ros-yolo-sort/
ls
cd YOLO_and_ROS_ver/
ls
roslaunch darknet_ros yolo_v3_tiny.launch
'/home/nvidia/ros-sort.py' 
python3 ros-sort.py
roslaunch realsense2_camera rs_camera.launch
rostopic list
rostopic echo /tracked_boxes
roslaunch darknet_ros yolo_v3_tiny.launch
python3 ros-sort.py
rostopic list
rostopic echo /tracked_boxes
sudo apt-get update
sudo apt-get upgrade
clear
roslaunch realsense2_camera rs_camera.launch
rostopic echo /person_tracking/bbox_center 
rostopic list
rostopic echo /cmd_vel
rostopic echo /person_tracking/bbox_center 
python 20211210_Drive.py 
exot
exit
/usr/bin/python /home/nvidia/20211210_Drive.py
clear
/usr/bin/python /home/nvidia/20211210_Drive.py
roslaunch darknet_ros yolo_v3_tiny.launch
roslaunch turtlebot3_bringup turtlebot3_robot.launch
python 20211203_BoundingBox.py 
scp /home/nvidia/Downloads/20211203_BoundingBox.py turtlebot@141.7.25.107:/home/turtlebot/Downloads
scp /home/nvidia/Downloads/20211210_Drive.py turtlebot@141.7.25.107:/home/turtlebot/Downloads
roslaunch turtlebot3_bringup turtlebot3_robot.launch
exit
roslaunch realsense2_camera rs_camera.launch 
exit
roslaunch darknet_ros yolo_v3_tiny.launch 
exit
/usr/bin/python /home/nvidia/20211210_Drive.py
python 20211203_BoundingBox.py 
roscore
exit
scp /home/nvidia/Downloads/20211210_Drive.py turtlebot@141.7.25.107:/home/turtlebot/Downloads
scp /home/nvidia/Downloads/20211203_BoundingBox.py turtlebot@141.7.25.107:/home/turtlebot/Downloads
scp /home/nvidia/Downloads/20211210_Drive.py turtlebot@141.7.25.107:/home/turtlebot/Downloads
roslaunch realsense2_camera rs_camera.launch 
exit
python 20211210_Drive.py 
exit
scp /home/nvidia/Downloads/20211215_Drive.py turtlebot@141.7.25.107:/home/turtlebot/Downloads
sudo apt-get update
sudo apt-get upgrade
roslaunch turtlebot3_bringup turtlebot3_robot.launch
exit
roscore
exit
cd Downloads/
ls
python 20211203_BoundingBox.py 
exit
roslaunch darknet_ros yolo_v3_tiny.launch 
exit
roslaunch realsense2_camera rs_camera.launch
exit
roslaunch turtlebot3_bringup turtlebot3_robot.launch
exit
roscore
exit
roslaunch darknet_ros yolo_v3_tiny.launch 
exit
ls
pythen 20211203_BoundingBox.py 
python 20211203_BoundingBox.py 
eixt
exit
python 20211210_Drive.py 
exit
roslaunch realsense2_camera rs_camera.launch 
rosrun smach_viewer smach_viewer.py 
exit
roscore
exit
sudo apt-get install ros-melodic-smach ros-melodic-smach-ros ros-melodic-executive-smach ros-melodic-smach-viewer
clear
sudo chmod 777 yolo_v3_tiny.launch 
roslaunch realsense2_camera rs_camera.launch
sudo apt-get dist-upgrade
sudo apt-get upgrade
cd catkin_ws/
catkin_make clean
catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
catkin_make install
cd
git clone https://github.com/JetsonHacksNano/installSwapfile.git
cd installSwapfile/
./installSwapfile.sh 
clear
cd
ls
git clone https://github.com/IntelRealSense/librealsense.git
clear
git clone https://github.com/JetsonHacksNano/installRealSenseROS.git
cd installRealSenseROS/
./installRealSenseROS.sh 
sudo apt-get install librealsense2-dkms
sudo apt install librealsense2
sudo apt-get install librealsense2-utils
sudo apt-get install librealsense2-dev
sudo apt-get install librealsense2-dbg
sudo find / -name librealsense2.so*
cd
roslaunch realsense2_camera rs_camera.launch
cd catkin_ws/
catkin_make clean
catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
cd
sudo apt-get update && sudo apt-get upgrade
git clone https://github.com/IntelRealSense/librealsense.git
sudo apt-get install libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev at
cmake ../ -DBUILD_EXAMPLES=true
cmake -BUILD_EXAMPLES=true
ros-melodic-smach-viewer
sudo apt install ros-melodic-smach-viewer
cd
clear
git clone https://github.com/JetsonHacksNano/installLibrealsense.git
cd installLibrealsense/
ls
./installLibrealsense.sh 
sudo apt-get install ros-melodic-realsense2-camera
cd
cd ..
cd
clear
git clone https://github.com/IntelRealSense/realsense-ros.git
cd catkin_ws/
catkin_make -j4
roslaunch realsense2_camera rs_camera.launch
clear
catkin_make clean
catkin init
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
roslaunch realsense2_camera rs_camera.launch
exit
roslaunch yolo_v3_tiny.launch 
roslaunch darknet_ros.launch
roslaunch darknet_ros yolo_v3_tiny.launch 
roslaunch darknet_ros yolo_v3_tiny.launch
roslaunch darknet_ros yolo_v3_tiny.launch 
roslaunch realsense2_camera rs_camera.launch
realsense-viewer
cd catkin_ws/src
ls
sudo apt install realsense2-dkms
sudo apt install realsense2-utils
sudo apt install 2
sudo apt-get install librealsense2-utils
sudo apt-get install librealsense2-dkms
cd
roslaunch realsense2_camera rs_aligned_depth.launch 
roslaunch turtlebot3_bringup turtlebot3_robot.launch
ls
roslaunch yolo_v3_tiny.launch 
chmod +x yolo_v3_tiny.launch 
roslaunch yolo_v3_tiny.launch 
roslaunch darknet_ros yolo_v3_tiny.launch 
roslaunch darknet_ros darknet_ros.launch
roslaunch realsense2_camera rs_camera.launch
rviz
sudo chmod 777 darknet
sudo chmod 777 darknet_ros
sudo chmod 777 darknet_ros_msgs
roslaunch darknet_ros darknet_ros.launch
clear
cd ..
catkin_make
cd ~/catkin_ws/
source devel/setup.bash
source ~/.bashrc
cd src/
sudo rm 60-librealsense2-udev-rules.rules 
roslaunch realsense2_camera rs_camera.launch
cd catkin_ws/src
cd ..
catkin_make
roslaunch realsense2_camera rs_camera.launch 
realsense-viewer
roslaunch realsense2_camera rs_camera.launch 
exit
roslaunch darknet_ros darknet_ros.launch
cd ca
cd catkin_ws/
catkin_make
roslaunch realsense2_camera rs_camera.launch 
locate realsense2
cd 
ld
ls
cd catkin_ws/src
ls
cd
cd installLibrealsense/
./installLibrealsense.sh 

dpkg -l | grep "realsense" | cut -d " " -f 3 | xargs sudo dpkg --purge
realsense-viewer
sudo make uninstall && make clean
cd
cd catkin_ws/src
cd realsense-ros/
sudo make uninstall && make clean
cd
cd installLibrealsense/
./installLibrealsense.sh 
cd
cd catkin_ws/
catkin_make
roslaunch realsense2_camera rs_camera.launch
exit
realsense-viewer
exit
realsense-viewer
sudo rm 99-realsense-libusb.rules 
cd
clear
roslaunch realsense2_camera rs_camera.launch
source ~/.bahsrc
source ~/.bashrc
roslaunch realsense2_camera rs_camera.launch
roscore
catkin_make
cd ..
catkin_make
ls
python 20211203_BoundingBox.py 
exit
python 20211210_Drive.py 
python 20211215_Drive.py 
exit
roslaunch darknet_ros yolo_v3_tiny.launch
exit
roslaunch realsense2_camera rs_camera.launch
exit
roslaunch turtlebot3_bringup turtlebot3_robot.launch
exit
sudo shutdown -h 0
/usr/bin/python /home/nvidia/20211210_Drive.py
/usr/bin/python /home/nvidia/20211215_Drive.py
vim plugin.xml 
sudo chmod 777 rqt_virtual_joy/
sudo touch plugin.xml
gedit plugin.xml 
rqt
rosun rqt
rosrun rqt
rosrun rqt rqt
rosrun rqt_
rqt
sudo apt-get install ros-melodic-rqt
clear
rosrun rqt-graph rqt-graph
rqt
sudo apt-get install ros-melodic-rqt-commong-plugins
sudo apt-get install ros-melodic-rqt-common-plugins
clear
rqt
sudo rm plugin.xml 
rosrun smach_viewer smach_viewer.py
rqt
rqt_graph
exit
ls
python3 depth_recognition.py 
rqt_graph
catkin_make
cd
/usr/bin/python /home/nvidia/depth_recognition.py
/usr/bin/python /home/nvidia/20211203_BoundingBox.py
/usr/bin/python /home/nvidia/depth_recognition.py
CLEAR
clear
rostopic list
rostopic echo /tracked_boxes
rqt
rqt_graph
clear
roslaunch darknet_ros yolo_v3_tiny.launch 
roslaunch realsense2_camera
roslaunch realsense2_camera rs_camera.launch
python3 ros-sort.py
rqt_graph
rqt_graph
rviz
roslaunch turtlebot3_bringup turtlebot3_robot.launch
realsense-viewer
clear
roslaunch realsense2_camera rs_camera.launch
roslaunch realsense2_camera rs_aligned_depth.launch 
clear
rostopic echo /tracked_boxes 
clear
cd catkin_ws/src/
ls
cd ros-yolo-sort/
ls
cd YOLO_and_ROS_ver/
ls
cd ..
cd SORT_and_ROS_ver/
ls
cd sort_ROS_python/
ls
python 3 ros-sort.py
ls
python3 ros-sort.py
clear
roslaunch realsense2_camera rs_camera.launch
clear
roslaunch realsense2_camera rs_camera.launch
clear
ls
python3 depth_recognition.py 
clear
roslaunch darknet_ros yolo_v3_tiny.launch 
clear
roslaunch darknet_ros yolo_v3_tiny.launch 
roslaunch darknet_ros yolo_v3_tiny.launch
clear
roslaunch turtlebot3_bringup turtlebot3_robot.launch
clear
roslaunch turtlebot3_bringup turtlebot3_robot.launch
clear
sudo apt-get update
sudo apt-get upgrade
sudo apt autoremove
sudo apt install ros-melodic-ddynamic-reconfigure
clear
roscore
clear
roscore
exit
roslaunch turtlebot3_navigation move_base.launch 
roslaunch turtlebot3_bringup turtlebot3_robot.launch 
roslaunch turtlebot3_slam turtlebot3_slam.launch
python3
roscore
source ~/.bashrc
realsense-viewer
sudo apt-get install libcanberra-gtk*
rosrun smach_viewer smach_viewer.py 
git clone https://github.com/qqwweee/keras-yolo3.git
cd keras-yolo3/
ls
wget https://pjreddie.com/media/files/yolov3-tiny.weights
clear
python3 convert.py yolov3-tiny.cfg yolov3-tiny.weights model_data/yolo-tiny.h5
cd
cd keras-tiny-yolo3_on_real_sence_tutorial/
python3 tiny-yolo.py 
python3
clear
python3 tiny-yolo.py 
pip3 install pyrealsense2
python3
python2 tiny-yolo.py 
python3 tiny-yolo.py
realsense-viewer
clear
roslaunch turtlebot3_bringup turtlebot3_robot.launch
catkin_make
cd ..
catkin_make
g++ version
g++ --version
sudo apt-get install build-essential software-properties-common -y
sudo add-apt-repository ppa:ubuntu-toolchain-r/test -y
sudo apt-get upddate
sudo apt-get update
sudo apt-get install gcc-snaphot -y
sudo apt-get install gcc-snapshot -y
clear
catkin_make -j4
sudo apt-get install g++-4.9
g++ --version
ls -l 'which g++'
cd
ls -l 'which g++'
sudo apt install g++
roslaunch darknet_ros_3d darknet_ros_3d.launch 
roslaunch darknet_ros_3d darknet_ros_3d.launch.py
rostopic list
roslaunch darknet_ros yolo_v3_tiny.launch
rostopic list
roslaunch darknet_ros yolo_v3_tiny.launch
roslaunch darknet_ros_3d darknet_ros_3d.launch
clear
realsense-viewer
cd keras-tiny-yolo3_on_real_sence_tutorial/
ls
python3 tiny-yolo.py 
sudo apt-get install libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev
sudo apt-get update && sudo apt-get upgrade && sudo apt-get dist-upgrade
git clone https://github.com/IntelRealSense/librealsense.git
cd installLibrealsense/
sudo apt-get install git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev
sudo apt-get install libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev
./scripts/setup_udev_rules.sh
cd scripts/
./setup_udev_rules.sh
cd
cd librealsense/
./scripts/setup_udev_rules.sh
./scripts/patch-realsense-ubuntu-lts.sh
cmake ../ -DCMAKE_BUILD_TYPE=release -DBUILD_EXAMPLES=true -DBUILD_GRAPHICAL_EXAMPLES=false
sudo make uninstall && make clean && make && sudo make install
g++ darknet3d_node.cpp 
clear
cd
roslaunch realsense2_camera rs_rgbd.launch 
cd catkin_ws/
catkin_make
clear
catkin_make
sudo apt-get install ros-melodic-find-object-2d
clear
roslaunch find_object_2d find_object_3d.launch
rotopic list
rostopic list
chmod 777 launch/
sudo chmod 777 launch/
roslaunch realsense2_camera rs_rgbd.launch
roslaunch find_object_2d find_object_3d_realsense.launch
roslaunch darknet_ros_3d darknet_ros_3d.launch
roslaunch darknet_ros yolo_v3_tiny.launch 
roslaunch darknet_ros_3d darknet_ros_3d.launch
rviz
roslaunch realsense2_camera rs_rgbd.launch
realsense-viewer
roslaunch realsense2_camera rs_rgbd.launch
python3
exit
python3
exit
rostopic list
source ~/.bashrc
rostopic list
source ~/.bashrc
exit
realsense-viewer
clear
python3
clear
exit
source ~/.bashrc
exit
export PATH=/usr/local/cuda-10.0/bin${PATH:+:${PATH}}
export LD_LIBRARY_PATH=/usr/local/cuda-10.0/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
exit
python3
cd
cd 
cd keras-tiny-yolo3_on_real_sence_tutorial/
ls
python3 tiny-yolo.py 
python3
clear
source ~/.bashrc
python3 tiny-yolo.py
python3
exit
whereis gcc
which gcc
gcc --version
realsense-viewer
clear
exit
roslaunch reclear
clear
source ~/.bashrc
exit
roslaunch darknet_ros_3d darknet_ros_3d.launch
roslaunch realsense2_camera rs_rgbd.launch 
clear
cd catkin_ws/
catkin_make
cd librealsense/
mkdir build && cd build
cmake ../ -DFORCE_RSUSB_BACKEND=ON -DBUILD_PYTHON_BINDINGS:bool=true -DPYTHON_EXECUTABLE=/usr/bin/python3
cd build/
cmake ../ -DFORCE_RSUSB_BACKEND=ON -DBUILD_PYTHON_BINDINGS:bool=true -DPYTHON_EXECUTABLE=/usr/bin/python3
make -j4
Okey, so... The problem is that the executable darknet3d_node is not being installed. If you look the CMakeLists, you will see that the library and the launch file is being installed but not the executable. That is the error and this is the reason by it is not found. When I did this package I used catkin to compile and catkin doesn't need that.

clear
sudo make install
sudo nano .bashrc
cd
sudo nano .bashrc
source ~/.bashrc
sudo nano .bashrc
source ~/.bashrc
clear
sudo apt-get update
sudo apt-get install libhdf5-serial-dev hdf5-tools libhdf5-dev zlib1g-dev zip libjpeg8-dev liblapack-dev libblas-dev gfortran
sudo apt-get install python3-pip
sudo pip3 install -U pip testresources setuptools==49.6.0 
sudo pip3 install -U --no-deps numpy==1.19.4 future==0.18.2 mock==3.0.5 keras_preprocessing==1.1.2 keras_applications==1.0.8 gast==0.4.0 protobuf pybind11 cython pkgconfig
sudo env H5PY_SETUP_REQUIRES=0 pip3 install -U h5py==3.1.0
sudo pip3 uninstall -y tensorflow
sudo pip3 uninstall -y tensorflow-gpu
sudo pip3 install --pre --extra-index-url https://developer.download.nvidia.com/compute/redist/jp/v46 'tensorflow<2'
sudo pip3 install -U --no-deps numpy==1.19.4 future==0.18.2 mock==3.0.5 keras_preprocessing==1.1.2 keras_applications==1.0.8 gast==0.4.0 protobuf pybind11 cython pkgconfig
sudo env H5PY_SETUP_REQUIRES=0 pip3 install -U h5py==3.1.0
sudo pip3 uninstall -y tensorflow
sudo pip3 install --pre --extra-index-url https://developer.download.nvidia.com/compute/redist/jp/v46 tensorflow
clear
cd catkin_ws/
catkin_make
clear
catkin-make -std=c++14
catkin_make -std=c++14
catkin_make -j4
catkin_make -j1
recipe for target 'gb_visual_detection_3d/darknet_ros_3d/CMakeFiles/darknet_ros_3d.dir/src/darknet_ros_3d/Darknet3D.cpp.o' failed
sudo apt-get install ros-melodic-rviz-visual-tools
roslaunch rviz_visual_tools demo_rviz.launch 
roslaunch rviz_visual_tools demo.launch
sudo apt-get remove ros-melodic-rviz-visual-tools
exit
exit
cd ca
cd catkin_ws/
catkin_make -DCMAKE_BUILD_TYPE=Release
clear
git clone --recursive https://github.com/IntelligentRoboticsLabs/gb_visual_detection_3d.git
clear
catkin_make -DCMAKE_BUILD_TYPE=Release
clear
git clone --melodic https://github.com/IntelligentRoboticsLabs/gb_visual_detection_3d.git
git clone -b melodic https://github.com/IntelligentRoboticsLabs/gb_visual_detection_3d.git
catkin_make -DCMAKE_BUILD_TYPE=Release
cd src
git clone https://github.com/PickNikRobotics/rviz_visual_tools.git
rosdep install --from-paths src --ignore-src --rosdistro melodic
cd ..
rosdep install --from-paths src --ignore-src --rosdistro melodic
sudo apt autoremove
clear
catkin_make -j4
sudo apt-get install ros-melodic-rviz-visual-tools
rosdep install --from-paths src --ignore-src --rosdistro melodic
clear
catkin_make
catkin_package() DEPENDS on 'PCL' but neither 'PCL_INCLUDE_DIRS' nor
git clone -b melodic https://github.com/ros-perception/perception_pcl.git
git clone -b melodic-devel https://github.com/ros-perception/perception_pcl.git
catkin_make -j4
sudo swapon --show
sudo apt install ros-melodic-pcl-ros
sudo apt install pcl-tools
catkin_make -j4
cd src
git clone -b melodic-devel https://github.com/ros-perception/perception_pcl.git
cd ..
catkin_make -j4
sudo apt-get remove pcl-tools
clear
catkin_make
sudo apt install ros-melodic-pcl-ros
sudo apt install pcl-tools
catkin_make
clear
rosdep install -h
sudo apt-get install python-rosdep python-rosinstall-generator python-vcstool python-rosinstall build-essential
sudo apt-get install python-rosdep
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro melodic -y
clear
catkin_make 
cd src
git clone -b melodic-devel https://github.com/ros-perception/perception_pcl.git
cd ..
catkin_make -j4
sudo apt install ros-melodic-pcl-ros
clear
sudo apt-get install ros-melodik-pcl-ros
sudo apt-get install ros-melodic-pcl-ros
cd src
git checkout origin/melodic-devel
git clone -b melodic-devel clone https://github.com/ros-perception/perception_pcl.git
git clone -b melodic-devel https://github.com/ros-perception/perception_pcl.git
mkdir build
cd ..
catkin-make -j4
catkin_make -j4
exit
