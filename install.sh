mv ~/Eurobot-Localization ~/Localization2023_ws
mv ~/Localization2023_ws/src/.YDLidar-SDK ~/Localization2023_ws/src/YDLidar-SDK
cd ~/Localization2023_ws/src/YDLidar-SDK && mkdir build && cd build
cmake ..
make
sudo make install
sudo apt update
sudo apt install ros-noetic-costmap-converter -y
sudo apt install ros-noetic-robot-localization -y
cd ~/Localization2023_ws
catkin_make
mv ~/Localization2023_ws/src/YDLidar-SDK ~/Localization2023_ws/src/.YDLidar-SDK

echo "source ~/Localization2023_ws/devel/setup.bash" >> ~/.bashrc
cp ydlidar_ros_driver ~/ws/src/ydlidar_ros_driver
roscd && cd ..
chmod 777 src/ydlidar_ros_driver/startup/*
sudo sh src/ydlidar_ros_driver/startup/initenv.sh
