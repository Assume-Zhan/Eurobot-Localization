mv ~/Eurobot-Localization ~/Localization2023_ws
mv ~/Localization2023_ws/src/.YDLidar-SDK ~/Localization2023_ws/src/YDLidar-SDK
cd ~/Localization2023_ws/src/YDLidar-SDK && mkdir build && cd build
cmake ..
make
sudo make install
sudo apt update
sudo apt install ros-noetic-costmap-converter -y
sudo apt install ros-noetic-robot-localization -y
sudo apt install ros-noetic-imu-tools -y
cd ~/Localization2023_ws
catkin_make
mv ~/Localization2023_ws/src/YDLidar-SDK ~/Localization2023_ws/src/.YDLidar-SDK

echo "source ~/Localization2023_ws/devel/setup.bash" >> ~/.bashrc
cd ~/Localization2023_ws

chmod 777 src/ydlidar_ros_driver/startup/*
sudo sh src/ydlidar_ros_driver/startup/initenv.sh
cd ~/Localization2023_ws/src/phidgets_drivers/phidgets_api
sudo cp debian/udev /etc/udev/rules.d/99-phidgets.rules
sudo udevadm control --reload-rules
