WS_PATH="$( find ~ -name Eurobot-Localization )"
NEW_WS_PATH="${WS_PATH/'Eurobot-Localization'/'Localization2023_ws'}"

mv $WS_PATH $NEW_WS_PATH
mv $NEW_WS_PATH/src/.YDLidar-SDK $NEW_WS_PATH/src/YDLidar-SDK
cd $NEW_WS_PATH/src/YDLidar-SDK && mkdir build && cd build
cmake ..
make
sudo make install
sudo apt update
sudo apt install ros-noetic-costmap-converter -y
sudo apt install ros-noetic-robot-localization -y
sudo apt install ros-noetic-imu-tools -y
cd $NEW_WS_PATH
catkin_make
mv $NEW_WS_PATH/src/YDLidar-SDK ~/Localization2023_ws/src/.YDLidar-SDK

echo "source $NEW_WS_PATH/devel/setup.bash" >> ~/.bashrc
cd $NEW_WS_PATH

chmod 777 src/ydlidar_ros_driver/startup/*
sudo sh src/ydlidar_ros_driver/startup/initenv.sh
cd $NEW_WS_PATH/src/phidgets_drivers/phidgets_api
sudo cp debian/udev /etc/udev/rules.d/99-phidgets.rules
sudo udevadm control --reload-rules
