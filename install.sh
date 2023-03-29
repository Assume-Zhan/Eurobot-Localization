# Install basic package
sudo apt update -y
sudo apt install ros-noetic-costmap-converter -y
sudo apt install ros-noetic-robot-localization -y
sudo apt install ros-noetic-imu-tools -y

# Rename the workspace
WS_PATH="$( find ~ -name Eurobot-Localization )"
NEW_WS_PATH="${WS_PATH/'Eurobot-Localization'/'localization'}"

mv $WS_PATH $NEW_WS_PATH

# Change visible og lidar sdk
# Build lidar sdk
mv $NEW_WS_PATH/.YDLidar-SDK $NEW_WS_PATH/YDLidar-SDK
cd $NEW_WS_PATH/YDLidar-SDK && mkdir build && cd build
# cmake ..
# make
# sudo make install
echo "Done make"
mv $NEW_WS_PATH/YDLidar-SDK $NEW_WS_PATH/.YDLidar-SDK

# Workspace setup
cd $NEW_WS_PATH
# catkin_make
echo "Done catkin_make"

# USB driver setup
cd $NEW_WS_PATH
cd $NEW_WS_PATH/local_filter/imu/phidgets_drivers/phidgets_api
sudo cp debian/udev /etc/udev/rules.d/99-phidgets.rules
sudo udevadm control --reload-rules
