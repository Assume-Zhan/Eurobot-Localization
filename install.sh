# Function for go back to workspace
function check_backto_ws(){
    if [ $(ls | grep devel | wc -l) = 1 ]; then
        echo "BACK TO WS"
        return 0
    else 
        cd ..
        if [ $(pwd) = "/home" ] || [ $(pwd) = "/" ]; then
            return -1
        else
            check_backto_ws
        fi
    fi
}


# Install basic package
sudo apt update -y
sudo apt install ros-noetic-costmap-converter \
                 ros-noetic-robot-localization \
                 ros-noetic-imu-tools \
                 libusb-1.0-0 libusb-1.0-0-dev -y

# Rename the workspace
WS_PATH="$( find ~ -name Eurobot-Localization | awk '{print $1}' | head -1)"

# Change visible of lidar sdk
# Build lidar sdk
mv $WS_PATH/.YDLidar-SDK $WS_PATH/YDLidar-SDK
cd $WS_PATH/YDLidar-SDK && mkdir build && cd build
cmake ..
make
sudo make install
# echo "Done make"
mv $WS_PATH/YDLidar-SDK $WS_PATH/.YDLidar-SDK

# Go back to Workspace
check_backto_ws
catkin_make

# USB driver setup
cd $WS_PATH
cd $WS_PATH/local_filter/imu/phidgets_drivers/phidgets_api
sudo cp debian/udev /etc/udev/rules.d/99-phidgets.rules
sudo udevadm control --reload-rules
