mv ~/Eurobot-Localization ~/Localization2023_ws
mv ~/Localization2023_ws/src/.YDLidar-SDK ~/Localization2023_ws/src/YDLidar-SDK
cd ~/Localization2023_ws/src/YDLidar-SDK/build
cmake ..
make
sudo make install
sudo apt update
sudo apt install ros-noetic-costmap-converter -y
sudo apt install ros-noetic-robot-localization -y
cd ~/Localization2023_ws
catkin_make
mv ~/Localization2023_ws/src/YDLidar-SDK ~/Localization2023_ws/src/.YDLidar-SDK
