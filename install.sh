mv ~/Eurobot-Localization ~/Localization2023_ws
cd ~/Localization2023_ws/src/.YDLidar-SDK/build
cmake ..
make
sudo make install
sudo apt install ros-noetic-costmap-converter -y
sudo apt install ros-noetic-robot-localization -y
cd ~/Localization2023_ws
catkin_make
