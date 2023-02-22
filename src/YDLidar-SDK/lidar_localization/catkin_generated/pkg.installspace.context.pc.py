# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;rviz;std_msgs;geometry_msgs;sensor_msgs;tf2;tf2_ros;laser_geometry;nodelet;obstacle_detector".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-llidar_localization;-llidar_localization_nodelets;-lbeacon_scan_tracker;-larea_obstacles_extractor".split(';') if "-llidar_localization;-llidar_localization_nodelets;-lbeacon_scan_tracker;-larea_obstacles_extractor" != "" else []
PROJECT_NAME = "lidar_localization"
PROJECT_SPACE_DIR = "/usr/local"
PROJECT_VERSION = "0.0.0"
