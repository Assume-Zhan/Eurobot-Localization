# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "nodelet;pluginlib;phidgets_api;roscpp;sensor_msgs;std_msgs;std_srvs".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lphidgets_spatial".split(';') if "-lphidgets_spatial" != "" else []
PROJECT_NAME = "phidgets_spatial"
PROJECT_SPACE_DIR = "/home/assume/Localization2023_ws/install"
PROJECT_VERSION = "1.0.6"
