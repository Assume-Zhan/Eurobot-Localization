# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "nodelet;phidgets_api;roscpp;sensor_msgs".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lphidgets_accelerometer".split(';') if "-lphidgets_accelerometer" != "" else []
PROJECT_NAME = "phidgets_accelerometer"
PROJECT_SPACE_DIR = "/home/ubuntu/Localization2023_ws/install"
PROJECT_VERSION = "1.0.6"
