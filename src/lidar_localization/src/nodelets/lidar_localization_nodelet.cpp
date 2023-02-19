/**
 *
 * @file lidar_localization_nodelet.cpp
 * @brief
 *
 * @code{.unparsed}
 *      _____
 *     /  /::\       ___           ___
 *    /  /:/\:\     /  /\         /  /\
 *   /  /:/  \:\   /  /:/        /  /:/
 *  /__/:/ \__\:| /__/::\       /  /:/
 *  \  \:\ /  /:/ \__\/\:\__   /  /::\
 *   \  \:\  /:/     \  \:\/\ /__/:/\:\
 *    \  \:\/:/       \__\::/ \__\/  \:\
 *     \  \::/        /__/:/       \  \:\
 *      \__\/         \__\/         \__\/
 * @endcode
 *
 * @author sunfu.chou (sunfu.chou@gmail.com)
 * @version 0.1
 * @date 2021-05-12
 *
 */

#include <nodelet/nodelet.h>
#include <memory>

#include "lidar_localization/lidar_localization.h"

namespace lidar_localization
{
class LidarLocalizationNodelet : public nodelet::Nodelet
{
public:
  virtual void onInit()
  {
    ros::NodeHandle nh = getNodeHandle();
    ros::NodeHandle nh_local = getPrivateNodeHandle();

    try
    {
      NODELET_INFO("[Lidar Localization]: Initializing nodelet");
      lidar_localization_instance = std::shared_ptr<LidarLocalization>(new LidarLocalization(nh, nh_local));
    }
    catch (const char* s)
    {
      NODELET_FATAL_STREAM("[Lidar Localization]: " << s);
    }
    catch (...)
    {
      NODELET_FATAL_STREAM("[Lidar Localization]: Unexpected error");
    }
  }

  virtual ~LidarLocalizationNodelet()
  {
    NODELET_INFO("[Lidar Localization]: Shutdown");
  }

private:
  std::shared_ptr<LidarLocalization> lidar_localization_instance;
};

}  // namespace lidar_localization

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(lidar_localization::LidarLocalizationNodelet, nodelet::Nodelet)
