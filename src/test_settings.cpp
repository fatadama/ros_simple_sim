#include <ros/ros.h>
#include <string>
#include <iostream>
#include "simple_sim/settings.h"

using namespace simple_sim_settings;

int main(int argc,char*argv[])
{
  ros::init(argc,argv,"test_settings");
  // ros handle object
  ros::NodeHandle node;

  std::string settingspath;
  if(node.getParam("simple_sim/settings_path",settingspath)){
    ROS_INFO("%s",settingspath.c_str());
    settings localsettings(settingspath);
    ROS_INFO("Execution rate: %f Hz",localsettings.get_rate_hz());
  }
  else
  {
    ROS_ERROR("Failed to find parameter simple_sim/settings_path");
  }
}
