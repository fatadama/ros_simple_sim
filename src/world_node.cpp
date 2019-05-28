#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <simple_sim/world.h>
#include "simple_sim/initVehicle.h"

using namespace simple_sim_world;

void dummycallback(const std_msgs::Float64MultiArray::ConstPtr& arr){
  return;
}

bool initVehicleCallback(simple_sim::initVehicle::Request &req,
    simple_sim::initVehicle::Response &res)
{
  return true;
}

int main(int argc, char** argv){
  // run ros
  ros::init(argc,argv,"simple_world");
  // roc handle object
  ros::NodeHandle node;

  // placeholder world object
  world worldObj;

  // init vehicle service
  ros::ServiceServer service = node.advertiseService("initVehicle",initVehicleCallback);
  // subscribe to simple_vel messages
  ros::Subscriber sub = node.subscribe("simple_vel", 1, dummycallback);
  // set rate
  ros::Rate r(10); // 10 hz
  int loopCounter = 0;
  while (ros::ok()){
    loopCounter++;
    if (loopCounter==10){
      // print status
      ROS_INFO("This is the world node at time t = %f.\n",ros::Time::now().toSec());
      //reset
      loopCounter = 0;
    }
    // spin once
    ros::spinOnce();
    // sleep to try to get steady execution
    r.sleep();
  }
}
