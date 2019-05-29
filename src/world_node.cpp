#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
// boost for boost::bind passing extra arguments
#include <boost/bind/bind.hpp>

#include "simple_sim/world.h"
#include "simple_sim/initVehicle.h"

using namespace simple_sim_world;

void dummycallback(const std_msgs::Float64MultiArray::ConstPtr& arr){
  return;
}

bool initVehicleCallback(simple_sim::initVehicle::Request &req,
    simple_sim::initVehicle::Response &res, world& worldObj)
{
  // initialize world object with id from req
  ROS_INFO("Got here with id: %d\n",(int)req.id);
  // check if ID is unique
  if (!(worldObj.is_known_vehicle((long)req.id)))
  {
    //set flag
    res.success = true;
    // add to list
    worldObj.init_vehicle((long)req.id);
  }
  else
  {
    // if the ID is already in use, return false
    res.success = false;
  }
  // return true to denote successful completion of service
  return true;
}

int main(int argc, char** argv){
  // run ros
  ros::init(argc,argv,"simple_world");
  // roc handle object
  ros::NodeHandle node;

  // placeholder world object
  world worldObj(ros::Time::now().toSec());

  // init vehicle service
  int i = 0;
  ros::ServiceServer service = node.advertiseService<simple_sim::initVehicle::Request,
      simple_sim::initVehicle::Response>("initVehicle",
    boost::bind(initVehicleCallback,_1,_2, worldObj));
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
    // integrate each vehicle to the current time
    worldObj.step(ros::Time::now().toSec());
    // spin once
    ros::spinOnce();
    // sleep to try to get steady execution
    r.sleep();
  }
}
