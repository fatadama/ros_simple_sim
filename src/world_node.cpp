#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
// boost for boost::bind passing extra arguments
#include <boost/bind/bind.hpp>

#include "simple_sim/world.h"       //!< World object
#include "simple_sim/initVehicle.h" //!< Vehicle initialization service
#include "simple_sim/simple_vel.h"  //!< Vehicle simple velocity message

using namespace simple_sim_world;

void simpleVelocityCallback(const simple_sim::simple_vel::ConstPtr& msg, world& worldOb){
// update velocity info
  worldOb.update_velocity(msg->u,msg->omega,(long)msg->id);
  return;
}

bool initVehicleCallback(simple_sim::initVehicle::Request &req,
    simple_sim::initVehicle::Response &res, world& worldOb)
{
  // check if ID is unique
  if (!(worldOb.is_known_vehicle((long)req.id)))
  {
    //set flag
    res.success = true;
    // add to list
    worldOb.init_vehicle((long)req.id);
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
  ros::init(argc,argv,"world");
  // roc handle object
  ros::NodeHandle node;

  // get the rate_hz parameter
  double rate_hz;
  node.param("simple_sim/rate_hz",rate_hz,10.0);

  // placeholder world object
  world worldObj(ros::Time::now().toSec());

  // init vehicle service
  ros::ServiceServer service = node.advertiseService<simple_sim::initVehicle::Request,
      simple_sim::initVehicle::Response>("initVehicle",
    boost::bind(initVehicleCallback,_1,_2, boost::ref(worldObj)));
  // subscribe to simple_vel messages
  ros::Subscriber sub = node.subscribe<simple_sim::simple_vel>("simple_vel", 100,
    boost::bind(&simpleVelocityCallback,_1,boost::ref(worldObj)));
  // set rate
  ros::Rate r(rate_hz); // 10 hz
  int loopCounter = 0;
  while (ros::ok()){
    loopCounter++;
    if (loopCounter==int(rate_hz)) // 1 Hz output
    {
      // print status
      //ROS_INFO("This is the world node.\n");
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
