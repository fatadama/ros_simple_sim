#include <iostream>
#include <ros/ros.h>
#include "simple_sim/initVehicle.h"

/** @brief simple vehicle that calls the initVehicle service of the world node
 */

int main(int argc, char* argv[]){
  ros::init(argc,argv,"vehicle");
  ros::NodeHandle node;
  // create service client
  ros::ServiceClient client = node.serviceClient<simple_sim::initVehicle>("initVehicle");
  // client request object
  simple_sim::initVehicle srv;
  // set ID
  srv.request.id = 1;
  if (client.call(srv))
  {
    ROS_INFO("Received %d from initVehicle service\n",srv.response.success);
  }
  else
  {
    ROS_INFO("No service response");
  }
  return 0;
}
