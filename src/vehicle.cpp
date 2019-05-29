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
  //
  // set ID
  srv.request.id = 0;
  while (true){
    if (client.call(srv))
    {
      if (srv.response.success){
        // accept the id
        ROS_INFO("Initializing with ID %d\n",srv.request.id);
        break;
      }
      else{
        ROS_INFO("Incrementing from ID %d\n",srv.request.id);
        // sleep
        ros::Duration(0.5).sleep();
        // increment ID and try again
        srv.request.id++;
      }
    }
    else
    {
      ROS_INFO("No service response");
      break;
    }
  }
  return 0;
}
