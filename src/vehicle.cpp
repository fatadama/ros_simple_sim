#include <iostream>
#include <ros/ros.h>
#include "simple_sim/initVehicle.h"
#include "simple_sim/simple_vel.h"

/** @brief simple vehicle that calls the initVehicle service of the world node
 */

int main(int argc, char* argv[]){
  ros::init(argc,argv,"vehicle");
  ros::NodeHandle node;

  // get the global rate
  double rate_hz;
  node.param("simple_sim/rate_hz",rate_hz,10.0);

  // create publisher to send simple_vel messages
  ros::Publisher velocity_publisher = node.advertise<simple_sim::simple_vel>("simple_vel",100);
  // create service client
  ros::ServiceClient client = node.serviceClient<simple_sim::initVehicle>("initVehicle");
  // client request object
  simple_sim::initVehicle srv;

  // set ID
  int id = -1;
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
        // sleep to avoid spamming
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
  id = srv.request.id;
  ros::Rate loop_rate(rate_hz);
  // HACK test velocity message
  while (ros::ok())
  {
    simple_sim::simple_vel msg;
    msg.u = 1.0;
    msg.omega = 0.0;
    msg.id = id;
    velocity_publisher.publish(msg);
    // spin and loop
    ros::spinOnce();
    loop_rate.sleep();
  }

  // TODO add a service to terminate vehicle at exit
}
