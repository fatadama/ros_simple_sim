#include "simple_sim/world.h"
#include <cmath>
#include <cstdlib>
#include <iostream>

world::world(){
  // DEBUG init a vehicle
  init_vehicle((long)1);
}

void world::init_vehicle(long id){
  // random longitude
  double phi = ( float(rand())/float(RAND_MAX) )*2.0*M_PI-M_PI;
  // random latitude
  double gamma = ( float(rand())/float(RAND_MAX) )*M_PI-0.5*M_PI;
  // store vehicle
  trackedVehicles.push_back(vehicle(id,phi,gamma));
  // call print debug_print
  trackedVehicles[0].debug_print();
}

vehicle::vehicle(){
  // random ID
  long idi = long(rand());
  // random longitude
  double phi = ( float(rand())/float(RAND_MAX) )*2.0*M_PI-M_PI;
  // random latitude
  double gamma = ( float(rand())/float(RAND_MAX) )*M_PI-0.5*M_PI;
  // call standard Constructor
  vehicle(idi,phi,gamma);
}

vehicle::vehicle(long idin, double longitude, double latitude){
  // store id
  id = idin;
  // trace of the DCM
  double zeta = cos(latitude)*cos(longitude)+cos(longitude)+cos(latitude);
  double coef = sqrt(zeta+1.0);
  // initialize quaternion
  quat.push_back(0.5*coef);
  quat.push_back(0.5*sin(latitude)*sin(longitude)/coef);
  quat.push_back(0.5*-sin(latitude)*(1+cos(longitude))/coef);
  quat.push_back(0.5*sin(longitude)*(1+cos(latitude))/coef);
}

void vehicle::debug_print(){
  std::cout<< " This is vehicle: "<< id <<"\n";
  std::cout<< " Current quaternion: " << quat[0] << "," << quat[1] << ","
   << quat[2] << "," << quat[3] << "\n";
}
