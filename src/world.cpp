#include "simple_sim/world.h"
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <ctime>
#include <cstdio>

void simple_sim_world::longLat2quat(double longitude, double latitude, std::vector <double>& quat)
{
  // trace of the DCM
  double zeta = cos(latitude)*cos(longitude)+cos(longitude)+cos(latitude);
  double coef = sqrt(zeta+1.0);
  if (quat.size() > 0)
  {
    quat[0] = 0.5*coef;
  }
  else
  {
    quat.push_back(0.5*coef);
  }
  if (quat.size() > 1)
  {
    quat[1] = 0.5*sin(latitude)*sin(longitude)/coef;
  }
  else
  {
    quat.push_back(0.5*sin(latitude)*sin(longitude)/coef);
  }
  if (quat.size() > 2)
  {
    quat[2] = 0.5*-sin(latitude)*(1+cos(longitude))/coef;
  }
  else
  {
    quat.push_back(0.5*-sin(latitude)*(1+cos(longitude))/coef);
  }
  if (quat.size() > 3)
  {
    quat[3] = 0.5*sin(longitude)*(1+cos(latitude))/coef;
  }
  else
  {
    quat.push_back(0.5*sin(longitude)*(1+cos(latitude))/coef);
  }
  return;
}

void simple_sim_world::quat2longLat(std::vector <double> beta, double& longitude, double& latitude){
  latitude = asin(2.0*(beta[1]*beta[3]-beta[0]*beta[2]));
  longitude = atan2(-2.0*(beta[1]*beta[2]-beta[0]*beta[3]),1.0-2.0*beta[1]*beta[1]-2.0*beta[3]*beta[3]);
}

simple_sim_world::vehicle::vehicle(){
  // random ID
  long idi = long(rand());
  // random longitude
  double phi = ( float(rand())/float(RAND_MAX) )*2.0*M_PI-M_PI;
  // random latitude
  double gamma = ( float(rand())/float(RAND_MAX) )*M_PI-0.5*M_PI;
  // call standard Constructor
  vehicle(idi,phi,gamma);
}

simple_sim_world::vehicle::vehicle(long idin, double longitude, double latitude){
  // store id
  id = idin;
  longLat2quat(longitude,latitude,quat);
}

void simple_sim_world::vehicle::debug_print(){
  // extract long lat
  double lon, lat;
  quat2longLat(quat,lon,lat);
  std::cout << " This is vehicle: "<< id <<"\n";
  std::cout << " Current quaternion: " << quat[0] << "," << quat[1] << ","
   << quat[2] << "," << quat[3] << "\n";
   std::cout << " Equivalent lon/lat: " << lon << "," << lat << "\n";
}

simple_sim_world::world::world(){
  // seed the RNG
  srand(time(NULL));
  // DEBUG init a vehicle
  init_vehicle((long)1);
}

void simple_sim_world::world::init_vehicle(long id){
  // random longitude
  double phi = ( float(rand())/float(RAND_MAX) )*2.0*M_PI-M_PI;
  // random latitude
  double gamma = ( float(rand())/float(RAND_MAX) )*M_PI-0.5*M_PI;
  // store vehicle
  trackedVehicles.push_back(vehicle(id,phi,gamma));
  // call print debug_print
  trackedVehicles[0].debug_print();
}
