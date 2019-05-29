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
  // random heading
  double theta = ( float(rand())/float(RAND_MAX) )*2.0*M_PI-M_PI;
  // call standard Constructor
  vehicle(idi,phi,gamma,theta);
}

simple_sim_world::vehicle::vehicle(long idin, double longitude, double latitude, double thetain){
  // store id
  id = idin;
  longLat2quat(longitude,latitude,quat);
  // heading
  theta = thetain;
  // zero speed and turn rate
  u = 0.0;
  omega = 0.0;
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

long simple_sim_world::vehicle::get_id()
{
  return id;
}

void simple_sim_world::vehicle::get_state(std::vector <double>& beta,double& thet,double& ui, double& omeg)
{
  //copy quaternion
  for(int i = 0;i<beta.size();i++)
  {
    beta[i] = quat[i];
    if (i == 3)
    {
      break;
    }
  }
  for(int i = beta.size();i<4;i++)
  {
    beta.push_back(quat[i]);
  }
  thet = theta;
  ui = u;
  omeg = omega;
  return;
}

simple_sim_world::world::world(double timein){
  // set the time
  t = timein;
  // seed the RNG
  srand(time(NULL));
}

void simple_sim_world::world::init_vehicle(long id)
{
  // random longitude
  double phi = ( float(rand())/float(RAND_MAX) )*2.0*M_PI-M_PI;
  // random latitude
  double gamma = ( float(rand())/float(RAND_MAX) )*M_PI-0.5*M_PI;
  // random heading
  double theta = ( float(rand())/float(RAND_MAX) )*2.0*M_PI-M_PI;
  // store vehicle
  trackedVehicles.push_back(vehicle(id,phi,gamma,theta));
  // HACK call print debug_print
  trackedVehicles[trackedVehicles.size()-1].debug_print();
}

bool simple_sim_world::world::is_known_vehicle(long id)
{
  for (int i = 0;i<trackedVehicles.size();i++)
  {
    if (id == trackedVehicles[i].get_id()){
      return true;
    }
  }
  return false;
}

void simple_sim_world::world::step(double timenow)
{
  // compute dt
  double dt = timenow - t;
  std::cout << "step print: t = " << timenow << " " << trackedVehicles.size() << " tracked vehicles\n";
  // loop over each vehicle
  for(int i = 0;i<trackedVehicles.size();i++)
  {
    // allocate state
    std::vector<double> x(7,0.0);
    // propagated state
    std::vector<double> xprop(7,0.0);
    // extract the current state
    trackedVehicles[i].get_state(x,x[4],x[5],x[6]);

    std::cout << "Vehicle " << i << " | time " << t << ": x = " << x[0] << ","
     << x[1] << "," << x[2] << "," << x[3] << "," << x[4] << "," << x[5] << ","
     << x[6] << "," << x[7] << std::endl;
  }
  // update the time
  t = timenow;
}
