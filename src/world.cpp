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
  // Earth radius
  R = 6371e6;
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

void::simple_sim_world::vehicle::propagate(double dt){
  // delta state
  std::vector<double> dx(5,0.0);
  // compute "angular velocity" for Earth-constrained velocity
  double w2 = -u/R*sin(theta);
  double w3 = u/R*cos(theta);
  // propagate the non-control state elements
  dx[0] = dt*0.5*(-quat[2]*w2-quat[3]*w3);
  dx[1] = dt*0.5*(-quat[3]*w2+quat[2]*w3);
  dx[2] = dt*0.5*(quat[0]*w2-quat[1]*w3);
  dx[3] = dt*0.5*(quat[1]*w2+quat[0]*w3);
  dx[4] = dt*omega;
  // update the state
  theta += dx[4];
  double normv = 0.0; // quaternion norm for renormalization
  for(int i = 0;i<4;i++)
  {
    quat[i] += dx[i];
    normv += quat[i]*quat[i];
  }
  normv = sqrt(normv);
  // renormalize quaternion
  for(int i = 0;i<4;i++)
  {
    quat[i] /= normv;
  }
  return;
}

void simple_sim_world::vehicle::set_control(double ui,double omegai)
{
  u = ui;
  omega = omegai;
}

simple_sim_world::world::world(double timein){
  // set the times
  t0 = timein;
  t = 0.0;
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
  double dt = timenow - t0 - t;
  // loop over each vehicle
  for(int i = 0;i<trackedVehicles.size();i++)
  {
    // allocate state
    std::vector<double> x(7,0.0);
    // propagated state
    std::vector<double> xprop(7,0.0);
    // extract the current state (which includes control terms for convenience)
    trackedVehicles[i].get_state(x,x[4],x[5],x[6]);
    trackedVehicles[i].propagate(dt);
    trackedVehicles[i].get_state(xprop,xprop[4],xprop[5],xprop[6]);
  }
  // update the time
  t = timenow-t0;
}

unsigned int simple_sim_world::world::get_num_vehicles()
{
  return (unsigned int)(trackedVehicles.size());
}

void simple_sim_world::world::update_velocity(double u, double theta, long id)
{
  for(int i = 0;i<trackedVehicles.size();i++)
  {
    // check for match
    if (id==trackedVehicles[i].get_id())
    {
      // update
      trackedVehicles[i].set_control(u,theta);
    }
  }
}
