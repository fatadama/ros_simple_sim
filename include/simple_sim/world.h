#ifndef __WORLD_H_INCLUDED__
#define __WORLD_H_INCLUDED__

#include <vector>

namespace simple_sim_world{
  //! convert longitude (gamma) and latitude (phi) to Euler parameters
  void longLat2quat(double longitude, double latitude, std::vector <double>& quat);
  void quat2longLat(std::vector <double> quat, double& longitude, double& latitude);

  /*! A generic vehicle class used by a world object for truth histories */
  class vehicle{
  public:
    vehicle();  //!< Constructor for random initial longitude/latitude
    vehicle(long idin,double longitude, double latitude, double theta); //!< constructor
    void debug_print();   //!< Debugging function that prints current state
    long get_id();        //!< Return the id
    void get_state(std::vector <double>& beta,double& thet,double& u, double& omega);     //!< Extract state information for integration
  private:
    std::vector <double> quat; //!< orientation described by Euler parameters, scalar first
    double u;     //!< speed
    double omega; //!< heading rate (rads)
    double theta; //!< heading (rad)
    long id;      //!< unique identifier
  };

  class world{
  public:
    world(double timein);
    void init_vehicle(long id);  //!< Initialize a vehicle at a random long/lat
    bool is_known_vehicle(long id); //!< Return true if the id is in the list of trackedVehicles
    void step(double timenow);  //!< Perform Euler integration for dt seconds
  private:
    std::vector <vehicle> trackedVehicles; //!< list of vehicles that are known to this object
    double t;  //!< Current time
  };

};

#endif
