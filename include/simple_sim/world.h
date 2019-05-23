#ifndef __WORLD_H_INCLUDED__
#define __WORLD_H_INCLUDED__

#include <vector>

class world{
public:
  world();
  double x;
private:
  vector <vehicle> trackedVehicles; //!< list of vehicles that are known to this object
};

/*! A generic vehicle class used by a world object for truth histories */
class vehicle{

}

#endif
