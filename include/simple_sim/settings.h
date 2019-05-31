#ifndef __SIMPLE_SETTINGS_H_INCLUDED__
#define __SIMPLE_SETTINGS_H_INCLUDED__

#include <string>

namespace simple_sim_settings{
  class settings{
  public:
    settings(std::string settingspath); //!< Constructor
    double get_rate_hz(); //!< Return the rate in Hz
  private:
    double rate_hz;       //!< The rate in Hz
  };
};

#endif
