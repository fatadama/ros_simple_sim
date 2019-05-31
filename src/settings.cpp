#include "simple_sim/settings.h"
#include <fstream>

#include <boost/program_options.hpp>
namespace po = boost::program_options;

simple_sim_settings::settings::settings(std::string settingspath){
  po::options_description config("Configuration");
  config.add_options()
    ("global.rate_hz",po::value<double>(&rate_hz)->default_value(10.0),"execution rate")
  ;
  po::variables_map vm;
  std::ifstream ifs(settingspath.c_str());
  store(parse_config_file(ifs, config), vm);
  notify(vm);
}

double simple_sim_settings::settings::get_rate_hz(){
  return rate_hz;
}
