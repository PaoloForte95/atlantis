#ifndef ATLANTIS_CORE__PLUGIN_CONFIG_HPP_
#define ATLANTIS_CORE__PLUGIN_CONFIG_HPP_

#include <string>

namespace atlantis_core
{

struct PluginConfig
{
  std::string name;  
  std::string type;   
  std::string topic;
};

}  

#endif 
