// Copyright 2026 Atlantis
//
// Configuration passed to a plugin during initialize().
// Holding this in a struct keeps the plugin contract stable
// when new fields are added later.

#ifndef ATLANTIS_CORE__PLUGIN_CONFIG_HPP_
#define ATLANTIS_CORE__PLUGIN_CONFIG_HPP_

#include <string>

namespace atlantis_core
{

struct PluginConfig
{
  std::string name;   // YAML key, used for parameter scoping
  std::string type;   // C++ class name that was loaded
  std::string topic;  // ROS topic or service path the plugin should use
};

}  // namespace atlantis_core

#endif  // ATLANTIS_CORE__PLUGIN_CONFIG_HPP_
