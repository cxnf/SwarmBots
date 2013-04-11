#pragma once

#include <libconfig.h++>
#include <iomanip>
#include "ros/ros.h"

//! Configuration for swarm bot.
/*!
  Storage for configurable values.
 */
struct Configuration
{
  //! Name used by the robot and node.
  std::string name;
  //! Namespace of RosAria.
  std::string basename;
};

//! Loads configuration from a config file.
/*!
  Loads configuration from a config file.
  \param cfg Pointer to a configuration.
  \param path Path to config file.
  \return Success of load operation.
 */
int32_t LoadConfig(Configuration *cfg, const char *path);
