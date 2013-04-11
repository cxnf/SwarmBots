#pragma once

#include <libconfig.h++>
#include "ros/ros.h"

//! Configuration for swarm bot.
/*!
  
 */
struct Configuration
{
  std::string name;
  std::string basename;
};

int32_t LoadConfig(Configuration *cfg, const char *path);
int32_t LoadParams(Configuration *cfg);
int32_t Load(Configuration *cfg);
