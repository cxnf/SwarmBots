#pragma once

#include <map>
#include <string>
#include "ros/ros.h"
#include "swarm_bot/formation.h"
#include "swarm_bot/robot.h"

#include "swarm_bot/Heartbeat.h"
#include "swarm_bot/Announce.h"
#include "swarm_bot/Task.h"

#define FREQUENCY 10                              // loop frequency

typedef std::map<int32_t, Robot> RobotMap;        // define map to link robot headers to static ids
typedef std::map<int32_t, Robot>::iterator RobotIterator; // define iterator of robot map

//! Returns a value indicating map contains given robot.
/*!
  Iterates through the map and returns a value indicating a robot with given static id exists in it.
  \return Value indicating robot is known.
 */
bool ContainsValue(RobotMap *map, int32_t StaticID);
