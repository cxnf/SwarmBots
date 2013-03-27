#pragma once

#include <map>
#include <string>
#include "ros/ros.h"
#include "swarm_bot/Heartbeat.h"
#include "swarm_bot/Announce.h"
#include "swarm_bot/Task.h"
#include "swarm_bot/robot.h"

#define FREQUENCY 10                              // loop frequency

typedef std::map<int32_t, Robot> RobotMap;        // define map to link robot headers to static ids
typedef std::map<int32_t, Robot>::iterator RobotIterator; // define iterator of robot map
