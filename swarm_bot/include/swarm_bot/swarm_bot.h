#pragma once

#include "ros/ros.h"
#include "swarm_bot/robot.h"
#include <cmath>
#include "tf/transform_datatypes.h"

#include "swarm_bot/Heartbeat.h"
#include "swarm_bot/Announce.h"
#include "sensor_msgs/PointCloud.h"
#include "nav_msgs/Odometry.h"

#define FREQUENCY 10                              // loop frequency
#define PI2       6.283185307                     // Pi times two

float inline Mod(float x, float y);
