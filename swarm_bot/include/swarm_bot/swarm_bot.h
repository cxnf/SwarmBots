#pragma once

#include <cmath>
#include <csignal>
#include <sys/utsname.h>
#include <boost/regex.hpp>
#include "ros/ros.h"
#include "swarm_bot/errcodes.h"
#include "swarm_bot/robot.h"
#include "swarm_bot/AriaInterface.h"
#include "tf/transform_datatypes.h"

#include "swarm_bot/Heartbeat.h"
#include "swarm_bot/Announce.h"
#include "sensor_msgs/PointCloud.h"
#include "nav_msgs/Odometry.h"

#define FREQUENCY 10                              // loop frequency
#define PI2       6.283185307                     // Pi times two


typedef enum
  {
    LS_SEEK,
    LS_WAIT_OTHERS,
    LS_SEEK_SIGNAL,
    LS_SEND_SIGNAL,
    
  }  LoopState;

//! Modulo for floats.
/*!
  Returns the remainder of a division.
  \param x Numinator.
  \param y Denominator.
  \return Remainder of division.
 */
float inline Mod(float x, float y);

//! Signal handler.
/*!
  Handles SIGINT signals.
  \param s Signal intger.
 */
void SIGINTHandler(int s);
