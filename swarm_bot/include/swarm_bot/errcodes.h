#pragma once

enum ErrCodes
  {
    OK_SUCCESS = 0,

    ERR_GENERAL,
    ERR_CONFIG,
    
    ERR_ROS_GENERAL,
    ERR_ROS_SERVICE,
    
    ERR_SWARM_CONTROLLER,

    ERR_ARIA_GENERAL,
    ERR_ARIA_CONNECTION,
    ERR_ARIA_PARSE,
    ERR_ARIA_SONAR,
    ERR_ARIA_LASER,

    _out_of_bounds,
  };
