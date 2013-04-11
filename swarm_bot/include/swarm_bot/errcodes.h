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

    _out_of_bounds,
  };
