#pragma once

/*! \file errcodes.h++
  \brief Error code list.
 */


/*! \enum ErrorCodes
  \brief Error codes.
  List of all error codes used by the SwarmBot project.
 */
enum ErrorCodes
  {
    // ------------- Success code ------------------------------------------------------------------
    OK_SUCCESS = 0,                               //!< no error, everything went fine

    // ------------- Error codes -------------------------------------------------------------------
    ERR_GENERAL,                                  //!< undefined error
    ERR_FAIL,                                     //!< out parameters values undefined, no internal error or fully recovered

    ERR_SWARM_CONTROLLER,                         //!< swarm controller response invalid
    ERR_SWARM_MAP,                                //!< robot map error
    ERR_SWARM_FINDER,                             //!< range finder error
    ERR_SWARM_CYCLE,                              //!< loop in graph error

    ERR_ROS_SERVICE,                              //!< ros service failed to send response

    ERR_ARIA_CONNECTION,                          //!< an aria connector failed to init or disconnected
    ERR_ARIA_LASER,                               //!< aria laser device related error

    ___OutOfBounds                                //!< if you get this, you screwed up
  };
