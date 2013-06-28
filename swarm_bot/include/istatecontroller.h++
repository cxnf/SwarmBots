#pragma once

/*! \file istatecontroller.h++
  \brief Interface for bot states.
*/
#include "Aria.h"
#include "assist.h++"
#include "errcodes.h++"
#include "objectfinder.h++"
#include "robotmap.h++"

/*! \enum BroadcastState
  \brief Broadcast states.
  States a robot can broadcast over InitProc.
*/
enum BroadcastState
  {
    BS_UNDEFINED = 0,                             //!< undefined behavior
    
    BS_START = 1,                                 //!< start state sequence
    BS_ACTIVE,                                    //!< source id is active
    BS_NEXT_SEARCH,                               //!< next robot should search
    BS_NEXT_SIGNAL,                               //!< next robot should signal
    BS_FOUND,                                     //!< source identified target
    BS_MOVE,                                      //!< swarm should start moving
    BS_STOP,                                      //!< swarm should stop moving
  };

/*! \enum FState
  \brief Main states.
  Main states a robot can enter.
  Each state must have a controller to handle it.
*/
enum FState
  {
    FS_UNDEFINED = 0,                             //!< undefined behavior

    FS_WAIT = 1,                                  //!< wait for message on InitProc, restores previous state if any
    FS_SEARCH,                                    //!< search, watch and find possible leaders
    FS_SIGNAL,                                    //!< signal other robots
    FS_FOLLOW,                                    //!< follows parent robot based on graph
    FS_LEADER,                                    //!< leader of graph
  };

/*! \struct Devices
  \brief Devices created by bot.
  All available devices for the robot.
*/
struct Devices
{
  RobotMap *map;                                  //!< pointer to robot map
  ObjectFinder *finder;                           //!< pointer to object finder
  ArRobot *robot;                                 //!< pointer to robot
  bool moveallowed;                               //!< set when swarm should be moving
};

/*! \class IStateController
  \brief Interface for state controllers.
  Provides an interface for state controllers.
*/
class IStateController
{
public:
  /* \brief Destructor.
     Define desturctor to prevent undefined behavior.
  */
  virtual ~IStateController() { }
  
  /*! \brief Cyclic update handle for state contorller.
    Update handle for state controllers, its called at the same frequency as the main loop.
    If robot should change state, the state parameter is set to the proposed state.
    \param bot Robot devices.
    \param state out - New state or FS_UNDEFINED if no change requested.
    \param broadcast out - State to broadcast, if any.
    \return OK_SUCCESS or error code.
  */
  virtual int UpdateState(Devices *bot, FState *state, BroadcastState *broadcast) = 0;
};
