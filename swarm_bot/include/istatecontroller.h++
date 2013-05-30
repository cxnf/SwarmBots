#pragma once

/*! \file istatecontroller.h++
  \brief Interface for bot states.
*/
#include "Aria.h"
#include "assist.h++"
#include "errcodes.h++"
#include "swarmbot.h++"

/*! \enum BroadcastState
  \brief Broadcast states.
  States a robot can broadcast over InitProc.
*/
enum BroadcastState
  {
    BS_START = 1,                                 //!< start state sequence
    BS_NEXT,                                      //!< turn goes to next robot in priority list
    BS_FOUND,                                     //!< source identified target
  };

/*! \enum FState
  \brief Main states.
  Main states a robot can enter.
  Each state must have a controller to handle it.
*/
enum FState
  {
    FS_UNDEFINED = 0,                             //!< undefined behavior

    FS_INI_WAIT = 1,                              //!< wait for message on InitProc
    FS_INI_SEARCH,                                //!< search, watch and find possible leaders
    FS_INI_SIGNAL,                                //!< signal other robots
    
    FS_RUN_FOLLOW,                                //!< follows parent robot based on graph
    FS_RUN_LEADER,                                //!< leader of graph
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
  ~IStateController() { }
  
  /*! \brief Cyclic update handle for state contorller.
    Update handle for state controllers, its called at the same frequency as the main loop.
    If robot should change state, the state parameter is set to the proposed state.
    \param bot Pointer to main robot object.
    \param state out - New state or FS_UNDEFINED if no change requested.
    \return OK_SUCCESS or error code.
  */
  virtual int UpdateState(SwarmBot *bot, FState *state) = 0;
}