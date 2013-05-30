#pragma once

/*! \file followstate.h++
  \brief Follow state controller.
*/
#define VERBOSE
#define CONSOLE_COLOR

#include "istatecontroller.h++"

/*! \class FollowState
  \brief State controller for follow state.
  State controller for following another robot.
*/
class FollowState
{
private:

public:
  /*! \brief Default constructor.
    Initializes data members.
  */
  FollowState();
  /*! \brief Destructor.
    Frees allocated memory.
  */
  ~FollowState();

  /*! \brief Update handle.
    Updates state controller.
    \param bot SwarmBot main object.
    \param state New state, if any.
    \return OK_SUCCESS or error code.
  */
  virtual int UpdateState(SwarmBot *bot, FState *state);
};
