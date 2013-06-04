#pragma once

/*! \file leaderstate.h++
  \brief Leader state controller.
*/
#define VERBOSE
#define CONSOLE_COLOR

#include "istatecontroller.h++"

/*! \class LeaderState
  \brief State controller for leader state.
  State controller for leading the formation.
*/
class LeaderState : public IStateController
{
private:

public:
  /*! \brief Default constructor.
    Initializes data members.
  */
  LeaderState();
  /*! \brief Destructor.
    Frees allocated memory.
   */
  ~LeaderState();

  /*! \brief Update handle.
    Updates state controller.
    \param bot Available devices.
    \param state New state, if any.
    \return OK_SUCCESS or error code.
  */
  virtual int UpdateState(Devices *bot, FState *state, BroadcastState *broadcast);
};
