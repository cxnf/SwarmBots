#pragma once

/*! \file signalstate.h++
  \brief Signal state controller.
*/
#define VERBOSE
#define CONSOLE_COLOR

#include "istatecontroller.h++"

/*! \class SignalState
  \brief State controller for signal state.
  State controller for signalling and restoring state.
*/
class SignalState
{
private:
  /*! \enum SignalSubState
    \brief Internal states.
    Different states used internally by the controller.
  */
  enum SignalSubState
    {
      SS_SEND = 1,
      SS_RESTORE,
      SS_FINALIZE,
    };
  
  SignalSubState substate;                        //<! current internal state of controller
  double restore;                                 //<! angle to restore to

public:
  /*! \brief Default constructor.
    Initializes data members.
  */
  SignalState();
  /*! \brief Destructor.
    Frees allocated memory.
   */
  ~SignalState();

  /*! \brief Update handle.
    Updates state controller.
    \param bot SwarmBot main object.
    \param state New state, if any.
    \return OK_SUCCESS or error code.
  */
  virtual int UpdateState(SwarmBot *bot, FState *state);
};
