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
class SignalState : public IStateController
{
private:
  /*! \enum SignalSubState
    \brief Internal states.
    Different states used internally by the controller.
  */
  enum SignalSubState
    {
      SIG_SEND = 1,                               //<! send sub state
      SIG_RESTORE,                                //<! restore sub state
      SIG_FINALIZE,                               //<! finalize sub state
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
    \param bot Available devices.
    \param state New state, if any.
    \param broadcast State to broadcast to other robots.
    \return OK_SUCCESS or error code.
  */
  virtual int UpdateState(Devices *bot, FState *state, BroadcastState *broadcast);

  /*! \brief Restore handle.
    Restores state controller.
    \param bot Available devices.
    \return OK_SUCCESS or error code.
  */
  virtual int Restoring(Devices *bot);
};
