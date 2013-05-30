#pragma once

/*! \file searchstate.h++
  \brief Search state contoller.
*/
#define VERBOSE
#define CONSOLE_COLOR

#include "istatecontroller.h++"

/*! \class SearchState
  \brief State controller for search state.
  State controller for searching, watching and finding of other robots.
*/
class SearchState : public IStateController
{
private:
  /*! \enum SearchSubState
    \brief Internal states.
    Different states used internally by the controller.
  */
  enum SearchSubState
    {
      SS_SEARCH = 1,                              //!< search for objects
      SS_LOCK,                                    //!< lock on object
      SS_WATCH,                                   //!< watch objects
    };
  
  SearchSubState substate;                        //!< current internal state of controller
  double lockangle;                               //!< angle to observed object
  
public:
  /*! \brief Default constructor.
    Initializes data members.
  */
  SearchState();
  /*! \brief Destructor.
    Frees allocated memory.
   */
  ~SearchState();
  
  /*! \brief Update handle.
    Updates state controller.
    \param bot SwarmBot main object.
    \param state New state, if any.
    \return OK_SUCCESS or error code.
  */
  virtual int UpdateState(SwarmBot *bot, FState *state);
};
