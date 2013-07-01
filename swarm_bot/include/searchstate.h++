#pragma once

/*! \file searchstate.h++
  \brief Search state contoller.
*/
#define VERBOSE
#define CONSOLE_COLOR

#include <algorithm>
#include <vector>
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
      SER_SEARCH = 1,                              //!< search for objects
      SER_LOCK,                                    //!< lock on object
      SER_WATCH,                                   //!< watch objects
    };
  
  SearchSubState substate;                        //!< current internal state of controller
  double lockangle;                               //!< angle to observed object
  std::vector<double> lockdist;                   //!< measured distances
  double median;                                  //!< distance median
  
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
