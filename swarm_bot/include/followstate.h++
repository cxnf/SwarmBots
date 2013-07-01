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
class FollowState : public IStateController
{
private:
  double prevx;                                   //!< initial x measurement
  double prevy;                                   //!< initial y measurement
  bool init;                                      //!< is initial measurement made

  /*! \brief Converts angle/distance to vector.
    Converts an angle distance pair to a vector.
    \param angle Angle to convert.
    \param distance Distance to convert.
    \param x out - X component of vector.
    \param y out - Y component of vector.
  */
  void Convert(double angle, double distance, double *x, double *y);

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
