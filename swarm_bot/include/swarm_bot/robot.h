#pragma once

#include <string>
#include "ros/ros.h"

//! Robot class.
/*!
  Logical representation of a robot.
 */
class Robot
{
 private:
  int32_t staticID;                               // static id by which a robot is known
  int32_t dynamicID;                              // dynamic id which positions a robot in the formation
  int32_t heartbeat;                              // time passed since last heartbeat
  int32_t missed;                                 // amount of heartbeats missed in succession
  std::string name;                               // robot name
  Vector3f *location;                             // estimated location of the robot

 public:
  //! Default constructor.
  /*!
    Constructor which zeroes out each member, causing StaticID to be invalid.
   */
  Robot();
  //! Main constructor.
  /*!
    Constructor which initializes the class with provided data.
    \param n Robot name, must be unique.
    \param sid StaticID assigned to robot, must be unique.
   */
  Robot(std::string n, int32_t sid);

  //! Returns StaticID.
  /*!
    Returns the StaticID which is assigned to this robot.
    Must be greater than 0 to be valid.
    \return StaticID of robot.
   */
  int32_t GetStaticID();

  //! Returns DynamicID.
  /*!
    Returns the DynamicID which is assigned to this robot.
    Must be greater than 0 to be valid.
    \return DynamicID of robot.
   */
  int32_t GetDynamicID();

  //! Returns robot name.
  /*!
    Returns the name of the robot.
    \return Name of robot.
   */
  std::string GetName();

  //! Returns Alive state.
  /*!
    Returns a value indicating if the actual robot keeps its StaticID alive.
    \return Value indicating StaticID is alive.
   */
  bool IsAlive();

  //! Returns missed heartbeats.
  /*!
    Returns the total amount of heartbeats missed in succession.
    \return Missed heartbeats since last heartbeat.
   */
  int32_t GetMissedHeartbeats();

  //! Updates StaticID state and returns if state changed.
  /*!
    Updates the state of the StaticID if the StaticID is still alive.
    It returns a value indicating if the state of StaticID changed.
    \return Value indicating StaticID state changed.
   */
  bool Increment();
  
  //! Updates heartbeat.
  /*!
    Refreshes the StaticID state if the StaticID is still alive.
   */
  void Heartbeat();

  //! Sets DynamicID.
  /*!
    Assigns a DynamicID to the robot.
    \param did New DynamicID.
   */
  void SetDynamicID(int32_t did);
  
  //! Get location.
  /*!
    Returns a value indicating the robot has a location.
    When the robot has a location it is set to 'v0'.
    \param v0 Location of the robot.
    \return Value indicating robot has a location.
   */
  bool GetLocation(Vector3f *v0);
};
