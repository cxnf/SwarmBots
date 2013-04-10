#pragma once

#include <vector>

#include "ros/ros.h"
#include "swarm_bot/vector3f.h"

//! Interface for a formation.
/*!
  Provides an interface which defines the location of a robot within a formation.
 */
class FormationProvider
{
 public:
  virtual ~FormationProvider() {}
  
  //! Returns offset from formation center. 
  /*!
    Returns the offset from the center of the formation based on the dynamic id.
    \param DynamicID Dynamic id.
    \return Offset from center.
   */
  virtual Vector3f GetOffset(int32_t DynamicID) = 0;
};

//! Formation manager.
/*!
  Manages formation of the swarm.
 */
class Formation
{
 private:
  std::vector<int32_t> tasklist;                  // links dynamic id (index+1) to a static id
  FormationProvider *fprov;                       // formation provider

 public:
  //! Creates a Formation.
  /*!
    Creates a Formation.
    A FormationProvider implementation must be set before it is usable.
   */
  Formation();
  //! Destroys the Formation.
  /*!
    Destroys the formation.
    If a FormationProvider is set, its is also destroyed.
   */
  ~Formation();

  //! Assigns a robot a static id.
  /*!
    Adds a robot by static id to the formation.
    A new robot is automaticly assigned a dynamic id.
    For a robot already in the formation, the assigned dynamic id is returned.
    \param StaticID Static id of the robot to add.
    \return Assigned Dynamic id.
   */
  int32_t Assign(int32_t StaticID);

  
  //! Unassings a robot.
  /*!
    Removes a robot by static id from the formation.
    The remaining robots are invalidated and returns a value indicating the formation must be restored.
    \param StaticID Static id of robot to remove.
    \param count out, contains amount of robots that are reassigned.
    \param invSID out, Static ids of reassigned robots.
    \param invDID out, Dynamic ids of reassigned robots.
    \return value indicating one or more robots where reassigned.
   */
  bool Unassign(int32_t StaticID, int32_t *count, int32_t **invSID, int32_t **invDID);
};
