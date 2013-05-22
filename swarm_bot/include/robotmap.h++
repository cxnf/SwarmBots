#pragma once

/*! \file robotmap.h++
  \brief Lists robots in swarm.
*/
#define DEBUG
#define VERBOSE
#define CONSOLE_COLOR

#include <algorithm>
#include <map>
#include <vector>

#include "assist.h++"
#include "errcodes.h++"
#include "node.h++"

/*! \class RobotMap
  \brief Robot list and graph.
  Lists a robots present is swarm.
  Maintains a graph of current formation.
*/
class RobotMap
{
private:
  std::map<int, Node*> robots;                    //<! list of known robots, including map owner
  std::vector<int> priorities;                    //<! robot priorities, value is id and index is priority, lower index is higher priority

public:
  /*! \brief Initializes fields.
    Initializes all fields.
  */
  RobotMap();

  bool isleader(int id);
  
  /*! \brief Add robot to map.
    Adds a robot with given id to the map.
    \param staticID Id of robot to add.
    \return OK_SUCCESS or error code.
  */
  int Add(int staticID);
  /*! \brief Handles heartbeat.
    Processes a heartbeat from robot with given id.
    \param staticID Id of robot.
   */
  void Heartbeat(int staticID);
  /*! \brief Increments heartbeat system.
    Increments all robots in map.
    \param lost List of robots no longer sending heartbeats, if any.
    \return Value indicating one or more robots where removed.
  */
  bool Increment(std::list<int> *lost);

  /*! \brief Links a robot to another.
    Creates a link from one robot to another.
    \param source Id of robot creating link.
    \param target Id of robot to create link to.
    \return OK_SUCCESS or error code.
  */
  int Link(int source, int target);
  
  /*! \brief Gets priority of id.
    Returns the priority the given id is mapped to.
    \param staticID Id to check.
    \return Priority of id or -1 when id is unkown.
  */
  int GetPriority(int staticID);
  /*! \brief Gets id with priority.
    Returns the id mapped to given priority.
    \param priority Priority to check.
    \return Id mapped priority or 0 when priority out of bounds.
  */
  int GetStaticID(unsigned int staticID);
  /*! \brief Gets the id of next robot.
    Returns the id of the robot after the robot with given id. This is based on priorities.
    \param staticID Id of current robot.
    \return Id of next robot or 0 if given robot is last.
  */
  int GetNextID(int staticID);

  /*! \brief Gets id of leader.
    Returns the id of the leader.
    \return Id of leader, or 0 if none found.
  */
  int GetLeader();
  /*! \brief Gets whether graph has multiple roots.
    Returns a value indicating the graph has more than one root node.
    \return More than one root.
  */
  bool HasMultipleLeaders();
};
