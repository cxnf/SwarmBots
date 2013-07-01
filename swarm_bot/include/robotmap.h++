#pragma once

/*! \file robotmap.h++
  \brief Lists robots in swarm.
*/
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
  /*! \brief Destructor.
    Frees allocated memory.
  */
  ~RobotMap();
  
  /*! \brief Add robot to map.
    Adds a robot with given id to the map.
    \param id Id of robot to add.
    \return OK_SUCCESS or error code.
  */
  int AddRobot(int id);
  /*! \brief Links a robot to another.
    Creates a link from one robot to another.
    \param source Id of robot creating link.
    \param target Id of robot to create link to.
    \return OK_SUCCESS or error code.
  */
  int LinkRobots(int source, int target);
  /*! \brief Returns graph count.
    Returns the amount of graphs in the swarm.
    \return Graph count.
  */
  int GetGraphCount();
  /*! \brief Returns leader of graph.
    Returns the id of the leading robot for the graph with given id.
    \return Graph leader.
  */
  int GetGraphLeader(int graph);
  
  
  /*! \brief Handles heartbeat.
    Processes a heartbeat from robot with given id.
    \param id Id of robot.
  */
  void Heartbeat(int id);
  /*! \brief Increments heartbeat system.
    Increments all robots in map.
    \param lost List of robots no longer sending heartbeats, if any.
    \return Value indicating one or more robots where removed.
  */
  bool Increment(std::list<int> *lost);
  
  /*! \brief Gets priority of id.
    Returns the priority the given id is mapped to.
    \param id Id to check.
    \return Priority of id or -1 when id is unkown.
  */
  int GetPriority(int id);
  /*! \brief Gets id with priority.
    Returns the id mapped to given priority.
    \param priority Priority to check.
    \return Id mapped priority or 0 when priority out of bounds.
  */
  int GetID(unsigned int priority);
  /*! \brief Gets the id of next robot.
    Returns the id of the robot after the robot with given id. This is based on priorities.
    \param id Id of current robot.
    \return Id of next robot or 0 if given robot is last.
  */
  int GetNextID(int id);
  
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

  /*! \brief Prints graph.
    Prints the first encountered graph.
    This is UNSAFE in cyclic graphs.
  */
  void Print();
};
