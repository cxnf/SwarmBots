#pragma once

/*! \file robotmap.h++
  \brief Lists robots in swarm.
*/

#include <algorithm>
#include <list>
#include <map>
#include <string>
#include <vector>
#include "errcodes.h++"

/*! \class Robot
  \brief Node in graph.
  A node in the robot graph.
*/
class Robot
{
public:
  int robot;                                      //<! id of robot represented by the node
  Robot *leader;                                  //<! pointer to robot followed by this one, if any
  std::list<Robot*> followers;                    //<! pointers to robots following this one, if any

  Robot(): robot(0), leader(0), followers() { }
  Robot(int staticID) : robot(staticID), leader(0), followers() { }
};

/*! \class RobotMap
  \brief Robot list and graph.
  Lists a robots present is swarm.
  Maintains a graph of current formation.
*/
class RobotMap
{
private:
  std::map<int, Robot> robots;                    //<! list of known robots, including map owner
  std::vector<int> priorities;                    //<! robot priorities, value is id and index is priority, lower index is higher priority

public:
  /*!
    \brief Initializes fields.
    Initializes all fields.
  */
  RobotMap();
  
  
  int Add(int staticID);
  void Heartbeat(int staticID);
  bool Increment(std::list<int> *lost);
  
  int GetPriority(int staticID);
  int GetStaticID(unsigned int priority);
};
