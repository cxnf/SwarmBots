#pragma once

/*! \file node.h++
  \brief Node in robot graph.
*/

#include <algorithm>
#include <list>
#include "errcodes.h++"


/*! \class Node
  \brief Node in graph.
  A node in the robot graph.
*/
class Node
{
private:
  int id;                                         //<! id of robot represented by the node
  Node *leader;                                   //<! pointer to robot followed by this one, if any
  std::list<Node*> followers;                     //<! pointers to robots following this one, if any

public:
  Node();
  Node(int id);

  /*! \brief Returns id.
    Returns the id of the robot.
    \return Robot id.
  */
  int GetID();

  /*! \brief Returns leader.
    Return the leader of this robot.
    \return Pointer to leader.
  */
  Node* GetLeader();

  /*! \brief Adds a node as child.
    Adds given node as a child node of this node.
    \param n Node to link to.
  */
  void Add(Node *n);

  /*! \brief Returns value indicating robot has child with id.
    Looks through followers recursively to determine if the robot is a follower.
    This is safe to use in a cyclic graph.
    \param id Id of robot to seek.
    \param l List of previously encountered ids, NULL if first call.
    \return Value indicating id 
  */
  bool HasChild(int id, std::list<int> *l = 0);
};
