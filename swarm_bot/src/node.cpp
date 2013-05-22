#include "node.h++"

// ----------------- Constructors ------------------------------------------------------------------
Node::Node() : id(0),
	       leader(0),
	       followers()
{
}
Node::Node(int id) : id(id),
		     leader(0),
		     followers()
{
}

// ----------------- Methods -----------------------------------------------------------------------
int Node::GetID()
{
  return this->id;
}

Node* Node::GetLeader()
{
  return this->leader;
}

void Node::Add(Node *n)
{
  this->followers.push_back(n);
  if (!n->leader)
    {
      n->leader = this;
    }
}

bool Node::HasChild(int id, std::list<int> *l)
{
  if (l == 0)
    {
      l = new std::list<int>;
      bool b = this->HasChild(id, l);
      delete l;
      return b;
    }
  else
    {
      for (std::list<Node*>::iterator it = this->followers.begin(); it != this->followers.end(); ++it)
	{
	  if ((*it)->id == id)
	    {
	      return true;
	    }
	  else
	    {
	      std::list<int>::iterator p = std::find(l->begin(), l->end(), (*it)->id);
	      // if (p == l->end())
	      // {
		  l->push_back((*it)->id);
		  if ((*it)->HasChild((*it)->id, l))
		    return true;
		  // }
	    }
	}
      return false;
    }
}
