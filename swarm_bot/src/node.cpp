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

// ----------------- Destructors -------------------------------------------------------------------
Node::~Node()
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

void Node::AddChild(Node *n)
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
	      if (p != l->end())
		{
		  l->push_back((*it)->id);
		}
	      else if ((*it)->HasChild(id, l))
		{
		  return true;
		}
	    }
	}
      return false;
    }
}

Node* Node::FindRoot(std::list<int> *l)
{
  if (this->leader)
    {
      std::list<int>::iterator p = std::find(l->begin(), l->end(), this->leader->id);
      if (p == l->end())
	{
	  l->push_back(this->leader->id);
	  return this->leader->FindRoot(l);
	}
      else
	{
	  return NULL;
	}
    }
  return this;
}

void Node::Print(int depth)
{
  for (int i = 0; i < depth; ++i)
    printf("|");
  PRINT(MAGENTA "-Node [%d]", this->id);
  for (std::list<Node*>::iterator it = this->followers.begin(); it != this->followers.end(); ++it)
    (*it)->Print(depth + 1);
}
