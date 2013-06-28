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
      PRINT(CYAN "New test");
      l = new std::list<int>;
      bool b = this->HasChild(id, l);
      delete l;
      return b;
    }
  else
    {
      if (this->followers.size()) { PRINT(CYAN " Test [%d][%d]", id, this->id); }
      else { PRINT(CYAN " No test"); }

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
		  PRINT(CYAN "  Skipping [%d]", (*it)->id);
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

void Node::Print(int depth)
{
  for (int i = 0; i < depth; ++i)
    printf(" ");
  PRINT(MAGENTA "Node [%d]", this->id);
  for (std::list<Node*>::iterator it = this->followers.begin(); it != this->followers.end(); ++it)
    (*it)->Print(depth + 1);
}
