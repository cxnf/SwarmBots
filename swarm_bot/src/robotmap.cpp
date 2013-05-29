#include "robotmap.h++"

// ----------------- Constructors ------------------------------------------------------------------
RobotMap::RobotMap() : robots()
{
}

// ----------------- Destructors -------------------------------------------------------------------

// ----------------- Methods -----------------------------------------------------------------------
int RobotMap::Add(int staticID)
{
  if (this->robots.count(staticID))
    {
      return ERR_SWARM_MAP;
    }
  this->robots[staticID] = new Node(staticID);
  this->priorities.push_back(staticID);
  std::sort(this->priorities.begin(), this->priorities.end());
  return OK_SUCCESS;
}

void RobotMap::Heartbeat(int staticID)
{
  if (this->robots.count(staticID))
    {
      // TODO: implement heartbeat system
    } else
    {
      this->Add(staticID);
    }
}

bool RobotMap::Increment(std::list<int> *lost)
{
  return false;
}

int RobotMap::Link(int source, int target)
{
  if (this->robots.count(source) && this->robots.count(target))
    {
      Node *r0 = this->robots[source];
      Node *r1 = this->robots[target];
      if (r1->HasChild(r0->GetID()))
	{
	  return ERR_SWARM_CYCLE;
	}
      else
	{
	  Node *l = r1->GetLeader();
	  if (l)
	    {
	      if (l->GetID() == source)
		{
		  return ERR_SWARM_CYCLE;
		}
	    }
	  r1->Add(r0);
	  return OK_SUCCESS;
	}
    } else
    {
      return ERR_FAIL;
    }
}

int RobotMap::GetPriority(int staticID)
{
  for (unsigned int i = 0; i < this->priorities.size(); ++i)
    {
      if (this->priorities[i] == staticID)
	{
	  return i;
	}
    }
  return -1;
}

int RobotMap::GetStaticID(unsigned int priority)
{
  if (priority >= this->priorities.size())
    {
      return 0;
    }
  return this->priorities[priority];
}

int RobotMap::GetNextID(int staticID)
{
  int p = this->GetPriority(staticID);
  if (p < 0)
    {
      return 0;
    }
  return this->GetStaticID(p + 1);
}

bool RobotMap::isleader(int id)
{
  if (this->robots.count(id))
    {
      Node *r = this->robots[id];
      return r->GetLeader() == 0;
    } else
    {
      return false;
    }
}

int RobotMap::GetLeader()
{
  for (std::map<int, Node*>::iterator it = this->robots.begin(); it != this->robots.end(); ++it)
    {
      if (!it->second->GetLeader())
	{
	  return it->second->GetID();
	}
    }
  return 0;
}

bool RobotMap::HasMultipleLeaders()
{
  int count = 0;
  for (std::map<int, Node*>::iterator it = this->robots.begin(); it != this->robots.end(); ++it)
    {
      if (!it->second->GetLeader())
	{
	  count++;
	}
    }
  return count > 1;
}
