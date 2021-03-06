#include "robotmap.h++"

// ----------------- Constructors ------------------------------------------------------------------
RobotMap::RobotMap() : robots(),
		       priorities()
{
}

// ----------------- Destructors -------------------------------------------------------------------
RobotMap::~RobotMap()
{
}

// ----------------- Methods -----------------------------------------------------------------------
int RobotMap::AddRobot(int id)
{
  if (this->robots.count(id))
    {
      return ERR_SWARM_MAP;
    }
  this->robots[id] = new Node(id);
  this->priorities.push_back(id);
  std::sort(this->priorities.begin(), this->priorities.end());
  return OK_SUCCESS;
}

int RobotMap::LinkRobots(int source, int target)
{
  PRINT(YELLOW "Linking... S:%d T:%d", source, target);
  if (this->robots.count(source) && this->robots.count(target))
    {
      Node *r0 = this->robots[source];
      Node *r1 = this->robots[target];

      std::list<int> l;
      Node *root = r1->FindRoot(&l);
      if (root)
	{
	  PRINT(BLUE "Root is [%d]", root->GetID());
	  if (root->GetID() == source)
	    {
	      PRINT(RED "Cycle attempt to root");
	      return ERR_SWARM_CYCLE;
	    }
	  else if (root->HasChild(r0->GetID()))
	    {
	      PRINT(RED "Cycle attempt");
	      return ERR_SWARM_CYCLE;
	    }
	  else
	    {
	      r1->AddChild(r0);
	      return OK_SUCCESS;
	    }
	}
      else
	{
	  PRINT(RED "Cyclic graph");
	  return ERR_SWARM_CYCLE;
	}

      /*
      if (r1->HasChild(r0->GetID()))
	{
	  PRINT(RED "Link to child attempt");
	  return ERR_SWARM_CYCLE;
	}
      else
	{
	  Node *l = r1->GetLeader();
	  if (l)
	    {
	      if (l->GetID() == source)
		{
		  PRINT(RED "Link to leader attempt");
		  return ERR_SWARM_CYCLE;
		}
	    }
	  r1->AddChild(r0);
	  return OK_SUCCESS;
	}
      */
    }
  else
    {
      PRINT(RED "Source or target missing");
      return ERR_FAIL;
    }
}

int RobotMap::GetGraphCount()
{
  int count = 0;
  for (std::map<int, Node*>::iterator it = this->robots.begin(); it != this->robots.end(); ++it)
    {
      if (!it->second->GetLeader())
	{
	  count++;
	}
    }
  return count;
}

int RobotMap::GetGraphLeader(int graph)
{
  int count = 0;
  for (std::map<int, Node*>::iterator it = this->robots.begin(); it != this->robots.end(); ++it)
    {
      if (!it->second->GetLeader())
	{
	  if (++count == graph || !graph)
	    {
	      return it->second->GetID();
	    }
	}
    }
  return 0;
}




void RobotMap::Heartbeat(int id)
{
  if (this->robots.count(id))
    {
      // TODO: implement heartbeat system
    }
  else
    {
      this->AddRobot(id);
    }
}

bool RobotMap::Increment(std::list<int> *lost)
{
  return false;
}





int RobotMap::GetPriority(int id)
{
  for (unsigned int i = 0; i < this->priorities.size(); ++i)
    {
      if (this->priorities[i] == id)
	{
	  return i;
	}
    }
  return -1;
}

int RobotMap::GetID(unsigned int priority)
{
  if (priority >= this->priorities.size())
    {
      return 0;
    }
  return this->priorities[priority];
}

int RobotMap::GetNextID(int id)
{
  int p = this->GetPriority(id);
  if (p < 0)
    {
      return 0;
    }
  return this->GetID(p + 1);
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

void RobotMap::Print()
{
  int id = this->GetGraphLeader(0);
  this->robots[id]->Print();
}
