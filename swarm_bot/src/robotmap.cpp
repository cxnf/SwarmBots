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
    return ERR_SWARM_MAP;
  this->robots[staticID] = Robot(staticID);
  this->priorities.push_back(staticID);
  std::sort(this->priorities.begin(), this->priorities.end());
  return OK_SUCCESS;
}

void RobotMap::Heartbeat(int staticID)
{
  if (this->robots.count(staticID))
    {
      
    }
  else this->Add(staticID);
}

bool RobotMap::Increment(std::list<int> *lost)
{
  return false;
}

int RobotMap::GetPriority(int staticID)
{
  for (unsigned int i = 0; i < this->priorities.size(); ++i)
    if (this->priorities[i] == staticID)
      return i;
  return -1;
}

int RobotMap::GetStaticID(unsigned int priority)
{
  if (priority >= this->priorities.size())
    return 0;
  return this->priorities[priority];
}
