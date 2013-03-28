#include "swarm_bot/formation.h"

Formation::Formation(): tasklist(), fprov(0)      // initialize non-pointer members
{
}
Formation::~Formation()
{
  if (this->fprov)                                // if a FormationProvider is set
    {
      delete this->fprov;                         // destroy the provider
    }
}


int32_t Formation::Assign(int32_t StaticID)
{
  int i = 1;
  for (std::vector<int32_t>::iterator it = this->tasklist.begin(); it != this->tasklist.end(); it++, i++)
    {
      if ((*it) == StaticID)
	{
	  return i;
	}
    }
  this->tasklist.push_back(StaticID);
  return this->tasklist.size();
}

bool Formation::Unassign(int32_t StaticID, int32_t *count, int32_t **invSID, int32_t **invDID)
{
  std::vector<int32_t> tmpstack;
  while (!this->tasklist.empty())
    {
      int32_t sid = this->tasklist.back();
      this->tasklist.pop_back();
      if (sid == StaticID)
	{
	  break;
	}
      else
	{
	  tmpstack.push_back(sid);
	}
    }
  (*count) = tmpstack.size();
  if (!(*count))
    {
      return false;
    }
  (*invSID) = new int32_t[(*count)];
  (*invDID) = new int32_t[(*count)];
  for (int i = 0; i < (*count) && !tmpstack.empty(); i++)
    {
      int32_t sid = tmpstack.back();
      this->tasklist.push_back(sid);
      (*invSID)[i] = sid;
      (*invDID)[i] = this->tasklist.size();
    }
  return true;
}
