#include "leaderstate.h++"

// ----------------- Constructors ------------------------------------------------------------------
LeaderState::LeaderState()
{
}

// ----------------- Destructors -------------------------------------------------------------------
LeaderState::~LeaderState()
{
}

// ----------------- Methods -----------------------------------------------------------------------
int LeaderState::UpdateState(Devices *bot, FState *state, BroadcastState *broadcast)
{
  if (!bot->moveallowed)
    {
      bot->robot->lock();
      bot->robot->setHeading(0);
      if (bot->robot->isHeadingDone())
	{
	  *broadcast = BS_MOVE;
	}
      bot->robot->unlock();
      return OK_SUCCESS;
    }
  else
    {
      bot->robot->lock();
      bot->robot->setVel2(80, 80);
      bot->robot->unlock();
    }

  return OK_SUCCESS;
}

int LeaderState::Restoring(Devices *bot)
{
  return ERR_FAIL;
}
