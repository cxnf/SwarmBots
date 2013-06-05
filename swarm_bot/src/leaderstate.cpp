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
  bot->robot->lock();
  bot->robot->setVel2(100, 100);
  bot->robot->unlock();

  return OK_SUCCESS;
}
