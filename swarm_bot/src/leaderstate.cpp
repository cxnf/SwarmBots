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
virtual int LeaderState::UpdateState(SwarmBot *bot, FState *state)
{
  return OK_SUCCESS;
}
