#include "followstate.h++"

// ----------------- Constructors ------------------------------------------------------------------
FollowState::FollowState()
{
}

// ----------------- Destructors -------------------------------------------------------------------
FollowState::~FollowState()
{
}

// ----------------- Methods -----------------------------------------------------------------------
virtual int FollowState::UpdateState(SwarmBot *bot, FState *state)
{
  return OK_SUCCESS;
}
