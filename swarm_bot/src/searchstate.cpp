#include "searchstate.h++"

// ----------------- Constructors ------------------------------------------------------------------
SearchState::SearchState() : substate(SS_SEARCH),
			     lockangle(0)
{
}

// ----------------- Destructors -------------------------------------------------------------------
SearchState::~SearchState()
{
}

// ----------------- Methods -----------------------------------------------------------------------
virtual int SearchState::UpdateState(SwarmBot *bot, FState *state)
{
  switch (this->substate)
    {
    case SS_SEARCH:
      {
	double d;
	bot->robot->lock();
	if (bot->robot->isHeadingDone())
	  {
	    ArPose p = bot->robot->GetEncoderPose();
	    if (!bot->finder->GetClosestObject(&this->lockangle, &d))
	      {
		this->substate = SS_LOCK;
	      }
	    else
	      {
		bot->robot->setHeading(p.getTh() + 90);
	      }
	  }
	robot->unlock();
      }
      break;
      
    case SS_LOCK:
      {
      }
      break;

    case SS_WATCH:
      {
      }
      break;
    }
  return OK_SUCCESS;
}
