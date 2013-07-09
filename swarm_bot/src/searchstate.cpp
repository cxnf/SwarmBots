#include "searchstate.h++"

// ----------------- Constructors ------------------------------------------------------------------
SearchState::SearchState() : substate(SER_SEARCH),
			     lockangle(0),
			     lockdist(),
			     median(0)
{
}

// ----------------- Destructors -------------------------------------------------------------------
SearchState::~SearchState()
{
}

// ----------------- Methods -----------------------------------------------------------------------
int SearchState::UpdateState(Devices *bot, FState *state, BroadcastState *broadcast)
{
/*
  switch (this->substate)
    {
    case SER_SEARCH:
      {
	double d;
	bot->robot->lock();
	if (bot->robot->isHeadingDone())
	  {
	    ArPose p = bot->robot->getEncoderPose();
	    if (!bot->finder->GetClosestObject(&this->lockangle, &d))
	      {
		/* double a = this->lockangle - p.getTh();
		   PRINT(GREEN "[%f] - [%f] = [%f]", p.getTh(), this->lockangle, a);
		   if (a > 80)
		   {
		   bot->robot->setHeading(p.getTh() + 10);
		   }
		   else if (a < -80)
		   {
		   bot->robot->setHeading(p.getTh() - 10);
		   }
		   else
		   {*/
/*
		this->substate = SER_LOCK;
		this->lockdist.clear();
		// }
	      }
	    else
	      {
		bot->robot->setHeading(p.getTh() + 90);
	      }
	  }
	bot->robot->unlock();
      }
      break;
      
    case SER_LOCK:
      {
	double d;
	if (this->lockdist.size() < 5)
	  {
	    if (!bot->finder->GetObjectAt(this->lockangle, &d))
	      {
		this->lockdist.push_back(d);
	      }
	    else
	      {
		PRINT(RED "Lock error.");
		return ERR_SWARM_FINDER;
	      }
	  }
	else
	  {
	    size_t n = this->lockdist.size() / 2;
	    nth_element(this->lockdist.begin(), this->lockdist.begin() + n, this->lockdist.end());
	    this->median = this->lockdist[n];
	    this->lockdist.clear();
	    this->substate = SER_WATCH;
	    *broadcast = BS_NEXT_SEARCH;
	  }
      }
      break;

    case SER_WATCH:
      {
	double d;
	if (!bot->finder->GetObjectAt(this->lockangle, &d))
	  {
	    double dif = fabs(this->median - d);
	    if (dif > 15.0)
	      {
		*state = FS_WAIT;
		*broadcast = BS_FOUND;
	      }
	  }
	else
	  {
	    PRINT(RED "Robot run away.");
	    return ERR_SWARM_FINDER;
	  }
      }
      break;
    }
*/

  bot->finder->ScanSurroundings();
  if (this->substate == SER_SEARCH)
    {
      this->substate = SER_LOCK;
      *broadcast = BS_NEXT_SEARCH;
      return OK_SUCCESS;
    }

  double angle, change;
  if (!bot->finder->GetLargestChange(&angle, &change))
    {
      // PRINT(BLACK "[%f]", change);
      if (change > 0.5)
	{
	  bot->finder->IdentifyObject(angle, bot->activebot);
	  if (this->substate == SER_LOCK)
	    {
	      this->substate = SER_WATCH;
	      *broadcast = BS_FOUND;
	    }
	}
    }


  return OK_SUCCESS;
}

int SearchState::Restoring(Devices *bot)
{
  double d;
  if (!bot->finder->GetObjectAt(this->lockangle, &d))
    {
      this->median = d;
      return OK_SUCCESS;
    }
  else
    {
      return ERR_SWARM_FINDER;
    }
}
