#include "signalstate.h++"

// ----------------- Constructors ------------------------------------------------------------------
SignalState::SignalState() : substate(SIG_SEND),
			     restore(0)
{
}

// ----------------- Destructors -------------------------------------------------------------------
SignalState::~SignalState()
{
}

// ----------------- Methods -----------------------------------------------------------------------
int SignalState::UpdateState(Devices *bot, FState *state, BroadcastState *broadcast)
{
  switch (this->substate)
    {
    case SIG_SEND:
      {
	bot->robot->lock();
	this->restore = bot->robot->getEncoderPose().getTh();
	bot->robot->setHeading(this->restore + 180);
	this->substate = SIG_RESTORE;
	bot->robot->unlock();
      }
      break;

    case SIG_RESTORE:
      {
	bot->robot->lock();
	if (bot->robot->isHeadingDone())
	  {
	    bot->robot->setHeading(this->restore);
	    this->substate = SIG_FINALIZE;
	  }
	bot->robot->unlock();
      }
      break;

    case SIG_FINALIZE:
      {
	bot->robot->lock();
	if (bot->robot->isHeadingDone())
	  {
	    double dif = fabs(this->restore - bot->robot->getEncoderPose().getTh());
	    if (dif < 0.1)
	      {
		*state = FS_WAIT;
		*broadcast = BS_NEXT_SIGNAL;
	      }
	    else
	      {
		bot->robot->setHeading(this->restore);
	      }
	  }
	bot->robot->unlock();
      }
      break;
    }
  return OK_SUCCESS;
}
