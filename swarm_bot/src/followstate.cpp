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
int FollowState::UpdateState(Devices *bot, FState *state, BroadcastState *broadcast)
{
  if (!this->init)
    {
      if (!bot->finder->GetClosestObject(&this->angle, &this->distance))
	{
	  this->init = true;
	}
    }
  
  double a, d;
  bot->robot->lock();
  if (!bot->finder->GetClosestObject(&a, &d))
    {
      double dif = this->distance - d;
      int speed = 100;
      if (dif > 25)
	{
	  speed += 10;
	}
      else if (dif < -25)
	{
	  speed -= 10;
	}
      dif = this->angle - a;
      if (dif > 1)
	{
	  bot->robot->setVel2(speed - 10, speed + 10);
	}
      else if (dif < -1)
	{
	  bot->robot->setVel2(speed + 10, speed - 10);
	}
      else
	{
	  bot->robot->setVel2(speed, speed);
	}
    }
  bot->robot->unlock();
  
  return OK_SUCCESS;
}
