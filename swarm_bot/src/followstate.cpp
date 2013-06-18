#include "followstate.h++"

// ----------------- Constructors ------------------------------------------------------------------
FollowState::FollowState() : prevx(0),
			     prevy(0),
			     init(false),
			     count(0)
{
}

// ----------------- Destructors -------------------------------------------------------------------
FollowState::~FollowState()
{
}

// ----------------- Methods -----------------------------------------------------------------------
int FollowState::UpdateState(Devices *bot, FState *state, BroadcastState *broadcast)
{
  double a, d;
  if (!this->init)
    {
      if (!bot->finder->GetClosestObject(&a, &d))
	{
	  this->Convert(a, d, &this->prevx, &this->prevy);
	  this->init = true;
	}
    }
  
  bot->robot->lock();
  if (!bot->finder->GetClosestObject(&a, &d))
    {
      double x, y, dif;
      this->Convert(a, d, &x, &y);
      int speed = 80;
      dif = y - this->prevy;
      if (dif < 0)
	{
	  speed -= 50;
	}
      else if (dif > 0)
	{
	  speed += 50;
	}
      dif = x - this->prevx;
      bot->robot->setVel2(speed, speed);
    }
  else
    {
      bot->robot->setVel2(0, 0);
      PRINT(RED "Panic!");
    }
  bot->robot->unlock();
  
  return OK_SUCCESS;
}

void FollowState::Convert(double angle, double distance, double *x, double *y)
{
  double t = angle * (M_PI / 180.0);
  *x = distance * sin(t);
  *y = distance * cos(t);
}
