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
  if (!bot->moveallowed)
    return OK_SUCCESS;

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
      double x, y;
      this->Convert(a, d, &x, &y);
      int speed = 60;
      double dify = y - this->prevy;
      speed += dify;
/*
      if (dify < 0)
	{
	  speed -= 50;
	}
      else if (dify > 0)
	{
	  speed += 50;
	}
*/
      double difx = (x - this->prevx) / 10;
      bot->robot->setVel2(speed + difx, speed - difx);
/*
      if (difx < 0)
	{
	  bot->robot->setVel2(speed - 10, speed + 10);
	}
      else if (difx > 0)
	{
	  bot->robot->setVel2(speed + 10, speed - 10);
	}
*/
      // bot->robot->setVel2(speed, speed);
    }
  else
    {
      bot->robot->setVel2(0, 0);
      // PRINT(RED "Panic!");
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
