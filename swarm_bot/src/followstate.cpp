#include "followstate.h++"

// ----------------- Constructors ------------------------------------------------------------------
FollowState::FollowState() : prevx(0),
			     prevy(0),
			     init(false)
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
      int speed = 80;
      int turn = 0;
      double dify = y - this->prevy;
      if (dify < 0)
	{
	  if (dify < 500)
	    {
	      speed = 0;
	    }
	  else
	    {
	      speed -= 20;
	    }
	}
      else if (dify > 0)
	{
	  if (dify > 500)
	    {
	      speed = 200;
	    }
	  else
	    {
	      speed += 20;
	    }
	}
      double difx = x - this->prevx;
      if (difx < 0)
	{
	  if (difx < 500)
	    {
	      turn = -25;
	    }
	  else
	    {
	      turn = -5;
	    }
	  // bot->robot->setVel2(speed - 5, speed + 5);
	}
      else if (difx > 0)
	{
	  if (difx > 500)
	    {
	      turn = 25;
	    }
	  else
	    {
	      turn = 5;
	    }
	  // bot->robot->setVel2(speed + 5, speed - 5);
	}
      bot->robot->setVel2(speed + turn, speed - turn);
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

int FollowState::Restoring(Devices *bot)
{
  return ERR_FAIL;
}
