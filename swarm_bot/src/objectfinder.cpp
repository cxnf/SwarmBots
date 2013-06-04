#include "objectfinder.h++"

// ----------------- Constructors ------------------------------------------------------------------
ObjectFinder::ObjectFinder()
{
}

// ----------------- Destructors -------------------------------------------------------------------
ObjectFinder::~ObjectFinder()
{
}

// ----------------- Methods -----------------------------------------------------------------------
int ObjectFinder::Setup(ArRobot *robot)
{
  robot->lock();
  std::map<int, ArLaser*> *lasermap = robot->getLaserMap();
  for (std::map<int, ArLaser*>::iterator it = lasermap->begin(); it != lasermap->end(); ++it)
    {
      this->laser = it->second;
      break;
    }
  robot->unlock();
  if (!this->laser)
    {
      return ERR_ARIA_LASER;
    }

  return OK_SUCCESS;
}

int ObjectFinder::GetClosestObject(double *angle, double *distance)
{
  this->laser->lockDevice();
  std::list<ArPoseWithTime*> *buffer = this->laser->getCurrentBuffer();
  ArPose p = this->laser->getRobot()->getEncoderPose();
  *distance = this->laser->getMaxRange();
  for (std::list<ArPoseWithTime*>::iterator it = buffer->begin(); it != buffer->end(); ++it)
    {
      double a = p.findAngleTo(**it);
      double d = p.findDistanceTo(**it);
      if (d < *distance)
	{
	  *distance = d;
	  *angle = a;
	}
    }
  this->laser->unlockDevice();

  if (*distance >= this->laser->getMaxRange())
    {
      return ERR_FAIL;
    }
  return OK_SUCCESS;
}

int ObjectFinder::GetObjectAt(double angle, double *distance)
{
  this->laser->lockDevice();
  std::list<ArPoseWithTime*> *buffer = this->laser->getCurrentBuffer();
  ArPose p = this->laser->getRobot()->getEncoderPose();
  *distance = this->laser->getMaxRange();
  double dif = 1.0;
  for (std::list<ArPoseWithTime*>::iterator it = buffer->begin(); it != buffer->end(); ++it)
    {
      double a = p.findAngleTo(**it);
      double b = fabs(angle - a);
      if (b < dif)
	{
	  dif = b;
	  *distance = p.findDistanceTo(**it);
	}
    }
  this->laser->unlockDevice();

  if (*distance >= this->laser->getMaxRange())
    {
      return ERR_FAIL;
    }
  return OK_SUCCESS;
}
