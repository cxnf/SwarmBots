#include "objectfinder.h++"

// ----------------- Constructors ------------------------------------------------------------------
ObjectFinder::ObjectFinder() : laser(NULL),
			       scanner()
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
  std::list<ArPose> objects;
  scanner.Analyse(p, buffer, &objects);
  this->laser->unlockDevice();
  
  if (!objects.size())
    {
      return ERR_FAIL;
    }

  /*
  *distance = this->laser->getMaxRange();
  for (std::list<ArPose>::iterator it = objects.begin(); it != objects.end(); ++it)
    {
      double d = p.findDistanceTo(*it);
      if (d < *distance)
	{
	  *distance = d;
	  *angle = p.findAngleTo(objects.front());	  
	}
    }
  */
  *distance = p.findDistanceTo(objects.front());
  *angle = p.findAngleTo(objects.front());

  return OK_SUCCESS;
}

int ObjectFinder::GetObjectAt(double angle, double *distance)
{
  this->laser->lockDevice();
  std::list<ArPoseWithTime*> *buffer = this->laser->getCurrentBuffer();
  ArPose p = this->laser->getRobot()->getEncoderPose();
  std::list<ArPose> objects;
  scanner.Analyse(p, buffer, &objects);
  this->laser->unlockDevice();

  if (!objects.size())
    {
      return ERR_FAIL;
    }
  double best = 180;
  for (std::list<ArPose>::iterator it = objects.begin(); it != objects.end(); ++it)
    {
      double a = p.findAngleTo(*it);
      double dif = fabs(angle - a);
      if (dif < best)
	{
	  *distance = p.findDistanceTo(*it);
	  best = dif;
	}
    }

  return OK_SUCCESS;
}
