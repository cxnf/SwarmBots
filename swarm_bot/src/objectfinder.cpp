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
#ifdef SIMULATOR
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
#endif

  return OK_SUCCESS;
}

int ObjectFinder::GetClosestObject(double *angle, double *distance)
{
#ifdef SIMULATOR
  this->laser->lockDevice();
  std::list<ArPoseWithTime*> *buffer = this->laser->getCurrentBuffer();
  ArPose p = this->laser->getRobot()->getEncoderPose();
  std::list<ArPose> objects;
  scanner.Analyse(p, buffer, &objects);
  this->laser->unlockDevice();
#else
  std::list<Scan> objects;
  scanner.AnalyseBuffer(&this->scanresults, &objects);
#endif

  if (!objects.size())
    {
      return ERR_FAIL;
    }

#ifdef SIMULATOR
  *distance = p.findDistanceTo(objects.front());
  *angle = p.findAngleTo(objects.front());
#else
  *distance = objects.front().distance;
  *angle = objects.front().angle;
#endif

  return OK_SUCCESS;
}

int ObjectFinder::GetObjectAt(double angle, double *distance)
{
#ifdef SIMULATOR
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
#else
  std::list<Scan> objects;
  scanner.AnalyseBuffer(&this->scanresults, &objects);
  double best = 180;
  for (std::list<Scan>::iterator it = objects.begin(); it != objects.end(); ++it)
    {
      double dif = fabs(angle - (*it).angle);
      if (dif < best)
	{
	  *distance = (*it).distance;
	  best = dif;
	}
    }
#endif

  return OK_SUCCESS;
}

void ObjectFinder::CallbackScan(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  double angle = msg->angle_min;
  bool isseperated = false;
  this->scanresults.clear();
  for (std::vector<float>::const_iterator it = msg->ranges.begin(); it != msg->ranges.end(); ++it)
    {
      if ((*it) >= msg->range_min && (*it) <= msg->range_max)
	{
	  isseperated = false;
	  this->scanresults.push_back(Scan(*it, angle));
	}
      else if (!isseperated)
	{
	  isseperated = true;
	  this->scanresults.push_back(Scan());
	}
      
      angle += msg->angle_increment;
    }
}
