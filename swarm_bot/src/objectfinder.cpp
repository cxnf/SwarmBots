#include "objectfinder.h++"

// ----------------- Constructors ------------------------------------------------------------------
ObjectFinder::ObjectFinder() : laser(NULL),
			       scanner(),
			       scanresults(),
			       surroundings()
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






int ObjectFinder::ScanSurroundings()
{
  std::list<Scan> objects;
#ifdef SIMULATOR
  this->laser->lockDevice();
  {
    std::list<ArPoseWithTime*> *buffer = this->laser->getCurrentBuffer();
    std::list<ArPose> objs;
    ArPose p = this->laser->getRobot()->getEncoderPose();
    this->scanner.Analyse(p, buffer, &objs);
    for (std::list<ArPose>::iterator it = objs.begin(); it != objs.end(); ++it)
      {
	double angle = p.findAngleTo(*it) * (M_PI / 180.0);
	objects.push_back(Scan(p.findDistanceTo(*it) / 1000.0, angle));
      }
  }
  this->laser->unlockDevice();
#else
  this->scanner.AnalyseBuffer(&this->scanresults, &objects);
#endif

  std::list<Scan>::iterator scan = objects.begin();
  std::list<ScanResult*>::iterator sur = this->surroundings.begin();
  while (scan != objects.end() && sur != this->surroundings.end())
    {
      double dif = fabs((*scan).angle - (*sur)->angle);
      if (dif < 0.1)
	{
	  dif = fabs((*scan).distance - (*sur)->distance);
	  (*sur)->change = dif;
	  (*sur)->angle = (*scan).angle;
	  (*sur)->distance = (*scan).distance;
	  ++sur;
	  ++scan;
	}
      else if ((*scan).angle > (*sur)->angle)
	{
	  this->surroundings.insert(sur, new ScanResult((*scan).distance, (*scan).angle));
	  ++scan;
	}
      else
	{
	  delete (*sur);
	  sur = this->surroundings.erase(sur);
	}
    }
  if (sur == this->surroundings.end() && scan != objects.end())
    {
      for (;scan != objects.end(); ++scan)
	{
	  this->surroundings.insert(sur, new ScanResult((*scan).distance, (*scan).angle));
	}
    }
  else if (scan == objects.end() && sur != this->surroundings.end())
    {
      while (sur != this->surroundings.end())
	{
	  delete (*sur);
	  sur = this->surroundings.erase(sur);
	}
    }

  return OK_SUCCESS;
}

int ObjectFinder::GetLargestChange(double *angle, double *change)
{
  *change = 0;
  for (std::list<ScanResult*>::iterator it = this->surroundings.begin(); it != this->surroundings.end(); ++it)
    {
      if ((*it)->change > *change)
	{
	  *change = (*it)->change;
	  *angle = (*it)->angle;
	}
    }

  if (*change == 0)
    {
      return ERR_FAIL;
    }
  return OK_SUCCESS;
}

int ObjectFinder::IdentifyObject(double angle, int id)
{
  for (std::list<ScanResult*>::iterator it = this->surroundings.begin(); it != this->surroundings.end(); ++it)
    {
      if ((*it)->angle == angle)
	{
	  (*it)->id = id;
	  return OK_SUCCESS;
	}
    }
  return ERR_FAIL;
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
