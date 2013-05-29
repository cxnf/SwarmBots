#include "rangefinder.h++"

// ----------------- Constructors ------------------------------------------------------------------
RangeFinder::RangeFinder() : laser(NULL),
			     ranges(),
			     lockonAngle(0),
			     median(0),
			     lockon(false)
{
}

// ----------------- Destructors -------------------------------------------------------------------
RangeFinder::~RangeFinder()
{
  laser = NULL;
}

// ----------------- Methods -----------------------------------------------------------------------
int RangeFinder::Setup(ArRobot *robot)
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


double RangeFinder::GetMedian()
{
  return this->median;
}

double RangeFinder::GetAngle()
{
  return this->lockonAngle;
}

void RangeFinder::LockOn(double angle)
{
  this->ResetLockOn();
  this->lockonAngle = angle;
  this->lockon = true;
}

void RangeFinder::ResetLockOn()
{
  this->lockon = false;
  this->lockonAngle = 0;
  this->median = 0;
  this->ranges.clear();
}

bool RangeFinder::HasLock()
{
  return this->lockon;
}

int RangeFinder::MeasureMedian()
{
  double range;
  if (!this->RangeAt(this->lockonAngle, &range))
    {
      this->ranges.push_back(range);
      if (this->ranges.size() > 25)
	{
	  size_t n = this->ranges.size() / 2;
	  nth_element(this->ranges.begin(), this->ranges.begin() + n, this->ranges.end());
	  this->median = this->ranges[n];
	  return OK_SUCCESS;
	}
    }
  return ERR_FAIL;
}


int RangeFinder::FindClosestRobot(double *angle, double *distance)
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

int RangeFinder::RangeAt(double angle, double *distance)
{
  this->laser->lockDevice();
  std::list<ArPoseWithTime*> *buffer = this->laser->getCurrentBuffer();
  ArPose p = this->laser->getRobot()->getEncoderPose();
  *distance = this->laser->getMaxRange();
  double diff = 1.0;
  for (std::list<ArPoseWithTime*>::iterator it = buffer->begin(); it != buffer->end(); ++it)
    {
      double a = p.findAngleTo(**it);
      double d = angle - a;
      if (d < 0) d *= -1;
      if (d < diff)
	{
	  diff = d;
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


void RangeFinder::PrintScan()
{
  this->laser->lockDevice();
  std::list<ArPoseWithTime*> *buffer = this->laser->getCurrentBuffer();
  ArPose p = this->laser->getRobot()->getEncoderPose();
  for (std::list<ArPoseWithTime*>::iterator it = buffer->begin(); it != buffer->end(); ++it)
    {
      double a = p.findAngleTo(**it);
      double d = p.findDistanceTo(**it);
      PRINT(BLACK "Scan [%f][%f]", a, d);
    }
  PRINT(RED "----------------------------------------");
  this->laser->unlockDevice();
}
