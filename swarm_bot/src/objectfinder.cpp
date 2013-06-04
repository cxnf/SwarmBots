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
  return OK_SUCCESS;
}

int ObjectFinder::GetClosestObject(double *angle, double *distance)
{
  return ERR_FAIL;
}

int ObjectFinder::GetObjectAt(double angle, double *distance)
{
  return ERR_FAIL;
}
