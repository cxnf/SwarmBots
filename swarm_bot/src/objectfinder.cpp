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
  return OK_SUCCES;
}

int ObjectFinder::GetClosestObject(double *angle, double *distance)
{
  return ERR_FAIL;
}
