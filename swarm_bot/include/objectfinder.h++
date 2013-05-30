#pragma once

/*! \file objectfinder.h++
  \brief Object finder.
*/
#define VERBOSE
#define CONSOLE_COLOR

#include "Aria.h"
#include "assist.h++"
#include "errcodes.h++"

/*! \class ObjectFinder
  \brief Object finder.
  Performs analysis on laser scans and possibly other input data to find and indentify objects.
*/
class ObjectFinder
{
private:

public:
  /*! \brief Default contructor.
    Initializes data members.
  */
  ObjectFinder();
  /*! \brief Destructor.
    Frees allocated memory.
  */
  ~ObjectFinder();

  /*! \brief Initializes object.
    Retrieves the laser scanner object.
    \return OK_SUCCESS or error code.
  */
  int Setup(ArRobot *robot);

  /*! \brief Finds closest object.
    Locates the closest object within view angle and returns the angle and distance to this object.
    \param angle out - Angle between robot and object.
    \param distance out - Distance between robot and object.
    \return OK_SUCCES, ERR_FAIL or error code.
  */
  int GetClosestObject(double *angle, double *distance);
};
