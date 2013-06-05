#pragma once

/*! \file scananalyser.h++
  \brief Detects objects with data from the point cloud.
*/

#define DEBUG
#define VERBOSE
#define CONSOLE_COLOR

// ----------------- Libraries ---------------------------------------------------------------------
#include <list>
#include "Aria.h"
#include <cmath>
// ----------------- Project Parts -----------------------------------------------------------------
#include "assist.h++"
#include "errcodes.h++"

/*! class ScanAnalyser
  \brief Detects objects with data from the point cloud.
*/

class ScanAnalyser
{
private:


public:
  /*! \brief Initializes fields.
    Initializes all fields.
    Pointers are set to NULL.
  */
  ScanAnalyser();
  /*! \brief Clean up.
    Clears pointers.
    Allocated data is deallocated.
  */
  ~ScanAnalyser();
  /*! \brief Returns the analysis of the point given by the laser.
    Returns the middle point of the object found by the laser scanner.
    \param Pos, The position of the object.
    \param Points, The array of points from the laser scanner.
    \param Objects out - The distance.
    \return List with the objects with ArPose.
  */
int Analyse(ArPos Pos, std::list<ArPoseWithTime*> Points, std::list<ArPose> *Objects);

}
