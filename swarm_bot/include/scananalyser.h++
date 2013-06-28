#pragma once

/*! \file scananalyser.h++
  \brief Detects objects with data from the point cloud.
*/

#define DEBUG
#define VERBOSE
#define CONSOLE_COLOR

#define OBJ_MARGIN 250.0

// ----------------- Libraries ---------------------------------------------------------------------
#include <cmath>
#include <list>
#include <vector>
#include "Aria.h"

// ----------------- Project Parts -----------------------------------------------------------------
#include "assist.h++"
#include "errcodes.h++"
#include "scan.h++"

/*! class ScanAnalyser
  \brief Detects objects with data from the point cloud.
*/

class ScanAnalyser
{
private:

  /*! \brief 
    \param obj out - 
    \return 
  */
  ArPose Avarage(std::list<ArPose*> *obj);

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
    \param pos The position of the object.
    \param points The array of points from the laser scanner.
    \param objects out - The distance.
    \return List with the objects with ArPose.
  */
  int Analyse(ArPose pos, std::list<ArPoseWithTime*> *points, std::list<ArPose> *objects);


  void AnalyseBuffer(std::vector<Scan> *buffer, std::list<Scan> *objects);
};
