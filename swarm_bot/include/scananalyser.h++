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

  /*! \brief Returns average.
    Takes the average of all positions in the input list.
    \param obj List to take average from.
    \return Average of input values.
  */
  ArPose Average(std::list<ArPose*> *obj);
  /*! \brief Returns average.
    Returns the average angle and distance of all given scans.
    \param scans List to take average from.
    \return Average of input values.
  */
  Scan AverageScan(std::vector<Scan> *scans);

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
    \param objects out - Object positions.
    \return List with the objects with ArPose.
  */
  int Analyse(ArPose pos, std::list<ArPoseWithTime*> *points, std::list<ArPose> *objects);

  /*! \brief Analysis the buffer.
    Finds all objects in the scan buffer and returns a list with objects and their estimated angle and distance.
    \param buffer Scan buffer.
    \param objects out - List of seperate objects.
  */
  void AnalyseBuffer(std::vector<Scan> *buffer, std::list<Scan> *objects);
};
