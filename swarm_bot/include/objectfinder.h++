#pragma once

/*! \file objectfinder.h++
  \brief Object finder.
*/
#define VERBOSE
#define CONSOLE_COLOR
#define SIMULATOR

#include <list>
#include <map>
#include <vector>
#include "Aria.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#include "assist.h++"
#include "errcodes.h++"
#include "scan.h++"
#include "scananalyser.h++"

/*! \class ObjectFinder
  \brief Object finder.
  Performs analysis on laser scans and possibly other input data to find and indentify objects.
*/
class ObjectFinder
{
private:
  ArLaser *laser;                                 //!< laser used for range measurements
  ScanAnalyser scanner;                           //!< laser scan analyser
  std::vector<Scan> scanresults;                  //!< scan buffer

  // -----------------------------------------------------------------------------------------------
  std::list<ScanResult*> surroundings;             //!< surrounding objects
  // -----------------------------------------------------------------------------------------------

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





  int ScanSurroundings();
  int GetLargestChange(double *angle, double *change);
  int IdentifyObject(double angle, int id);




  /*! \brief Finds closest object.
    Locates the closest object within view angle and returns the angle and distance to this object.
    \param angle out - Angle between robot and object.
    \param distance out - Distance between robot and object.
    \return OK_SUCCES, ERR_FAIL or error code.
  */
  int GetClosestObject(double *angle, double *distance);

  /*! \brief Finds distance at angle.
    Returns the distance at the given angle.
    \param angle Angle between robot and object.
    \param distance out - Distance between robot and object.
    \return OK_SUCCESS, ERR_FAIL or error code.
  */
  int GetObjectAt(double angle, double *distance);

  /*! \brief Filters scan results.
    Filters scan results and buffers range/angle pairs.
    \param msg Pointer to scan results.
  */
  void CallbackScan(const sensor_msgs::LaserScan::ConstPtr &msg);
};
