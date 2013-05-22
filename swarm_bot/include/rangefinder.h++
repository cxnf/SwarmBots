#pragma once

/*! \file rangefinder.h++
  \brief Finds and tracks robots.
*/
#define DEBUG
#define VERBOSE
#define CONSOLE_COLOR

// ----------------- Libraries ---------------------------------------------------------------------
#include <algorithm>
#include <list>
#include <map>
#include <vector>
#include "Aria.h"

// ----------------- Project Parts -----------------------------------------------------------------
#include "assist.h++"
#include "errcodes.h++"

/*! \class RangeFinder
  \brief Range and angle finder to objects.
  Finds the angle and distance between robot and objects.
*/
class RangeFinder
{
private:
  ArLaser *laser;                                 //!< laser used for range measurements
  std::vector<double> ranges;                     //!< list of range measurements to determine median and average deviation
  double lockonAngle;                             //!< current lock on angle
  double median;                                  //!< median of lock range measurements
  bool lockon;                                    //!< lock on enable flag

public:
  /*! \brief Initializes fields.
    Initializes all fields.
    Pointers are set to NULL.
  */
  RangeFinder();
  /*! \brief Clean up.
    Clears pointers.
    Allocated data is deallocated.
  */
  ~RangeFinder();

  /*! \brief Gets lockon median.
    Returns the current measured median.
    \return Median at locked angle.
  */
  double GetMedian();
  /*! \brief Gets lockon angle.
    Returns the current angle.
    \return Angle lock on is set to.
  */
  double GetAngle();
  /*! \brief Locks on angle.
    Resets current lock on and locks on new angle.
  */
  void LockOn(double angle);
  /*! \brief Resets lock on.
    Resets current lock on angle and clears median data.
  */
  void ResetLockOn();
  /*! \brief Returns lock on enabled.
    Returns a value indicating range finder is locked at angle.
    \return Value indicating lock on is enabled.
   */
  bool HasLock();

  /*! \brief Measures median at current lock on angle.
    Measures distance at current lock on angle.
    Computes the median with new and old measurements.
    Median is only computed when enough data is obtained, to obtain more data MeasureMedian must be called again at a later time(laser must make new measurements first).
    \return OK_SUCCESS when enough data is obtained or ERR_FAIL when more data must be obtained.
  */
  int MeasureMedian();
  
  /*! \brief Starts up the range finder.
    Locates a laser range finder on the robot.
    \param robot The robot to search for a laser on.
    \return OK_SUCCESS or an error code.
  */
  int Setup(ArRobot *robot);
  /*! \brief Search for the closest robot.
    Search through the range buffer for the nearest object that might be a robot.
    \param angle out - The angle at which the object is detected.
    \param distance out - The distance at which the object is detected.
    \return OK_SUCCESS, ERR_FAIL or an error code.
  */
  int FindClosestRobot(double *angle, double *distance);
  /*! \brief Returns the range at a given angle.
    Returns the latest measured range at the given angle.
    \param angle The angle.
    \param distance out - The distance.
    \return OK_SUCCESS, ERR_FAIL or an error code.
  */
  int RangeAt(double angle, double *distance);
  
  void PrintScan();
};
