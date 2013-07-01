#pragma once

/*! \file scan.h++
  \brief Scan result.
*/
#define VERBOSE
#define CONSOLE_COLOR

/*! \struct Scan
  \brief Scan result.
*/
struct Scan
{
  double distance;                                //!< distance measured
  double angle;                                   //!< angle of measurement
  bool isseperator;                               //!< flag indicating this is not a scan result but a seperator between objects

  /*! \brief Initializes a seperator.
    Initializes all fields as a seperator.
  */
  Scan() : distance(0), angle(0), isseperator(true) { };

  /*! \brief Initializes a scan result.
    Initializes all fields with result of a measurement.
    \param d Distance measured.
    \param a Angle of measurement.
  */
  Scan(double d, double a) : distance(d), angle(a), isseperator(false) { };
};
