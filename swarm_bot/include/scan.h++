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
  double distance;                                //!< distance measured (m)
  double angle;                                   //!< angle of measurement (rad)
  bool isseperator;                               //!< flag indicating this is not a scan result but a seperator between objects

  /*! \brief Initializes a seperator.
    Initializes all fields as a seperator.
  */
  Scan() : distance(0),
	   angle(0),
	   isseperator(true)
  { };

  /*! \brief Initializes a scan result.
    Initializes all fields with result of a measurement.
    \param d Distance measured.
    \param a Angle of measurement.
  */
  Scan(double d, double a) : distance(d),
			     angle(a),
			     isseperator(false)
  { };
};

/*! \struct ScanResult
  \brief Identified scan result.
*/
struct ScanResult : public Scan
{
  int id;                                         //!< id of robot represented by this scan, 0 if not identified/robot
  double change;                                  //!< amount of distance change

  /*! \brief Initializes a seperator.
    Initializes all fields as a seperator.
  */
  ScanResult() : Scan(),
		 id(0),
		 change(0)
  { };

  /*! \brief Initializes a scan result.
    Initializes all fields for an unidentified object.
    \param d Distance measured.
    \param a Angle of measurement.
  */
  ScanResult(double d, double a) : Scan(d, a),
				   id(0),
				   change(0)
  { };
};
