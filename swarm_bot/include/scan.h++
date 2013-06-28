#pragma once

/*! \file scan.h++
  \brief Scan result.
*/
#define VERBOSE
#define CONSOLE_COLOR

struct Scan
{
  double distance;
  double angle;
  bool isseperator;

  Scan() : distance(0), angle(0), isseperator(true) { };
  Scan(double d, double a) : distance(d), angle(a), isseperator(false) { };
};
