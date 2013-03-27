#include "swarm_bot/robot.h"

Robot::Robot() : staticID(0), dynamicID(0), heartbeat(0), missed(0), name() // zero out members
{
}
Robot::Robot(std::string n, int32_t sid) : staticID(sid), dynamicID(0), heartbeat(0), missed(0), name(n) // initialize members
{
}

int32_t Robot::GetStaticID()
{
  return this->staticID;                          // return static id
}

int32_t Robot::GetDynamicID()
{
  return this->dynamicID;                         // return dynamic id
}

std::string Robot::GetName()
{
  return this->name;                              // return name
}

bool Robot::IsAlive()
{
  return this->missed <= 2;                       // return value indicating heartbeat value is acceptable
}

int32_t Robot::GetMissedHeartbeats()
{
  return this->missed;                            // return the amount of heartbeats missed in a row
}

bool Robot::Increment()
{
  this->heartbeat++;                              // increment heartbeat
  if (this->heartbeat > 2)                        // if heartbeat exceeds interval
    {
      this->heartbeat = 0;                        // reset heartbeat
      this->missed++;                             // increment missed heartbeats
      return true;                                // return robot missed heartbeat
    }
  return false;                                   // return robot is still broadcasting heartbeat
}

void Robot::Heartbeat()
{
  this->heartbeat = 0;                            // reset heartbeat
  this->missed = 0;                               // reset missed counter
}

void Robot::SetDynamicID(int32_t did)
{
  this->dynamicID = did;                          // assign new dynamic id
}
