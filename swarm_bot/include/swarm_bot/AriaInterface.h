#pragma once

#include "Aria.h"
#include "swarm_bot/errcodes.h"

class AriaInterface
{

 private:
  ArRobot           robot;                          // instance of the aria client API
  ArRobotConnector *robotconnector;                 // pointer to the aria connection
  ArLaserConnector *laserconnector;
  ArSonarDevice     sonar;                          // instance of the sonar device
  ArActionAvoidFront        *avoidFrontNear;
  ArActionAvoidFront        *avoidFrontFar;
  ArActionConstantVelocity  *constantVelocity;

 public:

  //! Function to start Aria.
  /*!
    Initializes Aria, activates the argument parser for Aria,
    connects to the robot and starts the instance to do processing
    in his own tread.
    \param argc Pointer to the argument count.
    \param argv Multidimensional array of characters.
    \return Returns OK_SUCCES if succesfull or an error code.
  */
  int StartUp(int *argc, char **argv);
  
  //! Function to shut Aria down.
  /*!
    Ends the robot thread, but waits for it to finish first. After ending
    the tread Aria gets the exit command and we free any memory allocated.
  */
  void Shutdown();

  //! Function to move the robot
  /*!
    
    
   */
  int AvoidObjects();
  
  
}
