#include "swarm_bot/AriaInterface.h"

int AriaInterface::StartUp(int *argc, char **argv)
{
  Aria::init();                                   // initalisation of aria
  ArArgumentParser parser(argc, argv);            // 
  parser.loadDefaultArguments();                  // 
  
  robotconnector = new ArRobotConnector( &parser, &robot); // 
  laserconnector = new ArLaserConnector( &parser, &robot, robotconnector); // 

  parser.addDefaultArgument("-connectLaser");

  if (!robotconnector->connectRobot())            // 
    {
      return ERR_ARIA_CONNECTION;                 //
    }

  if (!Aria::parseArgs() || !parser.checkHelpAndWarnUnparsed()) // 
    {
      return ERR_ARIA_PARSE;                      // 
    }

  robot.addRangeDevice(&sonar);
    
  if(!laserconnector->connectLasers())
    {
      return ERR_ARIA_LASER;
    }
  else
    {
      ArLog::log(ArLog::Normal, "Laser works good.");
    }
    
  robot.runAsync(true);                           // 
  robot.enableMotors();
  return OK_SUCCESS;                              // 
}

void AriaInterface::Shutdown()
{
  robot.stopRunning();                            // 
  robot.waitForRunExit();                         // 
  Aria::exit();                                   // 
  
  delete robotconnector;
  delete laserconnector;
  delete avoidFrontNear;
  delete avoidFrontFar;
  delete constantVelocity;
}

void AriaInterface::AvoidObjects()
{
  avoidFrontNear = new ArActionAvoidFront("Avoid front Near", 225, 0);
  avoidFrontFar = new ArActionAvoidFront("Avoid front far", 250, 0);
  constantVelocity = new ArActionConstantVelocity("Constant Velocity", 400);

  robot.addAction(avoidFrontNear, 50);
  robot.addAction(avoidFrontFar, 49);
  robot.addAction(constantVelocity, 25);  
}
