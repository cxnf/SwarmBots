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

  // ----------------------------------------------------------------------
  robot.lock();
  std::map<int, ArLaser*> *lasers = robot.getLaserMap();
  for (std::map<int, ArLaser*>::iterator it = lasers->begin(); it != lasers->end(); ++it)
    {
      laser = (*it).second;
      break;
    }
  robot.unlock();
  if (!laser)
    return ERR_ARIA_LASER;
  
  return OK_SUCCESS;                              // 
}

void AriaInterface::Shutdown()
{
  robot.stopRunning();                            // 
  robot.waitForRunExit();                         // 
  Aria::shutdown();
  
  if (robotconnector) delete robotconnector;
  if (laserconnector) delete laserconnector;
  if (laser) delete laser;
  if (avoidFrontNear) delete avoidFrontNear;
  if (avoidFrontFar) delete avoidFrontFar;
  if (constantVelocity) delete constantVelocity;
}

void AriaInterface::AvoidObjects()
{
  avoidFrontNear = new ArActionAvoidFront("Avoid front Near", 225, 0);
  avoidFrontFar = new ArActionAvoidFront("Avoid front far", 450, 0);
  constantVelocity = new ArActionConstantVelocity("Constant Velocity", 400);

  robot.addAction(avoidFrontNear, 50);
  robot.addAction(avoidFrontFar, 49);
  robot.addAction(constantVelocity, 25);  
}


// ------------------------------------------------------------------------
float AriaInterface::GetRelativeAngle()
{
  float r;
  robot.lock();
  r = robot.getEncoderPose().getThRad();
  robot.unlock();
  return r;
}

int32_t AriaInterface::SearchRobotInView(float *angle, float *distance)
{
  if (!laser)
    {
      return ERR_ARIA_LASER;
    }
  double r;
  double d = laser->currentReadingPolar(laser->getStartDegrees(), laser->getEndDegrees(), &r);
  if (d >= laser->getMaxRange())
    {
      return ERR_ARIA_LASER;
    }
  *angle = (float)r;
  *distance = (float)d;
  return OK_SUCCESS;
}

float AriaInterface::GetDistanceAt(float angle)
{
  if (!laser)
    {
      return -1;
    }
  double r;
  double d = laser->currentReadingPolar(angle, angle, &r);
  return (float)d;
}

void AriaInterface::Rotate(float speed)
{
  robot.lock();
  robot.setRotVel(speed);
  robot.unlock();
}
