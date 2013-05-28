#pragma once

/*! \file swarmbot.h++
  \brief Contains main class.
 */
#define DEBUG
#define VERBOSE
#define CONSOLE_COLOR

// ----------------- Libraries ---------------------------------------------------------------------
#include <cmath>
#include <string>
#include "Aria.h"
#include "ros/ros.h"

// ----------------- Project Parts -----------------------------------------------------------------
#include "assist.h++"
#include "errcodes.h++"
#include "rangefinder.h++"
#include "robotmap.h++"

// ----------------- Messages and Services ---------------------------------------------------------
#include "swarm_bot/Heartbeat.h"
#include "swarm_bot/InitProc.h"
#include "swarm_bot/Announce.h"

// ----------------- Defines -----------------------------------------------------------------------
#define LASER_EPSILON 15

/*! \enum FormationState
  \brief States robot can enter to create/maintain a formation.
  Different states a robot can enter.
  A state defines the behaviour of the robot.
 */
typedef enum
  {
    FS_INI_WAIT = 1,                              //!< wait for a message on InitProc topic
    FS_INI_SEARCH,                                //!< search for a robot
    FS_INI_SEARCH_LOCK,                           //!< lock on possible robot
    FS_INI_SIGNAL_START,                          //!< send out a signal
    FS_INI_SIGNAL_RESTORE,                        //!< restore state changes caused by signal
    FS_INI_SIGNAL,                                //!< wait until finished signal
    FS_INI_WATCH,                                 //!< watch for a signal
    FS_INI_FOUND,                                 //!< found a robot
    
    FS_RUN_FORWARD,                               //!< leader moves forward
    FS_RUN_FOLLOW,                                //!< follow leader
  } FormationState;

/*! \class SwarmBot
  \brief Main class.
  The main class of a robot in the swarm.
  Manages the robots seperated parts.
 */
class SwarmBot
{
private:
  bool isRunning;                                 //!< main loop condition
  std::string name;                               //!< name of the robot
  int32_t staticID;                               //!< id of the robot
  FormationState fstate;                          //!< current state of the robot
  FormationState dstate;                          //!< delayed state to swap to
  int activeRobot;                                //!< id of signalling (or searching) robot
  int stateDelay;                                 //!< delay in msec before state swap
  
  RobotMap robotMap;                              //!< map (and graph if build) of swarm robots
  RangeFinder finder;                             //!< range finder of the robot
  
  ros::NodeHandle node;                           //!< handle to program node
  ros::Publisher heartbeatOut;                    //!< publisher for heartbeat topic
  ros::Subscriber heartbeatIn;                    //!< receiver for hearbeat topic
  ros::Publisher initprocOut;                     //!< init procedure writer
  ros::Subscriber initprocIn;                     //!< init procedure receiver
  ros::ServiceClient announce;                    //!< client for announce service
  
  ArArgumentParser *arguments;                    //!< argument parser for connectors
  ArRobot *robot;                                 //!< aria client
  ArRobotConnector *connector;                    //!< robot connector
  ArLaserConnector *laserConnector;               //!< connector for laser range finder
  ArTime delayTimer;                              //!< timer to delay steps
  
  
  /*! \brief Publishes to InitProc.
    Publishes a message to the InitProc topic.
    All posible fields are initialized with current member values.
    \param TargetID Static id of target robot, if any.
  */
  void PublishInitProc(int32_t TargetID);
  /*! \brief Changes robot state.
    Updates the robot formation state.
    \param newState State to change to.
    \param delay Optional delay before state change takes effect.
  */
  void ChangeState(FormationState newState, int delay = 0);
  
public:
  /*! \brief Initializes fields.
    Initializes all fields.
    Pointers are set to NULL.
  */
  SwarmBot();
  /*! \brief Stop Aria and clean up.
    Stops the robot and shuts down Aria.
    All allocated data is deallocated.
  */
  ~SwarmBot();
  
  /*! \brief Starts up the robot.
    Creates all objects required.
    Connecters are connected.
    \return OK_SUCCESS or an error code.
  */
  int Setup();
  /*! \brief Runs the main loop.
    Starts the main loop on calling thread.
    This method blocks until SwarmBot::Stop is called.
  */
  void Run();
  /*! \brief Stops the main loop.
    Stops the main loop.
    The main loop will finish its current iteration.
  */
  void Stop();
  
  /*! \brief InitProc callback.
    Processes messages from the initproc topic.
    \param msg Received message.
  */
  void CallbackInitProc(const swarm_bot::InitProc::ConstPtr &msg);
  /*! \brief Heartbeat callback.
    Maintains swarm list for a robot.
    \param msg Received message.
  */
  void CallbackHeartbeat(const swarm_bot::Heartbeat::ConstPtr &msg);
};
