#pragma once

/*! \file swarmbot.h++
  \brief Contains main class.
*/
#define DEBUG
#define VERBOSE
#define CONSOLE_COLOR
#define SIMULATOR

// ----------------- Libraries ---------------------------------------------------------------------
#include <cmath>
#include <string>
#include <boost/functional/hash.hpp>
#include "Aria.h"
#include "ros/ros.h"

// ----------------- Project Parts -----------------------------------------------------------------
#include "assist.h++"
#include "errcodes.h++"
#include "objectfinder.h++"
#include "robotmap.h++"
#include "istatecontroller.h++"
#include "searchstate.h++"
#include "signalstate.h++"
#include "followstate.h++"
#include "leaderstate.h++"

// ----------------- Messages and Services ---------------------------------------------------------
#include "swarm_bot/Heartbeat.h"
#include "swarm_bot/InitProc.h"

// ----------------- Defines -----------------------------------------------------------------------
#define LASER_EPSILON 15

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
  int myid;                                       //!< id of the robot
  int stateDelay;                                 //!< delay in msec before state swap
  int actDelay;                                   //!< delay set after activated

  Devices dev;                                    //!< devices for state controller
  IStateController *state;                        //!< current state controller
  IStateController *oldstate;                     //!< backup state controller
  FState nextstate;                               //!< type of state controller to load
  FState actstate;                                //!< type of state controller to load after activating
  bool activating;                                //!< flag indicating change state is waiting on active message

  RobotMap robotMap;                              //!< map (and graph if build) of swarm robots
  ObjectFinder finder;                            //!< object finder
  
  ros::NodeHandle node;                           //!< handle to program node
  ros::Publisher heartbeatOut;                    //!< publisher for heartbeat topic
  ros::Subscriber heartbeatIn;                    //!< receiver for hearbeat topic
  ros::Publisher initprocOut;                     //!< init procedure writer
  ros::Subscriber initprocIn;                     //!< init procedure receiver
  ros::Subscriber laserscan;                      //!< scan data from the laser scanner
  
  ArArgumentParser *arguments;                    //!< argument parser for connectors
  ArRobot *robot;                                 //!< aria client
  ArRobotConnector *connector;                    //!< robot connector
  ArLaserConnector *laserConnector;               //!< connector for laser range finder
  ArTime delayTimer;                              //!< timer to delay steps
  
  
  /*! \brief Loads state controller.
    Loads the state controller associated with the nextstate field.
    \return OK_SUCCESS or error code.
  */
  int LoadStateController();

  /*! \brief Requests a state change.
    Requests a state controller change.
    \param fstate State requested.
    \param backup Save current state.
    \param delay Delay in ms before change.
  */
  void ChangeState(FState fstate, bool backup = false, int delay = 1);

  /*! \brief Broadcasts a state.
    Broadcasts given state to all robots.
    \param state State to broadcast.
    \param target Target ID to send with broadcast.
  */
  void Broadcast(BroadcastState state, int target);

  /*! \brief Requests active state and state change.
    Broadcasts the active state and once received by this robot, requests a state change.
    \param fstate State request.
    \param backup Save current state.
    \param delay Delay in ms before change AFTER active message received.
  */
  void Activate(FState fstate, bool backup = false, int delay = 1);
  
  
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
