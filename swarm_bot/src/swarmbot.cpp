#include "swarmbot.h++"

// ----------------- Constructors ------------------------------------------------------------------
SwarmBot::SwarmBot() : isRunning(false),
		       name(),
		       staticID(0),
		       fstate(FS_INI_WAIT),
		       activeRobot(0),
		       robotMap(),
		       finder(),
		       node(),
		       heartbeatOut(),
		       heartbeatIn(),
		       initprocOut(),
		       initprocIn(),
		       announce(),
		       arguments(NULL),
		       robot(NULL),
		       connector(NULL),
		       laserConnector(NULL)
{
}

// ----------------- Destructors -------------------------------------------------------------------
SwarmBot::~SwarmBot()
{
  this->robot->disableMotors();
  this->robot->stopRunning();
  this->robot->waitForRunExit();
  Aria::shutdown();
  
  if (arguments) delete arguments;
  if (robot) delete robot;
  if (connector) delete connector;
  if (laserConnector) delete laserConnector;
}

// ----------------- Methods -----------------------------------------------------------------------
int SwarmBot::Setup()
{
  ros::NodeHandle ppnode("~");
  Aria::init();
  // ArLog::init(ArLog::None, ArLog::Verbose, "", false, false, false);

  // --------------- Aria Setup --------------------------------------------------------------------
  std::string port;
  ppnode.param<std::string>("port", port, "/dev/ttyUSB0");
  PRINT(BLUE "Connection using: [%s]", port.c_str());
  ArArgumentBuilder *arg = new ArArgumentBuilder();
  size_t split = port.find(":");
  if (split != std::string::npos)
    {
      arg->add("-rh");
      arg->add(port.substr(0, split).c_str());
      arg->add("-rrtp");
      arg->add(port.substr(split + 1).c_str());
    }
  else
    {
      arg->add("-rp");
      arg->add(port.c_str());
    }
  arg->add("-connectLaser");
  
  this->arguments = new ArArgumentParser(arg);
  this->robot = new ArRobot();
  this->connector = new ArRobotConnector(this->arguments, this->robot, true);
  this->laserConnector = new ArLaserConnector(this->arguments, this->robot, this->connector, true);

  if (!this->connector->connectRobot(this->robot))
    return ERR_ARIA_CONNECTION;
  if (!this->laserConnector->connectLasers())
    return ERR_ARIA_LASER;
  
  this->robot->enableMotors();
  this->robot->runAsync(true);
  
  delete arg;

  // --------------- Ros Setup ---------------------------------------------------------------------
  ppnode.param<std::string>("name", this->name, "swarmbot");
  
  this->heartbeatOut = this->node.advertise<swarm_bot::Heartbeat>("heartbeat", 32);
  this->heartbeatIn = this->node.subscribe("heartbeat", 32, &SwarmBot::CallbackHeartbeat, this);
  this->initprocOut = this->node.advertise<swarm_bot::InitProc>("initproc", 32);
  this->initprocIn = this->node.subscribe("initproc", 32, &SwarmBot::CallbackInitProc, this);
  this->announce = this->node.serviceClient<swarm_bot::Announce>("announce");
  
  swarm_bot::Announce request;
  request.request.Name = this->name.c_str();
  request.request.Shutdown = false;
  if (this->announce.call(request))
    {
      if (request.response.StaticID <= 0)
	return ERR_SWARM_CONTROLLER;
      this->staticID = request.response.StaticID;
    }
  else return ERR_ROS_SERVICE;

  // --------------- SwarmBot setup ----------------------------------------------------------------
  if (this->robotMap.Add(this->staticID))
    return ERR_SWARM_MAP;
  if (this->finder.Setup(this->robot))
    return ERR_SWARM_FINDER;

  this->isRunning = true;
  return OK_SUCCESS;
}

void SwarmBot::Run()
{
  ros::Rate rate(10);
  int second = 0;
  
  PRINT(GREEN "I am [%s] assigned to [%d]", this->name.c_str(), this->staticID);

  while (this->isRunning)
    {
      // ----------- Heartbeat ---------------------------------------------------------------------
      second++;
      if (second > 10)
	{
	  second = 0;
	  swarm_bot::Heartbeat pulse;
	  pulse.StaticID = this->staticID;
	  this->heartbeatOut.publish(pulse);
	}

      // ----------- Formation ---------------------------------------------------------------------
      switch (this->fstate)
	{
	case FS_INI_WAIT: break;

	case FS_INI_FOUND:
	  this->fstate = FS_INI_WAIT;
	  break;

	case FS_INI_SEARCH:
	  {
	    double a, d;
	    this->robot->lock();
	    ArPose p = this->robot->getEncoderPose();
	    if (!this->finder.FindClosestRobot(&a, &d))
	      {
		this->robot->setRotVel(0);
		this->robot->setHeading(p.getTh() + 5);
		this->fstate = FS_INI_SEARCH_LOCK;
	      }
	    else this->robot->setRotVel(10);
	    this->robot->unlock();
	  }
	  break;

	case FS_INI_SEARCH_LOCK:
	  {
	    double a, d;
	    this->robot->lock();
	    if (this->robot->isHeadingDone())
	      {
		if (!this->finder.MeasureMedian())
		  {
		    this->fstate = FS_INI_WATCH;
		    this->PublishInitProc(0);
		  }
	      }
	    else if (!this->finder.FindClosestRobot(&a, &d))
		this->finder.LockOn(a);
	    this->robot->unlock();
	  }
	  break;

	case FS_INI_SIGNAL_START:
	  {
	    this->robot->lock();
	    this->robot->setHeading(this->robot->getEncoderPose().getTh() + 180);
	    this->fstate = FS_INI_SIGNAL_RESTORE;
	    this->robot->unlock();
	  }
	  break;

	case FS_INI_SIGNAL_RESTORE:
	  {
	    this->robot->lock();
	    if (this->robot->isHeadingDone())
	      {
		this->robot->setHeading(this->robot->getEncoderPose().getTh() + 180);
		this->fstate = FS_INI_SIGNAL;
	      }
	    this->robot->unlock();
	  }
	  break;
	  
	case FS_INI_SIGNAL:
	  {
	    this->robot->lock();
	    if (this->robot->isHeadingDone())
	      {
		this->fstate = FS_INI_WAIT;
		this->PublishInitProc(this->staticID);
	      }
	    this->robot->unlock();
	  }
	  break;

	case FS_INI_WATCH:
	  {
	    double d;
	    if (!this->finder.RangeAt(this->finder.GetAngle(), &d))
	      {
		double diff = fabs(this->finder.GetMedian() - d);
		if (diff > 25)
		  {
		    PRINT(YELLOW "I think i am watching: [%d]", this->activeRobot);
		    this->fstate = FS_INI_FOUND;
		    this->PublishInitProc(this->activeRobot);
		  }
	      }
	    else
	      {
		PRINT(RED "Robot ran away!");
		this->fstate = FS_INI_WAIT;
	      }
	  }
	  break;
	}

      // -------------------------------------------------------------------------------------------
      ros::spinOnce();
      rate.sleep();
    }
  
  swarm_bot::Announce request;
  request.request.Name = this->name.c_str();
  request.request.Shutdown = true;
  if (!this->announce.call(request))
    {
      PRINT(RED "Failed to leave swarm.");
    }
  
  PRINT(BLUE "Good night.");
}

void SwarmBot::Stop()
{
  this->isRunning = false;
}


void SwarmBot::PublishInitProc(int32_t TargetID)
{
  swarm_bot::InitProc msg;
  msg.StaticID = this->staticID;
  msg.FormationState = this->fstate;
  msg.TargetID = TargetID;
  this->initprocOut.publish(msg);
}

// ----------------- Callbacks ---------------------------------------------------------------------
void SwarmBot::CallbackInitProc(const swarm_bot::InitProc::ConstPtr &msg)
{
  if (!msg->StaticID)
    {
      int p = this->robotMap.GetPriority(this->staticID);
      if (p == 0)
	  this->fstate = (FormationState)msg->FormationState;
    }
  else
    {
      switch (msg->FormationState)
	{
	case FS_INI_WATCH:
	  {
	    int sid = this->robotMap.GetNextID(msg->StaticID);
	    if (sid > 0)
	      {
		this->activeRobot = sid;
		if (sid == this->staticID)
		    this->fstate = FS_INI_SEARCH;
	      }
	    else if (sid == 0)
	      {
		int p = this->robotMap.GetPriority(this->staticID);
		if (p == 0)
		  {
		    this->fstate = FS_INI_SIGNAL_START;
		    this->PublishInitProc(0);
		    PRINT(YELLOW "Signal out.");
		  }
	      }
	  }
	  break;
	  
	case FS_INI_SIGNAL_START:
	  this->activeRobot = msg->StaticID;
	  break;

	case FS_INI_WAIT:
	  {
	    PRINT(BLACK "Test 01 [%d][%d][%d]", this->activeRobot, msg->StaticID, msg->TargetID);
	    if ((this->activeRobot == msg->StaticID) && (msg->StaticID == msg->TargetID))
	      {
		PRINT(BLACK "Test 02");
		int sid = this->robotMap.GetNextID(msg->StaticID);
		if (sid == this->staticID)
		  {
		    this->fstate = FS_INI_SIGNAL_START;
		    this->PublishInitProc(0);
		    PRINT(YELLOW "Next Signal out.");
		  }
	      }
	    PRINT(BLACK "Test End");
	  }
	  break;

	case FS_INI_FOUND:
	  // TODO: create graph link, also check if valid first
	  break;
	}
    }
}

void SwarmBot::CallbackHeartbeat(const swarm_bot::Heartbeat::ConstPtr &msg)
{
  this->robotMap.Heartbeat(msg->StaticID);
}
