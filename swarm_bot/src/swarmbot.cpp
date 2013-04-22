#include "swarmbot.h++"

// ----------------- Constructors ------------------------------------------------------------------
SwarmBot::SwarmBot() : isRunning(false),
		       name(),
		       staticID(0),
		       fstate(FS_INI_WAIT),
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
  printf("Connection using: [%s]\n", port.c_str());
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
  
  printf("\n\033[22;32mI am [%s] assigned to [%d].\n", this->name.c_str(), this->staticID);

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

	case FS_INI_SEARCH:
	  {
	    double a, d;
	    this->robot->lock();
	    ArPose p = this->robot->getEncoderPose();
	    if (!this->finder.FindClosestRobot(&a, &d))
	      {
		this->robot->setRotVel(0);
		this->robot->setHeading(p.getTh() + 5);
		this->finder.LockOn(a);
		this->fstate = FS_INI_SEARCH_LOCK;
	      }
	    else this->robot->setRotVel(10);
	    this->robot->unlock();
	  }
	  break;

	case FS_INI_SEARCH_LOCK:
	  {
	    if (!this->finder.MeasureMedian())
	      {
		this->fstate = FS_INI_WATCH;
		swarm_bot::InitProc msg;
		msg.StaticID = this->staticID;
		msg.FormationState = this->fstate;
		this->initprocOut.publish(msg);
	      }
	  }

	case FS_INI_SIGNAL:
	  {

	  }
	  break;

	case FS_INI_WATCH:
	  {
	    double d;
	    if (!this->finder.RangeAt(this->finder.GetAngle(), &d))
	      {
		double diff = fabs(this->finder.GetMedian() - d);
		// printf("Difference [%f]:[%f]\n", this->finder.GetMedian(), d);
		if (diff > 50)
		  {
		    printf("I think i am watching: [%d]\n", this->activeRobot);
		  }
	      }
	    else
	      {
		printf("Robot ran away!\n");
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
      printf("Failed to leave swarm.\n");
    }
  
  printf("Good night.\n");
}

void SwarmBot::Stop()
{
  this->isRunning = false;
}

// ----------------- Callbacks ---------------------------------------------------------------------

void SwarmBot::CallbackInitProc(const swarm_bot::InitProc::ConstPtr &msg)
{
  if (msg->StaticID)
    {
      if (msg->StaticID == this->staticID)
	{
	  this->fstate = (FormationState)msg->FormationState;
	  printf("Now in [%d] mode.\n", this->fstate);
	}
      else
	{
	  switch (msg->FormationState)
	    {
	    case FS_INI_WATCH:
	      {
		int p = this->robotMap.GetPriority(msg->StaticID);
		if (p < 0) break;
		int sid = this->robotMap.GetStaticID(p + 1);
		if (sid > 0)
		  {
		    this->activeRobot = sid;
		    if (sid == this->staticID)
		      {
			this->fstate = FS_INI_SEARCH;
		      }
		  }
	      }
	      break;
	    }
	}
    }
  else
    {
      printf("Something went horribly wrong. [%d]\n", msg->StaticID);
    }
}

void SwarmBot::CallbackHeartbeat(const swarm_bot::Heartbeat::ConstPtr &msg)
{
  this->robotMap.Heartbeat(msg->StaticID);
}
