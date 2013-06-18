#include "swarmbot.h++"

// ----------------- Constructors ------------------------------------------------------------------
SwarmBot::SwarmBot() : isRunning(false),
		       name(),
		       myid(0),
		       activeRobot(0),
		       stateDelay(0),
		       dev(),
		       state(NULL),
		       oldstate(NULL),
		       nextstate(FS_WAIT),
		       robotMap(),
		       finder(),
		       node(),
		       heartbeatOut(),
		       heartbeatIn(),
		       initprocOut(),
		       initprocIn(),
		       arguments(NULL),
		       robot(NULL),
		       connector(NULL),
		       laserConnector(NULL),
		       delayTimer()
{
}

// ----------------- Destructors -------------------------------------------------------------------
SwarmBot::~SwarmBot()
{
  this->robot->disableMotors();
  this->robot->stopRunning();
  this->robot->waitForRunExit();
  Aria::shutdown();
  
  if (state) delete state;
  if (oldstate) delete oldstate;
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
  ArLog::init(ArLog::StdOut, ArLog::Terse, "", false, false, false);

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
    {
      return ERR_ARIA_CONNECTION;
    }
  this->robot->disableSonar();
  if (!this->laserConnector->connectLasers())
    {
      return ERR_ARIA_LASER;
    }
  
  this->robot->enableMotors();
  this->robot->runAsync(true);
  
  delete arg;

  // --------------- Ros Setup ---------------------------------------------------------------------
  ppnode.param<std::string>("name", this->name, "swarmbot");
  
  this->heartbeatOut = this->node.advertise<swarm_bot::Heartbeat>("heartbeat", 32);
  this->heartbeatIn = this->node.subscribe("heartbeat", 32, &SwarmBot::CallbackHeartbeat, this);
  this->initprocOut = this->node.advertise<swarm_bot::InitProc>("initproc", 32);
  this->initprocIn = this->node.subscribe("initproc", 32, &SwarmBot::CallbackInitProc, this);

  // --------------- SwarmBot setup ----------------------------------------------------------------
  boost::hash<std::string> str_hash;
  this->myid = str_hash(this->name);

  if (this->robotMap.AddRobot(this->myid))
    {
      return ERR_SWARM_MAP;
    }
  if (this->finder.Setup(this->robot))
    {
      return ERR_SWARM_FINDER;
    }

  this->dev.map = &this->robotMap;
  this->dev.finder = &this->finder;
  this->dev.robot = this->robot;

  this->isRunning = true;
  return OK_SUCCESS;
}

void SwarmBot::Run()
{
  ros::Rate rate(10);
  int second = 0;
  
  PRINT(GREEN "I am [%s] assigned to [%d]", this->name.c_str(), this->myid);

  while (this->isRunning)
    {
      // ----------- Heartbeat ---------------------------------------------------------------------
      second++;
      if (second > 10)
	{
	  second = 0;
	  swarm_bot::Heartbeat pulse;
	  pulse.ID = this->myid;
	  this->heartbeatOut.publish(pulse);
	}
      
      // ----------- Delay -------------------------------------------------------------------------
      if (this->stateDelay > 0)
	{
	  if (this->delayTimer.mSecSince() > this->stateDelay)
	    {
	      this->LoadStateController();
	      this->stateDelay = 0;
	      this->nextstate = FS_UNDEFINED;
	    }
	}

      // ----------- Update ------------------------------------------------------------------------
      if (this->state)
	{
	  FState fs = FS_UNDEFINED;
	  BroadcastState bs = BS_UNDEFINED;
	  int code = this->state->UpdateState(&this->dev, &fs, &bs);
	  if (code) PRINT(BLUE "Code [%d]", code);
	  if (fs != FS_UNDEFINED)
	    {
	      this->ChangeState(fs, false, 0);
	    }
	  if (bs != BS_UNDEFINED)
	    {
	      this->Broadcast(bs, this->activeRobot);
	    }
	}

      // -------------------------------------------------------------------------------------------
      ros::spinOnce();
      rate.sleep();
    }
  PRINT(BLUE "Good night.");
}

void SwarmBot::Stop()
{
  this->isRunning = false;
}

void SwarmBot::Broadcast(BroadcastState state, int target)
{
  swarm_bot::InitProc msg;
  msg.ID = this->myid;
  msg.FormationState = state;
  msg.TargetID = target;
  this->initprocOut.publish(msg);
}

void SwarmBot::ChangeState(FState fstate, bool backup, int delay)
{
  this->stateDelay = delay;
  if (this->stateDelay <= 0)
    {
      this->stateDelay = 1;
    }
  if (backup)
    {
      if (this->oldstate) delete this->oldstate;
      this->oldstate = this->state;
    }
  else if (this->state) delete this->state;
  this->state = NULL;
  this->nextstate = fstate;
  this->delayTimer.setToNow();
}

int SwarmBot::LoadStateController()
{
  switch (this->nextstate)
    {
    case FS_WAIT:
      {
	if (this->oldstate)
	  {
	    this->state = this->oldstate;
	    this->oldstate = NULL;
	  }
	else
	  {
	    this->state = NULL;
	  }
      }
      break;

    case FS_SEARCH:
      this->state = new SearchState;
      break;
    case FS_SIGNAL:
      this->state = new SignalState;
      break;
    case FS_FOLLOW:
      this->state = new FollowState;
      break;
    case FS_LEADER:
      this->state = new LeaderState;
      break;
      
    default:
      this->nextstate = FS_WAIT;
      return this->LoadStateController();
    }
  return OK_SUCCESS;
}

// ----------------- Callbacks ---------------------------------------------------------------------
void SwarmBot::CallbackInitProc(const swarm_bot::InitProc::ConstPtr &msg)
{
  switch ((BroadcastState)msg->FormationState)
    {
    case BS_START:
      {
	int p = this->robotMap.GetPriority(this->myid);
	if (p == 0)
	  {
	    this->ChangeState(FS_SEARCH, false, 2500);
	  }
	this->activeRobot = this->robotMap.GetID(0);
      }
      break;

    case BS_NEXT_SEARCH:
      {
	int id = this->robotMap.GetNextID(msg->ID);
	if (id == 0)
	  {
	    int p = this->robotMap.GetPriority(this->myid);
	    if (p == 0)
	      {
		this->ChangeState(FS_SIGNAL, true, 3500);
	      }
	    this->activeRobot = this->robotMap.GetID(0);
	  }
	else
	  {
	    this->activeRobot = id;
	    if (id == this->myid)
	      {
		this->ChangeState(FS_SEARCH, true, 2500);
	      }
	  }
      }
      break;

    case BS_NEXT_SIGNAL:
      {
	int id = this->robotMap.GetNextID(msg->ID);
	if (id == 0)
	  {
	    // merge graphs
	    id = this->robotMap.GetGraphLeader(0);
	    if (id == this->myid)
	      {
		PRINT(YELLOW "Leader [%d]", id);
		this->ChangeState(FS_LEADER, false);
	      }
	    else
	      {
		this->ChangeState(FS_FOLLOW, false);
	      }
	  }
	else
	  {
	    this->activeRobot = id;
	    if (id == this->myid)
	      {
		this->ChangeState(FS_SIGNAL, true, 2500);
	      }
	  }
      }
      break;
      
    case BS_FOUND:
      {
	if (msg->ID != msg->TargetID && msg->ID > 0 && msg->TargetID > 0)
	  {
	    if (!this->robotMap.LinkRobots(msg->ID, msg->TargetID))
	      {
		// seek next
	      }
	    else
	      {
		PRINT(GREEN "Link [%d][%d]", msg->ID, msg->TargetID);
	      }
	  }
	else
	  {
	    PRINT(RED "Illegal link [%d]", msg->ID);
	  }
      }
      break;

    default:
      break;
    }
}

void SwarmBot::CallbackHeartbeat(const swarm_bot::Heartbeat::ConstPtr &msg)
{
  this->robotMap.Heartbeat(msg->ID);
}
