#include "swarm_controller/swarm_controller.h"

//! Announce service.
/*!
  Provides the service which assigns StaticIDs to robots.
  \param req Request from robot.
  \param res Response to robot.
  \return Value indicating request was handled.
*/
bool AnnounceService(swarm_bot::Announce::Request &req, swarm_bot::Announce::Response &res);
//! Task service.
/*!
  Provides the service whick dynamicly assigns tasks to robots.
  \param req Request from robot.
  \param res Response to robot.
  \return Value indicating request was handled.
*/
bool TaskService(swarm_bot::Task::Request &req, swarm_bot::Task::Response &res);

//! Hearbeat handler.
/*!
  Provides an implementation of the heartbeat system.
  \param msg Message holding the StaticID.
*/
void HeartbeatCallback(const swarm_bot::Heartbeat::ConstPtr &msg);

// Local variables
RobotMap robots;                                  // map linking static id to robot
int32_t nextID;                                   // next available number to assign as static id

//! Program entry point.
int main(int32_t argc, char **argv)
{
  int32_t second = 0;                             // measures when a second should have elapsed
  ros::init(argc, argv, "swarm_controller");      // initialize ROS
  ros::NodeHandle node;                           // obtain a handle to this node
  ros::Rate rate(FREQUENCY);                      // specify loop frequency
  ros::ServiceServer announce = node.advertiseService("announce", AnnounceService); // advertises the announce service
  ros::ServiceServer task = node.advertiseService("task", TaskService); // advertises the task service
  ros::Subscriber subscriber = node.subscribe("heartbeat", 32, HeartbeatCallback); // subscribe to the heartbeat
  ROS_INFO("Swarm controller started");           // log state change
  
  while (ros::ok())                               // remain in loop while node exists
    {
      second++;                                   // increment loops passed
      if (second > 5)                     // if loop iterations passed is greater then frequency, a second should have passed
	{
	  second = 0;                             // reset iteration counter
	  RobotIterator it = robots.begin();      // get iterator at begin of map
	  while (it != robots.end())              // while iterator is not at end of map
	    {
	      if (it->second.Increment())         // signal second elapsed, if robot missed heartbeat
		{
		  ROS_WARN("Robot missed heartbeat: [%d] amount [%d]", it->second.GetStaticID(), it->second.GetMissedHeartbeats()); // log warning
		  if (!it->second.IsAlive())      // if robot is not alive
		    {
		      ROS_WARN("Robot died: [%d]", it->second.GetStaticID()); // log warning
		      robots.erase(it);           // erase it from the map
		    }
		}
	      it++;                               // move to next iteration
	    }
	}
      
      ros::spinOnce();                            // allows callbacks to be called
      rate.sleep();                               // sleep the remaining time
    }
  
  ROS_INFO("Swarm controller stopped");           // log state change
  robots.clear();                                 // clear the map
  return 0;                                       // exit program
}

bool AnnounceService(swarm_bot::Announce::Request &req, swarm_bot::Announce::Response &res)
{
  for (RobotIterator it = robots.begin(); it != robots.end(); it++) // iterate through robots
    {
      if (!it->second.GetName().compare(req.Name)) // if requested name is equal to a known robot
	{
	  if (req.Shutdown)                       // if shutdown announced
	    {
	      ROS_INFO("Robot shutdown: [%d]", it->second.GetStaticID()); // log message
	      res.StaticID = 0;                   // confirm robot shutdown
	      robots.erase(it);                   // erase robot from known list
	      return true;                        // return success
	    }
	  else                                    // if startup announced
	    {
	      ROS_ERROR("Robot exists: [%s] as [%d]", req.Name.c_str(), it->second.GetStaticID()); // log error message
	      return false;                       // return failure
	    }
	}
    }
  if (req.Shutdown)                               // if shutdown announced
    {
      ROS_ERROR("Robot unknown: [%s]", req.Name.c_str()); // log error
      return false;                               // return failure
    }
  Robot r(req.Name, ++nextID);                    // create new robot with next available id
  robots[r.GetStaticID()] = r;                    // add it to the map
  res.StaticID = r.GetStaticID();                 // assigned static id must be returned to request source
  ROS_INFO("Robot announced: [%s] -> [%d]", r.GetName().c_str(), r.GetStaticID()); // log announcement
  return true;                                    // return success
}

bool TaskService(swarm_bot::Task::Request &req, swarm_bot::Task::Response &res)
{
  ROS_INFO("Task request: [%d]", req.StaticID);
  res.DynamicID = 0;
  return true;
}

void HeartbeatCallback(const swarm_bot::Heartbeat::ConstPtr &msg)
{
  RobotIterator it = robots.find(msg->StaticID);  // find robot with given static id
  if (it == robots.end())                         // if iterator at end, no robot is known with given static id
    {
      ROS_ERROR("Robot unknown: [%d]", msg->StaticID); // log error message
    }
  else                                            // if a robot with given static id is known
    {
      it->second.Heartbeat();                     // reset heartbeat time
    }
}