#include "swarm_bot/swarm_bot.h"

Robot *self;                                      // pointer to logical representation of self

int main(int argc, char **argv)
{
  int32_t seconds = 0;                            // counter to estimate when second elapsed
  self = 0;                                       // clear memory of pointer to self
  
  ros::init(argc, argv, "swarm_bot");             // initialize ros
  ros::NodeHandle node;                           // obtain handle to node
  ros::Publisher heartbeat = node.advertise<swarm_bot::Heartbeat>("heartbeat", 32); // create publisher to heartbeat topic
  ros::ServiceClient announce = node.serviceClient<swarm_bot::Announce>("announce"); // ceate client for announce service
  ros::Rate rate(FREQUENCY);                      // create loop rate with predefined frequency
  swarm_bot::Announce announceSrv;                // allocate request to service
  
  announceSrv.request.Name = "Test";              // initialize request name
  announceSrv.request.Shutdown = false;           // clear shutdown flag, robot is starting
  if (announce.call(announceSrv))                 // send request to the service, if no error
    {
      if (announceSrv.response.StaticID <= 0)     // if staticID is invalid
	{
	  return 1;                               // return error
	}
      self = new Robot("Test", announceSrv.response.StaticID); // create logical representation of self
    }
  else                                            // if announce service error
    {
      return 2;                                   // return error
    }
  
  while (ros::ok())                               // while ros is ok
    {
      seconds++;                                  // increment loop count
      if (seconds > FREQUENCY)                    // if counter exceeds frequency
	{
	  seconds = 0;                            // reset counter
	  swarm_bot::Heartbeat msg;               // allocate msg to heartbeat
	  msg.StaticID = self->GetStaticID();     // initialize staticID in the message
	  
	  heartbeat.publish(msg);                 // send message to the topic 
	}
      
      rate.sleep();                               // sleep off remaining time
    }
  
  announceSrv.request.Shutdown = true;            // set shutdown flag
  if (!announce.call(announceSrv))                // send request to the service, if error
    {
      ROS_ERROR("Could not shutdown. UNSTOPPABLE"); // log error
    }
  else
    {
      ROS_INFO("Response: [%d]", announceSrv.response.StaticID);
    }
  
  if (self)                                       // if self is allocated
    {
      delete self;                                // free self
    }
  return 0;                                       // return success
}
