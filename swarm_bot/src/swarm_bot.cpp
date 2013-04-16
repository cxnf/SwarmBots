#include "swarm_bot/swarm_bot.h"

// Prototypes.
//! Sonar handler.
/*! 
  Reads and processes sonar readings.
  \param msg Sonar readings.
 */ 
void SonarCallback(const sensor_msgs::PointCloud::ConstPtr &msg);
//! Pose handler.
/*! 
  Reads the odometry data.
  \param msg Odometry status.
 */
void PoseCallback(const nav_msgs::Odometry::ConstPtr &msg);

// Variables.
Robot *self;                                      // pointer to logical representation of self
std::string name;                                 // robot name
ros::Publisher heartbeat;                         // publisher to heartbeat topic
ros::ServiceClient announce;                      // client for announce service
ros::Subscriber sonar;                            // subscriber to sonar topic
ros::Subscriber pose;                             // subscriber to pose topic



int main(int argc, char **argv)
{
  int32_t seconds = 0;                            // counter to estimate when second elapsed
  self = 0;                                       // clear memory of pointer to self

  {
    utsname buf;                                  // temp storage for computer name
    if (!uname(&buf))                             // get computer name (and some other stuff), if data was obtained successfully
      {
	boost::regex r("[^a-zA-Z_]");             // get regex for replace
	name = boost::regex_replace(std::string(buf.nodename), r, "_"); // replace illegal chars with legal char
      }
    else                                          // if computer name was not obtained
      {
	return ERR_GENERAL;                       // return error
      }
  }
  
  ros::init(argc, argv, name.c_str());            // initialize ros
  ros::NodeHandle node;                           // obtain handle to node
  ros::Rate rate(FREQUENCY);                      // create loop rate with predefined frequency
  
  heartbeat = node.advertise<swarm_bot::Heartbeat>("heartbeat", 32); // create publisher to heartbeat topic
  announce = node.serviceClient<swarm_bot::Announce>("announce"); // ceate client for announce service
  // sonar = node.subscribe((cfg->basename + "/sonar").c_str(), 32, SonarCallback);  // create subscriber for the sonar callback
  // pose = node.subscribe((cfg->basename + "/pose").c_str(), 32, PoseCallback);  // create subscriber for the pose callback
  
  struct sigaction sih;                           // alloc struct for signal handling
  sih.sa_handler = SIGINTHandler;                 // set handler for SIGINT signal
  sigemptyset(&sih.sa_mask);                      // clear memory
  sih.sa_flags = 0;                               // clear flags
  sigaction(SIGINT, &sih, 0);                     // register signal handler

  swarm_bot::Announce announceSrv;                // allocate request to service
  announceSrv.request.Name = name.c_str();        // initialize request name
  announceSrv.request.Shutdown = false;           // clear shutdown flag, robot is starting
  if (announce.call(announceSrv))                 // send request to the service, if no error
    {
      if (announceSrv.response.StaticID <= 0)     // if staticID is invalid
	{
	  return ERR_SWARM_CONTROLLER;            // return error
	}
      self = new Robot(name.c_str(), announceSrv.response.StaticID); // create logical representation of self
    }
  else                                            // if announce service error
    {
      return ERR_ROS_SERVICE;                     // return error
    }
  
  ROS_INFO("I am [%s] aka [%d].", name.c_str(), self->GetStaticID()); // log summary of startup data
  while (ros::ok())                               // while ros is ok
    {
      seconds++;                                  // increment loop count
      if (seconds > FREQUENCY)                    // if counter exceeds frequency
	{
	  seconds = 0;                            // reset counter
	  swarm_bot::Heartbeat msg;               // allocate msg to heartbeat
	  msg.StaticID = self->GetStaticID();     // initialize staticID in the message
	  msg.X = self->GetLocation().GetX();     // present x coordinate of robot position
	  msg.Y = self->GetLocation().GetY();     // present y coordinate of robot position
	  msg.Z = self->GetLocation().GetZ();     // present z coordinate of robot position
	  heartbeat.publish(msg);                 // send message to the topic 
	}
      
      ros::spinOnce();                            // allows callbacks to be called
      rate.sleep();                               // sleep off remaining tiem
    }
  
  if (self)                                       // if self is allocated
    {
      delete self;                                // free self
    }
  return OK_SUCCESS;                              // return success
}

void SonarCallback(const sensor_msgs::PointCloud::ConstPtr &msg)
{
  for (unsigned int i = 0; i < msg->points.size(); i++) // loops through available ultrasonic sensors
    {
      //ROS_INFO("Point:  (%f;%f;%f)", msg->points[i].x, msg->points[i].y, msg->points[i].z);
      float o = atan2f(msg->points[i].x,msg->points[i].y); // extract yaw from vector
      o -= M_PI_2;                                // transform sensor space
      o = (Mod((o + M_PI), PI2) - M_PI);          // wrap around range pi .. -pi      
      // TODO: process angle and distance (if any)
      // vector -> vector3f(xyz) -> length
      // o hebben we de hoek bepaald
      float diff  = fabs(self->GetOrientation() - o); // angle for the sensor relatively to the robot
    }
}

void PoseCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
  if (self)                                       // if self is allocated
    {
      self->SetLocation(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z); // update robot position
      double r = tf::getYaw(msg->pose.pose.orientation); // extract yaw
      self->SetOrientation((float)r);             // set orientation
    }
}

void SIGINTHandler(int s)
{
  printf("\n");
  swarm_bot::Announce announceSrv;                // allocate service request
  announceSrv.request.Name = self->GetName().c_str(); // initialize request name
  announceSrv.request.Shutdown = true;            // set shutdown flag
  if (!announce.call(announceSrv))                // send request to the service, if error
    {
      ROS_ERROR("Could not shutdown. UNSTOPPABLE"); // log error
    }
  ros::shutdown();                                // shutdown node
}

float inline Mod(float x, float y) 
{
  if( 0 == y)                                     // denominator can't be zero
    {
      return x;                                   // return nominator
    }
  return x - y * floor(x / y);                    // perform modulo
}
