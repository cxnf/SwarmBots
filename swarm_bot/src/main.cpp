#include "ros/ros.h"
#include "swarmbot.h++"
#include "errcodes.h++"

// ----------------- Global variables --------------------------------
SwarmBot *bot;

// ----------------- Ctrl-C handler ----------------------------------
void HandleSIGINT(int signal)
{
  printf("\n");
  bot->Stop();
}

// ----------------- Main function -----------------------------------
int main(int argc, char **argv)
{
  int ec;
  ros::init(argc, argv, "SwarmBot");
  bot = new SwarmBot;
  if ((ec = bot->Setup()))
    {
      return ec;
    }
  
  struct sigaction sih;
  sih.sa_handler = HandleSIGINT;
  sigemptyset(&sih.sa_mask);
  sih.sa_flags = 0;
  sigaction(SIGINT, &sih, 0);
  
  bot->Run();
  
  delete bot;
  return OK_SUCCESS;
}
