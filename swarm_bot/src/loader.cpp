#include "swarm_bot/loader.h"

int32_t LoadConfig(Configuration *cfg, const char *path)
{
  libconfig::Config c;
  c.readFile(path);
  c.lookupValue("name", cfg->name);
  c.lookupValue("rosaria", cfg->basename);
  
  return 0;
}


int32_t LoadParams(Configuration *cfg)
{
  ros::NodeHandle pnode("~");
  if (pnode.hasParam("path"))
    {
      std::string tmp;
      pnode.param<std::string>("path", tmp, "swarm_bot.cfg");
      LoadConfig(cfg, tmp.c_str());
      return 0;
    }
  return 1;
}


int32_t Load(Configuration *cfg)
{
  if (LoadParams(cfg))
    {
      LoadConfig(cfg, "swarm_bot.cfg");
    }

  std::cout << cfg->name << std::endl;
  std::cout << cfg->basename << std::endl;
  return 0;
}
