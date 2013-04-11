#include "swarm_bot/loader.h"


int32_t LoadConfig(Configuration *cfg, const char *path)
{
  try
    {
      libconfig::Config c;
      c.readFile(path);
      c.lookupValue("name", cfg->name);
      c.lookupValue("rosaria", cfg->basename);
      return 0;
    }
  catch (const libconfig::FileIOException &fioe)
    {
      
      return 1;
    }
}
