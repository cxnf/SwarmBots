#include ScanAnalyser.h++

// ----------------- Constructors ------------------------------------------------------------------
ScanAnalyser::ScanAnalyser()
{
}

// ----------------- Destructors -------------------------------------------------------------------
ScanAnalyser::~ScanAnalyser()
{
}

// ----------------- Methods -----------------------------------------------------------------------

int ScanAnalyser::Analyse(ArPos pos, std::list<ArPoseWithTime*> *points, std::list<ArPose> *objects);
{
  ArPoseWithTime *prevPoint = NULL;

  for (std::list<ArPoseWithTime*>::iterator it = points->begin(); it != points->end(); it++)
    {
      if (!prevPoint)
	{
	  prevPoint = *it;
	}
      else
	{
	  double x = (*it)->getX() - prevPoint->getX();
	  double y = (*it)->getY() - prevPoint->getY();
	  
	  double length = sqrt(pow(x,2) + pow(y,2));
	  PRINT(BLUE "Verschil in length: %f", length);
	}
    }
  PRINT(GREEN "-----------------------------------------------");

  /*
  double tempA = 0;
  double tempD = 0;
  for (std::list<ArPoseWithTime*>::iterator it = buffer->begin(); it != buffer->end(); ++it)
    {
      double a = pos.findAngleTo(**it);
      double d = pos.findDistanceTo(**it);

      if( tempA == 0 && tempD == 0)
	{
	  tempA = a;
	  tempD = d;
	}
      else
	{
	  
	}
    }
  */
}
