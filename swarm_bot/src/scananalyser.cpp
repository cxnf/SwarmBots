#include "scananalyser.h++"

// ----------------- Constructors ------------------------------------------------------------------
ScanAnalyser::ScanAnalyser()
{
}

// ----------------- Destructors -------------------------------------------------------------------
ScanAnalyser::~ScanAnalyser()
{
}

// ----------------- Methods -----------------------------------------------------------------------

int ScanAnalyser::Analyse(ArPose pos, std::list<ArPoseWithTime*> *points, std::list<ArPose> *objects)
{
  ArPoseWithTime *prevPoint = NULL;
  std::list<ArPose*> obj;
  int debugint = 0;

  for (std::list<ArPoseWithTime*>::iterator it = points->begin(); it != points->end(); ++it, ++debugint)
    {
      if (prevPoint)
	{
	  double x = (*it)->getX() - prevPoint->getX();
	  double y = (*it)->getY() - prevPoint->getY();
	  
	  double length = sqrt(pow(x, 2) + pow(y, 2));
	  if (length > OBJ_MARGIN)
	    {
	      objects->push_back(this->Avarage(&obj));
	      obj.clear();
	    }
	}
      obj.push_back(*it);
      prevPoint = (*it);
    }
  if (obj.size())
    {
      objects->push_back(this->Avarage(&obj));
      obj.clear();
    }
  return OK_SUCCESS;
}

ArPose ScanAnalyser::Avarage(std::list<ArPose*> *obj)
{
  double tx = 0, ty = 0;
  if (!obj->size())
    {
      return ArPose();
    }

  for (std::list<ArPose*>::iterator itav = obj->begin(); itav != obj->end(); ++itav)
    {
      tx += (*itav)->getX();
      ty += (*itav)->getY();
    }
  tx /= obj->size();
  ty /= obj->size();

  return ArPose(tx, ty);
}

Scan ScanAnalyser::AvarageScan(std::vector<Scan> *scans)
{
  double d = 0, a = 0;
  if (!scans->size())
    {
      return Scan();
    }
  for (std::vector<Scan>::iterator it = scans->begin(); it != scans->end(); ++it)
    {
      d += (*it).distance;
      a += (*it).angle;
    }
  d /= scans->size();
  a /= scans->size();

  return Scan(d, a);
}


void ScanAnalyser::AnalyseBuffer(std::vector<Scan> *buffer, std::list<Scan> *objects)
{
  Scan *prev = NULL;
  std::vector<Scan> scans;
  for (std::vector<Scan>::iterator it = buffer->begin(); it != buffer->end(); ++it)
    {
      if (prev)
	{
	  double dif = fabs(prev->distance - (*it).distance);
	  if (dif > OBJ_MARGIN || (*it).isseperator)
	    {
	      objects->push_back(this->AvarageScan(&scans));
	      scans.clear();
	    }
	}
      scans.push_back(*it);
      prev = &(*it);
    }
  if (scans.size())
    {
      objects->push_back(this->AvarageScan(&scans));
      scans.clear();
    }
}
