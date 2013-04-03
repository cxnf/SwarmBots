#include "swarm_bot/vector3f.h"

Vector3f::Vector3f() : x(0), y(0), z(0)           // zero out members
{
}
Vector3f::Vector3f(float _x, float _y, float _z) : x(_x), y(_y), z(_z) // initialize members
{
}

float Vector3f::GetX()
{
  return this->x;                                 // return x coord
}
float Vector3f::GetY()
{
  return this->y;                                 // return y coord
}
float Vector3f::GetZ()
{
  return this->z;                                 // return z coord
}

void Vector3f::SetX(float _x)
{
  this->x = _x;                                   // set x coord
}
void Vector3f::SetY(float _y)
{
  this->y = _y;                                   // set y coord
}
void Vector3f::SetZ(float _z)
{
  this->z = _z;                                   // set z coord
}

void Vector3f::Add(Vector3f *v0)
{
  if (v0 == 0)                                    // if given pointer not valid
    {
      return;                                     // stop operation
    }
  this->x += v0->GetX();                          // add and assign x coord
  this->y += v0->GetY();                          // add and assign y coord
  this->z += v0->GetZ();                          // add and assign z coord
}

float Vector3f::Length()
{
  return sqrt(this->LengthSquared());             // return the square root of the squared length
}
float Vector3f::LengthSquared()
{
  return pow(this->x, 2) + pow(this->y, 2) + pow(this->z, 2); // return the squared length
}

void Vector3f::Normalize()
{
  float l = this->Length();                       // obtain current length
  this->x /= l;                                   // divide x coord by length
  this->y /= l;                                   // divide y coord by length
  this->z /= l;                                   // divide z coord by langth
}
