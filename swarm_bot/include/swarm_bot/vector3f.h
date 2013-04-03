#pragma once

#include "cmath"

//! Vector 3D float
/*!
  Position in 3D space using single-precision floating-point.
 */
class Vector3f
{
 private:
  float x;                                        // x coordinate
  float y;                                        // y coordinate
  float z;                                        // z coordinate

 public:
  //! Default constructor.
  /*!
    Constructor which zeroes out each member.
   */
  Vector3f();
  //! Helper constructor.
  /*!
    Constructor which initializes each member to the given values.
   */
  Vector3f(float _x, float _y, float _z);
  
  //! Gets x coordinate.
  /*!
    Returns the x component of the vector.
    \return X coordinate.
   */
  float GetX();
  //! Gets y coordinate.
  /*!
    Returns the y component of the vector.
    \return Y coordinate.
   */
  float GetY();
  //! Gets z coordinate.
  /*!
    Returns the z component of the vector.
    \return Z coordinate.
   */
  float GetZ();

  //! Sets x coordinate.
  /*!
    Changes the x component of the vector to the given value.
   */
  void SetX(float _x);
  //! Sets y coordinate.
  /*!
    Changes the y component of the vector to the given value.
   */
  void SetY(float _y);
  //! Sets z coordinate.
  /*!
    Changes the z component of the vector to the given value.
   */
  void SetZ(float _z);
  
  // TODO: add math functions as needed // ---------------------------

  //! Adds two vectors.
  /*!
    Adds the components of given vector to the components of this vector.
   */
  void Add(Vector3f *v0);

  //! Returns the length.
  /*!
    Returns the length of the vector.
    \return Vector length.
   */
  float Length();
  //! Returns the length before the square root.
  /*!
    Returns the length of the vector before the square root.
    Provides a faster way to obtain a valid value for length comparing, but requires only works when all lengths are squared.
    \return Squared vector length.
   */
  float LengthSquared();
  
  //! Normalize vector.
  /*!
    Normalizes the vector to a unit vector.
   */
  void Normalize();
};
