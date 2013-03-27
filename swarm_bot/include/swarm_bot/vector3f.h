#pragma once

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
   */
  float GetX();
  //! Gets y coordinate.
  /*!
    Returns the y component of the vector.
   */
  float GetY();
  //! Gets z coordinate.
  /*!
    Returns the z component of the vector.
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
  
  // TODO: add math functions as needed
  //! Adds two vectors.
  /*!
    Adds the components of given vector to the components of this vector.
   */
  void Add(Vector3f *v0);
};

