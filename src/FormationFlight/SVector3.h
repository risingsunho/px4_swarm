#pragma once
#include <math.h>
#include <geometry_msgs/Vector3.h>
class Vector3 {


public:
   double x, y, z;
   Vector3();
   Vector3( double _x, double _y, double _z)
   {
       x=_x;
       y=_y;
       z=_z;
   }
   double size();
   Vector3 normalize();
   Vector3 operator+( Vector3 &other );
   Vector3 operator*( Vector3 &other );
   Vector3 operator/( Vector3 &other );
   Vector3 operator-( Vector3 &other );
   Vector3 operator*( double p );
   Vector3 operator/( double p );

};
