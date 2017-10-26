#include "SVector3.h"

Vector3::Vector3(){
    x=0;
    y=0;
    z=0;
}

double Vector3::size()
{
    double sz=sqrt(x*x+y*y);
    return sz;
}

// Operator overloaded using a member function
Vector3 Vector3::normalize() {
    double tx,ty;
    double size=sqrt(x*x+y*y);
    tx=x/size;
    ty=y/size;
    return Vector3(tx,ty,0.0);
}

Vector3 Vector3::operator+( Vector3 &other ) {
   return Vector3( x + other.x, y + other.y , z + other.z );
}
Vector3 Vector3::operator-( Vector3 &other ) {
   return Vector3( x - other.x, y - other.y , z - other.z );
}
Vector3 Vector3::operator*( Vector3 &other ) {
   return Vector3( x * other.x, y * other.y , z * other.z );
}
Vector3 Vector3::operator/( Vector3 &other ) {
   return Vector3( x / other.x, y / other.y , z / other.z );
}

Vector3 Vector3::operator*(double p) {
   return Vector3( x *p , y*p , z *p);
}
Vector3 Vector3::operator/( double p ) {
   return Vector3( x / p, y / p , z / p );
}

