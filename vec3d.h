#ifndef VEC3D
#define VEC3D

#include <Arduino.h>

class vec3d {
public:

  static vec3d X;
  static vec3d Y;
  static vec3d Z;
  static vec3d zero;
  
  union{
    struct{float x, y, z;};
    float n[3];
  };
  
  vec3d(){
    x = 0; y = 0; z = 0;
  }

  vec3d(float X, float Y, float Z){
    x = X; y = Y; z = Z;
  }

  vec3d operator+(vec3d v){
    return vec3d(x + v.x, y + v.y, z + v.z);
  }

  vec3d operator-(vec3d v){
    return vec3d(x - v.x, y - v.y, z - v.z);
  }

  vec3d operator*(float a){
    return vec3d(x * a, y * a, z * a);
  }

  vec3d operator/(float a){
    if(a != 0){
      vec3d u(x / a, y / a, z / a);
      return u;
    }
    return vec3d(0,0,0);
  }
  
  void operator+=(vec3d &v){
    x += v.x;
    y += v.y;
    z += v.z;
  }

  void operator-=(vec3d &v){
    x -= v.x;
    y -= v.y;
    z -= v.z;
  }

  void operator*=(float a){
    x *= a;
    y *= a;
    z *= a;
  }

  void operator/=(float a){
    if(a != 0){
      x /= a;
      y /= a;
      z /= a;
    }
  }
  
  float angleBetween(vec3d &v, bool rads = false){
    float mags = mag() * v.mag();
    if(mags != 0){
      return acos(dot(v) / mags) * (rads ? 1 : (180 / 3.141592653589793));
    }
    return 0;
  }

  float dot(vec3d &v){
    return (x * v.x) + (y * v.y) + (z * v.z);
  }

  vec3d cross(vec3d &v){
    return vec3d(y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x);
  }

  String output(int decimal = 4){
    return "{ " + String(x, decimal) + ", " + String(y, decimal) + ", " + String(z, decimal) + " }";
  }

  void print(int decimal = 2){
    Serial.print(" { ");
    Serial.print(x, decimal);
    Serial.print(", ");
    Serial.print(y, decimal);
    Serial.print(", ");
    Serial.print(z, decimal);
    Serial.print(" } ");
  }

  void println(int decimal = 2){
    print(decimal);
    Serial.println();
  }

  float mag(){
    return sqrt(x*x + y*y + z*z);
  }

  vec3d unit(){
    float mag = this->mag();
    if(mag != 0){
      return *this / this->mag();
    }
    return vec3d(0, 0, 0);
  }

  void bound(float a, float b, float c){
    if(x > a) x = a;
    if(x < -a); x = -a;

    if(y > b) y = b;
    if(y < -b); y = -b;

    if(z > c) z = c;
    if(z < -c); z = -c;
  }

  bool isZero(){
    if(x == 0 && y == 0 && z == 0)
      return true;
    return false;
  }
};


#endif
