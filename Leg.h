#ifndef LEG
#define LEG

#include <Servo.h>
#include "vec3d.h"
#include "Quaternion.h"

class Leg {
private:

  Servo hipYaw;
  Servo hipPitch;
  Servo knee;
  int pin[3]; //which pins the servos are attached to
  
  
public:
  vec3d footPosition = vec3d(0.055, 0, 0); //current stored position
  vec3d targetPos; //current movement target
  vec3d stepTarget; //eventual final target for a step
  float moveSpeed; //current speed for a step
  float firstMoveSpeed; //initial speed for a step
  vec3d worldPosition; //position of hip joint relative to body origin
  
  Quaternion worldToLocalQuaternion; //rotation of leg space about body space z-axis
  bool isMoving = false;
  int8_t stepEventCounter = -1;
  float stepDelay;

  //relevant dimensions of leg in meters
  float hipAxisOffset = 0.0417;
  float femurLength = 0.055;
  float shinLength = 0.076;
  
  vec3d servoOffsets = vec3d(90, 90, 0); //degrees

  vec3d servoTrim = vec3d(0, 0, 0); //in degrees
  bool servoIsReversed[3] = {true, false, false};

  Leg(){}
  
  Leg(int hy_pin, int hp_pin, int kn_pin){
    pin[0] = hy_pin; pin[1] = hp_pin; pin[2] = kn_pin;
  }

  void moveTowardTarget(float dt){ //translates foot towards a target in a straight line at a certain speed

    if(isMoving){ //only works if leg is allowed to move
      vec3d stepdir = targetPos - footPosition;
      float stepSize = moveSpeed * dt;

      //if remaining distance to target is greater than twice the distance you can move in this timestep, and
      //remain at the specified speed, then move towards it at that speed. Else, if you're closer than that, 
      //jump straight to the target and stop moving.
      
      if(stepdir.mag() > 2 * stepSize){ 
        stepdir = stepdir.unit() * stepSize;
        translateLocalSpace(stepdir);
      }
      else{
        translateLocalSpace(stepdir);
        isMoving = false;
      }
    }
  }

  void stepping(float stepHeight){
    static long timer;

    //during a step, the leg goes through these cases in order

    if(stepEventCounter > -1){
      switch(stepEventCounter){
      case 0:
      {//move to top of step
        isMoving = true;//begin leg movement
        
        targetPos = stepTarget + footPosition;
        targetPos /= 2;//aim for a point above the point halfway to the target
        targetPos.z += stepHeight;
        
        stepEventCounter = 1;
        break;
      }
      case 1:
      {
        if(isMoving == false){//move to step target
          targetPos = stepTarget;
          isMoving = true;
          stepEventCounter = 2;
        }
        break;
      }
      case 2:
      {
        targetPos = stepTarget;
        if(isMoving == false){//step target is reached, start the timer for delay between steps
          stepEventCounter = 3;
          timer = millis();
          if(stepDelay <= 0)
            stepEventCounter = -1;
        }
        break;
      }
      case 3:
      {//end the step if timer is reached
        if((millis() - timer) > stepDelay * 1000)
          stepEventCounter = -1;
        break;
      }
      }
    }

  }
  
  void setStepDelay(float robotSpeed, float robotMaxSpeed, float stepDistance, float stepHeight){
    if(robotSpeed == 0){
      stepDelay = 0.1;
      return;
    }
    stepDelay = (stepDistance / 4 / robotSpeed) - (3*sqrt(sq(stepDistance) + 4*sq(stepHeight)) / 4 / moveSpeed);
  }

  void setMoveSpeed(float robotSpeed, float robotMaxSpeed, float stepDistance, float stepHeight){
    //moveSpeed = 3 * robotMaxSpeed * sqrt(sq(stepDistance) + 4*sq(stepHeight)) / stepDistance;
    if(robotSpeed == 0){
      robotSpeed = robotMaxSpeed;
    }
    float robotSpeedMix = robotMaxSpeed * sqrt(robotSpeed / robotMaxSpeed);
    moveSpeed = 3 * robotSpeedMix * sqrt(sq(stepDistance) + 4*sq(stepHeight)) / stepDistance;
    firstMoveSpeed = moveSpeed;
  }

  void attachServos(){
    hipYaw.attach(pin[0]);
    hipPitch.attach(pin[1]);
    knee.attach(pin[2]);
  }

  void attachServos(int hy_pin, int hp_pin, int kn_pin){
    pin[0] = hy_pin; pin[1] = hp_pin; pin[2] = kn_pin;
    hipYaw.attach(pin[0]);
    hipPitch.attach(pin[1]);
    knee.attach(pin[2]);
  }
  
  void setFootPosWorld(vec3d v){
    v = v - worldPosition;
    footPosition = worldToLocalQuaternion.inverse() * v;
  }

  vec3d getFootPosWorld(){
    vec3d result = worldToLocalQuaternion * footPosition;
    return result + worldPosition;
  }

  void translateWorldSpace(vec3d v){
    v = worldToLocalQuaternion.inverse() * v;
    footPosition += v;
  }

  vec3d localToWorld(vec3d v){
    vec3d result = worldToLocalQuaternion * v;
    return result + worldPosition;
  }

  vec3d worldToLocal(vec3d v){
    v = v - worldPosition;
    return worldToLocalQuaternion.inverse() * v;
  }

  void setFootPosLocal(vec3d v){
    footPosition = v;
  }

  void translateLocalSpace(vec3d v){
    footPosition += v;
  }

  void setLegOrigin(vec3d pos, Quaternion q){
    worldToLocalQuaternion = q;
    worldPosition = pos;
  }

  vec3d getAngles(){ //This is the inverse kinematics, returns servo angles based on stored foot position
    float theta, q1, q2, xprime;

    theta = atan2(footPosition.y, footPosition.x);
    xprime = sqrt(sq(footPosition.x) + sq(footPosition.y)) - hipAxisOffset;
    
    q2 = acos((sq(xprime) + sq(footPosition.z) - sq(femurLength) - sq(shinLength)) / (2 * shinLength * femurLength));
    q1 = atan2(footPosition.z, xprime) + atan2(shinLength * sin(q2), (femurLength + shinLength * cos(q2)));

    vec3d angles(theta, q1, q2);

    angles *= 180 / PI;
    
    return angles;
  }

  //setServos with no args will use the inverse kinematics to put the foot at footPosition. you can
  //call it with a vec3d of angles to set those specific angles instead.
  
  void setServos(){
    setServos(getAngles());
  }

  void setServos(vec3d a, bool skipBoundsCheck = false){
    const int upperBoundDeg = 180;
    const int lowerBoundDeg = 0;
    a += servoOffsets;
    for(byte i = 0; i < 3; i++)
      if(servoIsReversed[i])
        a.n[i] = 180 - a.n[i];
    a += servoTrim;
    
    if(a.x < upperBoundDeg && a.x > lowerBoundDeg || skipBoundsCheck)
      hipYaw.writeMicroseconds(degreesToMicros(a.x));
    if(a.y < upperBoundDeg && a.y > lowerBoundDeg || skipBoundsCheck)
      hipPitch.writeMicroseconds(degreesToMicros(a.y));
    if(a.z < upperBoundDeg && a.z > lowerBoundDeg || skipBoundsCheck)
      knee.writeMicroseconds(degreesToMicros(a.z));
  }

  
  const int lowerMicros = 650;
  const int upperMicros = 2350;

  
  //servos receive their target angle in the form of a high signal pulsed for a specific number of microseconds.
  //this function just maps 0-180 degrees onto the range specified by the constants above.
  
  int degreesToMicros(float deg){
    return upperMicros + (upperMicros - lowerMicros) * ( deg - 180) / 180;
  }
  
};

#endif
