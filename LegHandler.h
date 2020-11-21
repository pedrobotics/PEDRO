#ifndef LEGHANDLER
#define LEGHANDLER

#include <Servo.h>
#include <EEPROM.h>
#include "vec3d.h"
#include "Quaternion.h"
#include "Leg.h"


class LegHandler {
private:
  const uint8_t numLegs = 4;
  vec3d moveDir; //contains <left stick X, left stick Y, right stick Y>
  vec3d eulerAngles; //contains <right stick X, right stick Y, right stick X>

//  Quaternion rotation;
//  Quaternion targetRot;
  float maxSpeedTrot = 0.4;
  float maxSpeedCrawl = 0.2;
  float maxShift = 0.1;
  float legSpeed = 1.0;
  float maxRotRate = 90;
  float maxRot = 30;
  float stepRadius = 0.01;
  float stepHeight = 0.05;
  float stepDistance = 0.1;
  vec3d legHome = vec3d(0.09, 0, -0.05);
  bool confirmBounce = false;
  bool smoothInput = true;
  bool waitForJoyReset = false;
  
  //INTERACT STUFF
  byte interSelectedLeg = 0;
  vec3d interactShift = vec3d(-0.02, 0, -0.03);
  bool interactSetup = true;
  
  long prevMicros;
  long servoTimer;
  long bounceTimer;

  byte fwdPattern[4] = {2, 1, 3, 0};
  byte sidePattern[4] = {3, 2, 0, 1};
  int8_t prevLeg = -1;

  vec3d stepTarget[4];

  void translate(vec3d v, bool ignoreCheck = false){ //translates all non-stepping legs in the opposite direction indicated, thereby translating the body in the correct direction
    for(byte i = 0; i < numLegs; i++) {if(leg[i].stepEventCounter < 0 || ignoreCheck)leg[i].translateWorldSpace(v*-1);}
  }

  
  
public:
  
  class Trimmer { //handles EEPROM stuff for stored servo trim values
  private:
    int addr = 0;
    
  public:
    float values[4][3];
    byte legn = 0;

    void load(){
      EEPROM.get(addr, values);
    }

    void save(){
      EEPROM.put(addr, values);
    }

    void reset(){
      for(byte i = 0; i < 4; i++){
        for(byte j = 0; j < 3; j++){
          values[i][j] = 0;
        }
      }
      save();
    }

    void updateVals(vec3d v){
      for(byte i = 0; i < 3; i++){
        values[legn][i] += v.n[i];
      }
    }
    
  } servoTrims;
  
  enum Mode { //all possible modes (I KNOW)
    legTrimmer,
    walking,
    shifting,
    trotting,
    crawling,
    rideAdjust,
    stepAdjust, 
    shiftTrack,
    shiftRate,
    yawAndHeight,
    pitchAndRoll,
    modeChange,
    interact
  }mode = walking;

  Mode walkMode = crawling;
  Mode walkRightStick = rideAdjust;
  Mode shiftMode = shiftTrack;
  Mode shiftRightStick = pitchAndRoll;
  
  Leg leg[4];

  LegHandler(){
    prevMicros = micros();
    servoTimer = millis();
    mode = walking;
  }

  void update(int Hz = 1000){
    
    float dt = micros() - prevMicros;
    if(dt >= 1000000.0 / Hz){ //only performs update at the specified freq
      prevMicros = micros();
      dt /= 1000000;
      if(dt > 20.0 / Hz) dt = 20.0 / Hz;
      
      switch(mode){ //performs action based on current mode
      case modeChange: //the mode change menu
      { 
        smoothInput = false;
        
        float angle = atan2(moveDir.y, moveDir.x);
        angle += PI / 2; //X is forward, so must rotate 90 deg clockwise to match with stick directions
        
        if(moveDir.mag() > 0.7){
          
          if(-PI/4 < angle && angle < PI/4){
            //RIGHT
            mode = interact;
            interactSetup = true;
          }
          else if(PI/4 < angle && angle < 3*PI/4){
            //UP
            mode = walking;
            walkMode = crawling;
            walkRightStick = rideAdjust;
          }
          else if(-3*PI/4 < angle && angle < -PI/4){
            //DOWN
            mode = shifting;
            shiftMode = shiftTrack;
            shiftRightStick = yawAndHeight;
          }
          else {
            //LEFT

            //YOU CAN ADD AN ADDITIONAL MODE HERE, IF YOU WANT!
            
            //mode = yourModeHere;
            
            break;
          }

          waitForJoyReset = true;
          moveDir = vec3d::zero;
          bounce();
        }
        break;
      }

      case interact: //i.e. one leg raised off the ground mode
      {
        smoothInput = true;
        vec3d raisedHome(0.1, 0, 0);
        static byte prevLeg{};

        if(prevLeg != interSelectedLeg){
          interactSetup = true;
          prevLeg = interSelectedLeg;
        }
        
        if(interactSetup){
          vec3d shift = leg[interSelectedLeg].worldPosition.unit() * 0.04;
          for(byte i = 0; i < 4 ;i++){
            leg[i].isMoving = true;
            leg[i].moveSpeed = 0.05;
            leg[i].targetPos = legHome + leg[i].worldToLocalQuaternion.inverse() * shift;
          }
          interactSetup = false;
        }

        for(byte i = 0; i < 4 ;i++){
          //if(i == interSelectedLeg) continue;
          leg[i].moveTowardTarget(dt);
        }

        if(noLegMoving()){
          leg[interSelectedLeg].setFootPosLocal(raisedHome);
          leg[interSelectedLeg].translateWorldSpace(moveDir * 0.12);
        }
        
        break;
      }
      
      case legTrimmer:
      {
        smoothInput = false;
        vec3d ang(0,0,90); 
        for(byte legn = 0; legn < 4; legn++){
          leg[legn].setServos(ang, true);
        }
        vec3d dtrim = moveDir * dt * 10;
        servoTrims.updateVals(dtrim);
        leg[servoTrims.legn].servoTrim += dtrim;

        for(byte i = 0; i < 4; i++){
          for(byte j = 0; j < 3; j++){
            leg[i].servoTrim.n[j] = servoTrims.values[i][j];
          }
        }
        
        return;
      }
        
      case walking:
      {
        smoothInput = true;
        vec3d inputVect = moveDir;
        inputVect.z = 0;
        if(!inputVect.isZero() || !legsAreWithinBound(0.005) || !noLegStepping()){// skip if no input and legs at home
          if(noLegStepping()){
            int8_t legn;
            byte numLegs = 1;
            
            switch(walkMode){
              case crawling:
              {
                legn = legSelect();
                break;
              }
              case trotting:
              {
                static byte trotLegs{};
                legn = ++trotLegs & 1;
                numLegs = 2;
                break;
              }
            }
            
            if(legn >= 0){//legSelect returns -1 on fail
              float stepTimeTrot = 0.25;
              float stepDistTrot = stepTimeTrot * inputVect.mag() * maxSpeedTrot;
              if(stepDistTrot > stepDistance) {
                stepTimeTrot = stepDistance / (inputVect.mag() * maxSpeedTrot);
                stepDistTrot = stepDistance;
              }
              
              for(byte i = 0; i < numLegs + 1; i += 2){
                if(inputVect.isZero())// control input is zero
                  leg[legn + i].stepTarget = legHome;
                else{// translational input is present
                  leg[legn + i].stepTarget =  inputVect.unit() * (walkMode == crawling ?stepDistance * (0.35 + 0.15 * inputVect.mag()) : 0.3 * stepDistTrot);
                  leg[legn + i].stepTarget = leg[legn + i].worldToLocalQuaternion.inverse() * leg[legn + i].stepTarget;
                  leg[legn + i].stepTarget += legHome;
                }
                
                leg[legn + i].stepEventCounter = 0;
                if(walkMode == crawling) leg[legn + i].setMoveSpeed(inputVect.mag() * maxSpeedCrawl, maxSpeedCrawl, stepDistance, stepHeight);
                else {
                  leg[legn + i].moveSpeed = inputVect.isZero() ? 1 * (abs(eulerAngles.z) + 1) * maxSpeedTrot : sqrt(sq(stepDistTrot) + 4*sq(stepHeight)) / stepTimeTrot;
                  leg[legn + i].firstMoveSpeed = leg[legn + i].moveSpeed;
                }
                
              }
            }
          }
          else{// some leg is stepping
            
            for(byte i = 0; i < 4; i++){// find the stepping leg and update
              
              if(leg[i].stepEventCounter > -1){
                if(walkMode == crawling) leg[i].setStepDelay(inputVect.mag() * maxSpeedCrawl, maxSpeedCrawl, stepDistance, stepHeight);
                else leg[i].stepDelay = 0 ;

                if(!eulerAngles.isZero()){
                  //rotate stepTarget with yaw;
                  float d0 = (leg[i].stepTarget - leg[i].footPosition).mag();
                  vec3d temp = leg[i].worldToLocalQuaternion * leg[i].stepTarget;
                  vec3d yaw(0, 0, 1);
                  Quaternion q(yaw, -eulerAngles.z * maxRotRate * dt * 0.5 * leg[i].moveSpeed / (walkMode == crawling ? maxSpeedCrawl : maxSpeedTrot));
                  
                  leg[i].stepTarget = q * temp;
                  leg[i].stepTarget = leg[i].worldToLocalQuaternion.inverse() * leg[i].stepTarget;
                  
                  float d1 = (leg[i].stepTarget - leg[i].footPosition).mag();
                  leg[i].moveSpeed = leg[i].firstMoveSpeed + legHome.x * abs(eulerAngles.x) * maxRotRate * (walkMode == crawling ? 1 : 2) * PI / 180; 
                }
                
                leg[i].stepping(stepHeight);
                leg[i].moveTowardTarget(dt);
              }
              
              
            }
          }
        }
        switch(walkRightStick){
          case rideAdjust:
          {
            //adjust ride height
            if(legHome.z <= 0 && legHome.z >= -0.15 && noLegStepping()){
              float vert = moveDir.z * dt * 0.2;
              legHome.z -= vert;
              translate(vec3d(0, 0, vert), true);
            }
            else if(legHome.z > 0)
              legHome.z = 0;
            else if(legHome.z < -0.15)
              legHome.z = -0.15;
            break;
          }
          case stepAdjust:
          {
            //adjust step height
            if(stepHeight <= 0.1 && stepHeight >= 0.01 && !noLegStepping()){
              float vert = moveDir.z * dt * 0.2;
              stepHeight += vert;
            }
            else if(stepHeight > 0.1)
              stepHeight = 0.1;
            else if(stepHeight < 0.01)
              stepHeight = 0.01;
            break;
          }
        }


        if(!moveDir.isZero()){
          vec3d direction = moveDir * dt;
          direction.z = 0;
          
          if(walkMode == crawling)
            direction *= maxSpeedCrawl;
          else if(walkMode == trotting)
            direction *= maxSpeedTrot;
          
          translate(direction);
        }
        
        //turning    
        vec3d yaw = eulerAngles;
        yaw.x = 0; yaw.y = 0;
        rotate(yaw, maxRotRate * dt * (walkMode == trotting ? 2 : 1));
        break;
      }// END WALKING
        
      case shifting: //i.e. positioning mode
      {
        smoothInput = true;
        switch(shiftRightStick){
          case pitchAndRoll:
          {
            eulerAngles.z = 0;
            moveDir.z = 0;
            break;
          }
          case yawAndHeight:
          {
            eulerAngles.x = 0;
            eulerAngles.y = 0;
            break;
          }
        }
        switch(shiftMode){
          case shiftTrack:
          {
            legHome = vec3d(0.09, 0, -0.07);
            vec3d target = moveDir * -1 * maxShift;
            if(target.z > 0) target.z *= 1.5;
            
            for(byte i = 0; i < numLegs; i++) {
              leg[i].setFootPosLocal(legHome);
              leg[i].translateWorldSpace(target);
            }
            rotate(eulerAngles);
            
            break;
          }
          case shiftRate:
          {
            translate(moveDir * dt * 0.1, true);
            rotate(eulerAngles, maxRotRate * dt);
            
            //this is for limiting max angle to 30 deg
            vec3d a = leg[1].getFootPosWorld() - leg[0].getFootPosWorld();
            vec3d b = leg[2].getFootPosWorld() - leg[0].getFootPosWorld();
            vec3d c = a.cross(b);

            float alpha = c.angleBetween(vec3d::Z);
            if(alpha > 30){
              rotate(eulerAngles, -maxRotRate * dt);
            }
          }
        }
        
        
        break;
      }
      }// end switch
      
      //bounce that indicates a particular command has been received
      if(confirmBounce && millis() - bounceTimer > 250){
        confirmBounce = false;
        translate(vec3d::Z * 0.01);
        legHome.z -= 0.01;
      }
      
      setServos(); //servos only actually move at this point in the update loop
    }
  }/////  END UPDATE  /////

  int8_t legSelect(){ //figures out which leg should be called to take a step, returns leg number 0-3 or -1
    if(moveDir.isZero()){
      if(eulerAngles.z <= 0){
        int8_t leg = (prevLeg+1) & 3; //gives same answer as 'x % 4', but without returning negatives
        prevLeg = leg;
        return leg;
      }
      int8_t leg = (prevLeg-1) & 3;
      prevLeg = leg;
      return leg;
    }
      
    if(abs(moveDir.x) > abs(moveDir.y)){ //robot is being directed mostly forward or backward
      if(moveDir.x > 0){
        for(byte i = 0; i < numLegs; i++){
          if(fwdPattern[i] == prevLeg){
            prevLeg = fwdPattern[++i & 3];
            return prevLeg;
          }
        }
        //prevLeg not found in list, indicates prevleg is -1
        prevLeg = 2;
        return prevLeg;
      }
      else{
        for(byte i = 3; i != 254; i--){
          if(fwdPattern[i] == prevLeg){
            prevLeg = fwdPattern[--i & 3];
            return prevLeg;
          }
        }
        prevLeg = 0;
        return prevLeg;
      }
    }
    else{ // robot is being directed mostly to strafe
      if(moveDir.y > 0){
        for(byte i = 0; i < numLegs; i++){
          if(sidePattern[i] == prevLeg){
            prevLeg = sidePattern[++i & 3];
            return prevLeg;
          }
        }
        prevLeg = 3;
        return prevLeg;
      }
      else{
        for(byte i = 3; i != 254; i--){
          if(sidePattern[i] == prevLeg){
            prevLeg = sidePattern[--i & 3];
            return prevLeg;
          }
        }
        prevLeg = 1;
        return prevLeg;
      }
    }
    return -1;
  }

  void rotate(vec3d r, float scale = 30){ //rotates a point around a vector centered on body origin. rotation amount is vector mag in degrees, scaled by "scale"
    //I did it this way because it makes sense for interperating controller inputs as rotations.
    //Scale is used as the bounds when in coarse positioning, and the deg/s rate when walking or in fine positioning.
    Quaternion q(r, r.mag() * scale);
    for(byte i = 0; i < numLegs; i++){
      vec3d temp = leg[i].getFootPosWorld();
      temp = q * temp;
      leg[i].setFootPosWorld(temp);
    }
  }

  /////  TOOLS  /////
  
  bool noLegStepping(byte* which_leg = NULL){ //returns true if no leg is currently in the stepping process
    for(byte i = 0; i < numLegs; i++){
      if(leg[i].stepEventCounter > -1){
        *which_leg = i;
        return false;
      }
    }
    return true;
  }

  bool noLegMoving(){
    for(byte i = 0; i < numLegs; i++){
      if(leg[i].isMoving){
        return false;
      }
    }
    return true;
  }

  float farthestLegDist(){ //returns the greatest 'leg home to foot' distance of the four legs
    float dist[4];
    bool moving = false;
    if(!moveDir.isZero()) moving = true;
    for(byte i = 0; i < numLegs; i++){
      float directionMultiplier = 1;
      if(moving){
        vec3d v = leg[i].footPosition - legHome;
        v = leg[i].worldToLocalQuaternion * v;
        //directionMultiplier = -moveDir.dot(v);
        //if(moveDir.dot(v) < 0) directionMultiplier = 0;
        v = v.unit();
        directionMultiplier = -moveDir.dot(v);
      }
      dist[i] = sqrt(sq(leg[i].footPosition.x - legHome.x) + sq(leg[i].footPosition.y - legHome.y)) * directionMultiplier;
    }
    byte farthest = 0;
    for(byte i = 0; i < numLegs; i++){
      if(dist[i] > dist[farthest])
        farthest = i;
    }
    return dist[farthest];
  }

  bool legsAreWithinBound(float radius){
    for(byte i = 0; i < 4; i++){
      vec3d v = leg[i].footPosition - legHome;
      //v.z = 0;
      if(v.mag() > radius)
        return false;
    }
    return true;
  }

  void legsGtoL(byte* list){ //returns a list of the indices of the legs in order of
    float dist[4];           //greatest to least distance from home
    for(byte i = 0; i < numLegs; i++){
      vec3d v = leg[i].footPosition - legHome;
      v = leg[i].worldToLocalQuaternion.inverse() * v;
      float directionMultiplier = -moveDir.dot(v);
      dist[i] = sqrt(sq(leg[i].footPosition.x - legHome.x) + sq(leg[i].footPosition.y - legHome.y)) * directionMultiplier;
    }
    byte farthest;
    for(byte i = 0; i < numLegs; i++){ 
      farthest = i;
      for(byte j = i; j < numLegs; j++){
        if(dist[j] > dist[farthest]){
          farthest = j;
        }
      }
      byte temp = dist[i];
      dist[i] = dist[farthest];
      dist[farthest] = temp;
      list[i] = farthest;
    }
  }

  bool legIsRedundant(uint8_t legn){ // returns true if requested leg is able to be lifted 
    vec3d footPositions[3];          // without toppling the robot i.e. if the CoM is bounded
    byte j = 0;                      // by the 3 remaining legs
    for(byte i = 0; i < numLegs; i++){
      if(i == legn)
        continue;
      footPositions[j] = leg[i].getFootPosWorld();
      j++;
    }
    
    return PointInTriangle(footPositions, vec3d::zero);
  }

  bool PointInTriangle(vec3d* verts, vec3d point){

  vec3d w[3];
  w[0] = verts[2] - verts[0];
  w[1] = verts[1] - verts[0];
  w[2] = point - verts[0];

  float dot[5];
  dot[0] = w[0].dot(w[0]);
  dot[1] = w[0].dot(w[1]);
  dot[2] = w[0].dot(w[2]);
  dot[3] = w[1].dot(w[1]);
  dot[4] = w[1].dot(w[2]);

  float a = 1 / (dot[0] * dot[3] - sq(dot[1]));
  float u = (dot[3] * dot[2] - dot[1] * dot[4]) * a;
  float v = (dot[0] * dot[4] - dot[1] * dot[2]) * a;

  return (u >= 0) && (v >= 0) && (v + u < 1);
    
  } ///// END TOOLS  /////


  void setServos(){
    for(byte i = 0; i < numLegs; i++) leg[i].setServos();
  }

  void input(vec3d v, vec3d r){ //handles joystick inputs
    
    if(waitForJoyReset){ //when switching modes, waits for joystick to return to zero before accepting new input
      if(v.isZero())
        waitForJoyReset = false;
      else{
        v = vec3d::zero;
        r = vec3d::zero;
      }
    }
    
    float spd = 4; //values for input drag
    float rotspd = 10;
    
    vec3d u = v; u.z = 0;
    smoothVecCubic(u, 0.25); //filters x and y inputs
    v.x = u.x, v.y = u.y;

    vec3d dv = v - moveDir;

    if(dv.mag() < spd * 0.02 || !smoothInput) //adds drag to input, for smoothing
      moveDir = v;
    else
      moveDir = moveDir + dv.unit() * spd * 0.02;

    smoothVecCubic(r, 0.25); //filters rotation inputs

    dv = r - eulerAngles;

    if(dv.mag() < rotspd * 0.02 || !smoothInput) //rotation drag
      eulerAngles = r;
    else
      eulerAngles = eulerAngles + dv.unit() * rotspd * 0.02;

  }

  void bounce(){
    confirmBounce = true;
    bounceTimer = millis();

    translate(vec3d::Z * -0.01);
    legHome.z += 0.01;
  }

  void smoothVecCubic(vec3d& v, float m){ //makes values closer to 50% more precise, m is slope of line at 50%
      float mag = v.mag();
      if(mag > 1) mag = 1;
      mag = 3*pow((mag - 0.5), 3) + 0.25 * mag + 0.375;
      v = v.unit() * mag;
  }

  void buttonInput(byte buttons){ //button events are transmitted as a single byte, this function interprets that byte

    //right button held
    if(buttons & (1 << 2)){ //activates and deactivates servo trim mode
      if(mode != legTrimmer){
        mode = legTrimmer;
      }
      else{
        servoTrims.save();
        mode = walking;
        walkMode = crawling;
        bounce();
      }
    }
    
    //left button held
    if(buttons & (1 << 3)){ //activates mode switch menu
      if(mode == legTrimmer) servoTrims.reset();
      else {
        mode = modeChange;
        bounce();
      }
    }

    //left button pressed
    if(buttons & (1 << 1)){ //context specific
      switch(mode){
        case walking:
          switch(walkMode){
            case crawling: walkMode = trotting; break;
            case trotting: walkMode = crawling; break;
          }
          break;
          
        case shifting:
          switch(shiftMode){
            case shiftTrack: shiftMode = shiftRate; break;
            case shiftRate: shiftMode = shiftTrack; break;
          }
          break;
      }
    }

    //right button pressed
    if(buttons & (1 << 0)){ //context specific
      switch(mode){
        case walking:
          switch(walkRightStick){
            case rideAdjust: walkRightStick = stepAdjust; break;
            case stepAdjust: walkRightStick = rideAdjust; break;
          }
          break;
          
        case shifting:
          switch(shiftRightStick){
            case yawAndHeight: shiftRightStick = pitchAndRoll; break;
            case pitchAndRoll: shiftRightStick = yawAndHeight; break;
          }
          break;

        case legTrimmer:
          servoTrims.legn = ++servoTrims.legn & 3;
          break;

        case interact:
          ++interSelectedLeg %= 4;
          break;
      }
    }
    
  }

  void homeLegs(){
    for(byte i = 0; i < numLegs; i++) leg[i].setFootPosLocal(legHome);
  }
  
};



#endif
