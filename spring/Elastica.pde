// Author: Asheesh Sharma
// Msc project
// University of Bristol

// Class to describe a fixed spinning object

class Elastica {
  // Our object is two boxes and one joint
  // Consider making the fixed box much smaller and not drawing it
  RevoluteJoint revolutejoint1;
  RevoluteJoint revolutejoint2;
  WeldJoint weldjoint3;
  PrismaticJoint prismaticjoint1;
  PrismaticJoint prismaticjoint2;
  Box rotationElement1;
  Box rotationElement2;
  Box dummy;
  Box dummy2;
  Box translationElement1;
  Box translationElement2;
  // Degree to Radian conversion constant. 
  float DEGTORAD=PI/180;
  float krotation=19000;
  float ktranslation=0.5; //min is four //mac is 30
  
  Elastica(float x, float y) {

    // Initialize locations of two boxes
    dummy = new Box(x, y, 10, 10, true);
    dummy2 = new Box(x-10, y, 10, 5, false); 
    rotationElement1 = new Box(x, y, 10, 10, false); 
    translationElement1 = new Box(x, y-40, 10, 10, false); 
    rotationElement2 = new Box(x, y-40, 10, 10, false); 
    translationElement2 = new Box(x, y-80, 10, 10, false); 
    
    // Define joint as between two bodies
    revolutejoint1 = createrevolute(rotationElement1, dummy, new Vec2(0,0), new Vec2(0,0));
    prismaticjoint1=createprismatic(translationElement1, rotationElement1);
    revolutejoint2 = createrevolute(rotationElement2, translationElement1, new Vec2(0,0), new Vec2(0,0));
    prismaticjoint2=createprismatic(translationElement2, rotationElement2);
    weldjoint3=creatWeld(dummy, dummy2);
  }

  void display() {
    update();
    translationElement1.display();
    dummy2.display();
    dummy.display();
    /*rotationElement1.display();
    rotationElement2.display();*/
    translationElement2.display();
    
    // Draw anchor just for debug
    Vec2 anchor = box2d.coordWorldToPixels(rotationElement1.body.getWorldCenter());
    fill(255, 0, 0);
    stroke(0);
    ellipse(anchor.x, anchor.y, 4, 4);
    
     // Draw anchor just for debug
    anchor = box2d.coordWorldToPixels(rotationElement2.body.getWorldCenter());
    fill(255, 0, 0);
    stroke(0);
    ellipse(anchor.x, anchor.y, 4, 4);
        
  }
    
  void update(){
    updateprismatic(translationElement2, rotationElement2,prismaticjoint2);
    updaterevolute(revolutejoint2);
    updateprismatic(translationElement1, rotationElement1,prismaticjoint1);
    updaterevolute(revolutejoint1);
  }
 
 void updaterevolute(RevoluteJoint revolutejointie_){
       //Calculating the dynamics.  
    // 1. Get the current angle of bodies  
    double angle1=revolutejointie_.getJointAngle() * (180/PI);
    
    // 2. Calculate torques for the motors. The relation is that the torque must be 
    // increased as the angle increases.
    float T1 = krotation * abs((float)angle1/100);
    
    // 3. Calculate angular speed, motors should move in the opposite direction w.r.t
    // the tilt.
    float V1=(float)-angle1/(15);
    
    // 4. Update the the internal body joints    
    if (abs((float)angle1) > 0.2) {
      revolutejointie_.setMaxMotorTorque(T1);
      revolutejointie_.setMotorSpeed(V1);
      revolutejointie_.enableMotor(true);
    } else {
      revolutejointie_.enableMotor(false);
    }
 }
 
 void updateprismatic(Box a_, Box b_,PrismaticJoint prismaticjointe_){   
    Vec2 world2v = a_.body.m_linearVelocity;
    Vec2 world1v = b_.body.m_linearVelocity;
    Vec2 vdiff = world2v.sub(world1v);

    float  pjt = prismaticjointe_.getJointTranslation();
    float pjs = prismaticjointe_.getJointSpeed();
    float force = ktranslation*5*(pjt+100) +5* pjs; // ktranslation is the spring constant, 5 is the damping constant.
    //Note the the desired ditance is devided by two because the entire spring has been devided by 2.
    prismaticjointe_.setMaxMotorForce(force); 
    prismaticjointe_.setMotorSpeed(-prismaticjointe_.getJointTranslation()*vdiff.normalize());
 }
 
  PrismaticJoint createprismatic(Box a_, Box b_){
    PrismaticJointDef tjd= new PrismaticJointDef();
    tjd.initialize(a_.body,b_.body,b_.body.getWorldCenter(),new Vec2(0,1));
    //tjd.upperTranslation = 0.0f;
    //tjd.lowerTranslation = 3f;
    //tjd.enableLimit = true;
    tjd.enableMotor = true;
    return (PrismaticJoint) box2d.world.createJoint(tjd); 
  }

  RevoluteJoint createrevolute(Box a_, Box b_, Vec2 anchorA, Vec2 anchorB){
    // Define joint as between two bodies
    RevoluteJointDef rjd = new RevoluteJointDef();
    rjd.bodyA=a_.body;
    rjd.bodyB=b_.body;
    
    if (anchorA!=new Vec2(0,0) && anchorB != new Vec2(0,0)){
      rjd.localAnchorA = anchorA;
      rjd.localAnchorB = anchorB;
    }
    // Turning on a motor (optional)
    rjd.enableMotor = false;      // is it on?
    rjd.motorSpeed = 0;       // how fast?
    rjd.maxMotorTorque = 0; // how powerful?

   // 5. The motors must have upper and lower limits. This directly relates
    // How much the stiffness of the spring in the y direction (normal to the)
    // axis of the motor.
    rjd.enableLimit=true; //enable limits
    rjd.lowerAngle = (float) -90 * DEGTORAD;
    rjd.upperAngle = (float) 90 * DEGTORAD;

    // There are many other properties you can set for a Revolute joint
    // For example, you can limit its angle between a minimum and a maximum
    // See box2d manual for more
      // Create the joint
    return (RevoluteJoint) box2d.world.createJoint(rjd);

  }

 WeldJoint creatWeld(Box BoxA, Box BoxB){
    WeldJointDef welddef = new WeldJointDef();
    welddef.initialize(BoxA.body,BoxB.body,BoxA.body.getWorldCenter());
    welddef.frequencyHz=0;
    welddef.dampingRatio=1;
    return (WeldJoint) box2d.world.createJoint(welddef);
 }
 
//Utilities
float[] translat(float[] line, float x, float y){
  line[0]=line[0]+x;
  line[1]=line[1]+y;
  line[2]=line[2]+x;
  line[3]=line[3]+y;
  return line;
}
Vec2 rotate(Vec2 point, Vec2 center, float angle){
  float x1 = point.x - center.x;
  float y1 = point.y - center.y;
  float x2 = x1 * cos(angle) - y1 * sin(angle);
  float y2 = x1 * sin(angle) + y1 * cos(angle);
  point.x = x2 + center.x;
  point.y = y2 + center.y;
  return point;
}

 //Update the spring behaviour
float springupdate(){
      float currentDistance = 10*cos(frameCount*0.05); //Calculate the rest length of the spring
      //return currentDistance;
      return currentDistance;
}
}