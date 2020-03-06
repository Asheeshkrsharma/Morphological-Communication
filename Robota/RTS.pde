// Author: Asheesh Sharma
// Msc project
// University of Bristol
class RTS {
  // Our object is two boxes and one joint
  // Consider making the fixed box much smaller and not drawing it
  RevoluteJoint revolutejoint1;
  RevoluteJoint revolutejoint2;

  PrismaticJoint prismaticjoint1;

  rtsElement rotationElement1;

  rtsElement distal;

  rtsElement proximal;

  rtsElement translationElement1;


  // Degree to Radian conversion constant. 
  float DEGTORAD=PI/180;
  float krotation;
  float ktranslation=0.5;
  float restinglength;
  RTS(float x, float y) {

    // Initialize locations of two boxes
    proximal = new rtsElement(x, y, 10, 10, false); 
    rotationElement1 = new rtsElement(x, y, 10, 10, false); 
    translationElement1 = new rtsElement(x, y+10, 10, 10, false); 
    distal = new rtsElement(x, y+10, 10, 10, false); 

    // Define joint as between two bodies
    revolutejoint1 = createrevolute(rotationElement1, proximal);
    prismaticjoint1=createprismatic(translationElement1, rotationElement1);
    revolutejoint2 = createrevolute(distal, translationElement1);
  }

  float display() {
    float pe = update() * 0.01;
    translationElement1.display();
    proximal.display();
    rotationElement1.display();
    distal.display();

    // Draw anchor just for debug
    Vec2 anchor = box2d.coordWorldToPixels(rotationElement1.body.getWorldCenter());
    Vec2 linep1 = anchor;
    fill(255, 0, 0);
    stroke(0);
    ellipse(anchor.x, anchor.y, 4, 4);

    // Draw anchor just for debug
    anchor = box2d.coordWorldToPixels(distal.body.getWorldCenter());
    Vec2 linep2 = anchor;
    float widthline=7-sqrt(pow(linep1.x-linep2.x, 2)+pow(linep1.y-linep2.y, 2))*0.05;
    strokeWeight(abs(widthline));
    line(linep1.x, linep1.y, linep2.x, linep2.y);
    strokeWeight(1);
    
    fill(255, 0, 0);
    stroke(0);
    ellipse(anchor.x, anchor.y, 4, 4);
    return pe;
  }

  float update() {
    float rpe1 = updaterevolute(revolutejoint2);
    float pe1 = updateprismatic(translationElement1, rotationElement1, prismaticjoint1);
    float rpe2 = updaterevolute(revolutejoint1);
    return rpe1 + pe1 + rpe2;
  }

  float updaterevolute(RevoluteJoint revolutejointie_) {
    //Calculating the dynamics.  
    // 1. Get the current angle of bodies  
    krotation=cntrkrvalue*100000;

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
    return (0.5 * krotation * pow((float)angle1 / 180, 2));
  }

  float updateprismatic(rtsElement a_, rtsElement b_, PrismaticJoint prismaticjointe_) {
    ktranslation=2*cntrkvalue;

    Vec2 world2v = a_.body.m_linearVelocity;
    Vec2 world1v = b_.body.m_linearVelocity;    
    Vec2 vdiff = world2v.sub(world1v);

    float  pjt = prismaticjointe_.getJointTranslation();
    float pjs = prismaticjointe_.getJointSpeed();

    float force = ktranslation*(pjt-restinglength) + cntrdamping * pjs; // ktranslation is the spring constant, 5 is the damping constant.
    //Note the the desired ditance is devided by two because the entire spring has been devided by 2.
    prismaticjointe_.setMaxMotorForce(force); 
    prismaticjointe_.setMotorSpeed(-prismaticjointe_.getJointTranslation()*vdiff.normalize());
    return (0.5 * ktranslation * 5 * pow(prismaticjointe_.getJointTranslation(), 2));
  }

  PrismaticJoint createprismatic(rtsElement a_, rtsElement b_) {
    PrismaticJointDef tjd= new PrismaticJointDef();
    tjd.initialize(a_.body, b_.body, b_.body.getWorldCenter(), new Vec2(0, 1));
    tjd.enableMotor = true;
    return (PrismaticJoint) box2d.world.createJoint(tjd);
  }

  RevoluteJoint createrevolute(rtsElement a_, rtsElement b_) {
    // Define joint as between two bodies
    RevoluteJointDef rjd = new RevoluteJointDef();
    rjd.bodyA=a_.body;
    rjd.bodyB=b_.body;

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

  WeldJoint createWeld(rtsElement BoxA, rtsElement BoxB) {
    WeldJointDef welddef = new WeldJointDef();
    welddef.initialize(BoxA.body, BoxB.body, BoxA.body.getWorldCenter());
    welddef.frequencyHz=0;
    welddef.dampingRatio=1;
    return (WeldJoint) box2d.world.createJoint(welddef);
  }

  //Utilities
  float[] translat(float[] line, float x, float y) {
    line[0]=line[0]+x;
    line[1]=line[1]+y;
    line[2]=line[2]+x;
    line[3]=line[3]+y;
    return line;
  }
  Vec2 rotate(Vec2 point, Vec2 center, float angle) {
    float x1 = point.x - center.x;
    float y1 = point.y - center.y;
    float x2 = x1 * cos(angle) - y1 * sin(angle);
    float y2 = x1 * sin(angle) + y1 * cos(angle);
    point.x = x2 + center.x;
    point.y = y2 + center.y;
    return point;
  }
}