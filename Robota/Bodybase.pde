// Author: Asheesh Sharma
// Msc project
// University of Bristol

class Bodybase {
  float bodybaserotation;

  FlowField flowey; //Every robot experiences the Flow field

  float maxforce=300; // Maximum steering force
  float maxspeed=300; // Maximum speed

  boolean water=true; //Controls if the robot experiences the flow field

  Body base; //The base where all instrumentation lies 
  Body top; //This is the spining part which houses the gearbox
  Body topconnectionDistal; // This is where we will connect the spring

  float RTSintruptdelay; // How long the RTS is set to zero resting length

  Whiskerhub whiskers; //This are the sensors. This is part of the body. 
  //But defined in a saperate class so that we only listen to the collision events related to this

  public RTS rts; //This is the rts object for the spring.

  //Revolute joints.
  RevoluteJoint base2whisker;
  RevoluteJoint base2top;
  RevoluteJoint topconnection2rts;
  WeldJoint top2topconnectionweldProximal;
  WeldJoint top2topconnectionweldDistal;
  RevoluteJoint connectionDistal;

  //The mighty degree to rad conversion constant 
  float DEGTORAD=PI/180;  
  float w; //Width: These are the internal parameters do not touch
  float h; //Height: These are the internal parameters do not touch
  float r; // Radius of everything: These are the internal parameters do not touch
  color colour; //To visually display if the robot collided

  //Experimental, a radial joint at the base2top conection joint
  float krotation=19000; //Radial spring contant


  // Constructor for the entire robot.
  Bodybase(float x, float y, FlowField flowey_, float atAngle) {
    // Arguments: [x,y] position of the robot 
    w = 8; // Width: do not change
    h = 24; // Height: do not change
    r = 8; // radius: do not change
    bodybaserotation = atAngle; //The angle of the robot at initialisation
    flowey = flowey_;

    // Add the box to the box2d world
    makeBody(new Vec2(x, y)); //Function which contains all the internal routines to create the object.
    colour=color(175); //Assume the the robot has not collided when initialising.
  }

  // This function adds the rectangle to the box2d world
  void makeBody(Vec2 center) {

    // 1. Define the base and make it from the shape
    BodyDef basebody = new BodyDef(); //Definition
    basebody.type = BodyType.DYNAMIC; //The base is dynamic
    Vec2 c = box2d.coordPixelsToWorld(center);
    // 1.a We create the base at the prescribes coordinates
    basebody.position.set(c);
    //basebody.fixedRotation = true;
    base = box2d.createBody(basebody);
    //base.setFixedRotation(true);
    // 1.b Next we need to attach a shape to the body
    PolygonShape baseshape = new PolygonShape(); // A polygon shape (Rectangle)
    float box2dW = box2d.scalarPixelsToWorld(r);
    float box2dH = box2d.scalarPixelsToWorld(h/2);


    baseshape.setAsBox(box2dW, box2dH);

    // 1.c attach it to the base. Its density is 1.
    base.createFixture(baseshape, 1.0);

    // 1.d finally set the roation
    base.setTransform(base.getWorldCenter(), bodybaserotation);
    // 1.d Give it some initial random velocity
    base.setLinearVelocity(new Vec2(random(-5, 5), random(2, 5)));
    base.setAngularVelocity(random(-5, 5));

    //-----------------------------------------------------------------------------------------

    // 2. Create a connection b/w the rest of the body and the whiskers
    whiskers = new Whiskerhub(center.x, center.y-10); //This is where the whiskerbody is created.

    // 1.d finally set the roation
    rotat(whiskers.whiskay.getWorldCenter(), base.getWorldCenter(), bodybaserotation); //Ye kya chutiyapa h?
    whiskers.whiskay.setTransform(whiskers.whiskay.getWorldCenter(), bodybaserotation);
    //-----------------------------------------------------------------------------------------


    // 3. We joint the whisker to the base with a revolute joint.
    // Why revolute joint? it is a good enough way to simulate the whiskers actually moving
    // around.

    // 3.a Define joint as between base and the whiskers
    RevoluteJointDef basetowhisker = new RevoluteJointDef();

    basetowhisker.initialize(whiskers.whiskay, base, whiskers.whiskay.getWorldCenter());

    // 3.b We need to define some motor properties here. Essentially
    basetowhisker.enableMotor = false;      // is it on?
    basetowhisker.enableLimit=true; //enable limits
    basetowhisker.lowerAngle = (float) -10 * DEGTORAD;
    basetowhisker.upperAngle = (float) 10 * DEGTORAD;

    // 3.c Create the joint and make an object out of it which can be manipulated.
    base2whisker = (RevoluteJoint) box2d.world.createJoint(basetowhisker);

    //-----------------------------------------------------------------------------------------


    // 4. Define the top and make it from the shape
    BodyDef topbody = new BodyDef(); // Definition
    topbody.type = BodyType.DYNAMIC; // The top is dynamic.
    topbody.position.set(box2d.coordPixelsToWorld(new Vec2 (center.x, center.y+10))); //Create 
    //it at a prescribed coordintate   

    top = box2d.createBody(topbody);

    // 4.a Create a shape which will be attached to the topbody
    PolygonShape topshape = new PolygonShape(); // A polygon (rectangle)

    // 4.b Make the rectangle
    box2dW = box2d.scalarPixelsToWorld(r/2); //Shape width 
    box2dH = box2d.scalarPixelsToWorld(h/2); //Shape width  
    topshape.setAsBox(box2dW, box2dH);

    // 4.c The top does not collide with the base as it is above the base in the real world. 
    // So we need this fixture to be filtered out.
    FixtureDef topfixture = new FixtureDef(); // make a fixture
    topfixture.shape = topshape; // Its shape is the the topshape
    topfixture.density =1; // Its denisity is 1.
    topfixture.filter.groupIndex = 2; //This is the group index which represents all the 
    //bodies which do not collide 

    topfixture.filter.categoryBits = 1; //This is the category i.e how we recognise which 
    //type of shape it is

    topfixture.filter.maskBits = (short)0; //This is the mask. I dont really what this is. :P
    top.createFixture(topfixture); //Attach the shape to the top   

    rotat(top.getWorldCenter(), base.getWorldCenter(), bodybaserotation); //Ye kya chutiyapa h?
    top.setTransform(top.getWorldCenter(), bodybaserotation);


    //4.d The top also has a distal fixture which will be used to connect with other robots
    BodyDef topDistalbody = new BodyDef(); // Definition
    topDistalbody.type = BodyType.DYNAMIC; // The top is dynamic.
    topDistalbody.position.set(box2d.coordPixelsToWorld(new Vec2 (center.x-10, center.y))); //Create 
    //it at a prescribed coordintate   
    topconnectionDistal= box2d.createBody(topDistalbody);

    PolygonShape topDistalshape = new PolygonShape(); // A polygon (rectangle)

    // 4.e Make the rectangle
    box2dW = box2d.scalarPixelsToWorld(h/8); //Shape width 
    box2dH = box2d.scalarPixelsToWorld(r/2); //Shape width  
    topDistalshape.setAsBox(box2dW, box2dH);


    // 4.f The top does not collide with the base as it is above the base in the real world. 
    // So we need this fixture to be filtered out.
    FixtureDef topfixtureDistal = new FixtureDef(); // make a fixture
    topfixtureDistal.shape = topDistalshape; // Its shape is the the topshape
    topfixtureDistal.density =1; // Its denisity is 1.
    topfixtureDistal.filter.groupIndex = 2; //This is the group index which represents all the 
    //bodies which do not collide 

    topfixtureDistal.filter.categoryBits = 1; //This is the category i.e how we recognise which 
    //type of shape it is

    topfixtureDistal.filter.maskBits = (short)0; //This is the mask. I dont really what this is. :P


    topconnectionDistal.createFixture(topfixtureDistal); //Attach the shape to the top

    rotat(topconnectionDistal.getWorldCenter(), base.getWorldCenter(), bodybaserotation); //Ye kya chutiyapa h?
    topconnectionDistal.setTransform(topconnectionDistal.getWorldCenter(), bodybaserotation);



    // 4.g This body connects with the top with a weld joint
    WeldJointDef toptotopconnectionweldDistal = new WeldJointDef();
    toptotopconnectionweldDistal.initialize(top, topconnectionDistal, top.getWorldCenter());
    toptotopconnectionweldDistal.frequencyHz=0;
    toptotopconnectionweldDistal.dampingRatio=0;
    top2topconnectionweldDistal = (WeldJoint) box2d.world.createJoint(toptotopconnectionweldDistal);  


    //-----------------------------------------------------------------------------------------


    // 5. The top moves freely in ralation with the base. There is revolute joint b/w the
    // top and rest of the body.
    RevoluteJointDef basetotop = new RevoluteJointDef();

    // 5.a Initialise the joint and set motor properties.
    basetotop.initialize(base, top, base.getWorldCenter());
    basetotop.enableMotor = true;      // is it on?

    // 5.b The motors must have upper and lower limits. This directly relates
    // How much the stiffness of the spring in the y direction (normal to the)
    // axis of the motor.
    basetotop.enableLimit=true; //enable limits
    basetotop.lowerAngle = (float) -30 * DEGTORAD;
    basetotop.upperAngle = (float) 30 * DEGTORAD;

    // 5.b create the joint
    base2top = (RevoluteJoint) box2d.world.createJoint(basetotop);

    //-----------------------------------------------------------------------------------------


    // 6. Finally we make a connection which will connect the entire body with a non-liner spring.
    // To do this we will create a body which attached to the top with a revolute joint.      
    rts = new RTS(center.x, center.y+20);

    rotat(rts.rotationElement1.body.getWorldCenter(), base.getWorldCenter(), bodybaserotation); //Ye kya chutiyapa h?
    rts.rotationElement1.body.setTransform(rts.rotationElement1.body.getWorldCenter(), bodybaserotation);

    rotat(rts.distal.body.getWorldCenter(), base.getWorldCenter(), bodybaserotation); //Ye kya chutiyapa h?
    rts.distal.body.setTransform(rts.distal.body.getWorldCenter(), bodybaserotation);

    rotat(rts.proximal.body.getWorldCenter(), base.getWorldCenter(), bodybaserotation); //Ye kya chutiyapa h?
    rts.proximal.body.setTransform(rts.proximal.body.getWorldCenter(), bodybaserotation);

    rotat(rts.translationElement1.body.getWorldCenter(), base.getWorldCenter(), bodybaserotation); //Ye kya chutiyapa h?
    rts.translationElement1.body.setTransform(rts.translationElement1.body.getWorldCenter(), bodybaserotation);





    WeldJointDef toptotopconnectionweld = new WeldJointDef();
    toptotopconnectionweld.initialize(top, rts.proximal.body, top.getWorldCenter());
    toptotopconnectionweld.frequencyHz=0;
    toptotopconnectionweld.dampingRatio=0;
    top2topconnectionweldProximal = (WeldJoint) box2d.world.createJoint(toptotopconnectionweld);
  }

  // This function removes the particle from the box2d world
  void killBody() {
    box2d.destroyBody(base);
    box2d.destroyBody(whiskers.whiskay);
    box2d.destroyBody(top);
    box2d.destroyBody(topconnectionDistal);
    box2d.destroyBody(rts.rotationElement1.body);
    box2d.destroyBody(rts.distal.body);
    box2d.destroyBody(rts.proximal.body);
    box2d.destroyBody(rts.translationElement1.body);
  }

  // Is the robot ready for deletion?
  boolean done() {
    // Let's find the screen position of the particle
    Vec2 pos = box2d.getBodyPixelCoord(base);
    // Is it off the bottom of the screen?
    if (pos.y < 0) {
      killBody();
      return true;
    }
    return false;
  }

  // Drawing the robot
  float display() {
    listen();
    whiskers.display(); //Good thing is the whisker class has its own display method
    //But we need to draw everything else

    // 1. Draw the base
    // 1.a We look at each base and get its screen position
    Vec2 pos = box2d.getBodyPixelCoord(base);// Get its angle of rotation 
    float a = base.getAngle(); // Get its angle in the world

    // 1.b Lets draw
    rectMode(CENTER); //Put the screen mode the center. We rotate and translate in relation
    // with the center;

    pushMatrix();
    translate(pos.x, pos.y);
    rotate(-a);
    fill(colour);
    stroke(0);
    rect(0, 0, 2*r, h);
    popMatrix();

    // 2. Draw the top

    rectMode(CENTER);
    pos = box2d.getBodyPixelCoord(top);
    // Get its angle of rotation
    a = top.getAngle();
    pushMatrix();
    translate(pos.x, pos.y);
    rotate(-a);
    fill(175);
    stroke(0);
    rect(0, 0, r, h);
    popMatrix();


    // 3. Draw the top connection
    rectMode(CENTER);
    pos = box2d.getBodyPixelCoord(topconnectionDistal);
    // Get its angle of rotation
    a = topconnectionDistal.getAngle();
    pushMatrix();
    translate(pos.x, pos.y);
    rotate(-a);
    fill(175);
    stroke(0);
    rect(0, 0, h/4, r);
    popMatrix();    

    float pe = rts.display();
    return pe;
  }

  //This function listens to the Whisker and manipulate the spring.
  void listen() {
    if (whiskers.haveCollided==true) {
      colour = color(52, 73, 94);
      rts.restinglength=0; 
      //Wait for some time frames?
      RTSintruptdelay+=abs(sin(frameCount/100));
      if (RTSintruptdelay>=cntrRTSdelayintr) {
        RTSintruptdelay=0;
        whiskers.haveCollided=false;
        whiskers.colour=color(175);
        rts.restinglength=cntrRestinglen;
        colour = color(175);
      }
    } else
    {
      colour = color(175);
      rts.restinglength=cntrRestinglen;
    }
  }

  void follow() {
    //Work out the force to apply to us based on the flow field grid squares we are on.
    //we apply bilinear interpolation on the 4 grid squares nearest to us to work out our force.
    // http://en.wikipedia.org/wiki/Bilinear_interpolation#Nonlinear

    Vec2 position = box2d.getBodyPixelCoord(base);

    //Get the desired direction vector and covert it to world coordinated    
    Vec2 desiredDirection =  box2d.vectorPixelsToWorld(flowey.lookup(position));

    //Multiply the direction by speed with a fixed speed.
    Vec2 desiredVelocity = desiredDirection.mul(maxspeed);

    //The velocity change we want
    Vec2 velocityChange = desiredVelocity.sub(base.getLinearVelocity());

    //Get the force
    Vec2 force = velocityChange.mul(maxforce / maxspeed);

    //Work out the reynold flocking behaviours
    Vec2 ff =  force; //Flowfield
    Vec2 sep = new Vec2(0, 0); //Separation
    Vec2 alg = new Vec2(0, 0); //Alignment
    Vec2 coh = new Vec2(0, 0); //Cohesion

    //Calculate the desired force we need to apply
    Vec2 forceToApply = ff.add(sep.mul(2)).add(alg.mul(0.5)).add(coh.mul(0.2));

    //Make sure the force is never greater than the limit
    if (forceToApply.length() >  maxforce) {
      forceToApply.normalize();
      forceToApply.mulLocal(maxforce);
    }
    //Before applying the force, we must rotate the body towards the desiredDirection    float phi = (float)Math.atan2(-desiredDirection.x,desiredDirection.y);
    float phi = (float)Math.atan2(-desiredDirection.x, desiredDirection.y);

    //Problem is that applying this angle directly to the body results in a very jittery
    //motion. One way to deal with this is to apply a intantaneous spin

    //First we can calculate how much the angle overshoot.
    //The overshoot depends on the rate of change of angularvelocity.
    //Assuming the box2d world refreshrate is 60hz.
    float overShoot = base.getAngularVelocity()/10;
    //Thus the next angle will be...
    float nextAngle = base.getAngle()+overShoot;
    //So the toalRotation will be
    float totalRotation = phi - nextAngle;
    //limit this b/w 180 and -180
    while ( totalRotation < -180 * DEGTORAD ) totalRotation += 360 * DEGTORAD;
    while ( totalRotation >  180 * DEGTORAD ) totalRotation -= 360 * DEGTORAD;
    // Therefore the desired angular velocity can be given as 
    float desiredAngularVelocity = totalRotation * 10;
    // finally the torque we need to apply on the body 
    // can calculated using the formula T = Iv/t where T is the torque 
    // we want to know, I is the rotational inertia of the body, v 
    // is the rotational velocity and t is the time we will apply the torque, as before:

    float torque = base.getInertia() * desiredAngularVelocity / (1/10.0);
    base.applyTorque(torque);
    base.applyForceToCenter(forceToApply); //Apply the force
  }

  //Experimental, a radial spring at the topbaseconnection joint
  void updaterevolute(RevoluteJoint revolutejointie_) {
    //Calculating the dynamics.  
    // 1. Get the current angle of bodies  
    krotation=cntrkrvalue*100000;

    double angle1=revolutejointie_.getJointAngle() * (180/PI);

    // 2. Calculate torques for the motors. The relation is that the torque must be 
    // increased as the angle increases.
    float T1 = krotation * abs((float)angle1/40);

    // 3. Calculate angular speed, motors should move in the opposite direction w.r.t
    // the tilt.
    float V1=(float)-angle1/(30);

    // 4. Update the the internal body joints    
    if (abs((float)angle1) > 0.2) {
      revolutejointie_.setMaxMotorTorque(T1);
      revolutejointie_.setMotorSpeed(V1);
      revolutejointie_.enableMotor(true);
    } else {
      revolutejointie_.enableMotor(false);
    }
  }

  void update() {
    updaterevolute(base2whisker);
    if (water) {
      follow();
    }
  }

  void connect(Body RTSdistal) {
    RevoluteJointDef connection = new RevoluteJointDef();
    connection.initialize(topconnectionDistal, RTSdistal, topconnectionDistal.getWorldCenter());
    connection.localAnchorA = new Vec2(0, 0);
    connection.localAnchorB = new Vec2(0, 0);
    connectionDistal = (RevoluteJoint) box2d.world.createJoint(connection);
  }
  void rotat(Vec2 point, Vec2 center, float angle)
  {
    float x1 = point.x - center.x;
    float y1 = point.y - center.y;

    float x2 = (x1 * cos(angle)) - (y1 * sin(angle));
    float y2 = (x1 * sin(angle)) + (y1 * cos(angle));

    point.x = x2 + center.x;
    point.y = y2 + center.y;
  }
}