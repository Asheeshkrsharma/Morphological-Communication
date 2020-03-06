// Author: Asheesh Sharma
// Msc project
// University of Bristol
// A circular particle

class Particle {

  // We need to keep track of a Body and a radius
  Body body;
  float r;
  FlowField flowey;
  float maxforce=100;    // Maximum steering force
  float maxspeed=100;    // Maximum speed

  Particle(float x, float y, float r_, FlowField flowey_) {
    r = r_;
    flowey = flowey_;
    // This function puts the particle in the Box2d world
    makeBody(x,y,r);
  }

  // This function removes the particle from the box2d world
  void killBody() {
    box2d.destroyBody(body);
  }

  // Is the particle ready for deletion?
  boolean done() {
    // Let's find the screen position of the particle
    Vec2 pos = box2d.getBodyPixelCoord(body);
    // Is it off the bottom of the screen?
    if (pos.y > height+r*2) {
      killBody();
      return true;
    }
    return false;
  }

  // 
  void display() {
    follow(flowey);
    // We look at each body and get its screen position
    Vec2 pos = box2d.getBodyPixelCoord(body);
    // Get its angle of rotation
    float a = body.getAngle();
    pushMatrix();
    translate(pos.x,pos.y);
    rotate(-a);
    fill(175);
    stroke(0);
    strokeWeight(1);
    ellipse(0,0,r*2,r*2);
    // Let's add a line so we can see the rotation
    line(0,0,r,0);
    popMatrix();
  }

  // Here's our function that adds the particle to the Box2D world
  void makeBody(float x, float y, float r) {
    // Define a body
    BodyDef bd = new BodyDef();
    // Set its position
    bd.position = box2d.coordPixelsToWorld(x,y);
    bd.type = BodyType.DYNAMIC;
    body = box2d.world.createBody(bd);

    // Make the body's shape a circle
    CircleShape cs = new CircleShape();
    cs.m_radius = box2d.scalarPixelsToWorld(r);
    
    FixtureDef fd = new FixtureDef();
    fd.shape = cs;
    // Parameters that affect physics
    fd.density = 1;
    fd.friction = 0.01;
    fd.restitution = 0.3;
    
    //body.setFixedRotation(true);
    // Attach fixture to body
    body.createFixture(fd);

    // Give it a random initial velocity (and angular velocity)
    //body.setLinearVelocity(new Vec2(random(-10f,10f),random(5f,10f)));
    //body.setAngularVelocity(random(-10,10));
  }


  void follow(FlowField flow) {
     //Work out the force to apply to us based on the flow field grid squares we are on.
     //we apply bilinear interpolation on the 4 grid squares nearest to us to work out our force.
     // http://en.wikipedia.org/wiki/Bilinear_interpolation#Nonlinear
    
     Vec2 position = box2d.getBodyPixelCoord(body);
        
     //Get the desired direction vector and covert it to world coordinated
     Vec2 desiredDirection =  box2d.vectorPixelsToWorld(flow.lookup(position));
     
     //Multiply the direction by speed with a fixed speed.
     Vec2 desiredVelocity = desiredDirection.mul(maxspeed);

     //The velocity change we want
     Vec2 velocityChange = desiredVelocity.sub(body.getLinearVelocity());

     //Get the force
     Vec2 force = velocityChange.mul(maxforce / maxspeed);
    
     //Work out the reynold flocking behaviours
     Vec2 ff =  force; //Flowfield
     Vec2 sep = new Vec2(0,0); //Separation
     Vec2 alg = new Vec2(0,0); //Alignment
     Vec2 coh = new Vec2(0,0); //Cohesion

     //Calculate the desired force we need to apply
     Vec2 forceToApply = ff.add(sep.mul(2)).add(alg.mul(0.5)).add(coh.mul(0.2));
    
    //Make sure the force is never greater than the limit
    if (forceToApply.length() >  maxforce){
        forceToApply.normalize();
        forceToApply.mulLocal(maxforce);
    }
    
    //Before applying the force, we need to aign the body towards it.
    float phi = (float)Math.atan2(-desiredDirection.x,desiredDirection.y);
    //So lets apply it
    body.setFixedRotation(false);
    body.setTransform( body.getWorldCenter(), PI/2+phi );
    body.setFixedRotation(true);
    body.applyForceToCenter(forceToApply); //Apply the force
  }

}