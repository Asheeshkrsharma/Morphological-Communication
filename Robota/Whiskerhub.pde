// Author: Asheesh Sharma
// Msc project
// University of Bristol

//This class defines a whisker which is what we moniter for collisions.

class Whiskerhub {

  // We need to keep track of a Body and a width and height
  Body whiskay; //Whisky seems appropriate though.

  RevoluteJoint joint;
  RevoluteJoint joint2; 
  float DEGTORAD=PI/180;  
  float w;
  float h;
  float r;
  color colour;
  boolean haveCollided;
  // Constructor
  Whiskerhub(float x, float y) {
    w = 8; //internal parameters, do not change
    h = 24; //internal parameters, do not change
    r = 8;//internal parameters, do not change

    colour = color(175); //This property is shared with the ContactLisner
    //so that we can change the state of whisker

    haveCollided = false; //This property is also shared with the ContactLisner
    //so that we can change the state of whisker

    // Add the box to the box2d world
    makeBody(new Vec2(x, y));
    whiskay.setUserData(this); //We need to access the whisker from contactlistener
  }


  void makeBody(Vec2 center) {
    // Define a polygon
    PolygonShape sd = new PolygonShape();
    Vec2 vertices[] = new Vec2[]{
      new Vec2(0.5714, 0), 
      new Vec2(0.4623, 0.3359), 
      new Vec2(0.1766, 0.5435), 
      new Vec2(-0.1766, 0.5435), 
      new Vec2(-0.4623, 0.3359), 
      new Vec2(-0.5714, 0.0000), 
      new Vec2(0.5714, 0)
    };

    sd.set(vertices, vertices.length);

    FixtureDef fd = new FixtureDef();
    fd.shape = sd;
    fd.density =1;
    fd.filter.groupIndex = 1;

    // Define the body and make it from the shape
    BodyDef bd = new BodyDef();
    bd.type = BodyType.DYNAMIC;
    bd.position.set(box2d.coordPixelsToWorld(center));
    whiskay = box2d.createBody(bd);

    whiskay.createFixture(fd);
  }


  // This function removes the particle from the box2d world
  void killBody() {
    box2d.destroyBody(whiskay);
  }

  // Is the particle ready for deletion?
  boolean done() {
    // Let's find the screen position of the particle
    Vec2 pos = box2d.getBodyPixelCoord(whiskay);

    // Is it off the bottom of the screen?
    if (pos.y > height+w*h) {
      killBody();
      return true;
    }
    return false;
  }

  // Drawing the lollipop
  void display() {
    // We look at each body and get its screen position
    Vec2 pos = box2d.getBodyPixelCoord(whiskay);
    // Get its angle of rotation
    float a = whiskay.getAngle();

    Fixture f = whiskay.getFixtureList();
    PolygonShape ps = (PolygonShape) f.getShape();

    rectMode(CENTER);
    pushMatrix();
    translate(pos.x, pos.y);
    rotate(-a);
    fill(colour);
    stroke(0);
    beginShape();
    //println(vertices.length);
    // For every vertex, convert to pixel vector
    for (int i = 0; i < ps.getVertexCount(); i++) {
      Vec2 v = box2d.vectorWorldToPixels(ps.getVertex(i));
      vertex(v.x, v.y);
    }
    endShape(CLOSE);
    popMatrix();
  }
}