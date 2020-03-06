// Author: Asheesh Sharma
// Msc project
// University of Bristol

// An uneven surface boundary

class Surface {
  // We'll keep track of all of the surface points
  ArrayList<Vec2> surfaceRight;
  ArrayList<Vec2> surfaceLeft;
  
  java.awt.Polygon psurfaceRight = new java.awt.Polygon(); // A java inbuilt method for a polygon. Essensitally its a copy of the surface.
  java.awt.Polygon psurfaceLeft = new java.awt.Polygon(); // A java inbuilt method for a polygon. Essensitally its a copy of the surface.


  Surface() {
    surfaceRight = new ArrayList<Vec2>();
    surfaceLeft = new ArrayList<Vec2>();

    // This is what box2d uses to put the surface in its world
    ChainShape chainRight = new ChainShape();
    ChainShape chainLeft = new ChainShape();

    float theta = 0;
    float xoff=0;
    float roughness = 0.1;
    float saperation = width/2*1.25;
    
    // This has to go backwards so that the objects  bounce off the top of the surface
    // This "edgechain" will only work in one direction!
    for (float y = height; y > 0; y -= 5) {

      // Doing some stuff with perlin noise to calculate a surface that points down on one side
      // and up on the other
      float xRight = map(cos(theta),-1,1,saperation,width) + map(noise(xoff),0,1,-20,20);
      float xLeft = map(cos(theta),1,-1,0,width-saperation)+ map(noise(xoff),0,1,-20,20);
      
      theta += 0.07;

      // Store the vertex in screen coordinates
      surfaceRight.add(new Vec2(xRight,y));
      psurfaceRight.addPoint((int)xRight,(int)y);
      // Store the vertex in screen coordinates
      surfaceLeft.add(new Vec2(xLeft,y));
      psurfaceLeft.addPoint((int)xLeft,(int)y);

      xoff += roughness;
    }
    
    
    // Build an array of vertices in Box2D coordinates
    // from the ArrayList we made
    Vec2[] verticesRight = new Vec2[surfaceRight.size()];
    for (int i = 0; i < verticesRight.length; i++) {
      Vec2 edge = box2d.coordPixelsToWorld(surfaceRight.get(i));
      verticesRight[i] = edge;
    }
    
    Vec2[] verticesLeft = new Vec2[surfaceLeft.size()];
    for (int i = 0; i < verticesLeft.length; i++) {
      Vec2 edge = box2d.coordPixelsToWorld(surfaceLeft.get(i));
      verticesLeft[i] = edge;
    }
    
    // Create the chain!
    chainRight.createChain(verticesRight,verticesRight.length);
    chainLeft.createChain(verticesLeft,verticesLeft.length);

    // The edge chain is now attached to a body via a fixture
    BodyDef bdRight = new BodyDef();
    BodyDef bdLeft = new BodyDef();
    bdRight.position.set(0.0f,0.0f);
    bdLeft.position.set(0.0f,0.0f);
    
    Body bodyRight = box2d.createBody(bdRight);
    Body bodyLeft = box2d.createBody(bdLeft);

    // Shortcut, we could define a fixture if we
    // want to specify frictions, restitution, etc.
    bodyRight.createFixture(chainRight,1);
    bodyLeft.createFixture(chainLeft,1);

  }

  // A simple function to just draw the edge chain as a series of vertex points
  void display() {
    strokeWeight(2);
    stroke(0);
    noFill();
    beginShape();
    for (Vec2 v: surfaceRight) {
      vertex(v.x,v.y);
    }
    endShape();
    
    beginShape();
    for (Vec2 v: surfaceLeft) {
      vertex(v.x,v.y);
    }
    endShape();
  }
  
  // checks if the point exists inside the surface. Used to remove flowfiled vectors whici lie inside
  boolean amIInside(int x, int y){
    return !(psurfaceRight.contains(x, y) || psurfaceLeft.contains(x, y));
  }
  
}