// Author: Asheesh Sharma
// Msc project
// University of Bristol

// An uneven surface boundary

class Surface {
  // We'll keep track of all of the surface points
  ArrayList<Vec2> surfaceRight; //Right surface
  ArrayList<Vec2> surfaceLeft; //Left surface

  java.awt.Polygon psurfaceRight = new java.awt.Polygon(); // A java inbuilt method for a polygon. Essensitally its a copy of the surface.
  java.awt.Polygon psurfaceLeft = new java.awt.Polygon(); // A java inbuilt method for a polygon. Essensitally its a copy of the surface.


  Surface() {
    surfaceRight = new ArrayList<Vec2>(); //initialise
    surfaceLeft = new ArrayList<Vec2>(); //initialise

    // This is what box2d uses to put the surface in its world
    ChainShape chainRight = new ChainShape(); //box2d shape
    ChainShape chainLeft = new ChainShape(); //box2d shape

    float theta = 0; //For the cosine wave
    float xoff=0; //Perline noise parameter
    float roughness = 0.2; //Roughness of the terrain
    float saperation = width/2*1.09; //Saperation b/w the surfaces

    // This has to go backwards so that the objects  bounce off the top of the surface
    // This "edgechain" will only work in one direction!
    for (float y = height; y > 0; y -= 5) {

      float xRight = map(cos(theta), -1, 1, saperation, width) + map(noise(xoff), 0, 1, -20, 20);
      float xLeft = map(cos(theta), 1, -1, 0, width-saperation)+ map(noise(xoff), 0, 1, -20, 20);

      theta += 0.07;

      // Store the vertex in screen coordinates
      surfaceRight.add(new Vec2(xRight, y));
      psurfaceRight.addPoint((int)xRight, (int)y);
      // Store the vertex in screen coordinates
      surfaceLeft.add(new Vec2(xLeft, y));
      psurfaceLeft.addPoint((int)xLeft, (int)y);

      xoff += roughness;
    }

    //Complete the geometry;
    surfaceRight.add(new Vec2(width, 5));
    psurfaceRight.addPoint(width, 5);

    surfaceLeft.add(new Vec2(0, 5));
    psurfaceLeft.addPoint(0, 5);


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
    chainRight.createChain(verticesRight, verticesRight.length);
    chainLeft.createChain(verticesLeft, verticesLeft.length);

    // The edge chain is now attached to a body via a fixture
    BodyDef bdRight = new BodyDef();
    BodyDef bdLeft = new BodyDef();
    bdRight.position.set(0.0f, 0.0f);
    bdLeft.position.set(0.0f, 0.0f);

    Body bodyRight = box2d.createBody(bdRight);
    Body bodyLeft = box2d.createBody(bdLeft);

    // Shortcut, we could define a fixture if we
    // want to specify frictions, restitution, etc.
    bodyRight.createFixture(chainRight, 1);
    bodyLeft.createFixture(chainLeft, 1);
  }

  // A simple function to just draw the edge chain as a series of vertex points
  void display() {

    strokeWeight(2);
    stroke(0);
    beginShape();
    for (Vec2 v : surfaceRight) {
      vertex(v.x, v.y);
    }
    endShape();

    beginShape();
    for (Vec2 v : surfaceLeft) {
      vertex(v.x, v.y);
    }
    endShape();
  }

  // checks if the point exists inside the surface. Used to remove flowfiled vectors whici lie inside
  boolean amIInside(int x, int y) {
    return !(psurfaceRight.contains(x, y) || psurfaceLeft.contains(x, y));
  }

  // checks if the point exists inside the surface. Used to remove flowfiled vectors whici lie inside
  boolean amIIntersecting(int x, int y, int w, int h) {
    return !(psurfaceRight.intersects(x, y, w, h) || psurfaceLeft.intersects(x, y, w, h));
  }



  Vec2 getVectorCorrected(Vec2 given, float resolution, int i, int j, int scalefactor) {
    Vec2 result = new Vec2(0, 0);
    Vec2 input1 = new Vec2 ((int)given.x+i*resolution, (int)given.y+j*resolution);
    Vec2 input2 = new Vec2 ((int)(given.x+i*resolution)-(resolution+scalefactor)/2, (int)(given.y+j*resolution)-(resolution+scalefactor)/2);

    //If the vector is inside the 
    if (!(psurfaceRight.contains(input1.x, input1.y) || psurfaceLeft.contains(input1.x, input1.y))==false) {
      result.x=0;
      result.y=0;
    } else if (!(psurfaceRight.intersects(input2.x, input2.y, resolution+scalefactor, resolution+scalefactor))==false) {
      result = pointaway(input1, psurfaceRight, 1);
    } else if (!(psurfaceLeft.intersects(input2.x, input2.y, resolution+scalefactor, resolution+scalefactor))==false) {
      result = pointaway(input1, psurfaceLeft, -1);
    } else {
      result = given;
    }
    return result;
  }


  Vec2 pointaway(Vec2 given, java.awt.Polygon surface, int direction) {
    float[] line = new float[4]; //To store the line
    Vec2 closestonsegment=new Vec2(0, 0); // The final segment
    int i=0; //To ignore the first point
    for (java.awt.geom.PathIterator pi = surface.getPathIterator(null); !pi.isDone(); pi.next()) { //Iterate through all the points of polygon
      float R = 10000000000.0;
      if (i>0) {// if this is not the first point
        double[] coords = new double[6]; //Get the points (first and second fields will be the points)  
        // Because the Area is composed of straight lines
        pi.currentSegment(coords); //Store the the points in coords

        //last two points in the previous line becomes the first point in the next line segment 
        line[0]=(float)line[2];
        line[1]=(float)line[3];

        //The second point comes from the iterator
        line[2]=(float)coords[0];
        line[3]=(float)coords[1];

        stroke(0);
        line((float)line[0], (float)line[1], (float)line[2], (float)line[3]);

        //Get the closest point on this line
        Vec2 closestPoint = getClosestPointOnSegment(line[0], line[1], line[2], line[3], given.x, given.y);
        //ellipse(closestPoint.x,closestPoint.y,5,5);

        //Get the distance
        float r=sqrt(pow(closestPoint.x-given.x, 2)+pow(closestPoint.y-given.y, 2)); //Temp
        if ( r < R) {
          R=r;
          closestonsegment=closestPoint;
        }
      }
      i++;
    }
    //Calculate phi
    float phi = atan2(closestonsegment.y, closestonsegment.x);
    float theta = atan2(given.y, given.x);            
    phi+=(PI-direction*theta);
    //the given vector by phi
    ellipse(given.x, given.y, 5, 5);
    Vec2 tmp = given;
    given.x = tmp.x*cos(phi) - tmp.y*sin(phi);
    given.y = tmp.x*sin(phi) + tmp.y*cos(phi);
    ellipse(given.x, given.y, 5, 5);
    given.normalize();
    return given;
  }
}
public static Vec2 getClosestPointOnSegment(float sx1, float sy1, float sx2, float sy2, float px, float py)
{
  double xDelta = sx2 - sx1;
  double yDelta = sy2 - sy1;

  if ((xDelta == 0) && (yDelta == 0))
  {
    throw new IllegalArgumentException("Segment start equals segment end");
  }

  double u = ((px - sx1) * xDelta + (py - sy1) * yDelta) / (xDelta * xDelta + yDelta * yDelta);

  final Vec2 closestPoint;
  if (u < 0)
  {
    closestPoint = new Vec2(sx1, sy1);
  } else if (u > 1)
  {
    closestPoint = new Vec2(sx2, sy2);
  } else
  {
    closestPoint = new Vec2((int) Math.round(sx1 + u * xDelta), (int) Math.round(sy1 + u * yDelta));
  }

  return closestPoint;
}