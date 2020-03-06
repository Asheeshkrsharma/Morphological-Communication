// Author: Asheesh Sharma
// Msc project
// University of Bristol

//Flow field for the environment

class FlowField {
  Vec2[][] field;   // A flow field is a two dimensional array of Vectors
  int cols, rows; // Columns and Rows fir the  above grid

  int resolution; // How large is each "cell" of the flow field
  Surface mySurface; // We need this to look weather a flow vector is outside the terrain boundaries

  FlowField(int r, Surface mySurface_) {
    resolution = r;
    cols = floor(width/resolution); // Determine the number of columns based on width and height
    rows = floor(height/resolution); // Determine the number of columns based on width and height

    field = new Vec2[cols][rows]; //Initialise the field

    mySurface=mySurface_; //Initialie the surface
    init(); //Initiate the flow field
  }

  void init() {
    //Implements a 2 dimensional perin noise

    noiseSeed((int)random(10000));     // Reseed noise so we get a new flow field every time
    float xoff = 0; //Iterator for x axis
    for (int i = 0; i < cols; i++) { //For all columns

      float yoff = 0; //Iterator for y axis      
      for (int j = 0; j < rows; j++) { //For all rows

        float theta = map(noise(xoff, yoff), 0, 1, 0, -PI); //Create a theta value (for the angle of vector)

        // Polar to cartesian coordinate transformation to get x and y components of the vector
        Vec2 flowvec = new Vec2(cos(theta), sin(theta));

        // Returns a int array. First element is 1 if the vector is inside the surface
        // Second element is 1 if the vector is intersecting with the left surface
        // Third element is 1 if the vector is intersecting wirh the right surface
        // fourth and fith elements form the unit vector in case of intersection,otherwise they are zero
        field[i][j] = surface.getVectorCorrected(flowvec, resolution, i, j, 20);

        yoff += 0.2;
      }
      xoff += 0.2;
    }
  }

  // Draw every vector
  void display() {
    //surface.pointaway();
    for (int i = 0; i < cols; i++) { //For every column
      for (int j = 0; j < rows; j++) { //For every row
        Vec2 tmp = field[i][j]; //Get that vector
        if (tmp.x==0.0 && tmp.y==0.0f) { //if the vector was set to zero
        } else { //Otherwise draw the vector
          drawVector(tmp, i*resolution, j*resolution, resolution-5, color(0), true);
        }
      }
    }
  }

  // Renders a vector object 'v' as an arrow and a position 'x,y'
  void drawVector(Vec2 v, float x, float y, float scayl, color c, boolean head) {
    //Arguments 
    // v: The vector to draw
    // x,y: Coordinated
    // scayl: Scale for the vector
    // c: What color should be used.
    /*if (surface.amIInside((int)x,(int)y)==true){
     rect(x,y,resolution,resolution);
     } */

    /*if(surface.amIIntersecting((int)x, (int)y, resolution+50, resolution+50) == false){
     stroke(color(15,   27,  254));
     rect(x,y,resolution+50,resolution+50);
     }*/

    pushMatrix();
    float arrowsize = 4;

    // Translate to position to render vector
    translate(x, y);
    stroke(c); //Color
    // Call vector heading function to get direction (note that pointing to the right is a heading of 0) and rotate
    float angle = (float) Math.atan2(-v.y, v.x); 
    rotate(-1*angle);

    // Calculate length of vector & scale it to be bigger or smaller if necessary
    float len = v.length()*scayl;

    // Draw three lines to make an arrow (draw pointing up since we've rotate to the proper direction)
    line(0, 0, len, 0); //Line to represent the orientation of the vector

    //lines to draw the head
    if (head) {
      line(len, 0, len-arrowsize, +arrowsize/2);
      line(len, 0, len-arrowsize, -arrowsize/2);
    }
    popMatrix();
  }

  Vec2 lookup(Vec2 lookup) {
    // This function returns the heading which should be used for applying impulses on the Bodies.
    // First we floor the given vector. This is done because the array has absolute integer addresses.
    // Seondly, we can used the difference b/w the actual and floored vector to adjust the weights of
    // the neighbouring vector.

    // After querying the neighbours from the flowfield, we calculate th desired direction as follows.
    // Argument
    // lookup: Position of a body

    Vec2 floor = new Vec2(floor(lookup.x), floor(lookup.y)); //Floor vector

    //The 4 weights we'll interpolate, see http://en.wikipedia.org/wiki/File:Bilininterp.png for the coordinates
    float x00=floor.x, x01=floor.x, x10=floor.x+resolution, x11=floor.x+resolution; //to hold neighbours
    float y00=floor.y, y01=floor.y+resolution, y10=floor.y, y11=floor.y+resolution; //to hold neighbours

    //Corresponding rows and columns
    int r00 =int(constrain(y00/resolution, 0, rows-1)), c00 =int(constrain(x00/resolution, 0, cols-1));
    int r01 =int(constrain(y01/resolution, 0, rows-1)), c01 =int(constrain(x01/resolution, 0, cols-1));
    int r10 =int(constrain(y10/resolution, 0, rows-1)), c10 =int(constrain(x10/resolution, 0, cols-1));
    int r11 =int(constrain(y11/resolution, 0, rows-1)), c11 =int(constrain(x11/resolution, 0, cols-1));    

    //Corresponding Vectors
    Vec2 f00 = field[c00][r00];
    Vec2 f01 = field[c01][r01];
    Vec2 f10 = field[c10][r10];
    Vec2 f11 = field[c11][r11];

    //Do the x interpolations
    float xWeight = lookup.x - floor.x;
    Vec2 top = f00.mul(1 - xWeight).add(f10.mul(xWeight));
    Vec2 bottom = f01.mul(1 - xWeight).add(f11.mul(xWeight));

    //Do the y interpolation
    float yWeight = lookup.y - floor.y;

    //This is now the direction we want to be travelling in (needs to be normalized)
    Vec2 desiredDirection = top.mul(1 - yWeight).add(bottom.mul(yWeight));


    desiredDirection.normalize();

    //If we are centered on a grid square with no vector this will happen
    if (Float.isNaN(desiredDirection.lengthSquared())) {
      desiredDirection.setZero();
    }
    drawVector(desiredDirection, lookup.x, lookup.y, resolution, color(15, 27, 254), true); //Yellow

    return desiredDirection;
  }
}