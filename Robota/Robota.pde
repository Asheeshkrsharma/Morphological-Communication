// Author: Asheesh Sharma
// Msc project
// University of Bristol

import shiffman.box2d.*;
import org.jbox2d.common.*;
import org.jbox2d.dynamics.joints.*;
import org.jbox2d.collision.shapes.*;
import org.jbox2d.collision.shapes.Shape;
import org.jbox2d.common.*;
import org.jbox2d.dynamics.*;
import org.jbox2d.dynamics.contacts.*;
import controlP5.*;
import java.io.FileWriter;
import java.io.*;

int interation=0;
int failiteration=0;
int maxiterations=100;

//Global variables for the robot properties
float cntrRestinglen=20; //Initial resting length of the robot 
float cntrkvalue = 10; //Initial spring constant value
float cntrkrvalue = 0.19; //Initial radial spring constant
float cntrdamping = 5; // Linear damping
float cntrRTSdelayintr=50; //How much time we spend in the collision contraction case
boolean RTStoggle = true; //Set RTS active
boolean Collisiontoggle = true; //Are we looking at collision. Makes the bots passive.

boolean lensDrag=false;
boolean magnifierToggle=false;

String saveDir = "./data/";
//GUI elements
Slider2D s1;
Chart pulseo1;
ControlP5 cp51;
Accordion accordion;


Float kappa, damp;
int failiure=0;

//Create a new window to show a bar plot.
//Statplots plots;

color c = color(0, 160, 100); //This color is inherited by the robot when it collides

// Flowfield object
FlowField flowfield; //Initialises the flow field in the world.

//The mighty constant
float DEG2RAD = PI/180;

// An object to store information about the uneven surface
Surface surface;

// A reference to our box2d world
Box2DProcessing box2d;

// A list for all of our rectangles
ArrayList<Bodybase> pops;


// Time spent by the robot in the window
float elapsed=0; //How much time has passed since the robot was born
float epoch=0; //Time when the robot was born
float current=0; //Current time

String tPEString = "";

magnifier lens;

void writeFile(String path, String data) {
  PrintWriter output;
  output = createWriter(path);
  output.println(data);
  output.flush(); // Writes the remaining data to the file
  output.close(); // Finishes the file
}

public void settings() {
  size(800, 600);
}

void setup() {
  smooth();
  if (args != null) {
    for (int i = 1; i < args.length; i++) {
      String[] arg = split(args[i], '=');
      //println("------------->" + arg[0] + " " + (arg[0].equals("savdir")==true));
      if (arg[0].equals("k")==true) {
        cntrkvalue = float(arg[1]);
      } else if (arg[0].equals("kr")==true) {
        cntrkrvalue = float(arg[1]);
      } else if (arg[0].equals("damp")==true) {
        cntrdamping = float(arg[1]);
      } else if (arg[0].equals("rts")==true) {
        RTStoggle = boolean(arg[1]);
      } else if (arg[0].equals("collision")==true) {
        Collisiontoggle = boolean(arg[1]);
      } else if (arg[0].equals("iterations")==true) {
        maxiterations = int(arg[1]);
      } else if (arg[0].equals("savdir")==true) {
        saveDir = arg[1];
      }
    }
  } else {
    println("No arguments passed using defaults: ");
  }
  
  println("k: " + cntrkvalue);
  println("kr: " + cntrkrvalue);
  println("damping: " + cntrdamping);
  println("RTS: " + RTStoggle);
  println("Collision detection: " + Collisiontoggle);
  println("Maximum number of iterations: " + maxiterations);
  println("Data directory: " + saveDir);

  gui(); //Initialise the gui
  //plots = new Statplots(); //Initialise the plot window

  // Initialize box2d physics and create the world
  box2d = new Box2DProcessing(this, 20);
  box2d.createWorld();
  // We are setting a custom gravity
  box2d.setGravity(0, 0);

  // Create ArrayLists for all the bots
  pops = new ArrayList<Bodybase>();

  // Create the surface terrain
  surface = new Surface();

  // Make a new flow field with "resolution" of 25
  flowfield = new FlowField(25, surface);

  // Add a listener to listen for collisions!
  box2d.world.setContactListener(new CustomListener());

  //Puts a three bot system in the world
  startagain();
}

void draw() {

  background(255);
  // We must always step through time!
  box2d.step();

  // Draw the surface and the flow field
  surface.display();
  flowfield.display();

  // Display all the people and centroid 
  Vec2 center = new Vec2(0, 0); //Center of the traingle made by the robots
  Vec2 tmp = new Vec2(0, 0);
  float totalPe = 0.0;
  if (pops.size() > 0) { //check if pops is empty
    for (Bodybase p : pops) { //for all the robots
      p.update(); //update them
      totalPe += p.display(); //display then
      tmp = box2d.getBodyPixelCoord(p.base); //store its position 
      center= new Vec2(center.x+tmp.x, center.y+tmp.y); //add it to center
    }
    center=new Vec2(center.x/pops.size(), center.y/pops.size()); //get the centroid
    ellipse(center.x, center.y, 5, 5); //Draw the ellipse
  }
  tPEString += (float)(elapsed/1000) + " " + totalPe + "\n";
  // if center has crossed the finish line then we reset everything a start again.
  if (center.y <=0) {
    // people that leave the screen, we delete them
    // (note they have to be deleted from both the box2d world and our list
    for (int i = pops.size()-1; i >= 0; i--) { //for all the robots
      Bodybase p = pops.get(i); // get that robot
      p.killBody();
      pops.remove(i); //remove it.
    }
    epoch=0; //Set epoch to zero again.
    elapsed=0;
    current=0;
    startagain();
    writeFile(saveDir + "/PE" + interation + ".txt", tPEString);
    tPEString = "";
    interation++;
  } else {
    current = millis();
    elapsed=current-epoch;
    if (elapsed>60000) {
      for (int i = pops.size()-1; i >= 0; i--) { //for all the robots
        Bodybase p = pops.get(i); // get that robot
        p.killBody();
        pops.remove(i); //remove it.
      }
      failiteration++;
      failiure++;
      epoch=0; //Set epoch to zero again.
      elapsed=0;
      current=0;
      startagain();
      writeFile(saveDir + "/PE" + interation + ".txt", tPEString);
      tPEString = "";
    }
  }

  // Just drawing the framerate to see how many particles it can handle
  fill(0);
  rect(400, 0, width, 10);
  rect(width, 0, 10, height * 2);
  rect(0, 0, 10, height * 2);
  fill(255);
  // draw a rectanglw to display how much time it took for the robot to clear the the obstacle
  textSize(12);
  fill(0, 102, 153);
  text("FPS: " + (int)frameRate + " Elapsed: " + (float)(elapsed/1000), width/2 - 60, height-8);

  fill(255);
  textSize(10);
  text("Iterations: " + (int)interation, 10, 50);  
  text("Failiures: " + (int)failiure, 10, 60);
  fill(0);
  guiUpdate(center.x, center.y, tmp);
  center.setZero(); //Set it back to zero
  if ((int)interation >= maxiterations){
    exit();
  }
}

void mouseDragged() {
  lens.drag(mouseX, mouseY);
}
void keyPressed() {
  lensDrag=true;
}
void keyReleased() {
  lensDrag=false;
}

void startagain() { 

  setExperimentProperties(cntrRestinglen, cntrkvalue, cntrkrvalue, cntrdamping, cntrRTSdelayintr, RTStoggle, Collisiontoggle);

  float xstart=random(200, width-200);
  Bodybase p1 = new Bodybase(xstart, height, flowfield, -60*DEG2RAD);
  Bodybase p2 = new Bodybase(xstart-20, height-40, flowfield, 30*DEG2RAD); 
  Bodybase p3 = new Bodybase(xstart-40, height+20, flowfield, -150*DEG2RAD);


  p1.water = true;
  p2.water = true;
  p3.water = true;

  pops.add(p1);
  pops.add(p2);
  pops.add(p3);

  p1.connect(p2.rts.distal.body);
  p2.connect(p3.rts.distal.body);
  p3.connect(p1.rts.distal.body);
  epoch = millis();
  elapsed=0;
  current=0;
}

void setExperimentProperties(float l, float k, float kr, float d, float RTSdelay, boolean rtstoggle, boolean collisiontoggle) {
  cntrRestinglen=l; //Initial resting length of the robot 

  cntrkvalue = k; //Initial spring constant value
  cp51.getController("Spring constant").setValue(k);

  cntrkrvalue = kr; //Initial radial spring constant
  cp51.getController("Angular constant").setValue(kr);

  cntrdamping = d; // Linear damping
  cp51.getController("Damping").setValue(d);

  kappa=k;
  damp=d;

  cntrRTSdelayintr=RTSdelay; //How much time we spend in the collision contraction case
  cp51.getController("Collision Delay").setValue(RTSdelay);

  RTStoggle = rtstoggle; //Set RTS active
  if (rtstoggle==true) {
    cp51.getController("Disable \noscilation").setValue(1);
  } else {
    cp51.getController("Disable \noscilation").setValue(0);
  }

  Collisiontoggle = collisiontoggle; //Are we looking at collision. Makes the bots passive.
  if (collisiontoggle==true) {
    cp51.getController("Collision \nDetection").setValue(1);
  } else {
    cp51.getController("Collision \nDetection").setValue(0);
  }
}


void gui() {
  cp51 = new ControlP5(this);
  lens=new magnifier(width/2, height/2, 300, 300);
  cp51.addToggle("Lens")
    .setPosition(10, 10)
    .setValue(magnifierToggle)
    .setMode(ControlP5.SWITCH)
    .setSize(30, 10)
    ;

  // group number 1, contains 2 bangs
  Group g1 = cp51.addGroup("Spring Parameters")
    .setBackgroundColor(color(0, 150))
    .setSize(50, 50) 
    ;

  cp51.addSlider("Spring constant")
    .setPosition(10, 20)
    .setRange(4, 30)
    .moveTo(g1)
    .setValue(cntrkvalue);

  cp51.addSlider("Angular constant")
    .setPosition(10, 35)
    .setRange(0, 2)
    .moveTo(g1)
    .setValue(cntrkrvalue);

  cp51.addSlider("Damping")
    .setPosition(10, 50)
    .setRange(0, 5)
    .moveTo(g1)
    .setValue(cntrdamping);


  // group number 2, contains a radiobutton
  Group g2 = cp51.addGroup("RTS properties")
    .setBackgroundColor(color(0, 150))
    .setBackgroundHeight(150)
    ;

  s1=cp51.addSlider2D("RTS")
    .setPosition(10, 20)
    .setSize(60, 60)
    .setMinMax(1, 0, 5, 0.5)
    .setValue(5, 0.05)
    .moveTo(g2)
    ;  

  pulseo1=cp51.addChart("Plot")
    .setPosition(80, 20)
    .setSize(100, 60)
    .setRange(-20, 20)
    .setView(Chart.LINE) // use Chart.LINE, Chart.PIE, Chart.AREA, Chart.BAR_CENTERED
    .setStrokeWeight(1.5)
    .moveTo(g2)
    ;
  pulseo1.addDataSet("incoming");
  pulseo1.setData("incoming", new float[100]);

  cp51.addToggle("Disable \noscilation")
    .setPosition(10, 105)
    .setValue(RTStoggle)
    .setMode(ControlP5.SWITCH)
    .setSize(30, 10)
    .moveTo(g2)
    ;

  cp51.addSlider("Rest. length const.")
    .setPosition(45, 105)
    .setRange(0, 4)
    .setValue(cntrRestinglen)
    .setNumberOfTickMarks(25)
    .setSliderMode(Slider.FLEXIBLE)
    .setSize(135, 10)
    .moveTo(g2)
    ;
  cp51.getController("Rest. length const.").getCaptionLabel().align(ControlP5.RIGHT, ControlP5.BOTTOM_OUTSIDE).setPaddingX(0).setPaddingY(10);

  // group number 3, contains a bang and a slider
  Group g3 = cp51.addGroup("  Bot Behaviour")
    .setBackgroundColor(color(0, 150))
    .setBackgroundHeight(150)
    ;

  cp51.addToggle("Collision \nDetection")
    .setPosition(10, 20)
    .setSize(50, 20)
    .setValue(Collisiontoggle)
    .setMode(ControlP5.SWITCH)
    .setSize(30, 10)
    .moveTo(g3)
    ;

  cp51.addSlider("Collision Delay")
    .setPosition(45, 20)
    .setRange(5, 150)
    .setValue(cntrRestinglen)
    .setNumberOfTickMarks(25)
    .setSliderMode(Slider.FLEXIBLE)
    .setSize(135, 10)
    .moveTo(g3)
    ;    

  // reposition the Label for controller 'slider'
  cp51.getController("Collision Delay").getCaptionLabel().align(ControlP5.RIGHT, ControlP5.BOTTOM_OUTSIDE).setPaddingX(0).setPaddingY(10);

  // create a new accordion
  // add g1, g2, and g3 to the accordion.
  accordion = cp51.addAccordion("acc")
    .setPosition(width-240, 40)
    .setWidth(200)
    .addItem(g1)
    .addItem(g2)
    .addItem(g3)
    ;

  accordion.setCollapseMode(Accordion.SINGLE);
}

void guiUpdate(float lensX, float lensY, Vec2 tmp) {
  if (cp51.getController("Lens").getValue()==1.0) {
    magnifierToggle=true;
  } else {
    magnifierToggle=false;
  }

  if (cp51.getController("Disable \noscilation").getValue()==1.0) {
    RTStoggle=true;
  } else {
    RTStoggle=false;
  }

  if (cp51.getController("Collision \nDetection").getValue()==1.0) {
    Collisiontoggle=true;
  } else {
    Collisiontoggle=false;
  }

  if (magnifierToggle) {
    lens.display(lensX, lensY, tmp);
  }

  if (RTStoggle == false) {
    cntrkvalue=cp51.getController("Spring constant").getValue();
    cntrkrvalue=cp51.getController("Angular constant").getValue();
    cntrdamping=cp51.getController("Damping").getValue();
    cntrRestinglen=(s1.getArrayValue()[0]*sin(frameCount*s1.getArrayValue()[1]));

    if (cntrRestinglen < 0) {
      cntrRestinglen=0;
    }


    cp51.getController("Rest. length const.").setValue(cntrRestinglen);
    pulseo1.push("incoming", cntrRestinglen);
  } else {
    cntrRestinglen=cp51.getController("Rest. length const.").getValue();
    pulseo1.push("incoming", cntrRestinglen);
  }
}