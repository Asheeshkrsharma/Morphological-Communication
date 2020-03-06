// Author: Asheesh Sharma
// Msc project
// University of Bristol

// An uneven surface

import shiffman.box2d.*;
import org.jbox2d.collision.shapes.*;
import org.jbox2d.common.*;
import org.jbox2d.dynamics.*;
import org.jbox2d.dynamics.*;


// Using this variable to decide whether to draw all the stuff
boolean debug = true;

// Flowfield object
FlowField flowfield;

// A reference to our box2d world
Box2DProcessing box2d;

// An ArrayList of particles that will fall on the surface
ArrayList<Particle> particles;

// An object to store information about the uneven surface
Surface surface;

void setup() {
  size(640, 360);
  
  smooth();

  // Initialize box2d physics and create the world
  box2d = new Box2DProcessing(this);
  box2d.createWorld();
  // We are setting a custom gravity
  box2d.setGravity(0, 0);

  // Create the empty list
  particles = new ArrayList<Particle>();
  // Create the surface
  surface = new Surface();
  // Make a new flow field with "resolution" of 16
  flowfield = new FlowField(25,surface);
}

void draw() {
  background(255);
  
  // If the mouse is pressed, we make new particles
  if (mousePressed) {
    float sz = 5;
    particles.add(new Particle(mouseX,mouseY,sz,flowfield ));
  }

  // We must always step through time!
  box2d.step();

  // Draw the surface
  surface.display();
  if (debug) flowfield.display();
  // Draw all particles
  for (Particle p: particles) {
    p.display();
  }

  // Particles that leave the screen, we delete them
  // (note they have to be deleted from both the box2d world and our list
  for (int i = particles.size()-1; i >= 0; i--) {
    Particle p = particles.get(i);
    if (p.done()) {
      particles.remove(i);
    }
  }

  // Just drawing the framerate to see how many particles it can handle
  fill(0);
  text("Click to populate framerate: " + (int)frameRate,12,16);
}