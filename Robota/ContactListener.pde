// Author: Asheesh Sharma
// Msc project
// University of Bristol
// ContactListener to listen for collisions!

import org.jbox2d.callbacks.ContactImpulse;
import org.jbox2d.callbacks.ContactListener;
import org.jbox2d.collision.Manifold;
import org.jbox2d.dynamics.contacts.Contact;

class CustomListener implements ContactListener {
  CustomListener() {
  }

  // This function is called when a new collision occurs
  void beginContact(Contact cp) {
    // Get both fixtures
    Fixture f1 = cp.getFixtureA();
    Fixture f2 = cp.getFixtureB();
    // Get both bodies
    Body b1 = f1.getBody();
    Body b2 = f2.getBody();
    // Get our objects that reference these bodies
    Object o1 = b1.getUserData();
    Object o2 = b2.getUserData();

    //we dont have to bother about anything else in the world
    // That is we only have to know if the fixtures that collided were
    // Whiskers. Since we dont store the user data for everything
    // else in the world, we can run into collisions which has nothing
    // to do with whiskers, so we check if the object in concern is not
    // defined.
    if (o1 != null) {
      // new we need to check if the object is a whisker. This is not
      // necessary as nothing else will pass through the above loop.
      // But it is just there to make sure. You know physics engines are
      // full of glitches.
      if (o1.getClass() == Whiskerhub.class) { // if so,
        //The we change some properties
        Whiskerhub w = (Whiskerhub) o1; //First we access the object
        if (Collisiontoggle) {
          w.colour = color(231, 76, 60); //And then apply the change
          w.haveCollided=true;
        } else {
          w.colour = color(175); //And then apply the change
          w.haveCollided=false;
        }
      }
    }

    // The thing is, we dont know if the object one is the whisker
    // or the object two. So we repeat the same thing. But with 
    // object two.
    if (o2 != null) {  
      if (o2.getClass() == Whiskerhub.class) {
        Whiskerhub w = (Whiskerhub) o2;
        if (Collisiontoggle) {
          w.colour = color(231, 76, 60);
          w.haveCollided=true;
        } else {          
          w.colour = color(175); //And then apply the change
          w.haveCollided=false;
        }
      }
    }
  }

  void endContact(Contact contact) {
    // TODO Auto-generated method stub
  }

  void preSolve(Contact contact, Manifold oldManifold) {
    // TODO Auto-generated method stub
  }

  void postSolve(Contact contact, ContactImpulse impulse) {
    // TODO Auto-generated method stub
  }
}