class Flock {
  ArrayList<Boid> boids; // An ArrayList for all the boids

    Flock() {
    boids = new ArrayList<Boid>(); // Initialize the ArrayList
  }

  void run() {
    for (Boid b : boids) {
      b.col = color(175);
    }    

    Boid b1 = boids.get(0);
    b1.col = color(0, 0, 255);
    b1.view(boids);

    for (Boid b : boids) {
      b.flock(boids);  // Passing the entire list of boids to each boid individually
    }

    for (Boid b : boids) {
      b.run(boids);  // Passing the entire list of boids to each boid individually
    }
  }

  void addBoid(Boid b) {
    boids.add(b);
  }
}

int lasttime = 0;
float mouse_xrad=5;
float mouse_yrad=5;
class Boid {

  PVector position;
  PVector velocity;
  PVector acceleration;
  float r;
  float maxforce;    // Maximum steering force
  float maxspeed;    // Maximum speed
  int curr_t;

  color col;
  Boid(float x, float y) { //constructor
    acceleration = new PVector(0, 0);
    velocity = new PVector(random(-1, 1), random(-1, 1));
    position = new PVector(x, y);
    r = 5.0;
    maxspeed = 3;
    maxforce = 0.05;
    
    col = color(175);
  }

  void run(ArrayList<Boid> boids) { //runeverything
    flock(boids);
    curr_t = millis();
    update();
    borders();
    render();
  }

  void applyForce(PVector force) {
    
    acceleration.add(force); //vector addition-> .add
  }

  // adding effect vectors to the acceleration vector, change to velocity vector
  void flock(ArrayList<Boid> boids) {
    PVector sep = separate(boids);   // Separation
    PVector ali = align(boids);      // Alignment
    PVector coh = cohesion(boids);   // Cohesion

    // Not for every boid yet
    // PVector view = view(boids);      // view

    // Arbitrarily weight these forces->remove
    sep.mult(1.0);
    ali.mult(1.0);
    coh.mult(1.0);

    // Not for every boid yet
    // view.mult(1.0);

    // Add the force vectors to acceleration
    applyForce(sep);
    applyForce(ali);
    applyForce(coh);

    // Not for every boid yet
    // applyForce(view);
  }

  // Method to update position->this is the first equation of the paper. use helpers to make the equation
  void update() {
    // Update velocity
    velocity.add(acceleration);
    // Limit speed
    velocity.limit(maxspeed);
    position.add(velocity);
    // Reset accelertion to 0 each cycle
    acceleration.mult(0);
  }

  // A method that calculates and applies a steering force towards a target
  // STEER = DESIRED MINUS VELOCITY
  PVector seek(PVector target) {
    PVector desired = PVector.sub(target, position);  // A vector pointing from the position to the target
    // Normalize desired and scale to maximum speed
    desired.normalize();
    desired.mult(maxspeed);
    // Steering = Desired minus Velocity
    PVector steer = PVector.sub(desired, velocity);
    steer.limit(maxforce);  // Limit to maximum steering force
    return steer;
  }

  void render() {
    // Draw a triangle rotated in the direction of velocity
    float theta = velocity.heading() + radians(90);
    fill(col);
    stroke(0);
    pushMatrix();
    translate(position.x, position.y);
    rotate(theta);
    beginShape(TRIANGLES);
    vertex(0, -r*2);
    vertex(-r, r*2);
    vertex(r, r*2);
    endShape();
    popMatrix();
  }

  // Wraparound
  void borders() {
    if (position.x < -r) position.x = width+r;
    if (position.y < -r) position.y = height+r;
    if (position.x > width+r) position.x = -r;
    if (position.y > height+r) position.y = -r;
  }

  // Separation
  // Method checks for nearby boids and steers away
  //sir asked to keep this to some small extent
  PVector separate (ArrayList<Boid> boids) {
    float desiredseparation = 25.0f;
    PVector steer = new PVector(0, 0, 0);
    int count = 0;
    // For every boid in the system, check if it's too close
    for (Boid other : boids) {
      float d = PVector.dist(position, other.position);
      // If the distance is greater than 0 and less than an arbitrary amount (0 when you are yourself)
      if ((d > 0) && (d < desiredseparation)) {
        // Calculate vector pointing away from neighbor
        PVector diff = PVector.sub(position, other.position);
        diff.normalize();
        diff.div(d);        // Weight by distance
        steer.add(diff);
        count++;            // Keep track of how many
      }
    }
    // Average -- divide by how many
    if (count > 0) {
      steer.div((float)count);
    }

    // As long as the vector is greater than 0
    if (steer.mag() > 0) {
      // Implement Reynolds: Steering = Desired - Velocity
      steer.normalize();
      steer.mult(maxspeed);
      steer.sub(velocity);
      steer.limit(maxforce);
    }
    return steer;
  }

  // Alignment
  // For every nearby boid in the system, calculate the average velocity //done
  PVector align (ArrayList<Boid> boids) {
    float neighbordist = mouse_xrad;
    PVector V_T = new PVector(0, 0);
    int count = 0;
    float eta = random(0,1);
    PVector noise = PVector.random2D();
    float magnitude = 0.1 * (eta - 0.5); ///noise variable. we only change the first term, as the other is for the uniform distribution
    noise.setMag(magnitude);
    
    for (Boid other : boids) {
      float d = PVector.dist(position, other.position);
      if ((d > 0) && (d < neighbordist)) {
        PVector v_t = PVector.fromAngle(other.velocity.heading()); 
        V_T.add(v_t);
        V_T.add(noise);
        count++;
      }
    }
    if (count > 0) {
      float dt = (curr_t - lasttime) / 1000.0;
      V_T.div((float)count); //averaging
      V_T.mult(dt);
      V_T.normalize();
      V_T.mult(maxspeed); //stop it from blowing up?should i remove this?
      PVector steer = PVector.sub(V_T, velocity);
      steer.limit(maxforce);
      return steer;
    } 
    else {
      return new PVector(0, 0);
    }
  }

  // Cohesion
  // For the average position (i.e. center) of all nearby boids, calculate steering vector towards that position
  //not sure if this should still be there or not? i guess not?
  PVector cohesion (ArrayList<Boid> boids) {
    float neighbordist = 0;
    PVector sum = new PVector(0, 0);   // Start with empty vector to accumulate all positions
    int count = 0;
    for (Boid other : boids) {
      float d = PVector.dist(position, other.position);
      if ((d > 0) && (d < neighbordist)) {
        sum.add(other.position); // Add position
        count++;
      }
    }
    if (count > 0) {
      sum.div(count);
      return seek(sum);  // Steer towards the position
    } 
    else {
      return new PVector(0, 0);
    }
  }

  // View
  // move laterally away from any boid that blocks the view
  //the POV should be a full circle->done
  // Right now we are just drawing the view and highlighting boids
  PVector view (ArrayList<Boid> boids) {

    // How far can it see?
    float sightDistance = mouse_xrad;
    float periphery = 2*PI;

    for (Boid other : boids) {
      // A vector that points to another boid and that angle
      PVector comparison = PVector.sub(other.position, position);

      // How far is it
      float d = PVector.dist(position, other.position);

      // What is the angle between the other boid and this one's current direction
      float diff = PVector.angleBetween(comparison, velocity);

      // If it's within the periphery and close enough to see it
      if (diff < periphery && d > 0 && d < sightDistance) {
        // Just change its color
        other.highlight();
        //do the average velocity calculation here and return the vector.
      }
    }

//helpers dont touch
    // Debug Drawing
    float currentHeading = velocity.heading();
    pushMatrix();
    translate(position.x, position.y);
    rotate(currentHeading);
    fill(0, 100);
    arc(0, 0, sightDistance*2, sightDistance*2, -periphery, periphery);
    popMatrix();

    return new PVector();
  }

  void highlight() {
    col = color(255, 0, 0);
  }
}

Flock flock;

void setup() {
  size(540,540);
  flock = new Flock();
  // Add an initial set of boids into the system
  for (int i = 0; i < 200; i++) {
    Boid b = new Boid(width/2+random(0,75),height/2+random(0,75));
    flock.addBoid(b);
  }
}

void draw() {
  background(255);
  
  flock.run();
}

// Add a new boid into the System
void mouseDragged() {
  flock.addBoid(new Boid(mouseX,mouseY));
}



void mouseWheel(MouseEvent event) {
  float e = event.getCount();
  println(e);
  if (e>0) {
      mouse_xrad=mouse_xrad+1;
      mouse_yrad=mouse_yrad+1;
    }
   if (e<0) {
      mouse_xrad=mouse_xrad-1;
      mouse_yrad=mouse_yrad-1;
    }
}
