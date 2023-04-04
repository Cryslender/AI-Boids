import java.util.ArrayList;

// Define boid class
class Boid {
  PVector position;
  PVector velocity;
  PVector acceleration;
  float maxSpeed;              //maximum speed
  float maxForce;              //maximum steering force
  float perceptionRadius;      //obstacle radius that boids shoud avoid
  float r;                     //radius of obstacle to turn to the nighbour 

  //boid constructor 
  Boid(float x, float y) {
    position = new PVector(x, y);
    velocity = PVector.random2D();
    velocity.setMag(random(2, 4));
    acceleration = new PVector();
    maxSpeed = 2;
    maxForce = 0.1;
    perceptionRadius = 50;
    r = 2.0;
  }
  
 //add the force vector to acceleration
  void applyForce(PVector force) {
    acceleration.add(force);
  }
  //accumulate new acceleration each time base on three rules
  void flock(ArrayList<Boid> boids, ArrayList<PVector> obstacles) {
    PVector alignment = align(boids);
    PVector cohesion = cohesion(boids);
    PVector separation = separate(boids);
    PVector avoid = avoidObstacles(obstacles);
    
    //Arbitrary weight to the three laws of boids including avoiding obstacles
    alignment.mult(1);
    cohesion.mult(1);
    separation.mult(1.5);
    avoid.mult(2);
    //Add vectors to acceleration
    applyForce(alignment);
    applyForce(cohesion);
    applyForce(separation);
    applyForce(avoid);
  }
  //steer towards the avarage heading of local boids
  PVector align(ArrayList<Boid> boids) {
    PVector steering = new PVector();
    int total = 0;
    for (Boid other : boids) {
      float distance = PVector.dist(position, other.position);
      if (other != this && distance < perceptionRadius) {
        steering.add(other.velocity);
        total++;
      }
    }
    if (total > 0) {
      steering.div((float)total);
      steering.setMag(maxSpeed);
      steering.sub(velocity);
      steering.limit(maxForce);
    }
    return steering;
  }
  //steer to move towards the average position of other boids
  PVector cohesion(ArrayList<Boid> boids) {
    PVector steering = new PVector();
    int total = 0;
    for (Boid other : boids) {
      float distance = PVector.dist(position, other.position);
      if (other != this && distance < perceptionRadius) {
        steering.add(other.position);
        total++;
      }
    }
    if (total > 0) {
      steering.div(total);
      steering.sub(position);
      steering.setMag(maxSpeed);
      steering.sub(velocity);
      steering.limit(maxForce);
    }
    return steering;
  }
  //method to separate boids with a certain distance
  PVector separate(ArrayList<Boid> boids) {
    PVector steering = new PVector();
    int total = 0;
    for (Boid other : boids) {
      float distance = PVector.dist(position, other.position);
      if (other != this && distance < perceptionRadius/2) {
        PVector diff = PVector.sub(position, other.position);
        diff.div(distance*distance);
        steering.add(diff);
        total++;
      }
    }
    if (total > 0) {
      steering.div((float)total);
      steering.setMag(maxSpeed);
      steering.sub(velocity);
      steering.limit(maxForce);
    }
    return steering;
  }
  //method for boids to avoid obstacles and move to other direction
  PVector avoidObstacles(ArrayList<PVector> obstacles) {
      PVector steering = new PVector();
      int total = 0;
      for (PVector obstacle : obstacles) {
        float distance = PVector.dist(position, obstacle);
        if (distance < perceptionRadius) {
          PVector diff = PVector.sub(position, obstacle);
          diff.div(distance);
          steering.add(diff);
          total++;
        }
      }
      if (total > 0) {
        steering.div((float)total);
        steering.setMag(maxSpeed);
        steering.sub(velocity);
        steering.limit(maxForce);
  }
  return steering;
}
 //update the position of boids while in the move
  void update() {
    velocity.add(acceleration);
    velocity.limit(maxSpeed);
    position.add(velocity);
    acceleration.mult(0);
  }
  //wrap the boids around
  void edges() {
    if (position.x < -perceptionRadius) position.x = width + perceptionRadius;
    if (position.y < -perceptionRadius) position.y = height + perceptionRadius;
    if (position.x > width + perceptionRadius) position.x = -perceptionRadius;
    if (position.y > height + perceptionRadius) position.y = -perceptionRadius;
  }
  //show boids on the simulation in 2D
  void show() {
    // Draw a triangle rotated in the direction of velocity
    float theta = velocity.heading2D() + radians(90);
    // heading2D() above is now heading() but leaving old syntax until Processing.js catches up
    
    fill(100, 100);
    stroke(255);
    pushMatrix();
    translate(position.x, position.y);
    rotate(theta);
    beginShape(TRIANGLES);
    vertex(0, -r*2);
    vertex(-r, r*2);
    vertex(r, r*2);
    endShape();
    popMatrix();
    strokeWeight(2);
    }
  }
  // Initialize arrays
  ArrayList<Boid> boids;
  ArrayList<PVector> obstacles;
  int numObstacles = 60;        //number of obstacles
  float radius = 100;           //radius of the circumfrerence
  //set up the environment
  void setup() {
    size(640, 360);
    boids = new ArrayList<Boid>();
    obstacles = new ArrayList<PVector>();
    float angle = TWO_PI / numObstacles;
    for (int i = 0; i < numObstacles; i++) {
      float x = width/2 + cos(i*angle) * radius;
      float y = height/2 + sin(i*angle) * radius;
      obstacles.add(new PVector(x, y));
    }
  }
  //render the boid and obstacles on the screen
  void draw() {
    background(51);
    
    // Add obstacles to the screen
    for (PVector obstacle : obstacles) {
      //the size of the obstacles
      strokeWeight(10);      
      //the color of the obstacles
      stroke(255, 0, 0); 
      //position of the obstacles
      point(obstacle.x, obstacle.y);
    }
    
    // Update and show boids
    for (Boid boid : boids) {
      boid.flock(boids, obstacles);
      boid.update();
      boid.edges();
      boid.show();
     }
     //display boids and obstacle base on a click of a mouse button
     if (mouseButton == LEFT) {
      boids.add(new Boid(mouseX,mouseY));
    }else if (mouseButton == RIGHT) {
      PVector obstacle = new PVector(mouseX, mouseY);
      obstacles.add(obstacle);
    }
}
