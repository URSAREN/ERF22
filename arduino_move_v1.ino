#include <ros.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>
 
// Handles startup and shutdown of ROS
ros::NodeHandle nh;

////////////////// Motor Controller Variables and Constants ///////////////////
 
// Motor A connections Right
const int analog1 = 5;
const int analog2 = 6;
  
// Motor B connections Left
const int b_analog1 = 10;
const int b_analog2 = 11;

// Wheel radius in meters
const double WHEEL_RADIUS = 0.035;

// Distance from center of the left tire to the center of the right tire in m
const double WHEEL_BASE = 0.13;



/////////////////////// Motor Controller Functions ////////////////////////////


void forward() {     
  // Moves straight forward  
  analogWrite(analog1, 210);
  analogWrite(analog2, 0);
  analogWrite(b_analog1, 210);
  analogWrite(b_analog2, 0);
}

void backward() {      
  // Moves straight backward    
  analogWrite(analog1, 0);
  analogWrite(analog2, 210);
  analogWrite(b_analog1, 0);
  analogWrite(b_analog2, 210);
}

void rotate_slow_right() {      
  // Rotates around right wheel    
  digitalWrite(analog1, LOW);
  digitalWrite(analog2, LOW);
  analogWrite(b_analog1, 210);
  analogWrite(b_analog2, 0);
}

void rotate_hard_right() {      
  // Right rotation with both wheels   
  analogWrite(analog1, 0);
  analogWrite(analog2, 210);
  analogWrite(b_analog1, 210);
  analogWrite(b_analog2, 0);
}

void rotate_slow_left() {      
  // Rotates around left wheel    
  analogWrite(analog1, 210);
  analogWrite(analog2, 0);
  digitalWrite(b_analog1, LOW);
  digitalWrite(b_analog2, LOW);
}

void rotate_hard_left() {      
  // Left rotation with both wheels    
  analogWrite(analog1, 210);
  analogWrite(analog2, 0);
  analogWrite(b_analog1, 0);
  analogWrite(b_analog2, 210);
}

void stop() {            
  // Stops completely  
  digitalWrite(analog1, LOW);
  digitalWrite(analog2, LOW);
  digitalWrite(b_analog1, LOW);
  digitalWrite(b_analog2, LOW);
}

void move_robot(const geometry_msgs::Twist& cmdVel){
  // Moves robot in accordance to received Twist commands
  if (cmdVel.linear.x > 0) {
    forward();
  }
  if (cmdVel.linear.x < 0) {
    backward();
  }
  if (cmdVel.angular.z > 0 && cmdVel.angular.z < 10) {
    rotate_slow_left();
  }
  if (cmdVel.angular.z == 10) {
    rotate_hard_left();
  }
  if (cmdVel.angular.z < 0 && cmdVel.angular.z > -10) {
    rotate_slow_right();
  }
  if (cmdVel.angular.z == -10) {
    rotate_hard_right();
  }
  if (cmdVel.angular.z == 0 && cmdVel.linear.x == 0) {
    stop();
  }
}

// Set up ROS subscriber to the velocity command
ros::Subscriber<geometry_msgs::Twist> subCmdVel("cmd_vel", &move_robot);

void setup() {
 
   
  // Motor control pins are outputs
  pinMode(analog1, OUTPUT);
  pinMode(analog2, OUTPUT);
  pinMode(b_analog1, OUTPUT);
  pinMode(b_analog2, OUTPUT);

  // Initial -> down
  digitalWrite(analog1, LOW);
  digitalWrite(analog2, LOW);
  digitalWrite(b_analog1, LOW);
  digitalWrite(b_analog2, LOW);
  

  // ROS Setup
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(subCmdVel);
}

void loop() {
  // Will it work, or additional method required?
  nh.spinOnce();
   
}
