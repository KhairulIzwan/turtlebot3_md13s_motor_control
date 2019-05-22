/*
 * Created 22 May 2019
 * Basic dc motor control using md13s driver with ROS
 */
//different specific libraries
#include <ros.h>
#include <ros/time.h>

#include <geometry_msgs/Twist.h>

#define pinDirRight 7
#define pinDirLeft 8
#define pinPWMRight 9
#define pinPWMLeft 10

// create the ros node nh. The node will be used to publish to Arduino
ros::NodeHandle nh;

float transVelocity;
float rotVelocity;

float wheelSep = 0.165; //  16.5cm
float wheelRadius = 0.07; //  70mm

double velDiff;
double leftPower;
double rightPower;

double speed1;
double speed2;

void messageCb(const geometry_msgs::Twist &msg)
{
  transVelocity = msg.linear.x;
  rotVelocity = msg.angular.z;

  velDiff = (wheelSep * rotVelocity) / 2.0;
  leftPower = (transVelocity + velDiff) / wheelRadius;
  rightPower = (transVelocity - velDiff) / wheelRadius;

  motorDirection();
}

void motorDirection()
{
  if((abs(leftPower) > 0) && (abs(rightPower) > 0))
  {
    // set the speed
    // speed = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    speed2 = (abs(leftPower) - 0.00) * (255 - 0) / (6.49 - 0.00) + 0;
    speed1 = (abs(rightPower) - 0.00) * (255 - 0) / (6.49 - 0.00) + 0;

    if ((leftPower > 0) && (rightPower > 0))
    {
      moveForward(speed1, speed2);
    }
    else if ((leftPower > 0) && (rightPower < 0))
    {
      moveLeft(speed1, speed2);
    }
    else if ((leftPower < 0) && (rightPower < 0))
    {
      moveBackward(speed1, speed2);
    }
    else if ((leftPower < 0) && (rightPower > 0))
    {
      moveRight(speed1, speed2);
    }
  }
  else
  {
    moveStop();
  }
}

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &messageCb);

void setup() 
{
  nh.initNode();
  nh.subscribe(sub);
  
  pinMode(pinDirRight, OUTPUT);
  pinMode(pinDirLeft, OUTPUT);
  pinMode(pinPWMRight, OUTPUT);
  pinMode(pinPWMLeft, OUTPUT);
}

void loop()
{
  nh.spinOnce();
  delay(1);
}

void moveForward(int speed1, int speed2)
{
  digitalWrite(pinDirRight, HIGH);
  digitalWrite(pinDirLeft, LOW);
  analogWrite(pinPWMRight, speed1);
  analogWrite(pinPWMLeft, speed2);
}

void moveBackward(int speed1, int speed2)
{
  digitalWrite(pinDirRight, LOW);
  digitalWrite(pinDirLeft, HIGH);
  analogWrite(pinPWMRight, speed1);
  analogWrite(pinPWMLeft, speed2);
}

void moveLeft(int speed1, int speed2)
{
  digitalWrite(pinDirRight, LOW);
  digitalWrite(pinDirLeft, LOW);
  analogWrite(pinPWMRight, speed1);
  analogWrite(pinPWMLeft, speed2);
}

void moveRight(int speed1, int speed2)
{
  digitalWrite(pinDirRight, HIGH);
  digitalWrite(pinDirLeft, HIGH);
  analogWrite(pinPWMRight, speed1);
  analogWrite(pinPWMLeft, speed2);
}

void moveStop()
{
//  digitalWrite(pinDirRight, HIGH);
//  digitalWrite(pinDirLeft, HIGH);
  analogWrite(pinPWMRight, 0);
  analogWrite(pinPWMLeft, 0);
}
