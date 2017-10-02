#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"
#include <sstream>

double distance = 5.0; // max front distance we can see
double leftDist = 5.0; // max left distance we can see
double rightDist = 5.0; // max right distance we can see

double velocity = 1; // the constant velocity of this robot

void sensorCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  distance = msg->ranges[90];
  leftDist = msg->ranges[179];
  rightDist = msg->ranges[0];
  
  ROS_INFO("I see left  : [%f]", msg->ranges[179]);
  ROS_INFO("I see center: [%f]", msg->ranges[90]);
  ROS_INFO("I see right : [%f]", msg->ranges[0]);
}


/**
 * Simple node using PD control to follow the right side wall. 
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "bot");

  ros::NodeHandle n;

  /**
   * Setup the publisher and subscriber
   */
  ros::Publisher velocity_pub = n.advertise<geometry_msgs::Twist>("/robot/cmd_vel", 1000);
  ros::Subscriber sensor_sub = n.subscribe("/robot/base_scan", 1000, sensorCallback);

  ros::Rate loop_rate(10);

  /**
   * parameters for PD control
   */
  double kp = 1.0;
  double kd = 7;//4.0 @ 0.5 m/s;
  double dt = 10;

  double prevError = 0; // the previous distance error
  double setDist = 2.0; // how far away from the right wall we want to stay

  while (ros::ok())
  {
    /**
     * The geometry message we want to sent which controls the robot.
     */
    geometry_msgs::Twist twist;
     
    if(distance < 1.5) // stop! backup!
    {
      twist.linear.x = -velocity;
      twist.angular.z = 0;

     if(leftDist > rightDist) // turn left!
     {
       twist.angular.z = 10;
     }

    }
    else // The PD controller, just follow the wall please! 
    {
      double curError = setDist - rightDist; 
      double dError = (curError - prevError) / dt;
      double z = kp*curError + kd*dError;
      
      ROS_INFO("z : %f", z); // log the heading PD wants us to use
      
      twist.linear.x = velocity;
      twist.angular.z = z;
     
      prevError = curError;
    }
    
    /**
     * Publish the gemetry message.
     */
    velocity_pub.publish(twist);
    
    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
