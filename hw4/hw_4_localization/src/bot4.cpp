#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include <sstream>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"
#include <stdlib.h>

const int HISTOGRAM_SIZE = 600;
double histogram[HISTOGRAM_SIZE];

/**
 * Wall-Door sensor callback
 */
void sensorCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard : [%s]", msg->data.c_str());
  ROS_INFO("I heard : [%f]", histogram[154]);
}

/**
 * Initialize the histogram to a uniform distribution.
 */
void initHistogram()
{
  for(int i=0; i< HISTOGRAM_SIZE; i++)
  {
    histogram[i]=1/600.0;
  }
}

void updateHistogram(int position)
{
  for(int i=0; i<HISTOGRAM_SIZE; i++)
  {
    histogram[i] = 1/600.0;
  }

  for(int i=position-5; i<position+5; i++)
  {
    if(i > 0 && i < HISTOGRAM_SIZE)
      histogram[i]=rand();
  }

  ROS_INFO("position: %d", position);
}


/**
 * Simple node using localization in a 2d world 
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "bot4");

  ros::NodeHandle n;

  /**
   * Setup the publisher and subscriber
   */
  ros::Publisher velocity_pub = n.advertise<geometry_msgs::Twist>("/robot/cmd_vel", 1000);
  ros::Subscriber sensor_sub = n.subscribe("/robot/wall_door_sensor", 1000, sensorCallback);
  ros::Publisher histogram_pub = n.advertise<std_msgs::Float32MultiArray>("/robot/histogram", 1000);


  ros::Rate loop_rate(10);
  
  int velocity = 4;// m/s
  int position = 0;
  /**
   * Initialize the histogram
   */
   initHistogram();

  while (ros::ok())
  {

    updateHistogram(position);
    position+=velocity;

    std_msgs::Float32MultiArray array;
 
    array.data.clear();

    for(int i=0; i<HISTOGRAM_SIZE; i++)
    {
      array.data.push_back(histogram[i]);
    }
    
    histogram_pub.publish(array);


    /**
     * The geometry message we want to sent which controls the robot.
     */
    geometry_msgs::Twist twist;
     
    twist.linear.x = velocity;
     
    
    /**
     * Publish the gemetry message.
     */
    velocity_pub.publish(twist);
    
    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
