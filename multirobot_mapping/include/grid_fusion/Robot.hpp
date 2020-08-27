/**
 * @author nconlon
 */


#ifndef ROBOT_HPP
#define ROBOT_HPP

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int8MultiArray.h>

#include <iostream> //I/O
#include <string>
#include <memory>   //shared pointers

#include "Point.hpp"
#include "Helpers.hpp"

using std::cout;
using std::cerr;
using std::endl;
using std::string;

namespace robot
{
    class Robot
    {
        public:
            Robot(ros::NodeHandle nhPtr,
                  const char* laserScanSubTopic,
                  const char* odometrySubTopic,
                  const char* twistPubTopic,
                  const char* gridPubTopic,
                  const char *arrayPubTopic);

            /**
             * Callback for the LaserScan message
             *
             * @brief laserScanCallback
             * @param msg
             */
            void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

            /**
             * Callback for the Odometry message
             *
             * @brief odometryCallback
             * @param msg
             */
            void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);

            ~Robot();

        private:
            // pubs and subs
            ros::Subscriber laserScanSub;
            ros::Subscriber odometrySub;
            ros::Publisher twistPub;
            ros::Publisher occupancyGridMapPub;
            ros::Publisher logLikelihoodMapSub;

            // messages to publish
            geometry_msgs::Twist m_vel;
            std_msgs::Int8MultiArray logLikelihoodMap;
            nav_msgs::OccupancyGrid occupancyGridMap;

            // robot current state
            volatile double xPos;
            volatile double yPos;
            volatile double heading;
            bool moving;
            int timeStep;

            // initialize the node
            void init();

            // move the robot
            void runWallFollowing(const sensor_msgs::LaserScan::ConstPtr& msg);
            void runRandomWalk(const sensor_msgs::LaserScan::ConstPtr& msg);

            // detect any collitions
            bool collision(double x, double y);
    };
}

#endif // ROBOT_HPP

