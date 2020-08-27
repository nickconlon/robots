/**
 * @author nconlon
 */


#ifndef FUSER_HPP
#define FUSER_HPP

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
#include <algorithm>

#include "Constants.hpp"

using std::cout;
using std::cerr;
using std::endl;
using std::string;

namespace robot {

    class Fuser
    {
    public:
        Fuser(ros::NodeHandle nhPtr,
              const char* gridPubTopic,
              const char* floatArraySubTopic);

        /**
         * Callback for the Float 32 array message
         *
         * @brief arrayCallback
         * @param msg
         */
        void fusionCallback(const std_msgs::Int8MultiArray::ConstPtr& msg);


        ~Fuser();

    private:
        // pubs and subs
        ros::Subscriber arraySub;
        ros::Subscriber intArraySub;
        ros::Publisher gridPub;

        // the fused occupancy grid to publish
        nav_msgs::OccupancyGrid m_map;

        // local fused likelihood map
        std::vector<float> float_grid;
        std::vector<int16_t> int_grid;

        // fuse msg with grid
        void fuse(const std_msgs::Int8MultiArray::ConstPtr& msg);

        // run fusers
        void runMax(const std_msgs::Int8MultiArray::ConstPtr& msg);
        void runIOP(const std_msgs::Int8MultiArray::ConstPtr& msg);
        void runInt(const std_msgs::Int8MultiArray::ConstPtr& msg);

        // fuse based on some different methos
        float fuseUsingMaximization(float a, float b);
        float fuseUsingIndependentOpinionPool(float a, float b);
        int16_t fuseUsingIntegerArithmetic(int16_t m, int16_t n);

        // initialize the node
        void init();
    };

}
#endif // FUSER_HPP
