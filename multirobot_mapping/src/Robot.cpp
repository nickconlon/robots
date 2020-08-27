/**
 * @author nconlon
 */

#include "grid_fusion/Robot.hpp"

namespace robot
{
    Robot::Robot(ros::NodeHandle nhPtr,
                 const char* laserScanSubTopic,
                 const char* odometrySubTopic,
                 const char* twistPubTopic,
                 const char* gridPubTopic,
                 const char* arrayPubTopic)
    {
        laserScanSub = nhPtr.subscribe<sensor_msgs::LaserScan>(laserScanSubTopic, 1, &Robot::laserScanCallback, this);
        odometrySub = nhPtr.subscribe<nav_msgs::Odometry>(odometrySubTopic, 1, &Robot::odometryCallback, this);
        twistPub = nhPtr.advertise<geometry_msgs::Twist>(twistPubTopic, 1);
        occupancyGridMapPub = nhPtr.advertise<nav_msgs::OccupancyGrid>(gridPubTopic, 1);
        logLikelihoodMapSub = nhPtr.advertise<std_msgs::Int8MultiArray>(arrayPubTopic, 1);

        xPos = yPos = 0;
        timeStep = 0;
        moving = false;
        init();
        srand(RAND_SEED);
    }


    void Robot::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
        if(moving)
        {
            //runWallFollowing(msg);
            runRandomWalk(msg);

            runOccupancyGridMapping(msg, Point(xPos+SCALE/2.0, yPos+SCALE/2.0),
                                    heading, logLikelihoodMap.data);

            if(timeStep % SEND_FREQUENCY == 0)
            {
                logLikelihoodMapSub.publish(logLikelihoodMap);
            }
            twistPub.publish(m_vel);
        }

        timeStep++;
    }



    void Robot::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        moving = true;
        xPos = msg->pose.pose.position.x+(MAX_X/SCALE)/2;
        yPos = msg->pose.pose.position.y+(MAX_Y/SCALE)/2;
        heading = getYaw(msg->pose.pose.orientation);
    }

    void Robot::runWallFollowing(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
        m_vel.linear.x = MAX_SPEED;
        m_vel.angular.z = ZERO_HEADING;

        if(msg->ranges[2] < 2)
        {
            m_vel.linear.x = ZERO_SPEED;
            m_vel.angular.z = MAX_HEADING;
        }
        else if(msg->ranges[1] < msg->ranges[3])
        {
            m_vel.angular.z = 2;
        }
        else
        {
            m_vel.angular.z = -2;
        }
    }

    void Robot::runRandomWalk(const sensor_msgs::LaserScan::ConstPtr& msg)
    {

        m_vel.linear.x = MAX_SPEED;
        m_vel.angular.z = ZERO_HEADING;

        if(msg->ranges[2] < 2)
        {
           double r = (double)rand()/(double)RAND_MAX; // add a bit of randomness

           if(msg->ranges[1] <  msg->ranges[3] )
           {
               m_vel.angular.z = 2+2*r;
           }
           else
           {
               m_vel.angular.z = -2-2*r;
           }
           m_vel.linear.x = ZERO_SPEED;
        }
    }

    void Robot::init()
    {
        //Create a header, populate the fields.
        std_msgs::Header header = std_msgs::Header();
        header.stamp = ros::Time::now();
        header.frame_id = "/map";

        //Create the map meta data
        nav_msgs::MapMetaData metaD = nav_msgs::MapMetaData();
        metaD.map_load_time = ros::Time::now();
        metaD.resolution = RESOLUTION;
        metaD.width = MAX_X;
        metaD.height = MAX_Y;

        occupancyGridMap = nav_msgs::OccupancyGrid();
        occupancyGridMap.header = header;
        occupancyGridMap.info = metaD;

        for(int i = 0; i < MAX_X*MAX_Y; i++)
        {
            occupancyGridMap.data.push_back(-1);
            logLikelihoodMap.data.push_back(0);
        }
    }

    Robot::~Robot()
    {
        ;
    }
}

