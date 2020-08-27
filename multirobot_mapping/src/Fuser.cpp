/**
 * @author nconlon
 */

#include "grid_fusion/Fuser.hpp"

namespace robot
{
    Fuser::Fuser(ros::NodeHandle nhPtr,
                 const char *gridPubTopic,
                 const char *arraySubTopic)
    {
        arraySub = nhPtr.subscribe<std_msgs::Int8MultiArray>(arraySubTopic, 5, &Fuser::fusionCallback, this);

        gridPub = nhPtr.advertise<nav_msgs::OccupancyGrid>(gridPubTopic, 1);

        init();
    }

    void Fuser::fusionCallback(const std_msgs::Int8MultiArray::ConstPtr& msg)
    {
        fuse(msg);
        gridPub.publish(m_map);
    }


    void Fuser::fuse(const std_msgs::Int8MultiArray::ConstPtr& msg)
    {
        switch(FUSION)
        {
        case IOP:
            runIOP(msg);
            break;
         case MAX :
            runMax(msg);
            break;
        case IMSF:
            runInt(msg);
            break;
        }
    }

    void Fuser::runMax(const std_msgs::Int8MultiArray::ConstPtr& msg)
    {
        for(int i=0; i<msg->data.size(); i++)
        {
           float probNew = 1-(1.0/(1+exp(float(msg->data.at(i)))));
           float probOld = float_grid.at(i);

           float prob = fuseUsingMaximization(probNew, probOld);

           float_grid.at(i) = prob;

           if(prob < EMPTY_THRESHOLD_FLOAT)
               m_map.data.at(i) = 0; // empty
           else if(prob > OCCUPIED_THRESHOLD_FLOAT)
               m_map.data.at(i) = 100; // occupied
           else
               m_map.data.at(i) = -1; // unknown
        }
    }

    void Fuser::runIOP(const std_msgs::Int8MultiArray::ConstPtr& msg)
    {
        for(int i=0; i<msg->data.size(); i++)
        {
           float probNew = 1-(1.0/(1+exp(float(msg->data.at(i)))));
           float probOld = float_grid.at(i);

           float prob = fuseUsingIndependentOpinionPool(probNew, probOld);

           float_grid.at(i) = prob;

           if(prob < EMPTY_THRESHOLD_FLOAT)
               m_map.data.at(i) = 0; // empty
           else if(prob > OCCUPIED_THRESHOLD_FLOAT)
               m_map.data.at(i) = 100; // occupied
           else
               m_map.data.at(i) = -1; // unknown
        }
    }

    void Fuser::runInt(const std_msgs::Int8MultiArray::ConstPtr& msg)
    {
        for(int i=0; i<msg->data.size(); i++)
        {
           int16_t indexNew = msg->data.at(i);
           int16_t indexOld = int_grid.at(i);

           int16_t prob = fuseUsingIntegerArithmetic(indexNew, indexOld);

           int_grid.at(i) = prob;

           if(prob < EMPTY_THRESHOLD_INT) // -3
               m_map.data.at(i) = 0; // empty
           else if(prob > OCCUPIED_THRESHOLD_INT) // +5
               m_map.data.at(i) = 100; // occupied
           else
               m_map.data.at(i) = -1; // unknown
        }
    }

    float Fuser::fuseUsingMaximization(float probA, float probB)
    {
        if(probA < 0.5 || probB < 0.5)
        {
            return std::min(probA, probB);
        }
        else if(probA > 0.5 || probB > 0.5)
        {
            return std::max(probA, probB);
        }
        else
        {
            return 0.5;
        }
    }

    float Fuser::fuseUsingIndependentOpinionPool(float probA, float probB)
    {
        float result =  (probA*probB)/(probA*probB + (1-probA)*(1-probB));
        return result;
    }

    int16_t Fuser::fuseUsingIntegerArithmetic(int16_t m, int16_t n)
    {
        // correct for quantized inverse sensor model
        m = m < 0 ? -10 : m == 0 ? 0 : 10;
        n = n < 0 ? -10 : n == 0 ? 0 : 10;

        int q = 2;
        int result = m;

        if(m <= 0 && n <= 0)
        {
            result = round(m+n-q*m*n);
        }
        else if(m >= 0 && n >= 0)
        {
            result = round(m+n+q*m*n);
        }
        else if(n <= 0 && m >=0 && abs(n) <= abs(m))
        {
            result = round((m+n)/(1-q*n));
        }
        else if(n <= 0 && m <= 0 && abs(n) > abs(m)) // TODO check on this
        {
            result = round((m+n)/(q*m+1));
        }

        return result;
    }


    void Fuser::init()
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

        m_map = nav_msgs::OccupancyGrid();
        m_map.header = header;
        m_map.info = metaD;

        for(int i = 0; i < MAX_X*MAX_Y; i++)
        {
            float_grid.push_back(0.5);
            int_grid.push_back(0);
            m_map.data.push_back(-1);
        }
    }
}
