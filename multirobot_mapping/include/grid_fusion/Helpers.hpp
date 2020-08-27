/**
 * @author nconlon
 */


#ifndef HELPERS_HPP
#define HELPERS_HPP

#include <cmath>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/LaserScan.h>
#include "Point.hpp"
#include "Constants.hpp"

namespace robot {

    /**
     * @brief rad2deg
     * @param angle
     * @return degree value in positive ccw frame
     */
    double rad2deg(double angle);

    /**
     * @brief angleDiff
     * @param a1
     * @param a2
     * @return the angle difference between a1 and a1 in positive ccw frame
     */
    double angleDiff(double a1, double a2);

    /**
     * @brief getYaw
     * @param msg_q
     * @return the yaw
     */
    double getYaw(const geometry_msgs::Quaternion& msg_q);

    /**
     * @brief argMin
     * @param phi
     * @param thetas
     * @return the index of the angle in thetas closest to phi
     */
    int argMin(double phi, double* thetas);

    /**
     * Update the likelihood map given the parameters.
     *
     * @brief updateOccupancyGrid
     * @param readings
     * @param likelihoodMap
     * @param position
     * @param heading
     * @param beamWidth
     * @param beamMax
     * @param zMax
     */
    void updateLikelihoodMap(std::vector<float> readings,
                             std::vector<int8_t> &likelihoodMap,
                             Point position,
                             double heading,
                             double beamWidth,
                             double beamMax,
                             float zMax);

    /**
     * @brief getSensorHeadings
     * @param heading
     * @return an array of sensor headings
     */
    double* getSensorHeadings(double heading);

    /**
     * Calculate the log likelihood log(p(m|z,x)/(1-p(m|z,x))). From Table 9.2
     * of Probabilistic Robotics.
     *
     * @brief getInverseSensorModel
     * @param mp
     * @param cp
     * @param zMax
     * @param z
     * @param thetas
     * @param beamWidth
     * @return the log likelihood
     */
    int getInverseSensorModel(Point mp,
                                 Point cp,
                                 float zMax,
                                 std::vector<float> z,
                                 double* thetas,
                                 double beamWidth);

    /**
     * @brief inPerceptualField
     * @param other
     * @param cur
     * @param heading
     * @param beamMax
     * @param zMax
     * @return true if other is in the perceptual field of the robot at position cur
     */
    bool inPerceptualField(Point other,
                           Point cur,
                           double heading,
                           double beamMax,
                           float zMax);

    /**
     * run the occupancy grid mapping algorith from Table 9.1 of Probabilistic Robotics.
     *
     * @brief runOccupancyGridMapping
     * @param msg
     * @param position
     * @param heading
     * @param likelihoodMap
     */
    void runOccupancyGridMapping(const sensor_msgs::LaserScan::ConstPtr& msg,
                                 Point position,
                                 double heading,
                                 std::vector<int8_t> &likelihoodMap);


    std::vector<float> adjustReadings(std::vector<float> readings);
}

#endif // HELPERS_HPP
