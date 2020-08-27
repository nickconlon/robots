/**
 * @author nconlon
 */

#ifndef POINT_HPP
#define POINT_HPP

namespace robot {

    class Point
    {
    public:
        Point(double x, double y);

        /**
         * @brief getX
         * @return the x position
         */
        double getX();

        /**
         * @brief getY
         * @return the y position
         */
        double getY();

        /**
         * @brief getDistance
         * @param other
         * @return the distance from this point to other
         */
        double getDistance(Point other);

        /**
         * @brief getDistance
         * @param x
         * @param y
         * @return the distance from this point to the (x,y) location
         */
        double getDistance(double x, double y);


    private:
        double x;
        double y;

    };

}


#endif // POINT_HPP

