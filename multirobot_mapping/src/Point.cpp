/**
 * @author nconlon
 */

#include "grid_fusion/Point.hpp"
#include <cmath>

namespace robot
{
    Point::Point(double xval, double yval)
    {
        x = xval;
        y = yval;
    }

    double Point::getX()
    {
        return x;
    }

    double Point::getY()
    {
        return y;
    }

    double Point::getDistance(Point other)
    {
        return getDistance(other.getX(), other.getY());
    }

    double Point::getDistance(double xx, double yy)
    {
        return sqrt((x-xx)*(x-xx) + (y-yy)*(y-yy));
    }
}
