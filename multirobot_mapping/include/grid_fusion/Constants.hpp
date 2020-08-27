/**
 * @author nconlon
 */

#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

//TODO pull these into properties
namespace robot {
    // map constants
    const int MAX_X = 300;//1200;//0; //scale*map_size_metersX
    const int MAX_Y = 300;//320;//0; //scale*map_size_metersY
    const double RESOLUTION = 0.1; // 1/scale
    const double SCALE = 10.0; //30x30 meter map
    const int SEND_FREQUENCY = 10; // send the map once per sensor message received

    // algorithm constants
    enum FUSION_TYPE { MAX, IOP, IMSF };
    const FUSION_TYPE FUSION = IMSF;
    const float EMPTY_THRESHOLD_FLOAT = 1-(1.0/(1+exp(-20)));
    const float OCCUPIED_THRESHOLD_FLOAT = 1-(1.0/(1+exp(20)));
    const int EMPTY_THRESHOLD_INT = -3;
    const int OCCUPIED_THRESHOLD_INT = 5;

    // movement constants
    enum ROBOT_TYPE { CLEAN, DIRTY };
    const ROBOT_TYPE ROBOT = DIRTY;
    const unsigned int RAND_SEED = 1;
    const float MAX_SPEED = 0.5;
    const float ZERO_SPEED = 0.0;
    const float MAX_HEADING = 10.0;
    const float ZERO_HEADING = 0.0;

}

#endif // CONSTANTS_HPP

