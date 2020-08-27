/**
 * @author nconlon
 */

#include "grid_fusion/Robot.hpp"

using namespace robot;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Robot_Runner");

    ros::NodeHandle nh;

    if(argc != 4)
    {
        std::cout << "Wrong number of startup args! argc == " << argc << std::endl;
        return EXIT_FAILURE;
    }

    std::string robot = argv[1]; robot += "base_scan";
    std::string stage = argv[2]; stage += "base_pose_ground_truth";
    std::string cmd_vel = argv[3]; cmd_vel += "cmd_vel";

    std::cout << "Starting " << argv[1] << std::endl;
    std::cout << "Sensor in: " << robot << std::endl;
    std::cout << "State in: " << stage << std::endl;
    std::cout << "Command out: " << cmd_vel << std::endl;

    Robot* ptr = new Robot(nh,
                           robot.c_str(),//"/robot/base_scan",
                           stage.c_str(),//"stage/base_pose_ground_truth",
                           cmd_vel.c_str(),//"/robot/cmd_vel",
                           "/map",
                           "/array");

    ros::spin();

    return EXIT_SUCCESS;
}

