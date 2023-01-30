#include <common/lcm_config.h>
#include <mbot/mbot_channels.h>
#include <lcmtypes/robot_path_t.hpp>
#include <lcm/lcm-cpp.hpp>
#include <iostream>
#include <unistd.h>
#include <cmath>

int main(int argc, char** argv)
{std::cout << "here" <<std::endl;
    int numTimes = 1;
    
    if(argc > 1)
    {
        numTimes = std::atoi(argv[1]);
    }
    
    std::cout << "maze" << numTimes << " times.\n";
    
    robot_path_t path;
    path.path.resize(9);
    

    pose_xyt_t nextPose ;
    nextPose.x = 0.61f;
    nextPose.y = 0.0f;
    nextPose.theta = M_PI_2;
    path.path[0] = nextPose;
    
    nextPose.x = 0.61f;
    nextPose.y = 0.61f;
    nextPose.theta = 0;
    path.path[1] = nextPose;

    nextPose.x = 2*0.61f;
    nextPose.y = 0.61f;
    nextPose.theta = -M_PI_2;
    path.path[2] = nextPose;

    nextPose.x = 2*0.61f;
    nextPose.y = 0.61f - 1.22f;
    nextPose.theta = 0;
    path.path[3] = nextPose;

    // duplication

    nextPose.x = 2*0.61f + 0.61f;
    nextPose.y = 0.61f - 1.22f;
    nextPose.theta = M_PI_2;
    path.path[4] = nextPose;

    nextPose.x = 2*0.61f + 0.61f;
    nextPose.y = 0.61f;
    nextPose.theta = 0;
    path.path[5] = nextPose;

    nextPose.x = 2*0.61f + 2*0.61f;
    nextPose.y = 0.61f;
    nextPose.theta = -M_PI_2;
    path.path[6] = nextPose;

    nextPose.x = 2*0.61f + 2*0.61f;
    nextPose.y = 0;
    nextPose.theta = 0;
    path.path[7] = nextPose;

    nextPose.x = 2*0.61f + 3*0.61f;
    nextPose.y = 0;
    nextPose.theta = 0;
    path.path[8] = nextPose;

    /*
    nextPose.x = 0.61f + 2*0.61f;
    nextPose.y = 0.0f + 0.61f - 1.22f;
    nextPose.theta = M_PI_2;
    path.path[4] = nextPose;
    
    nextPose.x = 0.61f + 2 *0.61f;
    nextPose.y = 0.61f + 0.61f - 1.22f;
    nextPose.theta = 0;
    path.path[5] = nextPose;

    nextPose.x = 2*0.61f + 2*0.61f;
    nextPose.y = 0.61f + 0.61f - 1.22f;
    nextPose.theta = -M_PI_2;
    path.path[6] = nextPose;

    nextPose.x = 2*0.61f + 2*0.61f;
    nextPose.y = 0.61f - 1.22f + 0.61f - 1.22f;
    nextPose.theta = 0;
    path.path[7] = nextPose;
    */ 
    
    nextPose.x = 0.0f;
    nextPose.y = 0.0f;
    nextPose.theta = 0.0f;
    path.path.insert(path.path.begin(), nextPose);
    
    path.path_length = path.path.size();
    
    lcm::LCM lcmInstance(MULTICAST_URL);
	std::cout << "publish to: " << CONTROLLER_PATH_CHANNEL << std::endl;
    lcmInstance.publish(CONTROLLER_PATH_CHANNEL, &path);
    sleep(1);

    return 0;
}
