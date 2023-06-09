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
    
    std::cout << "Commanding robot to drive around 1m square " << numTimes << " times.\n";
    
    robot_path_t path;
    path.path.resize(numTimes * 4);

    int neg = -1;
    
    pose_xyt_t nextPose;
    float dist = 1;
    
    nextPose.x = dist;//1.0f;
    nextPose.y = 0.0f;//0.0f;
    nextPose.theta = M_PI_2 * neg;
    for(int n = 0; n < numTimes; ++n)
    {
        path.path[4*n] = nextPose;
    }
    
    nextPose.x = dist;
    nextPose.y = dist*neg;
    nextPose.theta = M_PI;//0;
    for(int n = 0; n < numTimes; ++n)
    {
        path.path[4*n + 1] = nextPose;
    }
    
    nextPose.x = 0.0f;
    nextPose.y = dist*neg;
    nextPose.theta = -M_PI;
    for(int n = 0; n < numTimes; ++n)
    {
        path.path[4*n + 2] = nextPose;
    }
    
    nextPose.x = 0.0f;
    nextPose.y = 0.0f;
    nextPose.theta = -M_PI_2*neg;
    for(int n = 0; n < numTimes; ++n)
    {
        path.path[4*n + 3] = nextPose;
    }
    
    // Return to original heading after completing all circuits
    nextPose.theta = 0.0f;
    path.path.push_back(nextPose);
    
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
