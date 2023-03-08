#include <common/grid_utils.hpp>
#include <common/timestamp.h>
#include <planning/motion_planner.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcm/lcm-cpp.hpp>
#include <common/getopt.h>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/median.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <chrono>
#include <thread>
#include <cassert>
#include <cmath>
#include <fstream>
#include <iostream>
#include <iterator>
#include <map>


// LCM DEFS TO SEND DATA TO A BOTGUI 
#define MULTICAST_URL "udpm://239.255.76.67:7667?ttl=2"
#define MAP_2GUI_CHANNEL "SLAM_MAP"
#define PATH_CHANNEL "CONTROLLER_PATH"

// ARGUMENT VARIABLES
bool useGui;                // Global Variable for using GUI 
bool animatePath;
int repeatTimes;            // Global Variable that sets number of repeat times, obtained from 
int pauseTime;           // Time to wait between executions of different cases
int selected_test;

// Setup Lcm
lcm::LCM lcmConnection(MULTICAST_URL); 

int main(int argc, char** argv)
{
    OccupancyGrid grid;
    grid.loadFromFile("../data/competition_map.map");

    pose_xyt_t start;
    pose_xyt_t goal;
    start.theta = 0.0;
    start.x = 0.0;
    start.y = 0.0;

    goal.theta = 0.0;
    goal.x = 0.41;
    goal.y = -1.32;
    
    MotionPlannerParams motion_params;
    SearchParams search_params{0.15, 100, 0.1};
    MotionPlanner planner(motion_params, search_params);
    
    planner.setMap(grid);
    
    robot_path_t path;
    path = planner.planPath(start, goal);
    lcmConnection.publish(PATH_CHANNEL, &path);
    
    if(path.path_length > 1)
    {
        std::cout << "path planned successfully" << std::endl;
    }
    else
    {
        std::cout << "path failed to plan" << std::endl;
    }

    path = planner.planPath(goal, start);
    lcmConnection.publish(PATH_CHANNEL, &path);
    
    if(path.path_length > 1)
    {
        std::cout << "path planned successfully" << std::endl;
    }
    else
    {
        std::cout << "path failed to plan" << std::endl;
    }
    return 0;
}
