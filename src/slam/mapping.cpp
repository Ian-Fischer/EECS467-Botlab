#include "common/timestamp.h"
#include "lcmtypes/pose_xyt_t.hpp"
#include <limits>
#include <slam/mapping.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <common/grid_utils.hpp>
#include <numeric>


Mapping::Mapping(float maxLaserDistance, int8_t hitOdds, int8_t missOdds)
: kMaxLaserDistance_(maxLaserDistance)
, kHitOdds_(hitOdds)
, kMissOdds_(missOdds)
, initialized_(false)
{
}


void Mapping::updateMap(const lidar_t& scan, const pose_xyt_t& pose, OccupancyGrid& map)
{
    //////////////// TODO: Implement your occupancy grid algorithm here ///////////////////////
    
    //prevPose_ = pose_xyt_t();
    //prevPose_.utime = utime_now();//12312;
    /*
    if(!initialized_) {
        //prevPose_ = pose;
        prevPose_.utime = 12312;

        //std::cout << pee.utime << std::endl;
        /*
        //return;
    }
    MovingLaserScan movingScan(scan, prevPose_, pose);

    for(auto& ray : movingScan) {
        scoreEndpoint(ray, map);
        scoreRay(ray, map);
    }

    initialized_ = true;
    prevPose_ = pose;
    */

    initialized_ = true;
}

void Mapping::scoreEndpoint(const adjusted_ray_t &ray, OccupancyGrid& map) {
    if(ray.range <= kMaxLaserDistance_) {
        Point<float> rayStart = global_position_to_grid_cell(ray.origin, map);
        Point<int> rayCell;
        rayCell.x = static_cast<int> (ray.range * std::cos(ray.theta) * map.cellsPerMeter() + rayStart.x);
        rayCell.y = static_cast<int> (ray.range * std::sin(ray.theta) * map.cellsPerMeter() + rayStart.y);

        if(map.isCellInGrid(rayCell.x, rayCell.y)) {
            increaseCellOdds(rayCell.x, rayCell.y, map);
        }
    }
}
void Mapping::scoreRay(const adjusted_ray_t &ray, OccupancyGrid& map) {
    
    
    bool maxDist = false;
    Point<int> rayCell;
    Point<float> rayStart = global_position_to_grid_cell(ray.origin, map);
    if(ray.range <= kMaxLaserDistance_) {
        rayCell.x = static_cast<int> (ray.range * std::cos(ray.theta) * map.cellsPerMeter() + rayStart.x);
        rayCell.y = static_cast<int> (ray.range * std::sin(ray.theta) * map.cellsPerMeter() + rayStart.y);
    }
    else {
        rayCell.x = static_cast<int> (kMaxLaserDistance_ * std::cos(ray.theta) * map.cellsPerMeter() + rayStart.x);
        rayCell.y = static_cast<int> (kMaxLaserDistance_ * std::sin(ray.theta) * map.cellsPerMeter() + rayStart.y);
        maxDist = true;
    }
    int x0 = ray.origin.x;
    int y0 = ray.origin.y;
    int x1 = rayCell.x;
    int y1 = rayCell.y;
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    bool sx = x0 < x1 ? 1 : -1;
    bool sy = y0 < y1 ? 1 : -1;
    int err = dx - dy;
    int x = x0;
    int y = y0;

    while (x != x1 || y != y1) {
        decreaseCellOdds(x, y, map);
        int e2 = 2 * err;
        if (e2 >= -dy) {
            err -= dy;
            x += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y += sy;
        }
    }
    if (maxDist) {
        decreaseCellOdds(x, y, map);
    }

    

}
void Mapping::decreaseCellOdds(int x, int y, OccupancyGrid &map) {
    if (map(x, y) - std::numeric_limits<CellOdds>::min() > kHitOdds_) {
        map(x, y) -= kHitOdds_;
    } else {
        map(x, y) = std::numeric_limits<CellOdds>::min();
    }
}
void Mapping::increaseCellOdds(int x, int y, OccupancyGrid &map) {
    if(std::numeric_limits<CellOdds>::max() - map(x, y) > kHitOdds_) {
        map(x,y) += kHitOdds_;
    } else {
        map(x, y) = std::numeric_limits<CellOdds>::max();
    }

}