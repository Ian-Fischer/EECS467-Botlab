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
{
}


void Mapping::updateMap(const lidar_t& scan, const pose_xyt_t& pose, OccupancyGrid& map)
{
    //////////////// TODO: Implement your occupancy grid algorithm here ///////////////////////
    if(!initialized_) {
        prevPose_= pose;
    }
    MovingLaserScan movingScan(scan, prevPose_, pose);

    for(auto& ray : movingScan) {
        scoreEndpoint(ray, map);
        scoreRay(ray, map);
    }

    initialized_ = true;
    prevPose_ = pose;
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

}
void Mapping::decreaseCellOdds(int x, int y, OccupancyGrid &map) {

}
void Mapping::increaseCellOdds(int x, int y, OccupancyGrid &map) {
    if(std::numeric_limits<CellOdds>::max() - map(x, y) > kHitOdds_) {
        map(x,y) += kHitOdds_;
    } else {
        map(x, y) = std::numeric_limits<CellOdds>::max();
    }

}