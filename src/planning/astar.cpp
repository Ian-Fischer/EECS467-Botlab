#include "common/grid_utils.hpp"
#include <planning/astar.hpp>
#include <planning/obstacle_distance_grid.hpp>
#include <queue>

struct Node {
    Node(Point<int> cell) : cell(cell) {}
    Point<int> cell;
    Node* parent = nullptr;
    double h_cost = 1e16;
    double g_cost = 1e16;
    double f_cost(void) const { return h_cost + g_cost; }
    bool operator==(const Node& rhs) const {
        return (cell==rhs.cell);
    }
};

struct CompareNode {
    bool operator() (Node* n1, Node* n2) {
        if(n1->f_cost() == n2->f_cost()) return n1->h_cost > n2->h_cost;
        return n1->f_cost() > n2->f_cost();
    }
};

robot_path_t search_for_path(pose_xyt_t start, 
                             pose_xyt_t goal, 
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params)
{
    ////////////////// TODO: Implement your A* search here //////////////////////////
    //global_position_to_grid_cell(start, distances);

    Point<int> robotCell = global_position_to_grid_cell(
        Point<float>(start.x, start.y), distances);

    Point<int> goalCell = global_position_to_grid_cell(
        Point<float>(start.x, start.y), distances);

    Node startNode(robotCell); 
    Node endNode(goalCell);

    std::priority_queue<Node*, std::vector<Node*>, CompareNode> pq;
    std::vector<Node> nodes;

    
    robot_path_t path;
    path.utime = start.utime;
    path.path.push_back(start);    
    path.path_length = path.path.size();
    return path;
}
