#ifndef PLANNING_ASTAR_HPP
#define PLANNING_ASTAR_HPP

#include <lcmtypes/robot_path_t.hpp>
#include <lcmtypes/pose_xyt_t.hpp>

#include <queue>
#include <common/grid_utils.hpp>
#include <planning/obstacle_distance_grid.hpp>

/*
typedef Point<int> cell;

struct Node {
    double h_cost;
    double g_cost;
    Node* parent;
    cell_t cell;
    Node(int a, int b) : h_cost(1.0E16), g_cost(1.0E16), parent(NULL), cell(a, b) {}
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

struct PriorityQueue {
    std::priority_queue<Node*, std::vector<Node*>, CompareNode> Q;
    std::vector<Node*> elements;

    bool empty() {
        return Q.empty();
    }

    bool is_member(Node* n) {
        for(auto &node : elements) {
            if(*n == *node) return true;
        }
        return false;
    }

    Node* get_member(Node* n) {
        for(auto &node : elements) {
            if(*n == *node) return node;
        }
        return nullptr;
    }

    Node* pop() {
        Node* n = Q.top();
        Q.pop();
        int idx = -1;
        for(unsigned i = 0; i < elements.size(); i++) {
            if(*elements[i] == *n) { 
                idx = i; 
                break;
            }
        }
        elements.erase(elements.begin() + idx);
        return n;
    }
}; */

struct Node {
    Node(Point<int> cell) : cell(cell) {}
    Point<int> cell;
    Node* parent = nullptr;
    double h_cost = 1e16;
    double g_cost = 1e16;
    bool visited = false;
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

class ObstacleDistanceGrid;

/**
* SearchParams defines the parameters to use when searching for a path. See associated comments for details
*/
struct SearchParams
{
    double minDistanceToObstacle;   ///< The minimum distance a robot can be from an obstacle before
                                    ///< a collision occurs
                                    
    double maxDistanceWithCost;     ///< The maximum distance from an obstacle that has an associated cost. The planned
                                    ///< path will attempt to stay at least this distance from obstacles unless it must
                                    ///< travel closer to actually find a path
                                    
    double distanceCostExponent;    ///< The exponent to apply to the distance cost, whose function is:
                                    ///<   pow(maxDistanceWithCost - cellDistance, distanceCostExponent)
                                    ///< for cellDistance > minDistanceToObstacle && cellDistance < maxDistanceWithCost
};

std::vector<Node*> get_neighbors(const Node* node, std::vector<std::vector<Node>> &nodes, const ObstacleDistanceGrid& grid, const SearchParams& params);

double get_h_cost(const cell_t node, const cell_t goal);

double get_g_cost(const cell_t cell, const ObstacleDistanceGrid& distances, const SearchParams& params);

robot_path_t backtrack_path(const Node* end_node, const ObstacleDistanceGrid& grid);

/**
* search_for_path uses an A* search to find a path from the start to goal poses. The search assumes a circular robot
* 
* \param    start           Starting pose of the robot
* \param    goal            Desired goal pose of the robot
* \param    distances       Distance to the nearest obstacle for each cell in the grid
* \param    params          Parameters specifying the behavior of the A* search
* \return   The path found to the goal, if one exists. If the goal is unreachable, then a path with just the initial
*   pose is returned, per the robot_path_t specification.
*/
robot_path_t search_for_path(pose_xyt_t start, 
                             pose_xyt_t goal, 
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params);

#endif // PLANNING_ASTAR_HPP
