#include "common/grid_utils.hpp"
#include "lcmtypes/pose_xyt_t.hpp"
#include "lcmtypes/robot_path_t.hpp"
#include <cmath>
#include <cstddef>
#include <planning/astar.hpp>
#include <planning/obstacle_distance_grid.hpp>
#include <queue>


std::vector<Node*> get_neighbors(const Node *node, std::vector<std::vector<Node>> nodes, const ObstacleDistanceGrid &grid, const SearchParams& params) {
    const int x_delta[8] = {1, -1, 0, 0, 1, -1, 1, -1};
    const int y_delta[8] = {0, 0, 1, -1, 1, -1, -1, 1};
    std::vector<Node*> neighbors;
    for(int n = 0; n < 8; ++n) {
        cell_t adjacent_cell(node->cell.x + x_delta[n], node->cell.y + y_delta[n]);
        if(grid.isCellInGrid(adjacent_cell.x, adjacent_cell.y) &&
          (grid(adjacent_cell.x, adjacent_cell.y) > params.minDistanceToObstacle)) {
            neighbors.push_back(&nodes[adjacent_cell.x][adjacent_cell.y]);
          }
    }
    return neighbors;
}

double get_h_cost(const cell_t node, const cell_t goal) {
    double dx = std::abs(node.x - goal.x);
    double dy = std::abs(node.y - goal.y);
    return (dx + dy) + (1.414 - 2) * std::min(dx, dy);
}

robot_path_t backtrack_path(Node* end_node, const ObstacleDistanceGrid& distances) {
    robot_path_t path;
    std::vector<Node*> backtrace;
    backtrace.push_back(end_node);
    while (backtrace.back()->parent != nullptr) {
        backtrace.push_back(backtrace.back()->parent);
    }

    for (auto it = backtrace.rbegin(); it != backtrace.rend(); ++it) {
        const Node* node = *it;
        Point<double> grid_pos(node->cell.x, node->cell.y);
        Point<double> global_pos = grid_position_to_global_position(grid_pos, distances);
        pose_xyt_t pose;
        pose.x = global_pos.x;
        pose.y = global_pos.y;
        // TODO: handle theta
        path.path.push_back(pose);    
    }
    path.path_length = path.path.size();
    return path;
}

robot_path_t search_for_path(pose_xyt_t start, 
                             pose_xyt_t goal, 
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params)
{
    Point<int> robotCell = global_position_to_grid_cell(
        Point<float>(start.x, start.y), distances);

    Point<int> goalCell = global_position_to_grid_cell(
        Point<float>(start.x, start.y), distances);

    Node startNode(robotCell); 
    Node endNode(goalCell);

    std::priority_queue<Node*, std::vector<Node*>, CompareNode> pq;
    // std::vector<std::vector<Node>> nodes(distances.widthInCells(), std::vector<Node>(distances.heightInCells()));
    std::vector<std::vector<Node>> nodes(distances.widthInCells(), std::vector<Node>());
    for (int x = 0; x < distances.widthInCells(); x++) {
        nodes[x].reserve(distances.heightInCells());
        for (int y = 0; y < distances.heightInCells(); y++) {
            cell_t cell(x, y);
            nodes[x].emplace_back(cell);
        }
    }

    startNode.g_cost = distances(robotCell.x, robotCell.y);
    startNode.h_cost = get_h_cost(robotCell, goalCell);

    nodes[robotCell.x][robotCell.y] = startNode;
    pq.push(&nodes[robotCell.x][robotCell.y]);

    while (!pq.empty()) {
        Node* current_node = pq.top();
        pq.pop();
        if (*current_node == endNode) {
            robot_path_t path = backtrack_path(current_node, distances);
            path.utime = start.utime;
        }

        std::vector<Node*> neighbors = get_neighbors(current_node, nodes, distances, params);
        for (auto neighbor : neighbors) {

            // // if neighbor hasnt already been enqueued
            // if (std::find_if(nodes.begin(), nodes.end(), [neighbor](Node* node) { return *node == *neighbor; }) != nodes.end()) {
            if (!neighbor->visited) {

                // set previous cost
                cell_t cell = neighbor->cell;
                neighbor->g_cost = current_node->g_cost + (params.maxDistanceWithCost - distances(cell.x, cell.y)) * params.distanceCostExponent;

                // set heuristic cost
                neighbor->h_cost = get_h_cost(cell, goalCell);
                
                // set parent to node
                neighbor->parent = current_node;

                // add to pq
                pq.push(neighbor);
            }
        }
    }

    // if we haven't found a valid path, return an empty path
    robot_path_t path;
    path.utime = start.utime;
    path.path_length = 0;
    return path;
}
