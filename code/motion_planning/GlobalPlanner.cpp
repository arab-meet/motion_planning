#include "GlobalPlanner.hpp"
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <cmath>
#include <limits>
#include <algorithm>
#include <iostream>


// Custom hash function for std::pair<int, int>
struct PairHash {
    std::size_t operator()(const std::pair<int, int>& p) const {
        return std::hash<int>()(p.first) ^ std::hash<int>()(p.second);
    }
};




GlobalPlanner::GlobalPlanner() : currentAlgorithm(PlannerType::AStar) {}

// Set the path planning algorithm type
void GlobalPlanner::setAlgorithm(PlannerType type) {
    currentAlgorithm = type;
}

// Plan a path using the selected algorithm
std::vector<std::pair<int, int>> GlobalPlanner::planPath(
    const std::pair<int, int>& start, 
    const std::pair<int, int>& goal
) {
    switch (currentAlgorithm) {
        case PlannerType::AStar:
            return aStarPath(start, goal);
        case PlannerType::Dijkstra:
            return dijkstraPath(start, goal);

        default:
            throw std::runtime_error("Unknown algorithm type");
    }
}

// Implement the A* algorithm 

// compute the heuristics : 

int heuristic (const std::pair<int ,int >&a , const std::pair<int , int >b)
{
    // manhaten distance 
    return std::abs(a.first-b.first)+std::abs(a.second-b.second);
}

std::vector<std::pair<int, int>> GlobalPlanner::aStarPath(
    const std::pair<int, int>& start, 
    const std::pair<int, int>& goal
) {
    std::vector<std::pair<int, int>> path;

    // fscore , gscore 
    std::unordered_set<std::pair<int, int>, PairHash> openSet = {start};

    std::unordered_map<std::pair<int ,int>, int,PairHash>fscore;
    std::unordered_map<std::pair<int ,int>, int , PairHashash>gscore;

    gscore[start]=0;
    fscore[start] = heuristic(start, goal);



    std::cout << "Running A* algorithm..." << std::endl;
    return path;
}

// Implement Dijkstra's algorithm 
std::vector<std::pair<int, int>> GlobalPlanner::dijkstraPath(
    const std::pair<int, int>& start, 
    const std::pair<int, int>& goal
) {
    std::vector<std::pair<int, int>> path;
    // Your Dijkstra pathfinding logic goes here
    std::cout << "Running Dijkstra's algorithm..." << std::endl;
    return path;
}


