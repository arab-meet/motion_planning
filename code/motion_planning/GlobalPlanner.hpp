#ifndef GLOBAL_PLANNER_H
#define GLOBAL_PLANNER_H

#include <vector>
#include <memory> // for std::unique_ptr
#include <stdexcept> // for std::runtime_error
#include <utility> // for std::pair
#include <iostream> /

// Define an enum for selecting algorithms
enum class PlannerType {
    AStar,
    Dijkstra,
};

class GlobalPlanner {
public:
    
    GlobalPlanner();

    // Set the path planning algorithm type
    void setAlgorithm(PlannerType type);

    // Plan a path using the selected algorithm
    std::vector<std::pair<int, int>> planPath(
        const std::pair<int, int>& start, 
        const std::pair<int, int>& goal
    );

private:
    std::vector<std::pair<int, int>> aStarPath(
        const std::pair<int, int>& start, 
        const std::pair<int, int>& goal
    );

    std::vector<std::pair<int, int>> dijkstraPath(
        const std::pair<int, int>& start, 
        const std::pair<int, int>& goal
    );


    // Current algorithm type
    PlannerType currentAlgorithm;
};


#endif // GLOBAL_PLANNER_H
