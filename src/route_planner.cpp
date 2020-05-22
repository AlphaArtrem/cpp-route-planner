#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes in the RoutePlanner's start_node and end_node attributes.
    start_node = &(m_Model.FindClosestNode(start_x, start_y));
    end_node = &(m_Model.FindClosestNode(end_x, end_y));
}


// Get the h value or manhattan distance of current node from the end node
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node -> distance(*end_node);
}


// Get the neighbours for current node and calculate their h and g value
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node -> FindNeighbors();
    for(RouteModel::Node *node : current_node -> neighbors){
        node -> parent = current_node;
        node -> h_value = CalculateHValue(node);
        node -> g_value = current_node -> g_value + current_node -> distance(*node);
        open_list.push_back(node);
        node -> visited = true;
    }
}


// Get the next optimal node to explore
RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(open_list.begin(), open_list.end(), [](const RouteModel::Node *a, const RouteModel::Node *b){
    float aScore = a -> h_value + a -> g_value;
    float bScore = b -> h_value + b -> g_value;
    return aScore > bScore;
    });
    
    RouteModel::Node *nextNode = open_list.back();
    open_list.pop_back();
    return nextNode;
}


// ConstructFinalPath method to return the final path found from your A* search.
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    std::cout<<"\nConstructing Path..\n";
    while (current_node -> parent != nullptr)
    {
        path_found.push_back(*current_node);
        distance += current_node -> distance(*(current_node -> parent));
        current_node = current_node -> parent;
    }
    
    path_found.push_back(*current_node);
    std::reverse(path_found.begin(), path_found.end());
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


// A* Search algorithm
void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
    start_node -> visited = true;
    open_list.push_back(start_node);
    
    std::cout<<"\nFinding Path...";
    while (!open_list.empty())
    {
        current_node = NextNode();
        if(current_node -> distance(*end_node) == 0)
        {
            m_Model.path = ConstructFinalPath(current_node);
            return;
        }
        else
        {
            AddNeighbors(current_node);
        }
    }
}