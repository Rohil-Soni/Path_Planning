#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <map>
#include<algorithm>
using namespace std;
// Represents a point in the grid
struct Point {
    int x, y;

    bool operator==(const Point& other) const {
        return x == other.x && y == other.y;
    }

    bool operator<(const Point& other) const {
        if (x != other.x) return x < other.x;
        return y < other.y;
    }
};

// Represents a node in the A* search
struct Node {
    Point p;
    double g_cost; // Cost from start to this node
    double h_cost; // Heuristic cost from this node to goal
    double f_cost; // g_cost + h_cost
    Point parent;

    // Custom comparator for priority queue (min-heap based on f_cost)
    bool operator>(const Node& other) const {
        return f_cost > other.f_cost;
    }
};

// Heuristic function (Manhattan distance)
double calculate_heuristic(Point p1, Point p2) {
    return std::abs(p1.x - p2.x) + std::abs(p1.y - p2.y);
}

// Check if a point is valid (within grid bounds and not an obstacle)
bool is_valid(Point p, int rows, int cols, const std::vector<std::vector<int>>& grid) {
    return p.x >= 0 && p.x < rows && p.y >= 0 && p.y < cols && grid[p.x][p.y] == 0; // 0 for empty, 1 for obstacle
}

// Find path using A*
std::vector<Point> a_star_search(Point start, Point goal, int rows, int cols, const std::vector<std::vector<int>>& grid) {
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_list;
    std::map<Point, double> g_costs;
    std::map<Point, Point> parents;

    Node start_node = {start, 0.0, calculate_heuristic(start, goal), calculate_heuristic(start, goal), {-1, -1}};
    open_list.push(start_node);
    g_costs[start] = 0.0;

    int dx[] = {-1, 1, 0, 0}; // Possible movements (up, down, left, right)
    int dy[] = {0, 0, -1, 1};

    while (!open_list.empty()) {
        Node current = open_list.top();
        open_list.pop();

        if (current.p == goal) {
            // Reconstruct path
            std::vector<Point> path;
            Point curr_path_point = goal;
            while (!(curr_path_point == start)) {
                path.push_back(curr_path_point);
                curr_path_point = parents[curr_path_point];
            }
            path.push_back(start);
            std::reverse(path.begin(), path.end());
            return path;
        }

        for (int i = 0; i < 4; ++i) {
            Point neighbor_p = {current.p.x + dx[i], current.p.y + dy[i]};

            if (is_valid(neighbor_p, rows, cols, grid)) {
                double new_g_cost = current.g_cost + 1.0; // Assuming uniform cost for movement

                if (g_costs.find(neighbor_p) == g_costs.end() || new_g_cost < g_costs[neighbor_p]) {
                    g_costs[neighbor_p] = new_g_cost;
                    parents[neighbor_p] = current.p;
                    double h_cost = calculate_heuristic(neighbor_p, goal);
                    double f_cost = new_g_cost + h_cost;
                    open_list.push({neighbor_p, new_g_cost, h_cost, f_cost, current.p});
                }
            }
        }
    }

    return {}; // No path found
}

int main() {
    int rows = 5, cols = 5;
    std::vector<std::vector<int>> grid = {
        {0, 0, 0, 0, 0},
        {0, 1, 0, 1, 0},
        {0, 0, 0, 0, 0},
        {0, 1, 0, 1, 0},
        {0, 0, 0, 0, 0}
    };

    Point start = {0, 0};
    Point goal = {4, 4};

    std::vector<Point> path = a_star_search(start, goal, rows, cols, grid);

    if (!path.empty()) {
        std::cout << "Path found:" << std::endl;
        for (const auto& p : path) {
            std::cout << "(" << p.x << ", " << p.y << ") -> ";
        }
        std::cout << "End" << std::endl;
    } else {
        std::cout << "No path found." << std::endl;
    }

    return 0;
}