#include <iostream>    // <iostream> provides std::cout, std::cin and other I/O streams
#include <vector>      // <vector> provides std::vector container
#include <queue>       // <queue> provides std::priority_queue (often used in Dijkstra)
#include <climits>     // <climits> defines integer limit macros like INT_MAX
using namespace std;   // bring std:: names into global namespace (convenience, but can pollute)

vector<vector<vector<int>>> constructAdj(vector<vector<int>>& edges, int V) { 
    // function: constructAdj
    // returns adjacency list: for each vertex u, a vector of {v, wt} pairs
    // params:
    //   edges - list of edges where each edge is {u, v, wt}
    //   V     - number of vertices

    // adj[u] = list of {v, wt}
    vector<vector<vector<int>>> adj(V); // create adj vector with V empty lists

    for (const auto &edge : edges) {    // iterate over all edges (edge is a vector<int> of size 3)
        int u = edge[0];                 // u = source vertex (first element)
        int v = edge[1];                 // v = destination vertex (second element)
        int wt = edge[2];                // wt = edge weight (third element)
        adj[u].push_back({v, wt});       // add directed edge u -> v with weight wt
        adj[v].push_back({u, wt});       // add reverse edge v -> u (making the graph undirected)
    }
    return adj; // return the built adjacency list (important: function must return adj)
} // end of constructAdj