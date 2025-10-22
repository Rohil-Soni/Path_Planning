#include<iostream>
#include<vector>
#include<queue>
#include<climits>
using namespace std;

//function to construct adjacency list
vector<vector<vector<int>>> constructadj(vector<vector<int>>& edges, int n) // Function to construct adjacency list
{
    vector<vector<vector<int>>> adj(V); // Adjacency list representation

    //adj[u] = list of {v, weight}
    for(const auto &edge: edges) //for loop for each edge in each direction
    {
        int u = edge[0];// u = starting node
        int v = edge[1];// v = ending node
        int wt = edge[2];// wt = weight of the edge
        adj[u].push_back({v, wt}); // Add edge from u to v with weight wt
        adj[v].push_back({u, wt}); // Add edge from v to u with weight wt (for undirected graph)
    }
    return adj; // Return the constructed adjacency list
}
int main()
{
    int v =5;
    int src = 0 ;

    vector<vector<int>> edge = {{0, 1, 4}, {0, 2, 8}, {1, 4, 6}, {2, 3, 2}, {3, 4, 10}};

    vector<int> result = dijkstra(v, edges, src);

    for(int dist: result)
    {
        cout<<dist<<" ";    
    }
    return 0;
}