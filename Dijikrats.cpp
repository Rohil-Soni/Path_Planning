#include<iostream>
#include<vector>
#include<queue>
#include<climits>
using namespace std;

//function to construct adjacency list
vector<vector<vector<int>>> constructadj(vector<vector<int>>& edges, int n) // Function to construct adjacency list
{
    vector<vector<vector<int>>> adj(v); // Adjacency list representation

    //adj[u] = list of {v, weight}
    for(const auto &edge: edges) //for loop for each edge in each direction
    {
        int u = edge[0];// u = starting node
        int V = edge[1];// v = ending node
        int wt = edge[2];// wt = weight of the edge
        adj[u].push_back({v, wt}); // Add edge from u to v with weight wt
        adj[v].push_back({u, wt}); // Add edge from v to u with weight wt (for undirected graph)
    }
    return adj; // Return the constructed adjacency list
}

vector<int> dijkstra(int V, vector<vector<int>> &edges, int src)
{
    vector<vector<vector<int>>> adj = constructadj(edges, V); // Construct adjacency list from edges
    
    // Create a priority queue to store vertices that are being preprocessed.
    priority_queue<vector<int>, vector<vector<int>>, greater<vector<int>>> pq; // Min-heap priority queue
    vector<int> dist(V, INT_MAX); // Initialize distances to all vertices as infinite
    pq.push({0,src});  // Insert source itself in priority queue and initialize its distance as 0.
    dist[src] = 0; // Distance to source is 0
    //looping till priority queue becomes empty(or all vertices are not finalized)
    while(!pq.empty()){
        //the first vertex in the pair is the min distance vertex, extract it from priority queue.
        int u = pq.top()[1];
        pq.pop();

        //get all adjacent vertices of u
         
    }
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