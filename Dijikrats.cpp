#include<iostream>
#include<vector>
#include<queue>
#include<climits>
using namespace std;

//function to construct adjacency list
vector<vector<vector<int>>> constructadj(vector<vector<int>>& edges, int n) // Function to construct adjacency list
{
    vector<vector<vector<int>>> adj(n); // Adjacency list representation

    // adj[u] = list of {v, weight}
    for (const auto &edge : edges) // iterate each edge
    {
        int u = edge[0]; // start node
        int v = edge[1]; // end node (fixed: use lowercase v for neighbor)
        int wt = edge[2]; // weight
        adj[u].push_back({v, wt}); // Add edge u -> v
        adj[v].push_back({u, wt}); // Add edge v -> u (undirected)
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
        for(auto x : adj[u]){
            //get vertex label and weight of current adjacent of u
            int v = x[0];
            int weight = x[1];

            //if there is a shorter path to v through u.
            if(dist[v] > dist[u] + weight)
            {
                //updating distance of v
                dist[v] = dist[u] + weight;
                pq.push({dist[v],v}); // Push updated distance to priority queue
            }
        }
    }
    return dist; // Return the distance array
}

int main()
{
    int v =5;
    int src = 0 ;

    vector<vector<int>> edges = {{0, 1, 4}, {0, 2, 8}, {1, 4, 6}, {2, 3, 2}, {3, 4, 10}};

    vector<int> result = dijkstra(v, edges, src);

    for(int dist: result)
    {
        cout<<dist<<" ";    
    }
    return 0;
}