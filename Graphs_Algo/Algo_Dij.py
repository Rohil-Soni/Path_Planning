import heapq
import time
from collections import defaultdict
import math

class GraphAlgorithms:
    """
    Implementation of Dijkstra's algorithm with performance metrics
    """
    
    def __init__(self, graph, graph_type='grid'):
        """
        Initialize with a graph
        graph_type: 'grid' for grid-based graphs with coordinates, 'weighted' for weighted graphs
        """
        self.graph = graph
        self.graph_type = graph_type
        self.nodes_explored = 0
        self.execution_time = 0
        self.path_length = 0
    
    def get_neighbors(self, node):
        """
        Get neighbors of a node based on graph type
        """
        if self.graph_type == 'grid':
            return self._get_grid_neighbors(node)
        else:
            return self.graph.get(node, [])
    
    def _get_grid_neighbors(self, node):
        """
        Get valid neighbors in a grid (4-directional movement)
        """
        x, y = node
        neighbors = []
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # Right, Down, Left, Up
        
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            if (nx, ny) in self.graph:
                # Return as (neighbor, cost) tuple
                neighbors.append(((nx, ny), self.graph[(nx, ny)]))
        
        return neighbors
    
    def dijkstra(self, start, goal):
        """
        Dijkstra's algorithm implementation
        Returns: (path, nodes_explored, execution_time, path_length)
        """
        start_time = time.time()
        
        # Priority queue: (cost, node)
        pq = [(0, start)]
        # Distance from start to each node
        distances = {start: 0}
        # To reconstruct the path
        parent = {start: None}
        # Track explored nodes
        explored = set()
        nodes_explored_count = 0
        
        while pq:
            current_cost, current_node = heapq.heappop(pq)
            
            # Skip if already explored
            if current_node in explored:
                continue
            
            explored.add(current_node)
            nodes_explored_count += 1
            
            # Goal found
            if current_node == goal:
                end_time = time.time()
                path = self._reconstruct_path(parent, start, goal)
                return {
                    'path': path,
                    'nodes_explored': nodes_explored_count,
                    'execution_time': (end_time - start_time) * 1000,  # Convert to milliseconds
                    'path_length': len(path)
                }
            
            # Explore neighbors
            for neighbor, weight in self.get_neighbors(current_node):
                if neighbor not in explored:
                    new_cost = current_cost + weight
                    
                    if neighbor not in distances or new_cost < distances[neighbor]:
                        distances[neighbor] = new_cost
                        parent[neighbor] = current_node
                        heapq.heappush(pq, (new_cost, neighbor))
        
        # No path found
        end_time = time.time()
        return {
            'path': [],
            'nodes_explored': nodes_explored_count,
            'execution_time': (end_time - start_time) * 1000,
            'path_length': 0
        }
    
    def _reconstruct_path(self, parent, start, goal):
        """
        Reconstruct path from start to goal using parent dictionary
        """
        path = []
        current = goal
        while current is not None:
            path.append(current)
            current = parent[current]
        path.reverse()
        return path


def input_weighted_graph():
    """
    Get weighted graph input from user
    """
    print("\n" + "="*80)
    print("WEIGHTED GRAPH INPUT")
    print("="*80)
    print("Enter graph as adjacency list.")
    print("Format: node neighbor1,weight1 neighbor2,weight2 ...")
    print("Example: A B,4 C,2")
    print("Enter 'done' when finished.\n")
    
    graph = {}
    while True:
        line = input("Enter node and edges (or 'done'): ").strip()
        if line.lower() == 'done':
            break
        
        parts = line.split()
        if len(parts) < 1:
            continue
        
        node = parts[0]
        neighbors = []
        
        for i in range(1, len(parts)):
            try:
                neighbor, weight = parts[i].split(',')
                neighbors.append((neighbor, int(weight)))
            except:
                print(f"  ✗ Invalid format for '{parts[i]}'. Use format: neighbor,weight")
        
        graph[node] = neighbors
        print(f"  ✓ Added node {node} with {len(neighbors)} connection(s)")
    
    start = input("\nEnter start node: ").strip()
    goal = input("Enter goal node: ").strip()
    description = input("Enter graph description: ").strip()
    
    return graph, start, goal, description


def input_grid_graph():
    """
    Get grid graph input from user
    """
    print("\n" + "="*80)
    print("GRID GRAPH INPUT")
    print("="*80)
    
    rows = int(input("Enter number of rows: "))
    cols = int(input("Enter number of columns: "))
    
    print("\nEnter default cost for cells (e.g., 1): ")
    default_cost = int(input("Default cost: "))
    
    graph = {}
    for i in range(rows):
        for j in range(cols):
            graph[(i, j)] = default_cost
    
    print(f"\n✓ Created {rows}x{cols} grid with default cost {default_cost}")
    
    print("\nDo you want to add obstacles? (y/n): ")
    if input().strip().lower() == 'y':
        print("Enter obstacles as 'row,col' (one per line, 'done' to finish):")
        while True:
            obs_input = input("Obstacle: ").strip()
            if obs_input.lower() == 'done':
                break
            try:
                r, c = map(int, obs_input.split(','))
                if (r, c) in graph:
                    del graph[(r, c)]
                    print(f"  ✓ Added obstacle at ({r}, {c})")
                else:
                    print(f"  ✗ Position ({r}, {c}) is out of bounds")
            except:
                print("  ✗ Invalid format. Use: row,col")
    
    print("\nDo you want to add varying costs for specific cells? (y/n): ")
    if input().strip().lower() == 'y':
        print("Enter cell costs as 'row,col,cost' (one per line, 'done' to finish):")
        while True:
            cost_input = input("Cell cost: ").strip()
            if cost_input.lower() == 'done':
                break
            try:
                r, c, cost = map(int, cost_input.split(','))
                if (r, c) in graph:
                    graph[(r, c)] = cost
                    print(f"  ✓ Set cost {cost} for cell ({r}, {c})")
                else:
                    print(f"  ✗ Position ({r}, {c}) is out of bounds or is an obstacle")
            except:
                print("  ✗ Invalid format. Use: row,col,cost")
    
    print("\nEnter start position (row,col): ")
    start_r, start_c = map(int, input().strip().split(','))
    start = (start_r, start_c)
    
    print("Enter goal position (row,col): ")
    goal_r, goal_c = map(int, input().strip().split(','))
    goal = (goal_r, goal_c)
    
    description = input("Enter graph description: ").strip()
    
    return graph, start, goal, description


def run_comparison():
    """
    Run Dijkstra's algorithm on user-inputted graphs and display results
    """
    print("\n" + "=" * 100)
    print("DIJKSTRA'S ALGORITHM - PATH FINDING ANALYSIS")
    print("=" * 100)
    
    num_graphs = int(input("\nHow many graphs do you want to test? "))
    
    graphs = []
    
    for i in range(num_graphs):
        print(f"\n{'='*100}")
        print(f"GRAPH {i+1} INPUT")
        print(f"{'='*100}")
        
        graph_type = input("\nEnter graph type (1=weighted, 2=grid): ").strip()
        
        if graph_type == '1':
            graph_data = input_weighted_graph()
            graphs.append(graph_data)
        else:
            graph_data = input_grid_graph()
            graphs.append(graph_data)
    
    # Run Dijkstra on all graphs
    print("\n\n" + "=" * 100)
    print("RUNNING DIJKSTRA'S ALGORITHM ON ALL GRAPHS")
    print("=" * 100)
    
    results_summary = []
    
    for idx, (graph, start, goal, description) in enumerate(graphs, 1):
        print(f"\n{'=' * 100}")
        print(f"GRAPH {idx}: {description}")
        print(f"{'=' * 100}")
        print(f"Start: {start}")
        print(f"Goal: {goal}")
        print(f"Total nodes in graph: {len(graph)}")
        print()
        
        # Determine graph type
        graph_type = 'grid' if isinstance(start, tuple) else 'weighted'
        
        # Run Dijkstra's
        algo_dijkstra = GraphAlgorithms(graph, graph_type)
        dijkstra_result = algo_dijkstra.dijkstra(start, goal)
        
        # Display results
        print("DIJKSTRA'S ALGORITHM RESULTS:")
        print("-" * 100)
        print(f"⏱️  Execution Time:    {dijkstra_result['execution_time']:>10.4f} ms")
        print(f"🔍 Nodes Explored:    {dijkstra_result['nodes_explored']:>10} nodes")
        print(f"📏 Path Length:       {dijkstra_result['path_length']:>10} nodes")
        print()
        
        if dijkstra_result['path']:
            if len(dijkstra_result['path']) <= 10:
                print(f"Path: {' -> '.join(map(str, dijkstra_result['path']))}")
            else:
                print(f"Path: {' -> '.join(map(str, dijkstra_result['path'][:5]))} ... {' -> '.join(map(str, dijkstra_result['path'][-3:]))}")
        else:
            print("Path: No path found")
        
        # Store for summary
        results_summary.append({
            'graph': description,
            'result': dijkstra_result
        })
    
    # Overall summary
    print(f"\n\n{'=' * 100}")
    print("OVERALL SUMMARY - DIJKSTRA'S ALGORITHM PERFORMANCE")
    print(f"{'=' * 100}\n")
    
    print(f"{'Graph':<40} {'Time (ms)':<15} {'Nodes Explored':<20} {'Path Length':<15}")
    print("-" * 100)
    
    total_time = 0
    total_nodes = 0
    total_path = 0
    
    for result in results_summary:
        print(f"{result['graph']:<40} {result['result']['execution_time']:>10.4f}     {result['result']['nodes_explored']:>15}     {result['result']['path_length']:>12}")
        total_time += result['result']['execution_time']
        total_nodes += result['result']['nodes_explored']
        total_path += result['result']['path_length']
    
    if num_graphs > 0:
        print("-" * 100)
        print(f"{'AVERAGE:':<40} {total_time/num_graphs:>10.4f}     {total_nodes/num_graphs:>15.1f}     {total_path/num_graphs:>12.1f}")
    
    print()
    print("KEY OBSERVATIONS:")
    print("-" * 100)
    print("✓ Dijkstra's algorithm guarantees finding the OPTIMAL (shortest) path.")
    print("✓ The algorithm explores nodes in order of their distance from the start.")
    print("✓ More complex graphs with obstacles require exploring more nodes.")
    print("✓ Execution time increases with graph size and complexity.")
    print()
    print("=" * 100)


if __name__ == "__main__":
    run_comparison()