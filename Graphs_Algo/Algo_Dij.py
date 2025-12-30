import heapq
import time

class DijkstraPathfinder:
    """
    Dijkstra's Algorithm Implementation for Research Analysis
    Unit cost: 1 for all paths
    """
    
    def __init__(self, grid, obstacles):
        """
        Initialize with grid dimensions and obstacles
        grid: (rows, cols) tuple
        obstacles: list of (row, col) tuples representing blocked cells
        """
        self.rows, self.cols = grid
        self.obstacles = set(obstacles)
        self.UNIT_COST = 1  # Fixed unit cost for all movements
        
    def get_neighbors(self, node):
        """
        Get valid neighbors (4-directional: up, down, left, right)
        """
        x, y = node
        neighbors = []
        # 4-directional movement: Right, Down, Left, Up
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
        
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            # Check if neighbor is within grid bounds and not an obstacle
            if (0 <= nx < self.rows and 0 <= ny < self.cols and 
                (nx, ny) not in self.obstacles):
                neighbors.append((nx, ny))
        
        return neighbors
    
    def dijkstra(self, start, goal):
        """
        Dijkstra's algorithm with performance metrics
        Returns: dictionary with path, metrics
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
        nodes_explored = 0
        
        while pq:
            current_cost, current_node = heapq.heappop(pq)
            
            # Skip if already explored
            if current_node in explored:
                continue
            
            explored.add(current_node)
            nodes_explored += 1
            
            # Goal found
            if current_node == goal:
                end_time = time.time()
                path = self._reconstruct_path(parent, start, goal)
                
                return {
                    'path': path,
                    'nodes_explored': nodes_explored,
                    'execution_time_ms': (end_time - start_time) * 1000,
                    'path_length': len(path),
                    'success': True
                }
            
            # Explore neighbors
            for neighbor in self.get_neighbors(current_node):
                if neighbor not in explored:
                    new_cost = current_cost + self.UNIT_COST
                    
                    if neighbor not in distances or new_cost < distances[neighbor]:
                        distances[neighbor] = new_cost
                        parent[neighbor] = current_node
                        heapq.heappush(pq, (new_cost, neighbor))
        
        # No path found
        end_time = time.time()
        return {
            'path': [],
            'nodes_explored': nodes_explored,
            'execution_time_ms': (end_time - start_time) * 1000,
            'path_length': 0,
            'success': False
        }
    
    def _reconstruct_path(self, parent, start, goal):
        """
        Reconstruct path from start to goal
        """
        path = []
        current = goal
        while current is not None:
            path.append(current)
            current = parent[current]
        path.reverse()
        return path 
def run_dijkstra_analysis():
    """
    Main function to run Dijkstra's algorithm with user input
    """
    print("\n" + "=" * 80)
    print("DIJKSTRA'S ALGORITHM - PATH FINDING RESEARCH ANALYSIS")
    print("=" * 80)
    print("Unit Cost: 1 (fixed for all movements)")
    print("Movement: 4-directional (up, down, left, right)")
    print("=" * 80)
    
    # Get grid dimensions
    print("\n📐 GRID SETUP")
    print("-" * 80)
    rows = int(input("Enter number of rows: "))
    cols = int(input("Enter number of columns: "))
    print(f"✓ Grid size: {rows} x {cols} ({rows * cols} total cells)")
    
    # Get obstacles
    print("\n🚧 OBSTACLES SETUP")
    print("-" * 80)
    print("Enter obstacles as 'row,col' (one per line)")
    print("Type 'done' when finished")
    print("Example: 1,2")
    
    obstacles = []
    while True:
        obstacle_input = input("Obstacle (or 'done'): ").strip()
        if obstacle_input.lower() == 'done':
            break
        try:
            r, c = map(int, obstacle_input.split(','))
            if 0 <= r < rows and 0 <= c < cols:
                obstacles.append((r, c))
                print(f"  ✓ Added obstacle at ({r}, {c})")
            else:
                print(f"  ✗ Position ({r}, {c}) is out of bounds!")
        except:
            print("  ✗ Invalid format! Use: row,col")
    
    print(f"\n✓ Total obstacles: {len(obstacles)}")
    
    # Get start and goal
    print("\n🎯 START & GOAL POSITIONS")
    print("-" * 80)
    
    while True:
        start_input = input("Enter start position (row,col): ").strip()
        try:
            sr, sc = map(int, start_input.split(','))
            if 0 <= sr < rows and 0 <= sc < cols:
                if (sr, sc) not in obstacles:
                    start = (sr, sc)
                    print(f"  ✓ Start: ({sr}, {sc})")
                    break
                else:
                    print("  ✗ Start position cannot be an obstacle!")
            else:
                print("  ✗ Position out of bounds!")
        except:
            print("  ✗ Invalid format! Use: row,col")
    
    while True:
        goal_input = input("Enter goal position (row,col): ").strip()
        try:
            gr, gc = map(int, goal_input.split(','))
            if 0 <= gr < rows and 0 <= gc < cols:
                if (gr, gc) not in obstacles:
                    goal = (gr, gc)
                    print(f"  ✓ Goal: ({gr}, {gc})")
                    break
                else:
                    print("  ✗ Goal position cannot be an obstacle!")
            else:
                print("  ✗ Position out of bounds!")
        except:
            print("  ✗ Invalid format! Use: row,col")
    
    # Run Dijkstra's algorithm
    print("\n\n" + "=" * 80)
    print("RUNNING DIJKSTRA'S ALGORITHM...")
    print("=" * 80)
    
    pathfinder = DijkstraPathfinder((rows, cols), obstacles)
    result = pathfinder.dijkstra(start, goal)
    
    # Display results
    print("\n" + "=" * 80)
    print("RESEARCH PARAMETERS - DIJKSTRA'S ALGORITHM RESULTS")
    print("=" * 80)
    
    if result['success']:
        print("\n✓ PATH FOUND!")
        print("-" * 80)
        print(f"\na) Total Time Taken:           {result['execution_time_ms']:.6f} ms")
        print(f"b) Nodes/Cells Explored:       {result['nodes_explored']} nodes")
        print(f"c) Nodes in Final Path:        {result['path_length']} nodes")
        print()
        print(f"Path Cost:                     {result['path_length'] - 1} (with unit cost = 1)")
        print()
        
        # Display path
        if result['path_length'] <= 20:
            print(f"Complete Path:")
            print(f"{' → '.join([f'({x},{y})' for x, y in result['path']])}")
        else:
            print(f"Path (first 5 and last 5 nodes):")
            first_5 = ' → '.join([f'({x},{y})' for x, y in result['path'][:5]])
            last_5 = ' → '.join([f'({x},{y})' for x, y in result['path'][-5:]])
            print(f"{first_5} ... {last_5}")
        
    else:
        print("\n✗ NO PATH FOUND!")
        print("-" * 80)
        print(f"\na) Total Time Taken:           {result['execution_time_ms']:.6f} ms")
        print(f"b) Nodes/Cells Explored:       {result['nodes_explored']} nodes")
        print(f"c) Nodes in Final Path:        0 nodes (no path exists)")
        print("\nReason: The goal is unreachable from the start position.")
    
    print("\n" + "=" * 80)
    print("ANALYSIS COMPLETE")
    print("=" * 80)
    
    # Ask if user wants to test another scenario
    print("\nWould you like to test another scenario? (y/n): ", end="")
    if input().strip().lower() == 'y':
        run_dijkstra_analysis()


if __name__ == "__main__":
    run_dijkstra_analysis()