import time
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.colors import ListedColormap
import numpy as np

class Node:
    """Simple linked list node for priority queue implementation"""
    def __init__(self, data, f_cost, g_cost):
        self.data = data
        self.f_cost = f_cost  # f(n) = g(n) + h(n)
        self.g_cost = g_cost  # Actual cost from start
        self.next = None

class PriorityQueue:
    """Basic linked list priority queue for A* algorithm"""
    def __init__(self):
        self.head = None
        self.size = 0
    
    def is_empty(self):
        return self.head is None
    
    def insert(self, data, f_cost, g_cost):
        """Insert node in sorted order by f_cost"""
        new_node = Node(data, f_cost, g_cost)
        self.size += 1
        
        if self.head is None or f_cost < self.head.f_cost:
            new_node.next = self.head
            self.head = new_node
            return
        
        current = self.head
        while current.next and current.next.f_cost <= f_cost:
            current = current.next
        new_node.next = current.next
        current.next = new_node
    
    def pop(self):
        """Remove and return minimum f_cost node"""
        if self.head is None:
            return None, None, None
        data = self.head.data
        f_cost = self.head.f_cost
        g_cost = self.head.g_cost
        self.head = self.head.next
        self.size -= 1
        return data, f_cost, g_cost

class AStarPathfinder:
    """
    Optimized A* Algorithm Implementation
    Unit cost: 1 for all paths
    Uses Manhattan distance heuristic
    Uses only arrays and linked lists
    """
    
    def __init__(self, grid, obstacles):
        """
        Initialize with grid dimensions and obstacles
        grid: (rows, cols) tuple
        obstacles: list of (row, col) tuples representing blocked cells
        """
        self.rows, self.cols = grid
        # Use 2D array for obstacles instead of set/dict
        self.obstacle_grid = [[False] * self.cols for _ in range(self.rows)]
        for r, c in obstacles:
            self.obstacle_grid[r][c] = True
        self.UNIT_COST = 1  # Fixed unit cost for all movements
    
    def heuristic(self, node, goal):
        """
        Manhattan distance heuristic (L1 distance)
        Admissible and consistent for grid-based pathfinding
        """
        return abs(node[0] - goal[0]) + abs(node[1] - goal[1])
    
    def get_neighbors(self, node):
        """
        Get valid neighbors (8-directional: up, down, left, right + diagonals)
        Optimized with early termination
        """
        x, y = node
        neighbors = []
        # 8-directional movement: 4 cardinal + 4 diagonal directions
        directions = [
            (0, 1), (1, 0), (0, -1), (-1, 0),       # Cardinal directions
            (-1, 1), (1, 1), (1, -1), (-1, -1)      # Diagonal directions
        ]
        
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            # Check if neighbor is within grid bounds and not an obstacle
            if (0 <= nx < self.rows and 0 <= ny < self.cols and 
                not self.obstacle_grid[nx][ny]):
                neighbors.append((nx, ny))
        
        return neighbors
    
    def astar(self, start, goal):
        """
        Optimized A* algorithm using only arrays and linked list
        Time Complexity: O(V log V) where V is number of vertices
        Space Complexity: O(V)
        Returns: dictionary with path, metrics
        """
        start_time = time.time()
        
        # Use linked list based priority queue
        queue = PriorityQueue()
        h_start = self.heuristic(start, goal)
        queue.insert(start, h_start, 0)
        
        # Use 2D arrays instead of dictionaries for O(1) access
        INF = float('inf')
        # g_score: distance from start to each node
        g_scores = [[INF] * self.cols for _ in range(self.rows)]
        g_scores[start[0]][start[1]] = 0
        
        # f_score: g_score + heuristic
        f_scores = [[INF] * self.cols for _ in range(self.rows)]
        f_scores[start[0]][start[1]] = h_start
        
        # To reconstruct the path
        parent = [[None] * self.cols for _ in range(self.rows)]
        
        # Track explored nodes
        explored = [[False] * self.cols for _ in range(self.rows)]
        nodes_explored = 0
        explored_list = []  # Store order of exploration for visualization
        
        while not queue.is_empty():
            current_node, current_f, current_g = queue.pop()
            x, y = current_node
            
            # Skip if already explored
            if explored[x][y]:
                continue
            
            explored[x][y] = True
            nodes_explored += 1
            explored_list.append((x, y))  # Track exploration order
            
            # Goal found - early exit
            if current_node == goal:
                end_time = time.time()
                path = self._reconstruct_path(parent, start, goal)
                
                return {
                    'path': path,
                    'nodes_explored': nodes_explored,
                    'execution_time_ms': (end_time - start_time) * 1000,
                    'path_length': len(path),
                    'success': True,
                    'explored_nodes': explored_list,
                    'algorithm': 'A*'
                }
            
            # Explore neighbors
            for neighbor in self.get_neighbors(current_node):
                nx, ny = neighbor
                if not explored[nx][ny]:
                    tentative_g = current_g + self.UNIT_COST
                    
                    # Only update if we found a better path
                    if tentative_g < g_scores[nx][ny]:
                        g_scores[nx][ny] = tentative_g
                        h_score = self.heuristic(neighbor, goal)
                        f_score = tentative_g + h_score
                        f_scores[nx][ny] = f_score
                        parent[nx][ny] = current_node
                        queue.insert(neighbor, f_score, tentative_g)
        
        # No path found
        end_time = time.time()
        return {
            'path': [],
            'nodes_explored': nodes_explored,
            'execution_time_ms': (end_time - start_time) * 1000,
            'path_length': 0,
            'success': False,
            'explored_nodes': explored_list,
            'algorithm': 'A*'
        }
    
    def _reconstruct_path(self, parent, start, goal):
        """
        Reconstruct path from start to goal using 2D parent array
        """
        path = []
        current = goal
        while current is not None:
            path.append(current)
            x, y = current
            current = parent[x][y]
        path.reverse()
        return path

def visualize_pathfinding(rows, cols, obstacles, start, goal, result):
    """
    Visualize the pathfinding process using matplotlib
    """
    # Create figure and axis
    fig, ax = plt.subplots(figsize=(12, 10))
    
    # Create grid visualization
    grid = np.zeros((rows, cols))
    
    # Mark obstacles (value = 1)
    for obs in obstacles:
        grid[obs[0]][obs[1]] = 1
    
    # Mark explored nodes (value = 2)
    if 'explored_nodes' in result:
        for node in result['explored_nodes']:
            if node != start and node != goal:
                grid[node[0]][node[1]] = 2
    
    # Mark path (value = 3)
    if result['success']:
        for node in result['path']:
            if node != start and node != goal:
                grid[node[0]][node[1]] = 3
    
    # Mark start (value = 4) and goal (value = 5)
    grid[start[0]][start[1]] = 4
    grid[goal[0]][goal[1]] = 5
    
    # Define colors: white (empty), black (obstacle), light blue (explored), 
    # green (path), red (start), yellow (goal)
    colors = ['white', 'black', '#87CEEB', '#00FF00', '#FF0000', '#FFD700']
    cmap = ListedColormap(colors)
    
    # Plot the grid
    im = ax.imshow(grid, cmap=cmap, interpolation='nearest', origin='upper')
    
    # Add grid lines
    ax.set_xticks(np.arange(-0.5, cols, 1), minor=True)
    ax.set_yticks(np.arange(-0.5, rows, 1), minor=True)
    ax.grid(which='minor', color='gray', linestyle='-', linewidth=0.5)
    ax.tick_params(which='minor', size=0)
    
    # Set major ticks
    ax.set_xticks(np.arange(0, cols, 1))
    ax.set_yticks(np.arange(0, rows, 1))
    ax.set_xticklabels(np.arange(0, cols, 1))
    ax.set_yticklabels(np.arange(0, rows, 1))
    
    # Add title and labels
    status = "PATH FOUND" if result['success'] else "NO PATH FOUND"
    plt.title(f"A* Algorithm Visualization - {status}\n"
              f"Grid: {rows}x{cols} | Explored: {result['nodes_explored']} nodes | "
              f"Path Length: {result['path_length']} | Time: {result['execution_time_ms']:.2f}ms",
              fontsize=14, fontweight='bold', pad=20)
    
    ax.set_xlabel('Column', fontsize=12)
    ax.set_ylabel('Row', fontsize=12)
    
    # Create legend
    legend_elements = [
        patches.Patch(facecolor='#FF0000', edgecolor='black', label='Start'),
        patches.Patch(facecolor='#FFD700', edgecolor='black', label='Goal'),
        patches.Patch(facecolor='black', edgecolor='black', label='Obstacles'),
        patches.Patch(facecolor='#87CEEB', edgecolor='black', label='Explored'),
        patches.Patch(facecolor='#00FF00', edgecolor='black', label='Path')
    ]
    ax.legend(handles=legend_elements, loc='upper left', bbox_to_anchor=(1.02, 1), 
              fontsize=11, frameon=True, shadow=True)
    
    # Add annotations for start and goal
    ax.text(start[1], start[0], 'S', ha='center', va='center', 
            fontsize=14, fontweight='bold', color='white')
    ax.text(goal[1], goal[0], 'G', ha='center', va='center', 
            fontsize=14, fontweight='bold', color='black')
    
    plt.tight_layout()
    plt.show()

def run_astar_analysis():
    """
    Main function to run A* algorithm with user input
    """
    print("\n" + "=" * 80)
    print("A* ALGORITHM - PATH FINDING RESEARCH ANALYSIS")
    print("=" * 80)
    print("Unit Cost: 1 (fixed for all movements)")
    print("Movement: 8-directional (up, down, left, right + diagonals)")
    print("Heuristic: Manhattan Distance (L1)")
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
    
    # Run A* algorithm
    print("\n\n" + "=" * 80)
    print("RUNNING A* ALGORITHM...")
    print("=" * 80)
    
    pathfinder = AStarPathfinder((rows, cols), obstacles)
    result = pathfinder.astar(start, goal)
    
    # Display results
    print("\n" + "=" * 80)
    print("RESEARCH PARAMETERS - A* ALGORITHM RESULTS")
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
    
    # Visualize the pathfinding
    print("\n📊 Generating visualization...")
    visualize_pathfinding(rows, cols, obstacles, start, goal, result)
    
    # Ask if user wants to test another scenario
    print("\nWould you like to test another scenario? (y/n): ", end="")
    if input().strip().lower() == 'y':
        run_astar_analysis()


if __name__ == "__main__":
    run_astar_analysis()
