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


def display_grid(rows, cols, start, goal, obstacles, path=None):
    """
    Visual representation of the grid
    """
    print("\nGRID VISUALIZATION:")
    print("=" * (cols * 4 + 1))
    
    for i in range(rows):
        row_str = ""
        for j in range(cols):
            cell = (i, j)
            if cell == start:
                row_str += " S "
            elif cell == goal:
                row_str += " G "
            elif cell in obstacles:
                row_str += " ■ "
            elif path and cell in path:
                row_str += " * "
            else:
                row_str += " . "
            row_str += "|" if j < cols - 1 else ""
        print(row_str)
        if i < rows - 1:
            print("-" * (cols * 4 + 1))
    
    print("=" * (cols * 4 + 1))
    print("Legend: S=Start, G=Goal, ■=Obstacle, *=Path, .=Empty")


def run_dijkstra_analysis():
    """
    Main function to run Dijkstra's algorithm with user input
    """
    print("\n" + "=" * 80)
    print("DIJKSTRA'S ALGORITHM - PATHFINDING RESEARCH ANALYSIS")
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
    
    # Display initial grid
    display_grid(rows, cols, start, goal, obstacles)
    
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
        
        # Display grid with path
        display_grid(rows, cols, start, goal, obstacles, result['path'])
        
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


# Example scenarios for quick testing
def run_example_scenarios():
    """
    Pre-defined example scenarios for quick testing
    """
    print("\n" + "=" * 80)
    print("EXAMPLE SCENARIOS - DIJKSTRA'S ALGORITHM")
    print("=" * 80)
    
    scenarios = [
        {
            'name': 'Simple 5x5 Grid - No Obstacles',
            'grid': (5, 5),
            'obstacles': [],
            'start': (0, 0),
            'goal': (4, 4)
        },
        {
            'name': '10x10 Grid - With Obstacles',
            'grid': (10, 10),
            'obstacles': [(2, 2), (2, 3), (2, 4), (3, 4), (4, 4), (5, 4), (6, 4)],
            'start': (0, 0),
            'goal': (9, 9)
        },
        {
            'name': '8x8 Grid - Maze-like',
            'grid': (8, 8),
            'obstacles': [(1, 1), (1, 2), (1, 3), (3, 1), (3, 3), (5, 1), (5, 2), (5, 3), (5, 5)],
            'start': (0, 0),
            'goal': (7, 7)
        }
    ]
    
    print("\nAvailable scenarios:")
    for i, scenario in enumerate(scenarios, 1):
        print(f"{i}. {scenario['name']}")
    print(f"{len(scenarios) + 1}. Custom input")
    
    choice = int(input("\nSelect scenario (1-4): "))
    
    if choice <= len(scenarios):
        scenario = scenarios[choice - 1]
        print(f"\n{'=' * 80}")
        print(f"SCENARIO: {scenario['name']}")
        print(f"{'=' * 80}")
        
        rows, cols = scenario['grid']
        obstacles = scenario['obstacles']
        start = scenario['start']
        goal = scenario['goal']
        
        print(f"Grid: {rows} x {cols}")
        print(f"Obstacles: {len(obstacles)}")
        print(f"Start: {start}")
        print(f"Goal: {goal}")
        
        display_grid(rows, cols, start, goal, obstacles)
        
        print("\nRunning Dijkstra's Algorithm...")
        pathfinder = DijkstraPathfinder(scenario['grid'], scenario['obstacles'])
        result = pathfinder.dijkstra(scenario['start'], scenario['goal'])
        
        print("\n" + "=" * 80)
        print("RESULTS")
        print("=" * 80)
        print(f"\na) Total Time Taken:           {result['execution_time_ms']:.6f} ms")
        print(f"b) Nodes/Cells Explored:       {result['nodes_explored']} nodes")
        print(f"c) Nodes in Final Path:        {result['path_length']} nodes")
        
        if result['success']:
            display_grid(rows, cols, start, goal, obstacles, result['path'])
        
    else:
        run_dijkstra_analysis()


if __name__ == "__main__":
    print("\n" + "=" * 80)
    print("DIJKSTRA'S ALGORITHM - RESEARCH IMPLEMENTATION")
    print("=" * 80)
    print("\nChoose mode:")
    print("1. Example scenarios (quick test)")
    print("2. Custom input")
    
    mode = input("\nEnter choice (1 or 2): ").strip()
    
    if mode == '1':
        run_example_scenarios()
    else:
        run_dijkstra_analysis()