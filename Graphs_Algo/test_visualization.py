#!/usr/bin/env python3
"""
Quick test script to demonstrate Dijkstra's pathfinding visualization
"""
from Algo_Dij import DijkstraPathfinder, visualize_pathfinding

# Test case 1: Simple grid with obstacles
print("=" * 80)
print("TESTING DIJKSTRA'S ALGORITHM WITH VISUALIZATION")
print("=" * 80)

# Setup a 10x10 grid
rows, cols = 10, 10

# Create obstacles (a wall pattern)
obstacles = [
    (2, 1), (2, 2), (2, 3), (2, 4), (2, 5), (2, 6),
    (5, 3), (5, 4), (5, 5), (5, 6), (5, 7), (5, 8),
    (7, 1), (7, 2), (7, 3), (7, 4), (7, 5)
]

# Start and goal positions
start = (0, 0)
goal = (9, 9)

print(f"\nGrid Size: {rows}x{cols}")
print(f"Start Position: {start}")
print(f"Goal Position: {goal}")
print(f"Obstacles: {len(obstacles)} cells")
print("\nRunning Dijkstra's algorithm...\n")

# Create pathfinder and run algorithm
pathfinder = DijkstraPathfinder((rows, cols), obstacles)
result = pathfinder.dijkstra(start, goal)

# Display results
print("=" * 80)
print("RESULTS")
print("=" * 80)

if result['success']:
    print(f"✓ PATH FOUND!")
    print(f"\nTime Taken:        {result['execution_time_ms']:.6f} ms")
    print(f"Nodes Explored:    {result['nodes_explored']}")
    print(f"Path Length:       {result['path_length']} nodes")
    print(f"Path Cost:         {result['path_length'] - 1}")
else:
    print(f"✗ NO PATH FOUND")
    print(f"\nTime Taken:        {result['execution_time_ms']:.6f} ms")
    print(f"Nodes Explored:    {result['nodes_explored']}")

print("\n📊 Opening visualization window...")
print("=" * 80)

# Visualize the result
visualize_pathfinding(rows, cols, obstacles, start, goal, result)

print("\nVisualization complete!")
