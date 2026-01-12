#!/usr/bin/env python3
"""
Test script that saves visualization to a PNG file
"""
import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend
import matplotlib.pyplot as plt

from Algo_Dij import DijkstraPathfinder
import matplotlib.patches as patches
from matplotlib.colors import ListedColormap
import numpy as np

# Test case: 10x10 grid with obstacles
rows, cols = 10, 10

# Create obstacles (wall pattern)
obstacles = [
    (2, 1), (2, 2), (2, 3), (2, 4), (2, 5), (2, 6),
    (5, 3), (5, 4), (5, 5), (5, 6), (5, 7), (5, 8),
    (7, 1), (7, 2), (7, 3), (7, 4), (7, 5)
]

start = (0, 0)
goal = (9, 9)

print("=" * 80)
print("DIJKSTRA'S ALGORITHM - VISUALIZATION TEST")
print("=" * 80)
print(f"\nGrid Size: {rows}x{cols}")
print(f"Start: {start} | Goal: {goal}")
print(f"Obstacles: {len(obstacles)} cells")
print("\nRunning algorithm...\n")

# Run algorithm
pathfinder = DijkstraPathfinder((rows, cols), obstacles)
result = pathfinder.dijkstra(start, goal)

print("=" * 80)
print("RESULTS")
print("=" * 80)
if result['success']:
    print(f"✓ PATH FOUND!")
    print(f"\nTime:              {result['execution_time_ms']:.6f} ms")
    print(f"Nodes Explored:    {result['nodes_explored']}")
    print(f"Path Length:       {result['path_length']}")
else:
    print(f"✗ NO PATH FOUND")

# Create visualization
fig, ax = plt.subplots(figsize=(12, 10))

# Create grid
grid = np.zeros((rows, cols))

# Mark obstacles
for obs in obstacles:
    grid[obs[0]][obs[1]] = 1

# Mark explored nodes
for node in result['explored_nodes']:
    if node != start and node != goal:
        grid[node[0]][node[1]] = 2

# Mark path
if result['success']:
    for node in result['path']:
        if node != start and node != goal:
            grid[node[0]][node[1]] = 3

# Mark start and goal
grid[start[0]][start[1]] = 4
grid[goal[0]][goal[1]] = 5

# Colors
colors = ['white', 'black', '#87CEEB', '#00FF00', '#FF0000', '#FFD700']
cmap = ListedColormap(colors)

# Plot
im = ax.imshow(grid, cmap=cmap, interpolation='nearest', origin='upper')

# Grid lines
ax.set_xticks(np.arange(-0.5, cols, 1), minor=True)
ax.set_yticks(np.arange(-0.5, rows, 1), minor=True)
ax.grid(which='minor', color='gray', linestyle='-', linewidth=0.5)
ax.tick_params(which='minor', size=0)

# Ticks
ax.set_xticks(np.arange(0, cols, 1))
ax.set_yticks(np.arange(0, rows, 1))

# Title
status = "PATH FOUND" if result['success'] else "NO PATH FOUND"
plt.title(f"Dijkstra's Algorithm Visualization - {status}\n"
          f"Grid: {rows}x{cols} | Explored: {result['nodes_explored']} nodes | "
          f"Path Length: {result['path_length']} | Time: {result['execution_time_ms']:.2f}ms",
          fontsize=14, fontweight='bold', pad=20)

ax.set_xlabel('Column', fontsize=12)
ax.set_ylabel('Row', fontsize=12)

# Legend
legend_elements = [
    patches.Patch(facecolor='#FF0000', edgecolor='black', label='Start'),
    patches.Patch(facecolor='#FFD700', edgecolor='black', label='Goal'),
    patches.Patch(facecolor='black', edgecolor='black', label='Obstacles'),
    patches.Patch(facecolor='#87CEEB', edgecolor='black', label='Explored'),
    patches.Patch(facecolor='#00FF00', edgecolor='black', label='Path')
]
ax.legend(handles=legend_elements, loc='upper left', bbox_to_anchor=(1.02, 1), 
          fontsize=11, frameon=True, shadow=True)

# Add S and G markers
ax.text(start[1], start[0], 'S', ha='center', va='center', 
        fontsize=14, fontweight='bold', color='white')
ax.text(goal[1], goal[0], 'G', ha='center', va='center', 
        fontsize=14, fontweight='bold', color='black')

plt.tight_layout()

# Save to file
output_file = 'dijkstra_visualization.png'
plt.savefig(output_file, dpi=150, bbox_inches='tight')
print(f"\n✓ Visualization saved to: {output_file}")
print("=" * 80)

print(f"\nOpen the file to view the visualization!")
print(f"Location: /home/rohil-soni/Code_Repo/Path_Planning/Graphs_Algo/{output_file}")
