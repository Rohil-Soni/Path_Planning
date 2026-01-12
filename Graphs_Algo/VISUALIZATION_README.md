# Dijkstra's Pathfinding Algorithm with Visualization

## Overview
Optimized implementation of Dijkstra's algorithm using only **arrays and linked lists** (no heapq or dictionaries for core logic). Includes matplotlib visualization showing the pathfinding process.

## Features
- ✅ Uses only basic data structures (arrays & linked lists)
- ✅ O(V²) time complexity
- ✅ O(1) array-based lookups (faster than dictionary hashing)
- ✅ Interactive matplotlib visualization
- ✅ Shows obstacles, explored nodes, and final path

## Data Structures Used
- **Linked List**: Custom priority queue implementation
- **2D Arrays**: For distances, parent tracking, explored nodes, and obstacles
- **1D Arrays**: For neighbor lists and path storage

## Installation

```bash
# Install required packages
pip install matplotlib numpy
```

## Usage

### Interactive Mode
Run the main program for interactive input:

```bash
python Algo_Dij.py
```

You'll be prompted to enter:
1. Grid dimensions (rows x cols)
2. Obstacle positions (row,col format)
3. Start position
4. Goal position

After completion, a visualization window will automatically open.

### Quick Test
Run the test script with a pre-configured example:

```bash
python test_visualization.py
```

### Programmatic Use

```python
from Algo_Dij import DijkstraPathfinder, visualize_pathfinding

# Setup
rows, cols = 10, 10
obstacles = [(2, 2), (2, 3), (2, 4), (5, 5), (5, 6)]
start = (0, 0)
goal = (9, 9)

# Run algorithm
pathfinder = DijkstraPathfinder((rows, cols), obstacles)
result = pathfinder.dijkstra(start, goal)

# Visualize
visualize_pathfinding(rows, cols, obstacles, start, goal, result)

# Access results
print(f"Path found: {result['success']}")
print(f"Path length: {result['path_length']}")
print(f"Nodes explored: {result['nodes_explored']}")
print(f"Execution time: {result['execution_time_ms']:.2f}ms")
```

## Visualization Legend

| Color | Meaning |
|-------|---------|
| 🔴 Red | Start position (marked with 'S') |
| 🟡 Yellow | Goal position (marked with 'G') |
| ⬛ Black | Obstacles |
| 🔵 Light Blue | Explored nodes |
| 🟢 Green | Final path |
| ⬜ White | Unexplored cells |

## Example Output

```
DIJKSTRA'S ALGORITHM - PATH FINDING RESEARCH ANALYSIS
================================================================================

Grid Size: 10x10
Start Position: (0, 0)
Goal Position: (9, 9)
Obstacles: 15 cells

RESULTS
================================================================================
✓ PATH FOUND!

Time Taken:        1.234567 ms
Nodes Explored:    45
Path Length:       19 nodes
Path Cost:         18

📊 Opening visualization window...
```

## Algorithm Complexity

- **Time Complexity**: O(V²) where V is the number of vertices (grid cells)
- **Space Complexity**: O(V) for storing distances, parent, and explored arrays
- **Access Time**: O(1) for all array lookups

## Performance Optimizations

1. **2D Arrays vs Dictionaries**: Direct indexing is faster than hash lookups
2. **Early Termination**: Stops immediately when goal is reached
3. **Sorted Linked List**: Maintains priority order during insertion
4. **Cache Locality**: Contiguous memory access for better performance

## Files

- `Algo_Dij.py` - Main algorithm implementation with visualization
- `test_visualization.py` - Quick test script with pre-configured example
- `VISUALIZATION_README.md` - This file

## Grid Coordinate System

```
     0   1   2   3   (columns)
   ┌───┬───┬───┬───┐
 0 │   │   │   │   │
   ├───┼───┼───┼───┤
 1 │   │   │   │   │
   ├───┼───┼───┼───┤
 2 │   │   │   │   │
   └───┴───┴───┴───┘
(rows)
```

Position format: `(row, column)`

## Tips

- Larger grids (20x20 or more) show more interesting pathfinding behavior
- Create walls by adding obstacle sequences
- Try different start/goal positions to see algorithm efficiency
- Close the visualization window to continue or run another test

## Research Parameters

The algorithm provides detailed metrics:
- **Total Time Taken**: Execution time in milliseconds
- **Nodes/Cells Explored**: Number of cells visited during search
- **Nodes in Final Path**: Length of the shortest path found
- **Path Cost**: Total cost with unit cost = 1

## License

Educational and research use.
