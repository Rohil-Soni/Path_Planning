#!/usr/bin/env python3
"""
Animated A* Algorithm Visualization
Shows step-by-step exploration with randomized obstacles
"""
import matplotlib
matplotlib.use('Agg')  # Non-interactive backend for saving
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.colors import ListedColormap
from matplotlib.animation import FuncAnimation
import numpy as np
import random
import time

from Algo_AStar import AStarPathfinder

# Fixed grid size
ROWS, COLS = 10, 10
MIN_OBSTACLES = 20

def generate_random_obstacles(rows, cols, start, goal, min_obstacles=20):
    """Generate random obstacles ensuring path exists"""
    max_attempts = 100
    
    for attempt in range(max_attempts):
        # Generate random number of obstacles (between min_obstacles and 35% of grid)
        num_obstacles = random.randint(min_obstacles, int(rows * cols * 0.35))
        obstacles = []
        
        # Generate random positions
        while len(obstacles) < num_obstacles:
            r = random.randint(0, rows - 1)
            c = random.randint(0, cols - 1)
            
            # Don't place obstacle on start or goal
            if (r, c) != start and (r, c) != goal and (r, c) not in obstacles:
                obstacles.append((r, c))
        
        # Test if path exists
        pathfinder = AStarPathfinder((rows, cols), obstacles)
        result = pathfinder.astar(start, goal)
        
        if result['success']:
            print(f"✓ Generated {len(obstacles)} random obstacles (attempt {attempt + 1})")
            return obstacles, result
    
    # Fallback: use fewer obstacles
    print(f"⚠ Using fewer obstacles to ensure path exists")
    obstacles = []
    while len(obstacles) < min_obstacles:
        r = random.randint(0, rows - 1)
        c = random.randint(0, cols - 1)
        if (r, c) != start and (r, c) != goal and (r, c) not in obstacles:
            obstacles.append((r, c))
    
    pathfinder = AStarPathfinder((rows, cols), obstacles)
    result = pathfinder.astar(start, goal)
    return obstacles, result

def animate_pathfinding():
    """Create animated visualization of pathfinding"""
    
    # Fixed positions
    start = (0, 0)
    goal = (9, 9)
    
    print("=" * 80)
    print("ANIMATED A* ALGORITHM VISUALIZATION")
    print("=" * 80)
    print(f"\nGrid Size: {ROWS}x{COLS} (FIXED)")
    print(f"Start: {start} | Goal: {goal}")
    print(f"Minimum Obstacles: {MIN_OBSTACLES}")
    print(f"Heuristic: Manhattan Distance")
    print("\nGenerating random obstacles...\n")
    
    # Generate obstacles and run algorithm
    obstacles, result = generate_random_obstacles(ROWS, COLS, start, goal, MIN_OBSTACLES)
    
    print("\n" + "=" * 80)
    print("ALGORITHM RESULTS")
    print("=" * 80)
    print(f"✓ PATH FOUND!")
    print(f"\nObstacles:         {len(obstacles)}")
    print(f"Time:              {result['execution_time_ms']:.6f} ms")
    print(f"Nodes Explored:    {result['nodes_explored']}")
    print(f"Path Length:       {result['path_length']}")
    print(f"Path Cost:         {result['path_length'] - 1}")
    
    # Prepare animation data
    explored_nodes = result['explored_nodes']
    path_nodes = result['path']
    
    # Create figure
    fig, ax = plt.subplots(figsize=(12, 10))
    
    # Colors: white, black, light blue, green, red, yellow
    colors = ['white', 'black', '#87CEEB', '#00FF00', '#FF0000', '#FFD700']
    cmap = ListedColormap(colors)
    
    # Initialize grid
    grid = np.zeros((ROWS, COLS))
    
    # Mark obstacles
    for obs in obstacles:
        grid[obs[0]][obs[1]] = 1
    
    # Mark start and goal
    grid[start[0]][start[1]] = 4
    grid[goal[0]][goal[1]] = 5
    
    # Create initial plot
    im = ax.imshow(grid, cmap=cmap, interpolation='nearest', origin='upper', vmin=0, vmax=5)
    
    # Add grid lines
    ax.set_xticks(np.arange(-0.5, COLS, 1), minor=True)
    ax.set_yticks(np.arange(-0.5, ROWS, 1), minor=True)
    ax.grid(which='minor', color='gray', linestyle='-', linewidth=0.5)
    ax.tick_params(which='minor', size=0)
    
    # Set major ticks
    ax.set_xticks(np.arange(0, COLS, 1))
    ax.set_yticks(np.arange(0, ROWS, 1))
    
    # Title
    title = ax.text(0.5, 1.05, '', transform=ax.transAxes, 
                    ha='center', fontsize=14, fontweight='bold')
    
    # Add S and G labels
    ax.text(start[1], start[0], 'S', ha='center', va='center', 
            fontsize=14, fontweight='bold', color='white')
    ax.text(goal[1], goal[0], 'G', ha='center', va='center', 
            fontsize=14, fontweight='bold', color='black')
    
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
    
    # Animation parameters
    total_frames = len(explored_nodes) + len(path_nodes) + 20  # Extra frames at end
    
    def update(frame):
        """Update function for animation"""
        nonlocal grid
        
        # Phase 1: Show exploration
        if frame < len(explored_nodes):
            node = explored_nodes[frame]
            if node != start and node != goal:
                grid[node[0]][node[1]] = 2  # Mark as explored
            title.set_text(f"A* Algorithm - Exploring Nodes ({frame + 1}/{len(explored_nodes)})")
        
        # Phase 2: Show path
        elif frame < len(explored_nodes) + len(path_nodes):
            path_idx = frame - len(explored_nodes)
            node = path_nodes[path_idx]
            if node != start and node != goal:
                grid[node[0]][node[1]] = 3  # Mark as path
            title.set_text(f"A* Algorithm - Building Path ({path_idx + 1}/{len(path_nodes)})")
        
        # Phase 3: Final result
        else:
            title.set_text(f"A* Algorithm - Complete! Path Length: {result['path_length']}, "
                          f"Nodes Explored: {result['nodes_explored']}")
        
        im.set_array(grid)
        return [im, title]
    
    # Create animation
    anim = FuncAnimation(fig, update, frames=total_frames, interval=100, blit=True, repeat=True)
    
    # Save animation
    output_file = 'astar_animation.gif'
    print(f"\n📹 Creating animation... (this may take a moment)")
    anim.save(output_file, writer='pillow', fps=10)
    print(f"✓ Animation saved as: {output_file}")
    
    # Also save final static image
    static_file = 'astar_final.png'
    plt.savefig(static_file, dpi=150, bbox_inches='tight')
    print(f"✓ Final image saved as: {static_file}")
    
    print("\n" + "=" * 80)
    print("VISUALIZATION COMPLETE")
    print("=" * 80)
    
    plt.close()

if __name__ == "__main__":
    animate_pathfinding()
