#!/usr/bin/env python3
"""
Animated Dijkstra's Algorithm Visualization
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

from Algo_Dij import DijkstraPathfinder

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
        pathfinder = DijkstraPathfinder((rows, cols), obstacles)
        result = pathfinder.dijkstra(start, goal)
        
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
    
    pathfinder = DijkstraPathfinder((rows, cols), obstacles)
    result = pathfinder.dijkstra(start, goal)
    return obstacles, result

def animate_pathfinding():
    """Create animated visualization of pathfinding"""
    
    # Fixed positions
    start = (0, 0)
    goal = (9, 9)
    
    print("=" * 80)
    print("ANIMATED DIJKSTRA'S ALGORITHM VISUALIZATION")
    print("=" * 80)
    print(f"\nGrid Size: {ROWS}x{COLS} (FIXED)")
    print(f"Start: {start} | Goal: {goal}")
    print(f"Minimum Obstacles: {MIN_OBSTACLES}")
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
    
    # Create image
    im = ax.imshow(grid, cmap=cmap, interpolation='nearest', origin='upper')
    
    # Grid lines
    ax.set_xticks(np.arange(-0.5, COLS, 1), minor=True)
    ax.set_yticks(np.arange(-0.5, ROWS, 1), minor=True)
    ax.grid(which='minor', color='gray', linestyle='-', linewidth=0.5)
    ax.tick_params(which='minor', size=0)
    
    # Ticks
    ax.set_xticks(np.arange(0, COLS, 1))
    ax.set_yticks(np.arange(0, ROWS, 1))
    
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
    
    # Title text
    title_text = ax.text(0.5, 1.08, '', transform=ax.transAxes,
                        ha='center', fontsize=14, fontweight='bold')
    
    # Animation state
    animation_phase = [0]  # 0: exploring, 1: showing path
    frame_count = [0]
    
    def update(frame):
        """Update function for animation"""
        phase = animation_phase[0]
        
        if phase == 0:  # Exploration phase
            if frame < len(explored_nodes):
                node = explored_nodes[frame]
                if node != start and node != goal:
                    grid[node[0]][node[1]] = 2  # Mark as explored
                
                im.set_array(grid)
                title_text.set_text(
                    f"Dijkstra's Algorithm - Exploring Nodes\n"
                    f"Explored: {frame + 1}/{len(explored_nodes)} | "
                    f"Obstacles: {len(obstacles)}"
                )
                frame_count[0] = frame
                return [im, title_text]
            else:
                animation_phase[0] = 1
                frame_count[0] = 0
                return [im, title_text]
        
        elif phase == 1:  # Path drawing phase
            adjusted_frame = frame_count[0]
            if adjusted_frame < len(path_nodes):
                node = path_nodes[adjusted_frame]
                if node != start and node != goal:
                    grid[node[0]][node[1]] = 3  # Mark as path
                
                im.set_array(grid)
                title_text.set_text(
                    f"Dijkstra's Algorithm - Final Path\n"
                    f"Path: {adjusted_frame + 1}/{len(path_nodes)} | "
                    f"Total Cost: {len(path_nodes) - 1}"
                )
                frame_count[0] += 1
                return [im, title_text]
            else:
                # Animation complete
                title_text.set_text(
                    f"Dijkstra's Algorithm - COMPLETE\n"
                    f"Explored: {result['nodes_explored']} | "
                    f"Path Length: {result['path_length']} | "
                    f"Time: {result['execution_time_ms']:.2f}ms"
                )
                return [im, title_text]
        
        return [im, title_text]
    
    # Create animation
    total_frames = len(explored_nodes) + len(path_nodes) + 30  # Extra frames at end
    anim = FuncAnimation(fig, update, frames=total_frames, 
                        interval=50, blit=True, repeat=False)
    
    plt.tight_layout()
    
    print("\n" + "=" * 80)
    print("🎬 SAVING ANIMATION...")
    print("=" * 80)
    
    # Save as GIF
    output_file = 'dijkstra_animation.gif'
    anim.save(output_file, writer='pillow', fps=20, dpi=100)
    
    print(f"✓ Animation saved to: {output_file}")
    print(f"Location: /home/rohil-soni/Code_Repo/Path_Planning/Graphs_Algo/{output_file}")
    
    # Also save final frame as PNG
    final_file = 'dijkstra_final.png'
    plt.savefig(final_file, dpi=150, bbox_inches='tight')
    print(f"✓ Final frame saved to: {final_file}")
    print("=" * 80 + "\n")

if __name__ == "__main__":
    animate_pathfinding()
