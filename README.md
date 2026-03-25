# Path Planning Algorithms

This repository contains implementations of various path planning algorithms in both C++ and Python. It includes basic console-based versions as well as interactive visualizations and Graphical User Interfaces (GUIs) to better understand how these algorithms explore spaces and find optimal paths.

## 📂 Repository Structure

### 1. C++ Implementations
These are fundamental console-based implementations of classic pathfinding algorithms:
- `A_Star.cpp` - A* (A-Star) Pathfinding Algorithm implementation.
- `Dijikrats.cpp` - Dijkstra's Algorithm implementation.

### 2. Python Visualizations & GUI (`Graphs_Algo/`)
This directory contains fully functional Python implementations with interactive visualizations and GUIs.

#### Algorithms
- `Algo_AStar.py` - A* Pathfinding Algorithm with visualization.
- `Algo_Dij.py` - Dijkstra's Pathfinding Algorithm with visualization.
- `Algo_LPA.py` - Lifelong Planning A* (LPA*) Algorithm with visualization.

#### Animations
- `animate_astar.py` - Animated visualization for A*.
- `animate_dijkstra.py` - Animated visualization for Dijkstra's.

#### Graphical User Interfaces (GUIs)
Interactive interfaces to draw obstacles, set start/goal points, and visualize the pathfinding process in real-time.
- `astar_gui.py` - GUI for A* Algorithm.
- `dijkstra_gui.py` - GUI for Dijkstra's Algorithm.
- `lpa_gui.py` - GUI for LPA* Algorithm.

### 3. Miscellaneous
- `Q_A` - A short Q&A reference file covering C++ basic concepts like Vectors (1D, 2D, 3D) and range-based loops.

## 🚀 Getting Started

### Prerequisites

For C++ files, you need a standard C++ compiler like `g++`:
```bash
g++ A_Star.cpp -o a_star
./a_star
```

For Python scripts, make sure you have Python 3 installed along with the required libraries (especially `matplotlib` and `numpy` for visualizations or `tkinter`/`PyQt` depending on the GUI bindings):
```bash
pip install matplotlib numpy
```

### Running Python Visualizations
Navigate to the `Graphs_Algo` directory and run any of the scripts:
```bash
cd Graphs_Algo
python Algo_Dij.py
```
*Note: Refer to `Graphs_Algo/VISUALIZATION_README.md` for detailed instructions on using the Python visualization tools.*

## 🧠 Algorithms Covered
* **Dijkstra's Algorithm**: Guarantees the shortest path but explores uniformly in all directions.
* **A* (A-Star) Algorithm**: Uses heuristics to guide the search towards the goal, making it significantly faster than Dijkstra's in most cases while still guaranteeing the shortest path.
* **LPA* (Lifelong Planning A*)**: An incremental version of A* that can adapt to changes in the graph without recalculating the entire path from scratch.

## 📝 License
This repository is intended for educational and research use.
