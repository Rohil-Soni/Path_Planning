import math
import heapq
import tkinter as tk

def load_static_grid(rows: int, cols: int, obstacles=None):
    """
    Build a rows×cols boolean matrix.
    False = free cell, True = blocked obstacle.
    obstacles is an optional iterable of (r, c) tuples.
    """
    if obstacles is None:
        obstacles = []

    grid = [[False for _ in range(cols)] for _ in range(rows)]

    for r, c in obstacles:
        if 0 <= r < rows and 0 <= c < cols:
            grid[r][c] = True
    return grid

class LPAStarPathfinder:
    """
    Lifelong Planning A* on an 8‑connected grid.
    The grid is a 2‑D list of booleans (False=free, True=blocked).
    """

    def __init__(self, grid):
        self.grid = grid
        self.rows = len(grid)
        self.cols = len(grid[0]) if self.rows else 0
        self.inf = float('inf')

        # g and rhs dictionaries (default ∞)
        self.g = {}
        self.rhs = {}
        for r in range(self.rows):
            for c in range(self.cols):
                self.g[(r, c)] = self.inf
                self.rhs[(r, c)] = self.inf

        # priority queue (min‑heap) + dict for fast update/removal
        self.open_heap = []          
        self.entry_finder = {}       

        self.sstart = None
        self.sgoal = None
        
        # Add a counter to prevent TypeError during heap tie-breaking
        self.counter = 0  

    # -----------------------------------------------------------------
    # 3.1  Heuristic (Euclidean, consistent for 8‑connected moves)
    # -----------------------------------------------------------------
    def heuristic(self, a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    # -----------------------------------------------------------------
    # 3.2  Key calculation – the two‑part LPA* priority
    # -----------------------------------------------------------------
    def calculate_key(self, s):
        min_g_rhs = min(self.g[s], self.rhs[s])
        return (min_g_rhs + self.heuristic(s, self.sgoal), min_g_rhs)

    # -----------------------------------------------------------------
    # 3.3  Priority‑queue helpers (insert / lazy‑remove / pop)
    # -----------------------------------------------------------------
    def _add_to_open(self, node):
        key = self.calculate_key(node)
        if node in self.entry_finder:
            # Mark the old entry as removed using index -1
            self.entry_finder[node][-1] = '<removed>'
            
        self.counter += 1
        # Insert count into the heap entry to guarantee unique tie-breaking
        entry = [key, self.counter, node]
        self.entry_finder[node] = entry
        heapq.heappush(self.open_heap, entry)

    def _remove_from_open(self, node):
        entry = self.entry_finder.pop(node, None)
        if entry:
            entry[-1] = '<removed>'

    def _pop_from_open(self):
        while self.open_heap:
            # Unpack three variables now: key, count, node
            key, count, node = heapq.heappop(self.open_heap)
            if node != '<removed>':
                self.entry_finder.pop(node, None)
                return node, key
        return None, None

    # ... [The rest of the LPAStarPathfinder class remains exactly the same] ...

    # -----------------------------------------------------------------
    # 3.4  Neighbor utilities (8‑connected moves)
    # -----------------------------------------------------------------
    def _in_bounds(self, r, c):
        return 0 <= r < self.rows and 0 <= c < self.cols

    def _is_blocked(self, r, c):
        return self.grid[r][c]

    def get_neighbors(self, s):
        dirs = [(-1, 0, 1), (1, 0, 1), (0, -1, 1), (0, 1, 1),
                (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)),
                (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2))]
        result = []
        r, c = s
        for dr, dc, cost in dirs:
            nr, nc = r + dr, c + dc
            if self._in_bounds(nr, nc) and not self._is_blocked(nr, nc):
                result.append(((nr, nc), cost))
        return result

    # -----------------------------------------------------------------
    # 3.5  Initialise start/goal for a fresh run
    # -----------------------------------------------------------------
    def initialize(self, start, goal):
        self.sstart = start
        self.sgoal = goal

        for r in range(self.rows):
            for c in range(self.cols):
                self.g[(r, c)] = self.inf
                self.rhs[(r, c)] = self.inf

        self.rhs[start] = 0
        self._add_to_open(start)

    # -----------------------------------------------------------------
    # 3.6  Update a single vertex (core of LPA*)
    # -----------------------------------------------------------------
    def update_vertex(self, u):
        if u != self.sstart:
            best = self.inf
            for pred, cost in self.get_neighbors(u):
                cand = self.g[pred] + cost
                if cand < best:
                    best = cand
            self.rhs[u] = best

        if self.g[u] != self.rhs[u]:
            self._add_to_open(u)
        else:
            self._remove_from_open(u)

    # -----------------------------------------------------------------
    # 3.7  ComputeShortestPath – main LPA* loop
    # -----------------------------------------------------------------
    def compute_shortest_path(self):
        while True:
            u, u_key = self._pop_from_open()
            if u is None:
                break

            goal_key = self.calculate_key(self.sgoal)

            # termination (same logic as A*)
            if (u_key >= goal_key) and (self.rhs[self.sgoal] == self.g[self.sgoal]):
                self._add_to_open(u)
                break

            if self.g[u] > self.rhs[u]:           # over‑consistent
                self.g[u] = self.rhs[u]
                for succ, _ in self.get_neighbors(u):
                    self.update_vertex(succ)
            else:                                 # under‑consistent
                self.g[u] = self.inf
                for succ, _ in self.get_neighbors(u) + [(u, 0)]:
                    self.update_vertex(succ)

    # -----------------------------------------------------------------
    # 3.8  Convenience wrapper – run once from start to goal
    # -----------------------------------------------------------------
    def plan(self, start, goal):
        self.initialize(start, goal)
        self.compute_shortest_path()
        return self._extract_path(start, goal)

    # -----------------------------------------------------------------
    # 3.9  Path extraction (used by the GUI)
    # -----------------------------------------------------------------
    def _extract_path(self, start, goal):
        if self.g[goal] == self.inf:
            return []
        path = [goal]
        cur = goal
        while cur != start:
            best_pred = None
            best_cost = self.inf
            for pred, cost in self.get_neighbors(cur):
                cand = self.g[pred] + cost
                if cand < best_cost:
                    best_cost = cand
                    best_pred = pred
            if best_pred is None:
                return []
            path.append(best_pred)
            cur = best_pred
        path.reverse()
        return path

    def current_path(self):
        """Public helper – return the most recent path (empty if none)."""
        return self._extract_path(self.sstart, self.sgoal)

    # -----------------------------------------------------------------
    # 3.10  UI‑friendly helpers – toggle an obstacle and re‑plan
    # -----------------------------------------------------------------
    def toggle_obstacle(self, r, c):
        if (r, c) == self.sstart or (r, c) == self.sgoal:
            return self.grid[r][c]          # never block start/goal

        self.grid[r][c] = not self.grid[r][c]

        # affected cells = the toggled cell + its neighbours
        affected = [(r, c)]
        for dr, dc, _ in [(-1, 0, 1), (1, 0, 1), (0, -1, 1), (0, 1, 1),
                          (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)),
                          (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2))]:
            nr, nc = r + dr, c + dc
            if self._in_bounds(nr, nc):
                affected.append((nr, nc))

        for cell in affected:
            self.update_vertex(cell)

        self.compute_shortest_path()

CELL_SIZE = 60   # pixels per cell, constant for simplicity

class LPAViewer(tk.Tk):
    """Tkinter window that draws the grid, the path, and handles clicks."""

    def __init__(self,
                 rows,
                 cols,
                 start,
                 goal,
                 initial_obstacles,
                 existing_planner=None):          # optional planner we already ran
        super().__init__()
        self.title("LPA* Interactive Demo (dynamic size)")
        self.resizable(False, False)

        # ---- store geometry -------------------------------------------------
        self.rows = rows
        self.cols = cols
        self.start = start
        self.goal = goal

        # ---- build the grid -------------------------------------------------
        self.grid_data = load_static_grid(rows, cols, initial_obstacles)

        # ---- planner ---------------------------------------------------------
        if existing_planner is not None:
            # we already called plan() once in the main block
            self.planner = existing_planner
        else:
            self.planner = LPAStarPathfinder(self.grid_data)
            self.planner.plan(start, goal)

        # ---- UI widgets ------------------------------------------------------
        self.canvas = tk.Canvas(self,
                                width=self.cols * CELL_SIZE,
                                height=self.rows * CELL_SIZE,
                                bg="#1e1e1e")
        self.canvas.pack()

        self.status_var = tk.StringVar()
        self.status_var.set("Ready")
        self.status_label = tk.Label(self,
                                     textvariable=self.status_var,
                                     anchor="w",
                                     bg="#2b2b2b",
                                     fg="#dddddd")
        self.status_label.pack(fill="x")

        # ---- initial drawing -------------------------------------------------
        self._draw_grid()
        self._draw_path()

        # ---- bind mouse clicks ------------------------------------------------
        self.canvas.bind("<Button-1>", self._on_click)

    # -----------------------------------------------------------------
    # 4.1  Drawing helpers
    # -----------------------------------------------------------------
    def _draw_grid(self):
        self.canvas.delete("cell")
        for r in range(self.rows):
            for c in range(self.cols):
                x0, y0 = c * CELL_SIZE, r * CELL_SIZE
                x1, y1 = x0 + CELL_SIZE, y0 + CELL_SIZE

                if (r, c) == self.start:
                    fill = "#ff5555"          # red
                elif (r, c) == self.goal:
                    fill = "#ffd700"          # gold
                elif self.grid_data[r][c]:
                    fill = "#000000"          # blocked
                else:
                    fill = "#ffffff"          # free

                self.canvas.create_rectangle(x0, y0, x1, y1,
                                             fill=fill,
                                             outline="#555555",
                                             tags="cell")

    def _draw_path(self):
        self.canvas.delete("path")
        path = self.planner.current_path()
        if not path:
            self.status_var.set("No path found!")
            return

        cost = self.planner.g[self.goal]
        self.status_var.set(f"Path length {len(path)} – cost {cost:.2f}")

        for (r, c) in path:
            if (r, c) in (self.start, self.goal):
                continue
            x0, y0 = c * CELL_SIZE, r * CELL_SIZE
            x1, y1 = x0 + CELL_SIZE, y0 + CELL_SIZE
            self.canvas.create_rectangle(x0, y0, x1, y1,
                                         fill="#90ee90",
                                         outline="#555555",
                                         tags="path")

    # -----------------------------------------------------------------
    # 4.2  Click handler – toggle obstacle, re‑plan, redraw, log to console
    # -----------------------------------------------------------------
    def _on_click(self, event):
        col = event.x // CELL_SIZE
        row = event.y // CELL_SIZE

        if not (0 <= row < self.rows and 0 <= col < self.cols):
            return

        self.planner.toggle_obstacle(row, col)
        self._draw_grid()
        self._draw_path()

        # ---- also echo the new result to the terminal --------------------
        cur_path = self.planner.current_path()
        if cur_path:
            print("\n--- Re‑plan after click ---")
            print(f"Path length : {len(cur_path)}")
            print(f"Cost        : {self.planner.g[self.goal]:.2f}")
            print(f"Path        : {cur_path}")
        else:
            print("\n--- Re‑plan after click ---")
            print("No path found!")

def ask_int(prompt, min_val=1):
    """Prompt until the user enters an integer >= min_val."""
    while True:
        try:
            val = int(input(prompt))
            if val >= min_val:
                return val
            print(f"Please enter an integer >= {min_val}.")
        except ValueError:
            print("That doesn't look like an integer – try again.")

def ask_obstacle_list(rows, cols):
    """
    Let the user type obstacle coordinates.
    Example input: 1,2 3,4 0,0   (press ENTER on an empty line when done)
    """
    print("\nEnter obstacle coordinates (row,col) separated by spaces.")
    print("Example: 1,2 3,4 0,0   (press ENTER when done)")
    line = input("> ").strip()
    if not line:
        return []                     # no obstacles

    obstacles = []
    for token in line.split():
        try:
            r_str, c_str = token.split(',')
            r, c = int(r_str), int(c_str)
            if 0 <= r < rows and 0 <= c < cols:
                obstacles.append((r, c))
            else:
                print(f"Ignoring out‑of‑bounds coordinate ({r},{c}).")
        except Exception:
            print(f"Could not parse '{token}'. Expected format row,col.")
    return obstacles

if __name__ == "__main__":
    print("\n=== Dynamic LPA* Demo ===\n")
    rows = ask_int("Number of rows (≥1): ")
    cols = ask_int("Number of columns (≥1): ")

    start = (0, 0)                 # top‑left corner
    goal  = (rows - 1, cols - 1)    # bottom‑right corner

    obstacles = ask_obstacle_list(rows, cols)

    # ------------------------------------------------------------------
    # 6.1  Build the planner and compute the initial path
    # ------------------------------------------------------------------
    grid = load_static_grid(rows, cols, obstacles)
    planner = LPAStarPathfinder(grid)
    planner.plan(start, goal)          # runs LPA* once

    # ----- 6.2  Console output of the initial solution -----------------
    init_path = planner.current_path()
    if init_path:
        print("\nInitial path found:")
        print(f"  Cells : {init_path}")
        print(f"  Cost  : {planner.g[goal]:.2f}")
    else:
        print("\nNo path found on the initial grid.")

    # ------------------------------------------------------------------
    # 6.3  Launch the interactive window, handing it the already‑run planner
    # ------------------------------------------------------------------
    app = LPAViewer(rows,
                    cols,
                    start,
                    goal,
                    obstacles,
                    existing_planner=planner)   # reuse the planner we just computed
    app.mainloop()