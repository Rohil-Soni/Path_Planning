import heapq
import math
import time
from typing import Dict, List, Optional, Set, Tuple

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
from matplotlib.colors import ListedColormap

GridNode = Tuple[int, int]


class LPAStarPathfinder:
    """
    Lifelong Planning A* (LPA*) on a grid with dynamic obstacle updates.

    Core equations:
    - rhs(s_start) = 0, rhs(s) = min_{s' in Pred(s)} (g(s') + c(s', s)) for s != s_start
    - key(s) = [min(g(s), rhs(s)) + h(s, goal), min(g(s), rhs(s))]
    - A state is consistent when g(s) == rhs(s)

    This implementation uses:
    - 2D arrays for g/rhs values
    - lazy-deletion binary heap for priority queue operations
    - incremental updates after obstacle changes
    """

    def __init__(
        self,
        grid: Tuple[int, int],
        obstacles: List[GridNode],
        straight_cost: float = 1.0,
        diagonal_cost: float = math.sqrt(2.0),
    ) -> None:
        self.rows, self.cols = grid
        self.straight_cost = float(straight_cost)
        self.diagonal_cost = float(diagonal_cost)
        self.inf = float("inf")

        self.obstacle_grid = [[False] * self.cols for _ in range(self.rows)]
        for r, c in obstacles:
            if 0 <= r < self.rows and 0 <= c < self.cols:
                self.obstacle_grid[r][c] = True

        self._directions = [
            (0, 1, self.straight_cost),
            (1, 0, self.straight_cost),
            (0, -1, self.straight_cost),
            (-1, 0, self.straight_cost),
            (-1, 1, self.diagonal_cost),
            (1, 1, self.diagonal_cost),
            (1, -1, self.diagonal_cost),
            (-1, -1, self.diagonal_cost),
        ]

        self.start: Optional[GridNode] = None
        self.goal: Optional[GridNode] = None

        self.g: List[List[float]] = [[self.inf] * self.cols for _ in range(self.rows)]
        self.rhs: List[List[float]] = [[self.inf] * self.cols for _ in range(self.rows)]

        # Priority queue with lazy deletion: heap entries are (k1, k2, tie, row, col)
        self.open_heap: List[Tuple[float, float, int, int, int]] = []
        self.entry_finder: Dict[GridNode, Tuple[float, float, int, int, int]] = {}
        self._tie_counter = 0

        self.initialized = False

    def _in_bounds(self, node: GridNode) -> bool:
        r, c = node
        return 0 <= r < self.rows and 0 <= c < self.cols

    def _is_blocked(self, node: GridNode) -> bool:
        r, c = node
        return self.obstacle_grid[r][c]

    def heuristic(self, node: GridNode, goal: GridNode) -> float:
        dx = abs(node[0] - goal[0])
        dy = abs(node[1] - goal[1])
        dmin, dmax = min(dx, dy), max(dx, dy)
        return self.diagonal_cost * dmin + self.straight_cost * (dmax - dmin)

    def _neighbor_coords_all(self, node: GridNode) -> List[GridNode]:
        x, y = node
        out: List[GridNode] = []
        for dx, dy, _ in self._directions:
            nx, ny = x + dx, y + dy
            if 0 <= nx < self.rows and 0 <= ny < self.cols:
                out.append((nx, ny))
        return out

    def get_neighbors(self, node: GridNode) -> List[Tuple[GridNode, float]]:
        if self._is_blocked(node):
            return []

        x, y = node
        out: List[Tuple[GridNode, float]] = []
        for dx, dy, move_cost in self._directions:
            nx, ny = x + dx, y + dy
            n = (nx, ny)
            if self._in_bounds(n) and not self._is_blocked(n):
                out.append((n, move_cost))
        return out

    def _move_cost(self, u: GridNode, v: GridNode) -> float:
        if not self._in_bounds(u) or not self._in_bounds(v):
            return self.inf
        if self._is_blocked(u) or self._is_blocked(v):
            return self.inf

        dr = abs(u[0] - v[0])
        dc = abs(u[1] - v[1])
        if dr == 1 and dc == 1:
            return self.diagonal_cost
        if dr + dc == 1:
            return self.straight_cost
        return self.inf

    def _calculate_key(self, node: GridNode) -> Tuple[float, float]:
        if self.goal is None:
            return (self.inf, self.inf)
        r, c = node
        base = min(self.g[r][c], self.rhs[r][c])
        return (base + self.heuristic(node, self.goal), base)

    def _add_or_update_open(self, node: GridNode) -> None:
        key = self._calculate_key(node)
        self._tie_counter += 1
        entry = (key[0], key[1], self._tie_counter, node[0], node[1])
        self.entry_finder[node] = entry
        heapq.heappush(self.open_heap, entry)

    def _remove_open(self, node: GridNode) -> None:
        if node in self.entry_finder:
            del self.entry_finder[node]

    def _peek_valid_key(self) -> Tuple[float, float]:
        while self.open_heap:
            k1, k2, tie, r, c = self.open_heap[0]
            node = (r, c)
            entry = self.entry_finder.get(node)
            if entry is None or entry != (k1, k2, tie, r, c):
                heapq.heappop(self.open_heap)
                continue
            return (k1, k2)
        return (self.inf, self.inf)

    def _pop_valid_node(self) -> Optional[GridNode]:
        while self.open_heap:
            k1, k2, tie, r, c = heapq.heappop(self.open_heap)
            node = (r, c)
            entry = self.entry_finder.get(node)
            if entry is None or entry != (k1, k2, tie, r, c):
                continue
            del self.entry_finder[node]
            return node
        return None

    def _initialize(self, start: GridNode, goal: GridNode) -> None:
        self.start = start
        self.goal = goal
        self.initialized = True

        self.g = [[self.inf] * self.cols for _ in range(self.rows)]
        self.rhs = [[self.inf] * self.cols for _ in range(self.rows)]

        self.open_heap = []
        self.entry_finder = {}
        self._tie_counter = 0

        sr, sc = start
        self.rhs[sr][sc] = 0.0
        self._add_or_update_open(start)

    def _update_vertex(self, u: GridNode) -> None:
        if self.start is None:
            return

        ur, uc = u
        if u != self.start:
            best_rhs = self.inf
            for pred, _ in self.get_neighbors(u):
                pr, pc = pred
                cost = self._move_cost(pred, u)
                cand = self.g[pr][pc] + cost
                if cand < best_rhs:
                    best_rhs = cand
            self.rhs[ur][uc] = best_rhs

        self._remove_open(u)
        if self.g[ur][uc] != self.rhs[ur][uc]:
            self._add_or_update_open(u)

    def _compute_shortest_path(self) -> Tuple[List[GridNode], int]:
        if self.goal is None:
            return [], 0

        expanded_nodes: List[GridNode] = []
        expanded_count = 0

        gr, gc = self.goal
        while (
            self._peek_valid_key() < self._calculate_key(self.goal)
            or self.rhs[gr][gc] != self.g[gr][gc]
        ):
            u = self._pop_valid_node()
            if u is None:
                break

            expanded_nodes.append(u)
            expanded_count += 1
            ur, uc = u

            if self.g[ur][uc] > self.rhs[ur][uc]:
                # Overconsistent state: set g to rhs and propagate improvements.
                self.g[ur][uc] = self.rhs[ur][uc]
                for succ, _ in self.get_neighbors(u):
                    self._update_vertex(succ)
            else:
                # Underconsistent state: invalidate g and re-evaluate local neighborhood.
                self.g[ur][uc] = self.inf
                self._update_vertex(u)
                for succ, _ in self.get_neighbors(u):
                    self._update_vertex(succ)

        return expanded_nodes, expanded_count

    def _extract_path(self) -> List[GridNode]:
        if self.start is None or self.goal is None:
            return []

        sr, sc = self.start
        gr, gc = self.goal
        if self.g[gr][gc] == self.inf:
            return []

        # g-values are forward costs from start, so reconstruct by walking
        # backward from goal using predecessor optimality.
        rev_path = [self.goal]
        visited: Set[GridNode] = {self.goal}
        current = self.goal
        eps = 1e-9

        limit = self.rows * self.cols
        for _ in range(limit):
            if current == self.start:
                break

            cr, cc = current
            current_g = self.g[cr][cc]

            best_pred: Optional[GridNode] = None
            best_pred_g = self.inf

            for pred, _ in self.get_neighbors(current):
                pr, pc = pred
                pred_g = self.g[pr][pc]
                if pred_g == self.inf:
                    continue

                # On an optimal predecessor, pred_g + c(pred,current) == current_g.
                if abs((pred_g + self._move_cost(pred, current)) - current_g) <= eps:
                    if pred_g < best_pred_g:
                        best_pred_g = pred_g
                        best_pred = pred

            if best_pred is None or best_pred in visited:
                return []

            rev_path.append(best_pred)
            visited.add(best_pred)
            current = best_pred

        if not rev_path or rev_path[-1] != self.start:
            return []

        rev_path.reverse()
        return rev_path

    def plan(self, start: GridNode, goal: GridNode) -> Dict[str, object]:
        if not self._in_bounds(start) or not self._in_bounds(goal):
            raise ValueError("Start/goal must be inside grid bounds")
        if self._is_blocked(start) or self._is_blocked(goal):
            raise ValueError("Start/goal cannot be blocked")

        t0 = time.time()

        if not self.initialized or self.start != start or self.goal != goal:
            self._initialize(start, goal)

        expanded_nodes, expanded_count = self._compute_shortest_path()
        path = self._extract_path()

        t1 = time.time()
        gr, gc = goal
        success = len(path) > 0

        # Equation snapshot helps inspect the algorithm's internal calculation state.
        calc_nodes: List[GridNode] = [start, goal]
        for node in path[1:4]:
            if node not in calc_nodes:
                calc_nodes.append(node)

        return {
            "algorithm": "LPA*",
            "success": success,
            "path": path,
            "path_length": len(path),
            "total_cost": self.g[gr][gc],
            "nodes_expanded": expanded_count,
            "expanded_nodes": expanded_nodes,
            "execution_time_ms": (t1 - t0) * 1000.0,
            "calculation_snapshot": self.get_calculation_snapshot(calc_nodes),
        }

    def apply_obstacle_changes(
        self, changes: List[Tuple[int, int, bool]]
    ) -> Dict[str, object]:
        """
        Apply obstacle updates incrementally and replan.

        changes: list of (row, col, is_blocked)
        """
        if not self.initialized or self.start is None or self.goal is None:
            raise RuntimeError("Call plan(start, goal) once before incremental updates")

        changed_cells: List[GridNode] = []

        for r, c, is_blocked in changes:
            if not (0 <= r < self.rows and 0 <= c < self.cols):
                continue
            node = (r, c)
            if node == self.start or node == self.goal:
                continue
            if self.obstacle_grid[r][c] == is_blocked:
                continue

            self.obstacle_grid[r][c] = is_blocked
            changed_cells.append(node)

        affected: Set[GridNode] = set()
        for node in changed_cells:
            affected.add(node)
            for nbr in self._neighbor_coords_all(node):
                affected.add(nbr)

        # For each changed incident edge (u, v), updating both endpoints keeps rhs values correct.
        for node in affected:
            self._update_vertex(node)

        result = self.plan(self.start, self.goal)
        result["changed_cells"] = changed_cells
        return result

    def get_calculation_snapshot(self, nodes: List[GridNode]) -> List[Dict[str, object]]:
        out: List[Dict[str, object]] = []
        for node in nodes:
            if not self._in_bounds(node):
                continue
            r, c = node
            k1, k2 = self._calculate_key(node)
            h = self.heuristic(node, self.goal) if self.goal is not None else self.inf
            out.append(
                {
                    "node": node,
                    "g": self.g[r][c],
                    "rhs": self.rhs[r][c],
                    "h": h,
                    "key": (k1, k2),
                    "consistent": self.g[r][c] == self.rhs[r][c],
                }
            )
        return out


def visualize_lpa_result(
    rows: int,
    cols: int,
    obstacles: List[GridNode],
    start: GridNode,
    goal: GridNode,
    result: Dict[str, object],
    title_prefix: str = "LPA*",
) -> None:
    fig, ax = plt.subplots(figsize=(12, 10))
    grid = np.zeros((rows, cols), dtype=np.int32)

    expanded_nodes_obj = result.get("expanded_nodes", [])
    expanded_nodes = expanded_nodes_obj if isinstance(expanded_nodes_obj, list) else []

    path_nodes_obj = result.get("path", [])
    path_nodes = path_nodes_obj if isinstance(path_nodes_obj, list) else []

    for obs in obstacles:
        grid[obs[0]][obs[1]] = 1

    for node in expanded_nodes:
        if node != start and node != goal:
            grid[node[0]][node[1]] = 2

    if result.get("success", False):
        for node in path_nodes:
            if node != start and node != goal:
                grid[node[0]][node[1]] = 3

    grid[start[0]][start[1]] = 4
    grid[goal[0]][goal[1]] = 5

    colors = ["white", "black", "#9bd4f8", "#47b647", "#d63636", "#ffd24d"]
    cmap = ListedColormap(colors)

    ax.imshow(grid, cmap=cmap, interpolation="nearest", origin="upper")

    ax.set_xticks(np.arange(-0.5, cols, 1), minor=True)
    ax.set_yticks(np.arange(-0.5, rows, 1), minor=True)
    ax.grid(which="minor", color="gray", linestyle="-", linewidth=0.5)
    ax.tick_params(which="minor", size=0)

    ax.set_xticks(np.arange(0, cols, 1))
    ax.set_yticks(np.arange(0, rows, 1))

    status = "PATH FOUND" if result.get("success", False) else "NO PATH"
    plt.title(
        f"{title_prefix} - {status} | Grid {rows}x{cols} | "
        f"Expanded: {result.get('nodes_expanded', 0)} | "
        f"Cost: {result.get('total_cost', float('inf')):.2f} | "
        f"Time: {result.get('execution_time_ms', 0.0):.2f}ms",
        fontsize=13,
        fontweight="bold",
    )

    legend_elements = [
        patches.Patch(facecolor="#d63636", edgecolor="black", label="Start"),
        patches.Patch(facecolor="#ffd24d", edgecolor="black", label="Goal"),
        patches.Patch(facecolor="black", edgecolor="black", label="Obstacle"),
        patches.Patch(facecolor="#9bd4f8", edgecolor="black", label="Expanded"),
        patches.Patch(facecolor="#47b647", edgecolor="black", label="Path"),
    ]
    ax.legend(handles=legend_elements, loc="upper left", bbox_to_anchor=(1.02, 1))

    ax.text(start[1], start[0], "S", ha="center", va="center", color="white", fontweight="bold")
    ax.text(goal[1], goal[0], "G", ha="center", va="center", color="black", fontweight="bold")

    plt.tight_layout()
    plt.show()


def run_lpa_analysis() -> None:
    rows, cols = 15, 15
    start = (0, 0)
    goal = (14, 14)
    obstacles = [
        (5, 5),
        (5, 6),
        (5, 7),
        (6, 7),
        (7, 7),
        (8, 7),
        (9, 7),
    ]

    planner = LPAStarPathfinder((rows, cols), obstacles)
    initial = planner.plan(start, goal)
    print("Initial search:")
    print(initial)

    updates = [(6, 6, True), (7, 6, True), (8, 6, True)]
    replanned = planner.apply_obstacle_changes(updates)
    print("After obstacle updates:")
    print(replanned)


if __name__ == "__main__":
    run_lpa_analysis()
