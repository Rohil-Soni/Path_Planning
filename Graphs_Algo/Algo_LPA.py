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
    - sparse dictionaries for g/rhs values
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

        # 8-neighbor grid for Euclidean-heuristic LPA*.
        self._directions = [
            (-1, 0),
            (1, 0),
            (0, -1),
            (0, 1),
            (-1, -1),
            (-1, 1),
            (1, -1),
            (1, 1),
        ]

        self.start: Optional[GridNode] = None
        self.goal: Optional[GridNode] = None

        self.g: Dict[GridNode, float] = {}
        self.rhs: Dict[GridNode, float] = {}

        # Priority queue entries are ((k1, k2), node); _in_q tracks currently valid key.
        self.open_heap: List[Tuple[Tuple[float, float], GridNode]] = []
        self._in_q: Dict[GridNode, Tuple[float, float]] = {}

        self.initialized = False

    def _in_bounds(self, node: GridNode) -> bool:
        r, c = node
        return 0 <= r < self.rows and 0 <= c < self.cols

    def _is_blocked(self, node: GridNode) -> bool:
        r, c = node
        return self.obstacle_grid[r][c]

    def heuristic(self, node: GridNode, goal: GridNode) -> float:
        return math.hypot(node[0] - goal[0], node[1] - goal[1])

    def _neighbor_coords_all(self, node: GridNode) -> List[GridNode]:
        x, y = node
        out: List[GridNode] = []
        for dx, dy in self._directions:
            nx, ny = x + dx, y + dy
            if 0 <= nx < self.rows and 0 <= ny < self.cols:
                out.append((nx, ny))
        return out

    def get_neighbors(self, node: GridNode) -> List[Tuple[GridNode, float]]:
        if self._is_blocked(node):
            return []

        x, y = node
        out: List[Tuple[GridNode, float]] = []
        for dx, dy in self._directions:
            nx, ny = x + dx, y + dy
            n = (nx, ny)
            if not self._in_bounds(n) or self._is_blocked(n):
                continue

            cost = self._move_cost(node, n)
            if cost < self.inf:
                out.append((n, cost))
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

    def _g(self, node: GridNode) -> float:
        return self.g.get(node, self.inf)

    def _rhs(self, node: GridNode) -> float:
        return self.rhs.get(node, self.inf)

    def _calculate_key(self, node: GridNode) -> Tuple[float, float]:
        if self.goal is None:
            return (self.inf, self.inf)
        base = min(self._g(node), self._rhs(node))
        return (base + self.heuristic(node, self.goal), base)

    def _add_or_update_open(self, node: GridNode) -> None:
        key = self._calculate_key(node)
        heapq.heappush(self.open_heap, (key, node))
        self._in_q[node] = key

    def _remove_open(self, node: GridNode) -> None:
        self._in_q.pop(node, None)

    def _peek_valid_key(self) -> Tuple[float, float]:
        while self.open_heap:
            key, node = self.open_heap[0]
            if self._in_q.get(node) != key:
                heapq.heappop(self.open_heap)
                continue
            return key
        return (self.inf, self.inf)

    def _pop_valid_node(self) -> Optional[GridNode]:
        while self.open_heap:
            key, node = heapq.heappop(self.open_heap)
            if self._in_q.get(node) != key:
                continue
            del self._in_q[node]
            return node
        return None

    def _initialize(self, start: GridNode, goal: GridNode) -> None:
        self.start = start
        self.goal = goal
        self.initialized = True

        self.g = {}
        self.rhs = {}

        self.open_heap = []
        self._in_q = {}

        self.rhs[start] = 0.0
        self._add_or_update_open(start)

    def _shift_start(self, new_start: GridNode) -> None:
        """
        Move the planning start without discarding accumulated search state.

        This keeps prior g/rhs information and only repairs local consistency,
        which is the behavior needed for obstacle hit -> predecessor replanning.
        """
        if self.start is None or self.goal is None:
            self._initialize(new_start, self.goal if self.goal is not None else new_start)
            return

        old_start = self.start
        if old_start == new_start:
            return

        self.start = new_start

        # Previous start is no longer the source.
        self.rhs[old_start] = self.inf
        self._update_vertex(old_start)

        # New start must satisfy rhs(start)=0 and stay in OPEN if inconsistent.
        self.rhs[new_start] = 0.0
        self._remove_open(new_start)
        if self._g(new_start) != self._rhs(new_start):
            self._add_or_update_open(new_start)

        # Local repairs around both old and new starts propagate quickly.
        for nbr in self._neighbor_coords_all(old_start):
            self._update_vertex(nbr)
        for nbr in self._neighbor_coords_all(new_start):
            self._update_vertex(nbr)

    def _update_vertex(self, u: GridNode) -> None:
        if self.start is None:
            return

        # Blocked cells (except start/goal) never need to stay in OPEN or keep rhs entries.
        if u != self.start and u != self.goal and self._is_blocked(u):
            self._remove_open(u)
            self.rhs.pop(u, None)
            self.g.pop(u, None)
            return

        if u != self.start:
            best_rhs = self.inf
            for pred, cost in self.get_neighbors(u):
                cand = self._g(pred) + cost
                if cand < best_rhs:
                    best_rhs = cand
            self.rhs[u] = best_rhs

        self._remove_open(u)
        if self._g(u) != self._rhs(u):
            self._add_or_update_open(u)

    def _compute_shortest_path(self) -> Tuple[List[GridNode], int]:
        if self.goal is None:
            return [], 0

        expanded_nodes: List[GridNode] = []
        expanded_count = 0

        while (
            self._peek_valid_key() < self._calculate_key(self.goal)
            or self._rhs(self.goal) != self._g(self.goal)
        ):
            u = self._pop_valid_node()
            if u is None:
                break

            expanded_nodes.append(u)
            expanded_count += 1

            if self._g(u) > self._rhs(u):
                self.g[u] = self._rhs(u)
                for succ, _ in self.get_neighbors(u):
                    self._update_vertex(succ)
            else:
                self.g[u] = self.inf
                self._update_vertex(u)
                for succ, _ in self.get_neighbors(u):
                    self._update_vertex(succ)

        return expanded_nodes, expanded_count

    def _extract_path(self) -> List[GridNode]:
        if self.start is None or self.goal is None:
            return []

        if self._g(self.goal) == self.inf:
            return []

        path = [self.goal]
        visited: Set[GridNode] = {self.goal}
        current = self.goal

        for _ in range(self.rows * self.cols):
            if current == self.start:
                break

            best_pred: Optional[GridNode] = None
            best_cost = self.inf
            for pred, cost in self.get_neighbors(current):
                cand = self._g(pred) + cost
                if cand < best_cost:
                    best_cost = cand
                    best_pred = pred

            if best_pred is None or best_pred in visited:
                return []

            path.append(best_pred)
            visited.add(best_pred)
            current = best_pred

        if not path or path[-1] != self.start:
            return []

        path.reverse()
        return path

    def plan(self, start: GridNode, goal: GridNode) -> Dict[str, object]:
        if not self._in_bounds(start) or not self._in_bounds(goal):
            raise ValueError("Start/goal must be inside grid bounds")
        if self._is_blocked(start) or self._is_blocked(goal):
            raise ValueError("Start/goal cannot be blocked")

        t0 = time.time()

        if not self.initialized or self.goal != goal:
            self._initialize(start, goal)
        elif self.start != start:
            self._shift_start(start)

        expanded_nodes, expanded_count = self._compute_shortest_path()
        path = self._extract_path()

        t1 = time.time()
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
            "total_cost": self._g(goal),
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

        self.update_graph(changed_cells)

        result = self.plan(self.start, self.goal)
        result["changed_cells"] = changed_cells
        return result

    def update_graph(self, changed_nodes: List[GridNode]) -> None:
        if not changed_nodes:
            return

        # Deduplicate affected cells so each vertex is repaired at most once per update.
        affected: Set[GridNode] = set()
        for node in changed_nodes:
            if not self._in_bounds(node):
                continue
            affected.add(node)
            for nbr in self._neighbor_coords_all(node):
                affected.add(nbr)

        for node in affected:
            self._update_vertex(node)

    def get_calculation_snapshot(self, nodes: List[GridNode]) -> List[Dict[str, object]]:
        out: List[Dict[str, object]] = []
        for node in nodes:
            if not self._in_bounds(node):
                continue
            k1, k2 = self._calculate_key(node)
            h = self.heuristic(node, self.goal) if self.goal is not None else self.inf
            out.append(
                {
                    "node": node,
                    "g": self._g(node),
                    "rhs": self._rhs(node),
                    "h": h,
                    "key": (k1, k2),
                    "consistent": self._g(node) == self._rhs(node),
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
