from typing import Any, Dict, List, Optional, Sequence, Tuple, Union
import math
import pickle
import numpy as np


Point = Union[Tuple[float, float], List[float], np.ndarray]
GridPoint = Union[Tuple[int, int], List[int], np.ndarray]


class MapManager:
    def __init__(self) -> None:
        self.point_cloud: Optional[np.ndarray] = None
        self.bounds_min: Optional[np.ndarray] = None
        self.bounds_max: Optional[np.ndarray] = None
        self.resolution_from_pkl: Optional[float] = None
        self.semantic_mapping: Optional[Dict[str, Any]] = None
        self.grid: Optional[np.ndarray] = None
        self.meta: Optional[Dict[str, float]] = None

    def load_map(self, pkl_path: str) -> None:
        """
        Load voxel PKL saved by the map generator and cache the metadata.
        """
        with open(pkl_path, "rb") as f:
            data = pickle.load(f)
        if "point_cloud" not in data:
            raise ValueError("PKL missing 'point_cloud' key.")
        pc = np.asarray(data["point_cloud"])
        if pc.ndim != 2 or pc.shape[1] < 2:
            raise ValueError("point_cloud must be a 2D array with at least 2 columns.")
        self.point_cloud = pc[:, :3]
        self.bounds_min = np.asarray(data.get("bounds_min", [float("nan")] * 3), dtype=float)
        self.bounds_max = np.asarray(data.get("bounds_max", [float("nan")] * 3), dtype=float)
        self.resolution_from_pkl = float(data.get("resolution", 1.0))
        self.semantic_mapping = data.get("semantic_mapping", None)
        print(f"Map loaded: {pkl_path}")
        print(f"Bounds min: {self.bounds_min}, Bounds max: {self.bounds_max}, Resolution: {self.resolution_from_pkl}")
        print(f"Semantic mapping: {self.semantic_mapping}")
        print(f"Point cloud: {self.point_cloud.shape}")

    def world_to_grid(self, point_xy: Union[Point, Sequence[Point]]) -> Union[Tuple[int, int], List[Tuple[int, int]]]:
        """
        Convert world (x,y) to grid (col,row). Accepts one point or a sequence of points.
        """
        if self.meta is None:
            raise RuntimeError("Occupancy grid not built yet.")
        def _one(p: Point) -> Tuple[int, int]:
            x, y = float(p[0]), float(p[1])
            col = int(math.floor((x - self.meta['xmin']) / self.meta['resolution']))
            row = int(math.floor((y - self.meta['ymin']) / self.meta['resolution']))
            return (col, row)
        if isinstance(point_xy, (list, tuple, np.ndarray)) and len(point_xy) > 0 and isinstance(point_xy[0], (list, tuple, np.ndarray)):
            return [_one(p) for p in point_xy]  # type: ignore
        return _one(point_xy)  # type: ignore

    def grid_to_world(self, point_grid_xy: Union[GridPoint, Sequence[GridPoint]]) -> Union[Tuple[float, float], List[Tuple[float, float]]]:
        """
        Convert grid (col,row) to world (x,y). Accepts one point or a sequence of points.
        """
        if self.meta is None:
            raise RuntimeError("Occupancy grid not built yet.")
        def _one(p: GridPoint) -> Tuple[float, float]:
            col, row = int(p[0]), int(p[1])
            x = self.meta['xmin'] + (col + 0.5) * self.meta['resolution']
            y = self.meta['ymin'] + (row + 0.5) * self.meta['resolution']
            return (x, y)
        if isinstance(point_grid_xy, (list, tuple, np.ndarray)) and len(point_grid_xy) > 0 and isinstance(point_grid_xy[0], (list, tuple, np.ndarray)):
            return [_one(p) for p in point_grid_xy]  # type: ignore
        return _one(point_grid_xy)  # type: ignore

    def get_occupancy_grid(
        self,
        height_z: Optional[float],
        z_band: float = 1.0,
        override_resolution: Optional[float] = None
    ) -> Tuple[np.ndarray, Dict[str, float]]:
        """
        Build an occupancy grid at a given height slice. Returns (grid, meta).
        """
        if self.point_cloud is None or self.bounds_min is None or self.bounds_max is None:
            raise RuntimeError("Map not loaded. Call load_map() first.")
        ox, oy, _ = self._extract_2d_obstacles_from_height_layer(
            self.point_cloud, height_z, z_band=z_band
        )
        if len(ox) == 0:
            raise RuntimeError("No obstacle points extracted from PKL at requested height.")
        res = override_resolution if override_resolution is not None else (self.resolution_from_pkl if self.resolution_from_pkl is not None else 1.0)
        grid, meta = self._create_occupancy_grid(ox, oy, self.bounds_min, self.bounds_max, override_resolution=res)
        self.grid = grid
        self.meta = meta
        return grid, meta

    def get_free_corner(self, opposite_of: Optional[Tuple[int, int]] = None) -> Tuple[int, int]:
        """
        Pick a free cell from corners (and a couple of near-corners). If opposite_of is provided,
        pick the farthest candidate to encourage separation.
        """
        if self.grid is None:
            raise RuntimeError("Occupancy grid not built yet.")
        rows, cols = self.grid.shape
        candidates = [
            (0, 0),
            (cols - 1, 0),
            (0, rows - 1),
            (cols - 1, rows - 1),
            (cols // 10, rows // 10),
            (max(cols - 2, 0), max(rows - 2, 0)),
        ]
        free_candidates: List[Tuple[int, int]] = []
        for c in candidates:
            col, row = c
            if 0 <= row < rows and 0 <= col < cols and self.grid[row, col] == 0:
                free_candidates.append(c)
        if not free_candidates:
            free = np.argwhere(self.grid == 0)
            if free.size == 0:
                raise RuntimeError("No free cells in occupancy grid")
            r0, c0 = free[0]
            return (int(c0), int(r0))
        if opposite_of is None:
            return free_candidates[0]
        ox, oy = opposite_of
        best = max(free_candidates, key=lambda p: (p[0] - ox) ** 2 + (p[1] - oy) ** 2)
        return best

    def _extract_2d_obstacles_from_height_layer(
        self,
        pc: np.ndarray,
        z_height: Optional[float],
        z_band: float
    ) -> Tuple[List[float], List[float], Optional[float]]:
        """
        Convert a 3D (or 2D) point cloud to 2D obstacle points (ox, oy).
        """
        if pc.shape[1] >= 3:
            x = pc[:, 0].astype(float)
            y = pc[:, 1].astype(float)
            z = pc[:, 2].astype(float)
            if z_height is None:
                z_height = float(np.median(z))
            half = max(z_band / 2.0, 1e-6)
            mask = (z >= (z_height - half)) & (z <= (z_height + half))
            x2d = x[mask]
            y2d = y[mask]
            used_z = z_height
        else:
            x2d = pc[:, 0].astype(float)
            y2d = pc[:, 1].astype(float)
            used_z = None
        if len(x2d) == 0:
            return [], [], used_z
        return x2d.tolist(), y2d.tolist(), used_z

    def _create_occupancy_grid(
        self,
        ox: List[float],
        oy: List[float],
        bounds_min: np.ndarray,
        bounds_max: np.ndarray,
        override_resolution: Optional[float] = None
    ) -> Tuple[np.ndarray, Dict[str, float]]:
        """
        Build occupancy grid with obstacles marked as 50.
        """
        xmin, ymin = float(bounds_min[0]), float(bounds_min[1])
        xmax, ymax = float(bounds_max[0]), float(bounds_max[1])
        resolution = float(override_resolution) if override_resolution is not None else 1.0
        if xmax <= xmin:
            xmax = xmin + 1.0
        if ymax <= ymin:
            ymax = ymin + 1.0
        cols = int(math.ceil((xmax - xmin) / resolution))
        rows = int(math.ceil((ymax - ymin) / resolution))
        grid = np.zeros((rows, cols), dtype=float)
        for x, y in zip(ox, oy):
            col = int(math.floor((x - xmin) / resolution))
            row = int(math.floor((y - ymin) / resolution))
            if 0 <= row < rows and 0 <= col < cols:
                grid[row, col] = 50
        meta = {
            "xmin": xmin,
            "ymin": ymin,
            "xmax": xmax,
            "ymax": ymax,
            "rows": rows,
            "cols": cols,
            "resolution": resolution,
        }
        return grid, meta