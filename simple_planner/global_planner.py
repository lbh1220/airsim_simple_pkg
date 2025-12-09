"""
 *  Convex Safety Corridor Generator with Normal-Slice Expansion
 *  and Minimum-Snap Trajectory Optimization
 *  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 *  Copyright (c) 2025 Gerry (刘耕睿)
 *  https://github.com/Gerrylgr/ 
 *  https://space.bilibili.com/673367025
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 *  Author        : 刘耕睿 (Gerry)
 *  Contact       : 2717915639@qq.com
 *  Created       : 2025-11
 *  Last Modified : 2025-11-30
 *
 *  Description   : Strictly convex safety corridor generation based on
 *                  waypoint-centered inflation and greedy normal slicing.
 *                  Integrated with 4th-order Minimum-Snap optimization.
"""

import numpy as np
import math
import heapq
import matplotlib.pyplot as plt
import cvxpy as cp 
from matplotlib.patches import Polygon as MplPolygon
from shapely.geometry import Polygon, LineString
from shapely import vectorized 
from shapely.ops import split
import time

# ------------------ A*算法 ------------------
def astar(start, goal, grid):
    rows, cols = grid.shape
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}

    def heuristic(a, b):
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

    visited = set()
    while open_set:
        _, current = heapq.heappop(open_set)
        if current in visited:
            continue
        visited.add(current)

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]

        for dx, dy in [(1,0),(-1,0),(0,1),(0,-1),(1,1),(-1,1),(-1,-1),(1,-1)]:
            neighbor = (current[0]+dx, current[1]+dy)
            if (0 <= neighbor[0] < cols and 0 <= neighbor[1] < rows and grid[neighbor[1], neighbor[0]] == 0):
                tentative_g = g_score[current] + math.sqrt(dx**2 + dy**2)
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f = tentative_g + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f, neighbor))
    return None

# ------------------ JPS算法 ------------------
def jps(start, goal, grid):
    """
    Jump Point Search on 8-connected grid.
    - Coordinates are (x, y) with x = col, y = row.
    - Free cell == 0, obstacle otherwise (kept consistent with astar()).
    Returns list of points from start to goal (inclusive) or None.
    """
    rows, cols = grid.shape

    def in_bounds(x, y):
        return 0 <= x < cols and 0 <= y < rows

    def is_passable(x, y):
        return in_bounds(x, y) and grid[y, x] == 0

    def octile(a, b):
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])
        dmin = min(dx, dy)
        dmax = max(dx, dy)
        return dmin * math.sqrt(2.0) + (dmax - dmin) * 1.0

    def reconstruct_path(came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def get_pruned_directions(cx, cy, parent):
        """
        Return movement directions to consider from current based on JPS pruning.
        Each direction is a (dx, dy) with components in {-1, 0, 1} excluding (0,0).
        """
        if parent is None:
            directions = []
            for dx, dy in [(1,0),(-1,0),(0,1),(0,-1),(1,1),(-1,1),(-1,-1),(1,-1)]:
                nx, ny = cx + dx, cy + dy
                if dx != 0 and dy != 0:
                    # No corner cutting
                    if is_passable(cx + dx, cy) and is_passable(cx, cy + dy) and is_passable(nx, ny):
                        directions.append((dx, dy))
                else:
                    if is_passable(nx, ny):
                        directions.append((dx, dy))
            return directions

        px, py = parent
        dx = int(math.copysign(1, cx - px)) if cx != px else 0
        dy = int(math.copysign(1, cy - py)) if cy != py else 0

        directions = []
        if dx != 0 and dy != 0:
            # Diagonal move: natural neighbors are (dx, dy), (dx, 0), (0, dy)
            if is_passable(cx + dx, cy) and is_passable(cx, cy + dy):
                if is_passable(cx + dx, cy + dy):
                    directions.append((dx, dy))
            if is_passable(cx + dx, cy):
                directions.append((dx, 0))
            if is_passable(cx, cy + dy):
                directions.append((0, dy))

            # Forced neighbors for diagonal
            if not is_passable(cx - dx, cy + dy) and is_passable(cx, cy + dy):
                directions.append((0, dy))
            if not is_passable(cx + dx, cy - dy) and is_passable(cx + dx, cy):
                directions.append((dx, 0))
        elif dx != 0:
            # Horizontal move: natural neighbor is (dx, 0)
            if is_passable(cx + dx, cy):
                directions.append((dx, 0))
            # Forced up/down diagonals
            if not is_passable(cx, cy + 1) and is_passable(cx + dx, cy + 1):
                directions.append((dx, 1))
            if not is_passable(cx, cy - 1) and is_passable(cx + dx, cy - 1):
                directions.append((dx, -1))
        else:
            # Vertical move: natural neighbor is (0, dy)
            if is_passable(cx, cy + dy):
                directions.append((0, dy))
            # Forced left/right diagonals
            if not is_passable(cx + 1, cy) and is_passable(cx + 1, cy + dy):
                directions.append((1, dy))
            if not is_passable(cx - 1, cy) and is_passable(cx - 1, cy + dy):
                directions.append((-1, dy))

        # Deduplicate
        uniq = []
        seen = set()
        for d in directions:
            if d not in seen and d != (0, 0):
                seen.add(d)
                uniq.append(d)
        return uniq

    def jump(x, y, dx, dy):
        """
        Move from (x, y) step by step in (dx, dy) until:
        - reaches goal
        - hits a forced neighbor
        - blocked
        Return (jx, jy) if a jump point is found, else None.
        """
        nx = x
        ny = y
        while True:
            nx += dx
            ny += dy
            if not is_passable(nx, ny):
                return None
            if (nx, ny) == goal:
                return (nx, ny)

            # Forced neighbor checks
            if dx != 0 and dy != 0:
                # Diagonal: if either side is blocked while the forward-side is open, it's forced
                if (not is_passable(nx - dx, ny + dy) and is_passable(nx, ny + dy)) or \
                   (not is_passable(nx + dx, ny - dy) and is_passable(nx + dx, ny)):
                    return (nx, ny)
                # When moving diagonally, also check straight jumps
                if jump(nx, ny, dx, 0) is not None or jump(nx, ny, 0, dy) is not None:
                    return (nx, ny)
                # Corner cutting prevention for next step
                if not (is_passable(nx + dx, ny) and is_passable(nx, ny + dy)):
                    return None
            elif dx != 0:
                # Horizontal
                if (not is_passable(nx, ny + 1) and is_passable(nx + dx, ny + 1)) or \
                   (not is_passable(nx, ny - 1) and is_passable(nx + dx, ny - 1)):
                    return (nx, ny)
            else:
                # Vertical
                if (not is_passable(nx + 1, ny) and is_passable(nx + 1, ny + dy)) or \
                   (not is_passable(nx - 1, ny) and is_passable(nx - 1, ny + dy)):
                    return (nx, ny)

    if not (is_passable(start[0], start[1]) and is_passable(goal[0], goal[1])):
        return None

    open_heap = []
    entry_id = 0
    g_score = {start: 0.0}
    f_score = {start: octile(start, goal)}
    heapq.heappush(open_heap, (f_score[start], entry_id, start, None))  # (f, id, node, parent)
    entry_id += 1

    came_from = {}
    closed = set()

    while open_heap:
        _, _, current, parent = heapq.heappop(open_heap)
        if current in closed:
            continue
        closed.add(current)

        if current == goal:
            return reconstruct_path(came_from, current)

        curx, cury = current
        directions = get_pruned_directions(curx, cury, parent)
        for dx, dy in directions:
            jp = jump(curx, cury, dx, dy)
            if jp is None:
                continue
            tentative_g = g_score[current] + octile(current, jp)
            if jp not in g_score or tentative_g < g_score[jp]:
                g_score[jp] = tentative_g
                came_from[jp] = current
                fval = tentative_g + octile(jp, goal)
                heapq.heappush(open_heap, (fval, entry_id, jp, current))
                entry_id += 1
    return None

def create_normal_line(point1, point2, length=50):
    dx, dy = np.array(point2) - np.array(point1)
    vector = np.array([-dy, dx]) 
    norm = np.linalg.norm(vector)
    dir_vec = vector / norm if norm > 1e-6 else vector
    start = point2 + dir_vec * length / 2
    end = point2 - dir_vec * length / 2
    return LineString([(start[0], start[1]), (end[0], end[1])])

def convex_corridor(path, grid, max_width=10.0, extend=8.0):
    """
    修正版 convex_corridor，内部将 path (col,row) -> (x,y) 统一处理。
    返回的 rectangles 为 list of np.array of shape (4,2)（点为 (x,y) 顺序）。
    """
    rows, cols = grid.shape
    rectangles = []

    # 把 path 转为 (x, y) 语义：x = col, y = row
    path_xy = [ (float(p[0]), float(p[1])) for p in path ]  # (x, y)

    for i in range(len(path_xy) - 1):
        p1 = np.array(path_xy[i], dtype=float)    # (x, y)
        p2 = np.array(path_xy[i+1], dtype=float)  # (x, y)

        seg_vec = p2 - p1
        seg_len = np.linalg.norm(seg_vec)
        if seg_len < 1e-6:
            half = max_width * 0.5
            corners = np.array([
                [p1[0]-half, p1[1]-half],
                [p1[0]+half, p1[1]-half],
                [p1[0]+half, p1[1]+half],
                [p1[0]-half, p1[1]+half],
            ])
        else:
            unit = seg_vec / seg_len         # 沿线单位向量 (x,y)
            orth = np.array([-unit[1], unit[0]])   # 垂直单位向量 (x,y)

            half_w = max_width

            c1 = p1 + orth * half_w - unit * extend
            c2 = p1 - orth * half_w - unit * extend
            c3 = p2 - orth * half_w + unit * extend
            c4 = p2 + orth * half_w + unit * extend
            corners = np.vstack((c1, c2, c3, c4))

        corners[:, 0] = np.clip(corners[:, 0], 0, cols - 1)  # x in [0, cols-1]
        corners[:, 1] = np.clip(corners[:, 1], 0, rows - 1)  # y in [0, rows-1]

        rectangles.append(corners)

    return rectangles

def points_inside_polygon_mask_fast(poly, rows, cols):  
    if len(poly) < 3:
        return np.zeros((rows, cols), dtype=bool)

    xs = np.arange(cols) + 0.5
    ys = np.arange(rows) + 0.5
    XX, YY = np.meshgrid(xs, ys)

    mask = vectorized.contains(Polygon(poly), XX, YY)
    return mask

def corridor_generator_optimized(path, corridor, grid, max_width=10.0):
    path_xy = [ (float(p[0]), float(p[1])) for p in path ]
    rows, cols = grid.shape

    # 对每个初始膨胀多边形进行切割
    for i in range(len(corridor)):
        poly = np.array(corridor[i], dtype=float)
        if poly.size == 0:
            continue

        # 获取多边形顶点坐标
        min_x = int(np.floor(np.min(poly[:,0])))
        max_x = int(np.ceil (np.max(poly[:,0])))
        min_y = int(np.floor(np.min(poly[:,1])))
        max_y = int(np.ceil (np.max(poly[:,1])))

        # 计算子地图顶点坐标（后续切割在子地图中进行可以大量节省计算资源）
        min_x = int(max(0, min_x - int(max_width)))
        max_x = int(min(cols - 1, max_x + int(max_width)))
        min_y = int(max(0, min_y - int(max_width)))
        max_y = int(min(rows - 1, max_y + int(max_width)))

        # 提取子地图
        sub_grid = grid[min_y:max_y+1, min_x:max_x+1]
        delta_row, delta_col = sub_grid.shape

        # 将多边形坐标转换到子地图坐标系下
        poly_local = poly.copy()
        poly_local[:,0] -= min_x
        poly_local[:,1] -= min_y

        # 获取布尔掩码
        mask = points_inside_polygon_mask_fast(poly_local, delta_row, delta_col)

        # mask 中为 True 并且代价值为50的就是障碍物点
        obs_cells = np.argwhere(mask & (sub_grid == 50))
        if obs_cells.size == 0:
            continue
        # 计算世界坐标系下的障碍物点坐标
        obs_cells = obs_cells + np.array([min_y, min_x])

        del_array = []
        # 递归根据障碍物点切割多边形
        while obs_cells.size > 0:
            r, c = obs_cells[0]     # 障碍物点坐标
            print(f"Point ({c}, {r}) is inside rectangle {i}")

            # 路径中点
            point1 = ((path_xy[i][0] + path_xy[i+1][0]) / 2.0,
                    (path_xy[i][1] + path_xy[i+1][1]) / 2.0)
            point2 = (float(c), float(r))
            # 创建切割线
            line = create_normal_line(point1, point2, length=50)
            del_array.append((r, c))

            poly_full = Polygon(corridor[i])
            # 获取切割结果
            result = split(poly_full, line)

            # 遍历切割结果
            for m in range(len(result.geoms)):
                candi_polygon = result.geoms[m]  
                # 顶点坐标   
                coords = np.array(candi_polygon.exterior.coords[:-1])
                coords_local = coords.copy()
                # 顶点坐标转换到子地图坐标系下
                coords_local[:,0] -= min_x
                coords_local[:,1] -= min_y

                # 新的布尔掩码
                mask1 = points_inside_polygon_mask_fast(coords_local, delta_row, delta_col)

                # 路径中点在子地图中坐标
                mid_x = (path_xy[i][0] + path_xy[i+1][0]) / 2.0 - min_x
                mid_y = (path_xy[i][1] + path_xy[i+1][1]) / 2.0 - min_y
                
                # 确保路径中点在子地图中
                if not (0 <= int(np.floor(mid_y)) < delta_row and 0 <= int(np.floor(mid_x)) < delta_col):
                    continue

                # 包含路径中点的多边形是要保留的多边形
                if mask1[int(np.floor(mid_y)), int(np.floor(mid_x))]:
                    corridor[i] = coords    # 更新多边形
                    mask = mask1    # 更新布尔掩码
                    obs_cells_local = np.argwhere(mask & (sub_grid == 50))      # 更新障碍物点
                    obs_cells = obs_cells_local + np.array([min_y, min_x])
                    # 删除切割过的点
                    if del_array:
                        obs_cells = np.array([row for row in obs_cells if not any((row == np.array(del_array)).all(axis=1))])
                    break
    return corridor

def compute_angles(points):
    points = np.asarray(points, dtype=float)
    n = len(points)
    if n < 3:
        return np.array([np.pi] * n, dtype=float)
    angles = np.empty(n, dtype=float)
    angles[0] = np.pi
    angles[-1] = np.pi
    for i in range(1, n - 1):
        v1 = points[i]   - points[i - 1]
        v2 = points[i+1] - points[i]
        n1 = np.linalg.norm(v1)
        n2 = np.linalg.norm(v2)
        if n1 < 1e-12 or n2 < 1e-12:
            angles[i] = 0.0
            continue
        cos_angle = np.dot(v1, v2) / (n1 * n2)
        cos_angle = np.clip(cos_angle, -1.0, 1.0)
        angles[i] = np.arccos(cos_angle)
    return angles

def simplify_path(points, corner_deg=30, corner_dilate=1):
    points = np.asarray(points, dtype=float)
    n = len(points)
    if n <= 2:
        return points.copy()

    angles = compute_angles(points)
    theta = np.deg2rad(corner_deg)      
    corner_idx = np.where(angles >= theta)[0]
    corner_idx = np.unique(np.concatenate(([0], corner_idx, [n-1])))

    if corner_dilate > 0:
        extra = []
        for idx in corner_idx:
            for j in range(idx - corner_dilate, idx + corner_dilate + 1):
                if 0 <= j < n:
                    extra.append(j)
        corner_idx = np.unique(np.concatenate((corner_idx, np.array(extra, dtype=int))))

    out = points[corner_idx]
    return np.array(out, dtype=float)

def polygon_to_inequalities(vertices):
        n = len(vertices)
        A, b = [], []
        for i in range(n):
            p1 = vertices[i]    # 当前顶点 
            p2 = vertices[(i + 1) % n]      # 下一个顶点
            edge = p2 - p1
            normal = np.array([edge[1], -edge[0]])  # 法向量

            center = np.mean(vertices, axis=0)      # 计算所有顶点的几何中心（质心）
            # 点积 > 0，说明法向量与内部向量的夹角小于90度，即法向量指向了多边形内部（此时将法向量取反 normal = -normal，确保它始终指向外部）
            if np.dot(normal, center - p1) > 0:
                normal = -normal

            A.append(normal)
            b.append(np.dot(normal, p1))
        return np.array(A), np.array(b)

def minimum_snap_solver(corridor, grid, path, N, dim, solver="OSQP", lambda_center=2.5):
        """
        Minimumsnap问题求解函数
        输入: corridor:凸多边形膨胀走廊
            grid:栅格代价地图
            path: 路径点
            N: 路径长度
            dim: 维度数
            solver: 求解器选择
            lambda_center: 中点偏移量权重系数
        输出: 优化后的路径点
        """
        # path = [ (float(p[1]), float(p[0])) for p in path ] 
        # ---------------- 差分矩阵 ----------------
        # 构造差分矩阵 S（用于 snap，即 4 阶差分），若 N<5 用 2 阶差分矩阵（加速度最小化）
        def make_diff_matrix(order, N):
            # 差分阶数 order 大于或等于点的数量 N，我们无法计算任何差分。例如，你不能用3个点去计算4阶差分
            if order >= N:
                return np.zeros((0, N))
            S = np.zeros((N - order, N))
            coeff = np.array([1.])
            for _ in range(order):
                # 通过卷积来递归地计算差分系数
                coeff = np.convolve(coeff, np.array([1, -1]))
            # 循环 N - order 次，对应矩阵的每一行
            for i in range(N - order):
                S[i, i:i+order+1] = coeff       # 选中了第 i 行，从第 i 列到第 i+order 列（共 order+1 个元素）
            return S

        if N >= 5:
            S = make_diff_matrix(4, N)   # snap
            # X[:,0] 和 X[:,1] 分别取路径点的 x 和 y 坐标；sum_squares 用于计算平方和
            obj_expr = lambda X: cp.sum_squares(S @ X[:,0]) + cp.sum_squares(S @ X[:,1])
        elif N >= 3:
            S = make_diff_matrix(2, N)   # accel
            obj_expr = lambda X: cp.sum_squares(S @ X[:,0]) + cp.sum_squares(S @ X[:,1])
        else:
            S = make_diff_matrix(1, N)   # velocity
            obj_expr = lambda X: cp.sum_squares(S @ X[:,0]) + cp.sum_squares(S @ X[:,1])

        # ---------------- 变量 ----------------
        # N个点，二维坐标
        traj = cp.Variable((N, dim))

        constraints = []
        # 起点终点硬约束
        constraints += [traj[0, :] == path[0]]
        constraints += [traj[-1, :] == path[-1]]

        # ---------------- 走廊约束 ----------------
        # 每个轨迹点落在对应的 corridor[i] 内
        for i in range(N-1):    # corridor 数量是 N-1（每段一个多边形）
            poly = corridor[i]
            if poly is None or len(poly) < 3:
                # 限制在网格边界内
                constraints += [traj[i,0] >= 0, traj[i,0] <= grid.shape[1]-1]
                constraints += [traj[i,1] >= 0, traj[i,1] <= grid.shape[0]-1]
            else:
                A_poly, b_poly = polygon_to_inequalities(np.array(poly))
                # 对 A_poly @ p <= b_poly 添加约束（允许一点容差）
                # cvxpy 需要将每行单独加约束
                for row_idx in range(A_poly.shape[0]):
                    arow = A_poly[row_idx]
                    brow = b_poly[row_idx] + 1e-6  # 少量容差
                    # 约束在走廊内部（Ax <= b）
                    constraints += [arow[0]*traj[i,0] + arow[1]*traj[i,1] <= brow]

        # 也可以对最后一点 traj[-1] 附加最后段的走廊（保证终点可行）
        if len(corridor) >= 1:
            A_poly, b_poly = polygon_to_inequalities(np.array(corridor[-1]))
            for row_idx in range(A_poly.shape[0]):
                arow = A_poly[row_idx]
                brow = b_poly[row_idx] + 1e-6
                constraints += [arow[0]*traj[-1,0] + arow[1]*traj[-1,1] <= brow]

        # ---------------- 目标函数 ----------------
        # 计算质心坐标
        def compute_centroid(polygon):
            x_coords = [p[0] for p in polygon]      # x坐标
            y_coords = [p[1] for p in polygon]      # y坐标
            n = len(polygon)
            area = 0.0
            cx = 0.0
            cy = 0.0
            for i in range(n):
                x0, y0 = polygon[i]
                x1, y1 = polygon[(i + 1) % n]
                # |OA × OB| 的值等于由向量 OA 和 OB 构成的平行四边形的面积。这个平行四边形的面积，正好是由三角形 OAB 构成的面积的两倍。
                cross = (x0 * y1) - (x1 * y0)
                area += cross
                """
                三角形 OAB 的质心 x 坐标是 (x0 + x1 + 0) / 3
                三角形 OAB 的面积是 |cross| / 2
                所以 (质心x * 面积) 这一项正比于 ( (x0+x1)/3 * |cross|/2 )
                公式中的 (x0 + x1) * cross 是这个值的6倍(包含了符号和常数因子)。常数因子不影响最终结果
                """
                cx += (x0 + x1) * cross
                cy += (y0 + y1) * cross
            area *= 0.5
            if area == 0:       # 处理退化情况（例如所有点共线，面积为0）
                return (sum(x_coords) / n, sum(y_coords) / n)
            cx /= (6 * area)        # 质心x坐标 = (累加的x分量) / (6 * 总面积)
            cy /= (6 * area)        # 质心y坐标 = (累加的y分量) / (6 * 总面积)
            return (cx, cy)

        ### 虽然前边已经有了硬约束，但是即使落在走廊内，轨迹可能贴边走，在数值误差或走廊过窄时容易“蹭到”障碍物
        ### 因此加入下边的软约束
        # 计算每个轨迹点对应的走廊中心
        centers = []
        for i in range(len(corridor)):
            poly = corridor[i]
            if len(poly) < 3:       # 三条边以内
                center_x = (path[i][0] + path[i+1][0]) / 2
                center_y = (path[i][1] + path[i+1][1]) / 2
                centers.append((center_x, center_y))
            else:
                centroid = compute_centroid(poly)
                centers.append(centroid)

        # 之所以将最后一段走廊单独再处理一遍，是因为traj有N个点，那么centers也必须有N个值
        poly = corridor[-1]
        if len(poly) < 3:
            # --- 最后一个多边形退化 ---
            # 逻辑：取连接该走廊的两个原始路径点的中点
            # path[-2] 是倒数第二个点，path[-1] 是最后一个点（终点）
            center_x = (path[-2][0] + path[-1][0]) / 2
            center_y = (path[-2][1] + path[-1][1]) / 2
            centers.append((center_x, center_y))
        else:
            centroid = compute_centroid(poly)
            centers.append(centroid)
        centers = np.array(centers)

        # 添加中心约束到优化目标
        centers_const = cp.Parameter(shape=(N, 2))      # 代表一个在优化问题中值是已知且固定的符号变量
        centers_const.value = centers
        # traj - centers_const: 这个结果矩阵代表了每个轨迹点与其对应参考点之间的坐标差向量
        center_term = cp.sum(cp.sum_squares(traj - centers_const))

        objective = cp.Minimize(obj_expr(traj) + lambda_center*center_term)

        # ---------------- 求解 ----------------
        prob = cp.Problem(objective, constraints)
        try:
            if solver == "ECOS":
                prob.solve(solver=cp.ECOS, verbose=False)
            elif solver == "OSQP":
                prob.solve(solver=cp.OSQP, verbose=False, eps_abs=1e-3, eps_rel=1e-3, max_iter=20000)
            else:
                print(f"INVALID solver-param: {solver}")
                raise NameError("Minimum-snap FAILED, returning!!!")
        except Exception as e:
            print("Primary solver failed:", e)
            # fallback
            prob.solve(solver=cp.SCS, verbose=False)

        if traj.value is None:
            print(f"Optimization failed, traj.value is None! Returning original path!")
            path = [ (float(p[1]), float(p[0])) for p in path ] 
            return path
        return traj.value


# I write a class to wrap the above functions
class GlobalPlanner:
    def __init__(self, max_width: float = 7.0, extend: float = 8.0, solver: str = "OSQP", lambda_center: float = 2.5, use_jps: bool = False) -> None:
        """
        Wrapper around A*, convex corridor generation and minimum-snap optimization.
        """
        self.max_width = float(max_width)
        self.extend = float(extend)
        self.solver = str(solver)
        self.lambda_center = float(lambda_center)
        self.use_jps = use_jps
    def find_path(self, start, goal, grid):
        """
        Run selected search on the occupancy grid.
        """
        if self.use_jps:
            path = jps(start, goal, grid)
            if path is not None:
                return path
            # Fallback to A* if JPS fails
            return astar(start, goal, grid)
        # Default A*
        return astar(start, goal, grid)

    def simplify(self, path_points, corner_deg: float = 30.0, corner_dilate: int = 1):
        """
        Simplify the path by preserving corner points.
        """
        pts = np.asarray(path_points, dtype=float)
        return simplify_path(pts, corner_deg=corner_deg, corner_dilate=corner_dilate)

    def build_corridor(self, simplified_path, grid):
        """
        Build convex safety corridor along the simplified path and refine it by cutting with obstacles.
        """
        base = convex_corridor(simplified_path, grid, max_width=self.max_width, extend=self.extend)
        refined = corridor_generator_optimized(simplified_path, base, grid, max_width=self.max_width)
        return refined

    def optimize(self, corridor, grid, path):
        """
        Run minimum-snap trajectory optimization within the corridor.
        """
        N = len(path) if not isinstance(path, np.ndarray) else path.shape[0]
        dim = 2
        traj = minimum_snap_solver(
            corridor=corridor,
            grid=grid,
            path=path,
            N=N,
            dim=dim,
            solver=self.solver,
            lambda_center=self.lambda_center,
        )
        return np.array(traj) if traj is not None else None

    def plan(self, grid, start, goal, corner_deg: float = 30.0, corner_dilate: int = 1):
        """
        Full pipeline: A* -> simplify -> corridor -> minimum-snap.
        Returns (path, simplified_path, corridor, optimized_traj).
        """
        path = self.find_path(start, goal, grid)
        if path is None:
            return None, None, None, None
        # simplified = self.simplify(path, corner_deg=corner_deg, corner_dilate=corner_dilate)
        simplified = np.array(path)
        corridor = self.build_corridor(simplified, grid)
        traj = self.optimize(corridor, grid, simplified)
        return path, simplified, corridor, traj