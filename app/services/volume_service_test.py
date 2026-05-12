# app/services/volume_service.py
import numpy as np
from typing import List, Optional
from matplotlib.path import Path as MplPath
from scipy.spatial import cKDTree
import logging

logger = logging.getLogger(__name__)
class VolumeEstimator:
    def __init__(
        self,
        grid_size: float = 0.1,
        enable_advanced: bool = True,          # 是否启用增强
        angle_of_repose_deg: float = 35.0,     # 安息角（度）
        wall_compensation: bool = True,        # 苍壁补偿
        base_model: str = "local_min"          # "global_min", "local_min", 或未来扩展
    ):
        if grid_size <= 0:
            raise ValueError("grid_size must be positive")
        self.grid_size = grid_size
        self.enable_advanced = enable_advanced
        self.angle_of_repose = np.radians(angle_of_repose_deg)
        self.wall_compensation = wall_compensation
        self.base_model = base_model

    def estimate_full_volume(self, points: np.ndarray) -> float:
        if points.size == 0 or points.shape[1] != 3:
            return 0.0
        return self._estimate_core(points)

    def estimate_volume_in_polygon_roi(self, points: np.ndarray, polygon: List[List[float]]) -> float:
        if points.size == 0 or len(polygon) < 3:
            return 0.0
        path = MplPath(polygon)
        inside_mask = path.contains_points(points[:, :2])
        roi_points = points[inside_mask]
        vol = self._estimate_core(roi_points)
        
        # 苍壁补偿（仅在 ROI 模式下启用）
        if self.enable_advanced and self.wall_compensation:
            wall_vol = self._compensate_wall_volume(roi_points, polygon)
            vol += wall_vol
        return vol

    def estimate_volumes_with_rois(self, points: np.ndarray, rois: List[List[List[float]]]) -> dict:
        total_vol = self.estimate_full_volume(points)
        results = []
        part_total = 0.0
        for idx, poly in enumerate(rois):
            if not isinstance(poly, list) or len(poly) < 3:
                vol = 0.0
                pt_count = 0
            else:
                path = MplPath(poly)
                mask = path.contains_points(points[:, :2])
                pt_count = int(np.sum(mask))
                vol = self.estimate_volume_in_polygon_roi(points, poly)
            part_total += vol
            results.append({
                "roi_index": idx,
                "polygon": poly,
                "point_count": pt_count,
                "volume_m3": round(vol, 3)
            })
        return {
            "total_volumes": round(total_vol, 3),
            "part_total_volume": round(part_total, 3),
            "grid_size_m": self.grid_size,
            "roi_volumes": results
        }

    def _estimate_core(self, points: np.ndarray) -> float:
        """核心体积计算"""
        if len(points) == 0:
            return 0.0

        # Step 1: 安息角约束修正（降低过陡区域高度）
        if self.enable_advanced:
            points = self._apply_angle_of_repose(points)

        # Step 2: 构建参考底面
        if self.enable_advanced and self.base_model == "local_min":
            base_z_map = self._build_local_base_surface(points)
        else:
            global_min_z = np.min(points[:, 2])
            base_z_map = None
        logger.info(f"Estimating volume with {len(base_z_map)} points - {base_z_map.size}")
        # Step 3: 栅格化计算体积
        gx = np.floor(points[:, 0] / self.grid_size).astype(np.int64)
        gy = np.floor(points[:, 1] / self.grid_size).astype(np.int64)
        cell_ids = gx * (np.max(gy) - np.min(gy) + 1) + gy

        sort_idx = np.argsort(cell_ids)
        sorted_cells = cell_ids[sort_idx]
        sorted_z = points[sort_idx, 2]
        sorted_x = points[sort_idx, 0]
        sorted_y = points[sort_idx, 1]

        _, idx_start = np.unique(sorted_cells, return_index=True)
        idx_end = np.append(idx_start[1:], len(sorted_cells))

        vol = 0.0
        cell_area = self.grid_size * self.grid_size

        for i, j in zip(idx_start, idx_end):
            max_z = np.max(sorted_z[i:j])
            x_center = sorted_x[i]
            y_center = sorted_y[i]

            if base_z_map is not None and base_z_map.size > 0:
                # 查找最近的底面高度（简化：用 cell 中心查）
                base_z = self._query_base_z(base_z_map, x_center, y_center)
            else:
                base_z = global_min_z

            dh = max_z - base_z
            if dh > 1e-6:
                vol += dh * cell_area

        return vol

    def _apply_angle_of_repose(self, points: np.ndarray) -> np.ndarray:
        if len(points) < 5:
            return points

        from scipy.spatial.distance import cdist

        xy = points[:, :2]
        z = points[:, 2]
        radius = 2.0 * self.grid_size
        max_slope = np.tan(self.angle_of_repose)

        # 计算所有点对的距离矩阵（仅适用于中小点云 < 50k）
        if len(points) > 50_000:
            # 点太多，跳过或降采样
            return points

        dist_matrix = cdist(xy, xy)
        corrected_z = z.copy()

        for i in range(len(points)):
            mask = (dist_matrix[i] <= radius) & (dist_matrix[i] > 1e-6)
            if np.sum(mask) < 3:
                continue

            dz = np.abs(z[mask] - z[i])
            dx = dist_matrix[i][mask]
            slopes = dz / dx

            if np.any(slopes > max_slope):
                corrected_z[i] = np.median(z[mask])

        new_points = points.copy()
        new_points[:, 2] = corrected_z
        return new_points

    def _build_local_base_surface(self, points: np.ndarray):
        """超快版：在粗网格上采样最低 Z 作为底面控制点"""
        if len(points) < 1000:
            # 点太少，直接返回全局最小点
            idx = np.argmin(points[:, 2])
            return points[idx:idx+1]

        # 粗网格（比 volume grid 大 5～10 倍）
        coarse_grid = 5 * self.grid_size
        x_min, x_max = points[:, 0].min(), points[:, 0].max()
        y_min, y_max = points[:, 1].min(), points[:, 1].max()

        x_bins = np.arange(x_min, x_max + coarse_grid, coarse_grid)
        y_bins = np.arange(y_min, y_max + coarse_grid, coarse_grid)

        base_points = []
        for i in range(len(x_bins) - 1):
            for j in range(len(y_bins) - 1):
                mask = (
                    (points[:, 0] >= x_bins[i]) & (points[:, 0] < x_bins[i+1]) &
                    (points[:, 1] >= y_bins[j]) & (points[:, 1] < y_bins[j+1])
                )
                if np.any(mask):
                    min_idx = np.argmin(points[mask, 2])
                    pt = points[np.where(mask)[0][min_idx]]
                    base_points.append(pt)

        return np.array(base_points) if base_points else np.array([points[np.argmin(points[:, 2])]])

    def _query_base_z(self, base_points: list, x: float, y: float) -> float:
        """查询最近底面点的高度（简化版）"""
        if not base_points or base_points.size == 0:
            return 0.0
        base_arr = np.array(base_points)
        dx = base_arr[:, 0] - x
        dy = base_arr[:, 1] - y
        dists = np.hypot(dx, dy)
        nearest_idx = np.argmin(dists)
        return base_arr[nearest_idx, 2]

    def _compensate_wall_volume(self, points: np.ndarray, polygon: List[List[float]]) -> float:
        """苍壁补偿：对靠近 ROI 边界的高点，补偿缺失底部体积"""
        if len(points) < 10 or len(polygon) < 3:
            return 0.0

        # 计算 ROI 的 bbox
        poly_arr = np.array(polygon)
        x_min, y_min = poly_arr.min(axis=0)
        x_max, y_max = poly_arr.max(axis=0)

        # 找出靠近边界的点（距离边界 < 1.5 * grid_size）
        margin = 1.5 * self.grid_size
        near_boundary = (
            (points[:, 0] <= x_min + margin) |
            (points[:, 0] >= x_max - margin) |
            (points[:, 1] <= y_min + margin) |
            (points[:, 1] >= y_max - margin)
        )

        boundary_points = points[near_boundary]
        if len(boundary_points) == 0:
            return 0.0

        # 假设这些点下方有“缺失”的柱体（从底面到当前高度）
        global_min_z = np.min(points[:, 2])
        compensation_vol = 0.0
        cell_area = self.grid_size * self.grid_size

        for z in boundary_points[:, 2]:
            dh = z - global_min_z
            if dh > 0:
                compensation_vol += dh * cell_area * 0.3  # 补偿系数（经验值）

        return compensation_vol