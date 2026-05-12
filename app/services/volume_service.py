# app/services/volume_service.py
import numpy as np
from typing import List, Optional, Tuple
from matplotlib.path import Path as MplPath
import logging

logger = logging.getLogger(__name__)
class VolumeEstimator:
    def __init__(self, grid_size: float = 0.1):
        self.grid_size = grid_size

    def estimate_full_volume(self, points: np.ndarray) -> float:
        if points.size == 0:
            return 0.0
        
        # 防止 Z 轴为空
        if points.shape[1] < 3:
            return 0.0

        global_min_z = np.min(points[:, 2])
        
        # 处理可能的除零或无穷大
        with np.errstate(divide='ignore', invalid='ignore'):
            gx = np.floor(points[:, 0] / self.grid_size).astype(int)
            gy = np.floor(points[:, 1] / self.grid_size).astype(int)
        
        keys = [f"{x},{y}" for x, y in zip(gx, gy)]
        grid_map = {}
        
        for i, key in enumerate(keys):
            z = points[i, 2]
            # 忽略 NaN 或 Inf
            if not np.isfinite(z):
                continue
            if key not in grid_map or z > grid_map[key]:
                grid_map[key] = z
        
        vol = 0.0
        cell_area = self.grid_size * self.grid_size
        for h in grid_map.values():
            dh = h - global_min_z
            if dh > 1e-6:
                vol += dh * cell_area
        return vol

    def estimate_volume_in_polygon_roi(self, points: np.ndarray, polygon: List[List[float]]) -> Tuple[float, int]:
        """
        返回体积和内部点数，避免重复计算 mask
        """
        if len(points) == 0:
            return 0.0, 0
        
        if len(polygon) < 3:
            return 0.0, 0

        try:
            path = MplPath(polygon)
            # contains_points 返回 numpy 布尔数组
            inside_mask = path.contains_points(points[:, :2])
            
            # 【关键修复】：确保 mask 是布尔数组后再使用
            # 虽然 contains_points 通常返回布尔数组，但显式转换更安全
            inside_mask = np.asarray(inside_mask, dtype=bool)
            
            roi_points = points[inside_mask]
            count = int(np.sum(inside_mask)) # 直接统计 True 的数量，比 len(roi_points) 快
            
            vol = self.estimate_full_volume(roi_points)
            return vol, count
        except Exception as e:
            # 如果多边形无效，返回 0
            logging.warning(f"Invalid polygon or error in calculation: {e}")
            return 0.0, 0

    def estimate_volumes_with_rois(self, points: np.ndarray, rois: List[List[List[float]]]) -> dict:
        total_vol = self.estimate_full_volume(points)
        results = []
        part_total = 0.0
        
        for idx, poly in enumerate(rois):
            # 【优化】：一次性获取体积和点数，不再重复创建 MplPath 或计算 mask
            vol, count = self.estimate_volume_in_polygon_roi(points, poly)
            
            part_total += vol
            results.append({
                "roi_index": idx,
                "polygon": poly,
                "point_count": count,
                "volume_m3": round(vol, 3)
            })
            
        return {
            "total_volumes": round(total_vol, 3),
            "part_total_volume": round(part_total, 3),
            "grid_size_m": self.grid_size,
            "roi_volumes": results
        }