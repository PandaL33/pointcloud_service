# app/services/volume_service.py
import numpy as np
import open3d as o3d
from collections import defaultdict
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
        # global_min_z = np.min(points[:, 2])
        global_min_z = self.detect_ground(points)
        logger.info(f"地面高度: {global_min_z:.5f}")
        # 处理可能的除零或无穷大
        with np.errstate(divide='ignore', invalid='ignore'):
            gx = np.floor(points[:, 0] / self.grid_size).astype(int)
            gy = np.floor(points[:, 1] / self.grid_size).astype(int)
        
        keys = [f"{x},{y}" for x, y in zip(gx, gy)]
        
        # 使用字典存储每个网格单元的所有 Z 值列表
        grid_map = {}
        
        for i, key in enumerate(keys):
            z = points[i, 2]
            # 忽略 NaN 或 Inf
            if not np.isfinite(z):
                continue
            if key not in grid_map:
                grid_map[key] = []
            grid_map[key].append(z)
        
        vol = 0.0
        cell_area = self.grid_size * self.grid_size
        # 计算每个网格单元的平均高度
        for z_values in grid_map.values():
            h = np.mean(z_values)
            dh = h - global_min_z
            if dh > 1e-6:
                vol += dh * cell_area
        return vol

    def estimate_volume_in_polygon_roi(self, points: np.ndarray, polygon: List[List[float]], cluster_voxel_size: float = 0.1, cluster_min_points_per_component: int = 10) -> Tuple[float, int]:
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

            if len(roi_points) > 0:
                roi_pcd = o3d.geometry.PointCloud()
                roi_pcd.points = o3d.utility.Vector3dVector(roi_points)
                roi_pcd = self.connectivity_cluster_filter(
                    roi_pcd, voxel_size=cluster_voxel_size, min_points_per_component=cluster_min_points_per_component
                )
                roi_points = np.asarray(roi_pcd.points)

            count = len(roi_points)
            vol = self.estimate_full_volume(roi_points)
            return vol, count
        except Exception as e:
              
            # 如果多边形无效，返回 0
            logging.warning(f"Invalid polygon or error in calculation: {e}")
            return 0.0, 0

    def estimate_volumes_with_rois(self, points: np.ndarray, rois: List[List[List[float]]], cluster_voxel_size: float = 0.1, cluster_min_points_per_component: int = 10) -> dict:
        total_vol = self.estimate_full_volume(points)
        results = []
        part_total = 0.0
        
        for idx, poly in enumerate(rois):
            # 【优化】：一次性获取体积和点数，不再重复创建 MplPath 或计算 mask
            vol, count = self.estimate_volume_in_polygon_roi(
                points, poly, 
                cluster_voxel_size=cluster_voxel_size, cluster_min_points_per_component=cluster_min_points_per_component
            )
            
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
        
    def save_roi_points_as_pcd(self, points: np.ndarray, polygon: List[List[float]], output_path: str, cluster_voxel_size: float = 0.1, cluster_min_points_per_component: int = 10) -> bool:
        try:
            path = MplPath(polygon)
            inside_mask = path.contains_points(points[:, :2])
            inside_mask = np.asarray(inside_mask, dtype=bool)

            roi_points = points[inside_mask]

            roi_pcd = o3d.geometry.PointCloud()
            roi_pcd.points = o3d.utility.Vector3dVector(roi_points)

            logger.info(f"ROI区域聚类前点数: {len(roi_points)}")
            roi_pcd = self.connectivity_cluster_filter(
                roi_pcd, voxel_size=cluster_voxel_size, min_points_per_component=cluster_min_points_per_component
            )
            logger.info(f"ROI区域聚类后点数: {len(roi_pcd.points)}")

            logger.info(f"ROI区域内有 {len(roi_pcd.points)} 个点，正在保存到 {output_path}")
            success = o3d.io.write_point_cloud(output_path, roi_pcd)
            if success:
                logger.info(f"ROI点云已成功保存到 {output_path}")
            else:
                logger.error(f"保存ROI点云失败: {output_path}")
            return success
        except Exception as e:
            logger.error(f"保存ROI点云时发生错误: {e}")
            return False 
           
    def connectivity_cluster_filter(self, pcd, voxel_size=0.1, min_points_per_component=10):
        points = np.asarray(pcd.points)
        if len(points) == 0:
            return o3d.geometry.PointCloud()

        if voxel_size <= 0:
            voxel_size = 0.01

        min_bound = np.min(points, axis=0)
        voxel_indices = np.floor((points - min_bound) / voxel_size).astype(np.int32)

        voxel_to_points = defaultdict(list)
        for i, vi in enumerate(voxel_indices):
            voxel_to_points[tuple(vi)].append(i)

        occupied = set(voxel_to_points.keys())
        visited = set()
        components = []

        neighbor_offsets = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    if dx == 0 and dy == 0 and dz == 0:
                        continue
                    neighbor_offsets.append((dx, dy, dz))

        for voxel in occupied:
            if voxel in visited:
                continue
            component_voxels = []
            queue = [voxel]
            visited.add(voxel)
            while queue:
                current = queue.pop(0)
                component_voxels.append(current)
                for dx, dy, dz in neighbor_offsets:
                    nb = (current[0] + dx, current[1] + dy, current[2] + dz)
                    if nb in occupied and nb not in visited:
                        visited.add(nb)
                        queue.append(nb)
            components.append(component_voxels)

        component_sizes = []
        for comp in components:
            total_pts = sum(len(voxel_to_points[v]) for v in comp)
            component_sizes.append(total_pts)

        valid_components = [(comp, sz) for comp, sz in zip(components, component_sizes)
                           if sz >= min_points_per_component]

        if not valid_components:
            logger.warning("连通性聚类未找到满足最小点数要求的类，返回原始点云")
            return pcd

        valid_components.sort(key=lambda x: x[1], reverse=True)
        largest_comp, largest_size = valid_components[0]

        keep_indices = []
        for v in largest_comp:
            keep_indices.extend(voxel_to_points[v])
        keep_indices = np.array(keep_indices, dtype=np.int64)

        logger.info(f"连通性聚类: 共发现 {len(components)} 个连通域, "
                    f"有效类数={len(valid_components)}, "
                    f"最大类点数={largest_size}, "
                    f"去除噪点数={len(points) - largest_size}")

        filtered_pcd = o3d.geometry.PointCloud()
        filtered_pcd.points = o3d.utility.Vector3dVector(points[keep_indices])

        if pcd.has_colors():
            filtered_pcd.colors = o3d.utility.Vector3dVector(np.asarray(pcd.colors)[keep_indices])
        if pcd.has_normals():
            filtered_pcd.normals = o3d.utility.Vector3dVector(np.asarray(pcd.normals)[keep_indices])

        return filtered_pcd
    
    def detect_ground(self, points: np.ndarray, percentile=50.0):
        """
        基于边缘单元格最小值的地面检测。

        适用于堆料(煤堆、粮堆等)场景:堆料下方无地面点云数据,
        地面仅在堆料边缘可见。内部单元格的最小 Z 是料堆底面而非地面。

        算法流程:
        1. 将 XY 平面划分为网格,计算每个单元的最小 Z 值
        2. 识别边界单元(8邻域中有空邻居的单元)
            - 边界单元位于点云边缘,其最小 Z 代表地面高度
            - 内部单元被料堆覆盖,其最小 Z 是料堆底面,远高于地面
        3. 从边界单元的最小 Z 中,渐进式稳健估计地面高度
            - 以最低 20% 为种子,MAD 估计散布,逐步扩展
            - 从下方 3σ 裁剪噪声异常值
        4. 取地面候选集的指定百分位作为最终地面高度
        """
        if len(points) == 0:
            return 0.0

        z_min = np.min(points[:, 2])
        z_max = np.max(points[:, 2])
        z_range = z_max - z_min

        if z_range < 1e-6:
            return float(z_min)

        # ---- Step 1: 网格化，提取每个单元的最小 Z ----
        span_x = float(points[:, 0].max() - points[:, 0].min())
        span_y = float(points[:, 1].max() - points[:, 1].min())
        max_span = max(span_x, span_y)

        cell_size = max_span / 30.0
        cell_size = min(cell_size, max_span / 5.0)
        cell_size = max(cell_size, max_span / 100.0)

        min_x = float(points[:, 0].min())
        min_y = float(points[:, 1].min())

        ix = ((points[:, 0] - min_x) / cell_size).astype(np.int64)
        iy = ((points[:, 1] - min_y) / cell_size).astype(np.int64)

        ny = max(1, int(np.ceil(span_y / cell_size)) + 1)
        cell_key = ix * ny + iy

        order = np.argsort(cell_key)
        sorted_keys = cell_key[order]
        sorted_z = points[order, 2]

        unique_keys, start_pos = np.unique(sorted_keys, return_index=True)

        cell_min_z = np.empty(len(unique_keys))
        for i in range(len(unique_keys)):
            s = start_pos[i]
            e = start_pos[i + 1] if i + 1 < len(unique_keys) else len(sorted_z)
            cell_min_z[i] = np.min(sorted_z[s:e])

        # ---- Step 2: 识别边界单元 ----
        occupied = set(unique_keys.tolist())
        boundary_mask = np.zeros(len(unique_keys), dtype=bool)

        for idx in range(len(unique_keys)):
            k = int(unique_keys[idx])
            ci, cj = k // ny, k % ny
            has_empty = False
            for di in (-1, 0, 1):
                for dj in (-1, 0, 1):
                    if di == 0 and dj == 0:
                        continue
                    nk = (ci + di) * ny + (cj + dj)
                    if nk not in occupied:
                        has_empty = True
                        break
                if has_empty:
                    break
            boundary_mask[idx] = has_empty

        boundary_min_z = cell_min_z[boundary_mask]

        if len(boundary_min_z) < 3:
            boundary_min_z = cell_min_z

        # ---- Step 3: 渐进式稳健估计 ----
        sorted_mins = np.sort(boundary_min_z)

        n_initial = max(3, int(len(sorted_mins) * 0.2))
        ground_candidates = sorted_mins[:n_initial]

        for _ in range(10):
            median_z = np.median(ground_candidates)
            mad = np.median(np.abs(ground_candidates - median_z))
            sigma = 1.4826 * mad

            if sigma < 1e-6:
                break

            threshold = median_z + 2.0 * sigma
            new_candidates = sorted_mins[sorted_mins <= threshold]

            if len(new_candidates) == len(ground_candidates):
                break

            ground_candidates = new_candidates

        # 从下方移除噪声异常值
        if len(ground_candidates) >= 3:
            median_z = np.median(ground_candidates)
            mad = np.median(np.abs(ground_candidates - median_z))
            sigma = 1.4826 * mad
            if sigma > 1e-6:
                ground_candidates = ground_candidates[
                    ground_candidates >= median_z - 3.0 * sigma
                ]

        if len(ground_candidates) == 0:
            return float(np.min(boundary_min_z))

        # ---- Step 4: 稳健估计地面高度 ----
        ground_z = np.percentile(ground_candidates, percentile)

        n_boundary = int(np.sum(boundary_mask))
        print(f"  地面检测: 网格单元数={len(cell_min_z)}, "
            f"边界单元数={n_boundary}, "
            f"地面候选数={len(ground_candidates)}, "
            f"候选Z范围=[{np.min(ground_candidates):.4f}, {np.max(ground_candidates):.4f}]")

        return float(ground_z)