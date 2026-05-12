# app/utils/pointcloud_utils.py
import numpy as np
from typing import List

def polygon_to_bbox(polygon: List[List[float]]) -> List[float]:
    if len(polygon) < 3:
        raise ValueError("Polygon must have at least 3 points")
    coords = np.array(polygon, dtype=float)
    if coords.shape[1] != 2:
        raise ValueError("Each point must be [x, y]")
    x_min, y_min = coords.min(axis=0)
    x_max, y_max = coords.max(axis=0)
    return [float(x_min), float(y_min), float(x_max), float(y_max)]

def parse_rois(roi_data) -> List[List[float]]:
    if not isinstance(roi_data, list):
        raise ValueError("rois must be a list")
    parsed = []
    for i, item in enumerate(roi_data):
        if isinstance(item, str):
            parts = item.split(',')
            if len(parts) != 4:
                raise ValueError(f"ROI string {i} must contain 4 comma-separated numbers")
            roi = [float(p.strip()) for p in parts]
        elif isinstance(item, list):
            roi = polygon_to_bbox(item)
        else:
            raise ValueError(f"ROI {i} must be a string or a list of points")
        parsed.append(roi)
    return parsed