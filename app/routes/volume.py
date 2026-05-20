# app/routes/volume.py
# Author: hzc
# Created: 2026-01-27
# Description: 点云预体积估算接口
import os
from fastapi import APIRouter, Form, HTTPException
from app.services.volume_service import VolumeEstimator
from app.utils.file_downloader import download_pcd_to_temp
from app.config import settings
import uuid
import open3d as o3d
import numpy as np
import json
import logging
router = APIRouter()

@router.post("/estimate_volume")
async def estimate_volume(
    file_url: str = Form(...),
    rois: str = Form(None), # 保持类型提示为 str
    grid_size: float = Form(0.1)
):
    tmp_path = None
    logging.info(f"Received request to estimate volume for file: {file_url}")
    try:
        if file_url.startswith(('http://', 'https://')):
            tmp_path = download_pcd_to_temp(file_url, settings.max_file_size)
            logging.info(f"Downloaded file to: {tmp_path}")
            pcd = o3d.io.read_point_cloud(tmp_path)
        else:
            pcd = o3d.io.read_point_cloud(file_url)
            
        if pcd.is_empty():
            raise HTTPException(status_code=400, detail="Empty point cloud")
        
        points = np.asarray(pcd.points)
        estimator = VolumeEstimator(grid_size=grid_size)

        # 不要直接使用 "if not rois"，因为如果 rois 意外变成 numpy 数组会报错
        is_rois_empty = False
        if rois is None:
            is_rois_empty = True
        elif isinstance(rois, str):
            if not rois.strip():
                is_rois_empty = True
        elif isinstance(rois, (list, tuple, np.ndarray)):
            # 防御性编程：如果 rois 被解析成了列表/数组，且为空，则视为无 ROI
            if len(rois) == 0:
                is_rois_empty = True
            # 如果不为空，说明传参类型有误但数据存在，我们将其转回 JSON 字符串继续处理
            else:
                logging.warning(f"rois received as {type(rois)}, converting back to JSON string.")
                rois = json.dumps(rois)
        
        if is_rois_empty:
            total_vol = estimator.estimate_full_volume(points)
            return {"total_volumes": round(total_vol, 3), "roi_volumes": None}

        # 解析 ROI 数据
        try:
            roi_data = json.loads(rois)
        except json.JSONDecodeError as e:
            raise HTTPException(status_code=400, detail=f"Invalid JSON in rois: {str(e)}")

        if not isinstance(roi_data, list):
            raise ValueError("rois must be a list of polygons")
      
        output_path = settings.preprocessed_dir / f"{uuid.uuid4()}_preprocessed.pcd"
        output_path.parent.mkdir(parents=True, exist_ok=True)
         # 如果指定了ROI，同时保存ROI点云
        for idx, poly in enumerate(roi_data):
            roi_output_path = output_path.parent / f"{output_path.stem}_roi_{idx}{output_path.suffix}"
            estimator.save_roi_points_as_pcd(points, poly, str(roi_output_path),0.1, 50)
        result = estimator.estimate_volumes_with_rois(points, roi_data,0.1, 50)
        # 计算体积
        #result = estimator.estimate_volumes_with_rois(points, roi_data)
        
        return result

    except Exception as e:
        # 记录详细错误日志
        logging.error(f"Volume estimation failed: {str(e)}", exc_info=True)
        raise HTTPException(status_code=400, detail=str(e))
    finally:
        if tmp_path and os.path.exists(tmp_path):
            os.unlink(tmp_path)
