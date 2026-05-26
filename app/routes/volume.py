# app/routes/volume.py
# Author: hzc
# Created: 2026-01-27
# Description: 点云预体积估算接口
import os
from fastapi import APIRouter, Form, HTTPException, BackgroundTasks, Query
from app.services.volume_service import VolumeEstimator
from app.utils.file_downloader import download_pcd_to_temp
from app.utils.task_manager import get_task_manager
from app.config import settings
import open3d as o3d
import numpy as np
import json
import logging
import uuid

from typing import Optional

router = APIRouter()

def estimate_volume_background_task(
    task_id: str,
    file_url: str,
    rois: Optional[str] = None,
    grid_size: float = 0.1
):
    """后台执行体积估算任务"""
    tmp_path = None
    task_manager = get_task_manager()
    
    try:
        # 更新任务状态为处理中
        task_manager.update_task(task_id, {
            "task_id": task_id,
            "status": "processing",
            "progress": 0,
            "message": "开始处理...",
            "result": None
        })
        
        # 1. 下载或加载点云文件
        task_manager.update_task(task_id, {
            "task_id": task_id,
            "status": "processing",
            "progress": 10,
            "message": "加载点云文件...",
            "result": None
        })
        
        file_name = None
        if file_url.startswith(('http://', 'https://')):
            tmp_path = download_pcd_to_temp(file_url, settings.max_file_size)
            logging.info(f"Downloaded file to: {tmp_path}")
            pcd = o3d.io.read_point_cloud(tmp_path)
            file_name = os.path.basename(file_url)
        else:
            pcd = o3d.io.read_point_cloud(file_url)
            file_name = file_url.rsplit('.', 1)[0] 
            
        if pcd.is_empty():
            raise ValueError("Empty point cloud")
        
        points = np.asarray(pcd.points)
        
        task_manager.update_task(task_id, {
            "task_id": task_id,
            "status": "processing",
            "progress": 30,
            "message": "点云加载完成",
            "result": None
        })
        
        # 2. 解析 ROI 数据
        task_manager.update_task(task_id, {
            "task_id": task_id,
            "status": "processing",
            "progress": 40,
            "message": "解析ROI数据...",
            "result": None
        })
        
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
            task_manager.update_task(task_id, {
                "task_id": task_id,
                "status": "processing",
                "progress": 60,
                "message": "计算全局体积...",
                "result": None
            })
            
            estimator = VolumeEstimator(grid_size=grid_size)
            total_vol = estimator.estimate_full_volume(points)
            
            # 更新任务状态为完成
            task_manager.update_task(task_id, {
                "task_id": task_id,
                "status": "completed",
                "progress": 100,
                "message": "处理完成",
                "result": {
                    "total_volumes": round(total_vol, 3),
                    "roi_volumes": None
                }
            })
            return
        
        # 解析 ROI 数据
        try:
            roi_data = json.loads(rois)
        except json.JSONDecodeError as e:
            raise ValueError(f"Invalid JSON in rois: {str(e)}")

        if not isinstance(roi_data, list):
            raise ValueError("rois must be a list of polygons")
        
        task_manager.update_task(task_id, {
            "task_id": task_id,
            "status": "processing",
            "progress": 50,
            "message": f"解析到 {len(roi_data)} 个ROI区域",
            "result": None
        })
        
        # 3. 保存 ROI 点云
        task_manager.update_task(task_id, {
            "task_id": task_id,
            "status": "processing",
            "progress": 60,
            "message": "保存ROI点云...",
            "result": None
        })
        
        output_path = settings.preprocessed_dir / f"{file_name}_preprocessed.pcd"
        output_path.parent.mkdir(parents=True, exist_ok=True)
        
        estimator = VolumeEstimator(grid_size=grid_size)
        
        for idx, poly in enumerate(roi_data):
            roi_output_path = output_path.parent / f"{output_path.stem}_roi_{idx}{output_path.suffix}"
            estimator.save_roi_points_as_pcd(points, poly, str(roi_output_path), 0.1, 50)
            
            progress = 60 + int((idx + 1) / len(roi_data) * 20)
            task_manager.update_task(task_id, {
                "task_id": task_id,
                "status": "processing",
                "progress": progress,
                "message": f"已保存 {idx + 1}/{len(roi_data)} 个ROI点云",
                "result": None
            })
        
        # 4. 计算体积
        task_manager.update_task(task_id, {
            "task_id": task_id,
            "status": "processing",
            "progress": 85,
            "message": "计算ROI体积...",
            "result": None
        })
        
        result = estimator.estimate_volumes_with_rois(points, roi_data, 0.1, 50)
        logging.info(f"estimate_volumes_with_rois for {file_name}, result: {result}")
        
        # 更新任务状态为完成
        task_manager.update_task(task_id, {
            "task_id": task_id,
            "status": "completed",
            "progress": 100,
            "message": "处理完成",
            "result": result
        })
        
    except Exception as e:
        # 更新任务状态为失败
        task_manager.update_task(task_id, {
            "task_id": task_id,
            "status": "failed",
            "progress": -1,
            "message": f"处理失败: {str(e)}",
            "result": None,
            "error": str(e)
        })
        logging.error(f"任务 {task_id} 处理失败: {str(e)}", exc_info=True)
    finally:
        # 清理临时文件
        if tmp_path and os.path.exists(tmp_path):
            os.unlink(tmp_path)


@router.post("/estimate_volume/async")
async def estimate_volume_async(
    file_url: str = Form(...),
    rois: str = Form(None),
    grid_size: float = Form(0.1),
    background_tasks: BackgroundTasks = None
):
    """
    异步点云体积估算接口
    立即返回 task_id，后台执行处理任务
    """
    task_id = str(uuid.uuid4())
    
    # 立即返回，将任务创建也放到后台执行
    background_tasks.add_task(
        _create_and_execute_task,
        task_id=task_id,
        file_url=file_url,
        rois=rois,
        grid_size=grid_size
    )
    
    return {
        "task_id": task_id,
        "status": "accepted",
        "message": "任务已接受，正在后台处理"
    }


def _create_and_execute_task(
    task_id: str,
    file_url: str,
    rois: Optional[str] = None,
    grid_size: float = 0.1
):
    """在后台创建任务状态并执行体积估算"""
    task_manager = get_task_manager()
    
    # 创建初始任务状态
    task_manager.create_task(task_id, {
        "task_id": task_id,
        "status": "accepted",
        "progress": 0,
        "message": "任务已接受，准备开始处理",
        "result": None
    })
    
    # 执行实际的体积估算任务
    estimate_volume_background_task(
        task_id=task_id,
        file_url=file_url,
        rois=rois,
        grid_size=grid_size
    )


@router.get("/estimate_volume/status/{task_id}")
async def get_task_status(task_id: str):
    """查询体积估算任务状态"""
    task_manager = get_task_manager()
    task_status = task_manager.get_task(task_id)
    
    if task_status is None:
        raise HTTPException(status_code=404, detail="任务不存在")
    
    return task_status


@router.post("/estimate_volume")
async def estimate_volume(
    file_url: str = Form(...),
    rois: str = Form(None), # 保持类型提示为 str
    grid_size: float = Form(0.1)
):
    tmp_path = None
    logging.info(f"Received request to estimate volume for file: {file_url}")
    try: 
        file_name = None
        if file_url.startswith(('http://', 'https://')):
            tmp_path = download_pcd_to_temp(file_url, settings.max_file_size)
            logging.info(f"Downloaded file to: {tmp_path}")
            pcd = o3d.io.read_point_cloud(tmp_path)
            file_name = os.path.basename(file_url)
        else:
            pcd = o3d.io.read_point_cloud(file_url)
            file_name = file_url.rsplit('.', 1)[0] 
            
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

        output_path = settings.preprocessed_dir / f"{file_name}_preprocessed.pcd"
        output_path.parent.mkdir(parents=True, exist_ok=True)
         # 如果指定了ROI，同时保存ROI点云
        for idx, poly in enumerate(roi_data):
            roi_output_path = output_path.parent / f"{output_path.stem}_roi_{idx}{output_path.suffix}"
            estimator.save_roi_points_as_pcd(points, poly, str(roi_output_path),0.1, 50)
        
        # 计算体积
        result = estimator.estimate_volumes_with_rois(points, roi_data,0.1, 50)
        logging.info(f"estimate_volumes_with_rois for {file_name}, result: {result}")
        
        return result

    except Exception as e:
        # 记录详细错误日志
        logging.error(f"Volume estimation failed: {str(e)}", exc_info=True)
        raise HTTPException(status_code=400, detail=str(e))
    finally:
        if tmp_path and os.path.exists(tmp_path):
            os.unlink(tmp_path)