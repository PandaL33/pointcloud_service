# app/routes/preprocess.py
# Author: hzc
# Created: 2026-01-27
# Description: 点云预处理接口
from fastapi import APIRouter, Form, HTTPException, BackgroundTasks, Query
from app.services.preprocessing_service import PointCloudPreprocessor
from app.services.file_upload_service import FileUploadService
from app.utils.file_downloader import download_pcd_to_temp
from app.utils.task_manager import get_task_manager
from app.config import settings
import open3d as o3d
import os
from pathlib import Path
from app.services.cloud_compare_icp import CloudCompareIcp
router = APIRouter()
import logging
import uuid

from typing import Optional

logger = logging.getLogger(__name__)

def preprocess_background_task(
    task_id: str,
    file_url: str,
    map_file_url: Optional[str] = None
):
    """后台执行点云预处理任务"""
    temp_input = None
    map_temp_input = None
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
        
        if file_url.startswith(('http://', 'https://')):
            temp_input = download_pcd_to_temp(file_url, settings.max_file_size)
            logging.info(f"Downloaded file to: {temp_input}")
            pcd = o3d.io.read_point_cloud(temp_input)
            file_name = os.path.basename(file_url)
            if pcd.is_empty():
                raise ValueError("Empty point cloud")
        else:
            temp_input = file_url
            pcd = o3d.io.read_point_cloud(file_url)
            file_name = file_url.rsplit('.', 1)[0] 
        
        task_manager.update_task(task_id, {
            "task_id": task_id,
            "status": "processing",
            "progress": 30,
            "message": "点云加载完成",
            "result": None
        })
        
        # 2. ICP 配准（如果提供了地图文件）
        if map_file_url:
            task_manager.update_task(task_id, {
                "task_id": task_id,
                "status": "processing",
                "progress": 40,
                "message": "开始点云配准...",
                "result": None
            })
            
            if map_file_url.startswith(('http://', 'https://')):
                map_temp_input = download_pcd_to_temp(map_file_url, settings.max_file_size)
                map_pcd = o3d.io.read_point_cloud(map_temp_input) 
            else:
                map_temp_input = map_file_url
                map_pcd = o3d.io.read_point_cloud(map_file_url)
                  
            logger.info(f"点云配准,align_cloud:{temp_input},map_cloud:{map_temp_input}")    
            if not map_pcd.is_empty():
                cloud_compare_icp = CloudCompareIcp()
                icp_pcd_file_path = settings.preprocessed_dir / f"{file_name}_icp.pcd"
                transform_matrix, fitness, iterations, icp_pcd = cloud_compare_icp.run_registrator(temp_input, map_temp_input, icp_pcd_file_path, 0.25)
                pcd = icp_pcd
                
                task_manager.update_task(task_id, {
                    "task_id": task_id,
                    "status": "processing",
                    "progress": 60,
                    "message": f"点云配准完成 (迭代{iterations}次, 适应度:{fitness:.4f})",
                    "result": None
                })
        
        # 3. 预处理
        task_manager.update_task(task_id, {
            "task_id": task_id,
            "status": "processing",
            "progress": 70,
            "message": "开始点云预处理...",
            "result": None
        })
        
        logger.info(f"点云预处理开始")
        preprocessor = PointCloudPreprocessor()
        cleaned = preprocessor.preprocess(pcd)

        output_path = settings.preprocessed_dir / f"{file_name}_preprocessed.pcd"
        if not o3d.io.write_point_cloud(str(output_path), cleaned):
            raise RuntimeError("保存失败")
        
        task_manager.update_task(task_id, {
            "task_id": task_id,
            "status": "processing",
            "progress": 85,
            "message": "预处理完成，上传文件...",
            "result": None
        })
        logger.info(f"点云预处理完成，点云文件：{output_path}")
        
        # 4. 上传文件到文件服务器
        uploader = FileUploadService()
        file_id = uploader.upload_file(str(output_path))
        
        # 更新任务状态为完成
        task_manager.update_task(task_id, {
            "task_id": task_id,
            "status": "completed",
            "progress": 100,
            "message": "处理完成",
            "result": {"file_id": file_id}
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
        logger.error(f"任务 {task_id} 处理失败: {str(e)}")
    finally:
        # 清理临时文件
        if temp_input and os.path.exists(temp_input) and temp_input != file_url:
            os.unlink(temp_input)
        if map_temp_input and os.path.exists(map_temp_input) and map_temp_input != map_file_url:
            os.unlink(map_temp_input)


@router.post("/preprocess/async")
async def preprocess_endpoint_async(
    file_url: str = Form(...),
    map_file_url: str = Form(None),
    background_tasks: BackgroundTasks = None
):
    """
    异步点云预处理接口
    立即返回 task_id，后台执行处理任务
    """
    task_id = str(uuid.uuid4())

    # 将任务创建和执行都放到后台，避免 Redis 操作阻塞响应
    background_tasks.add_task(
        _create_and_execute_task,
        task_id=task_id,
        file_url=file_url,
        map_file_url=map_file_url
    )

    return {
        "task_id": task_id,
        "status": "accepted",
        "message": "任务已接受，正在后台处理"
    }


def _create_and_execute_task(
    task_id: str,
    file_url: str,
    map_file_url: Optional[str] = None
):
    """在后台创建任务状态并执行预处理"""
    task_manager = get_task_manager()

    # 创建初始任务状态
    task_manager.create_task(task_id, {
        "task_id": task_id,
        "status": "accepted",
        "progress": 0,
        "message": "任务已接受，准备开始处理",
        "result": None
    })

    # 执行实际的预处理任务
    preprocess_background_task(
        task_id=task_id,
        file_url=file_url,
        map_file_url=map_file_url
    )


@router.get("/preprocess/status/{task_id}")
async def get_task_status(task_id: str):
    """查询任务状态"""
    task_manager = get_task_manager()
    task_status = task_manager.get_task(task_id)
    
    if task_status is None:
        raise HTTPException(status_code=404, detail="任务不存在")
    
    return task_status


@router.post("/preprocess")
async def preprocess_endpoint(
    file_url: str = Form(...),
    map_file_url: str = Form(None),  # 修改：将map_file_url设为可选参数
    background_tasks: BackgroundTasks = None
):
    temp_input = None
    map_temp_input = None
    try:
        if file_url.startswith(('http://', 'https://')):
            temp_input = download_pcd_to_temp(file_url, settings.max_file_size)
            logging.info(f"Downloaded file to: {temp_input}")
            pcd = o3d.io.read_point_cloud(temp_input)
            file_name = os.path.basename(file_url)
            if pcd.is_empty():
                raise ValueError("Empty point cloud")
        else:
            temp_input = file_url  # 修复：使用本地文件路径
            pcd = o3d.io.read_point_cloud(file_url)
            file_name = file_url.rsplit('.', 1)[0] 
        
        if map_file_url:
            if map_file_url.startswith(('http://', 'https://')):
                map_temp_input = download_pcd_to_temp(map_file_url, settings.max_file_size)
                map_pcd = o3d.io.read_point_cloud(map_temp_input) 
            else:
                map_temp_input = map_file_url  # 修复：使用本地文件路径
                map_pcd = o3d.io.read_point_cloud(map_file_url)
                  
            logger.info(f"点云配准,align_cloud:{temp_input},map_cloud:{map_temp_input}")    
            if not map_pcd.is_empty():
                cloud_compare_icp = CloudCompareIcp()
                icp_pcd_file_path = settings.preprocessed_dir / f"{file_name}_icp.pcd"
                transform_matrix, fitness, iterations, icp_pcd = cloud_compare_icp.run_registrator(temp_input, map_temp_input, icp_pcd_file_path, 0.25)
                pcd = icp_pcd   
        
        # 预处理
        logger.info(f"点云预处理开始")
        preprocessor = PointCloudPreprocessor()
        cleaned = preprocessor.preprocess(pcd)

        output_path = settings.preprocessed_dir / f"{file_name}_preprocessed.pcd"
        if not o3d.io.write_point_cloud(str(output_path), cleaned):
            raise RuntimeError("Save failed")
        logger.info(f"点云预处理完成，点云文件：{output_path}")
        
        # 上传文件到文件服务器
        uploader = FileUploadService()
        file_id = uploader.upload_file(str(output_path))

        return {"file_id": file_id}
    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e))
    finally:
        # 清理临时文件
        if temp_input and os.path.exists(temp_input) and temp_input != file_url:
            os.unlink(temp_input)
        if map_temp_input and os.path.exists(map_temp_input) and map_temp_input != map_file_url:
            os.unlink(map_temp_input)