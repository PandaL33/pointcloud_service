# app/routes/preprocess.py
# Author: hzc
# Created: 2026-01-27
# Description: 点云预处理接口
from fastapi import APIRouter, Form, HTTPException, BackgroundTasks
from app.services.preprocessing_service import PointCloudPreprocessor
from app.services.file_upload_service import FileUploadService
from app.utils.file_downloader import download_pcd_to_temp
from app.config import settings
import open3d as o3d
import os
from pathlib import Path
from app.services.cloud_compare_icp import CloudCompareIcp
router = APIRouter()
import logging
logger = logging.getLogger(__name__)
@router.post("/preprocess")
async def preprocess_endpoint(
    file_url: str = Form(...),
    map_file_url: str = Form(None),  # 修改：将map_file_url设为可选参数
    background_tasks: BackgroundTasks = None
):
    temp_input = None
    try:
        # 下载文件
        temp_input = download_pcd_to_temp(file_url, settings.max_file_size)
        pcd = o3d.io.read_point_cloud(temp_input)
        if pcd.is_empty():
            raise ValueError("Empty point cloud")
        
        file_name = os.path.basename(file_url)
        
        if map_file_url:
            map_input = download_pcd_to_temp(map_file_url, settings.max_file_size)
            map_pcd = o3d.io.read_point_cloud(map_input)
            logger.info(f"点云配准,align_cloud:{temp_input},map_cloud:{map_input}")
            if not map_pcd.is_empty():
                cloud_compare_icp = CloudCompareIcp()
                icp_pcd_file_path = settings.preprocessed_dir / f"{file_name}_icp.pcd"
                transform_matrix, fitness, iterations, icp_pcd = cloud_compare_icp.run_registrator(temp_input, map_input, icp_pcd_file_path,0.25)
                pcd = icp_pcd
        
        # 预处理
        logger.info(f"点云预处理开始")
        preprocessor = PointCloudPreprocessor()
        cleaned = preprocessor.preprocess(pcd)

        output_path = settings.preprocessed_dir / f"{file_name}_preprocessed.pcd"
        if not o3d.io.write_point_cloud(str(output_path), cleaned):
            raise RuntimeError("Save failed")

        # 上传文件到文件服务器
        uploader = FileUploadService()
        file_id = uploader.upload_file(str(output_path))

        return {"file_id": file_id}
    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e))
    finally:
        if temp_input and os.path.exists(temp_input):
            os.unlink(temp_input)