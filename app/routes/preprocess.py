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
import uuid
import os
from pathlib import Path

router = APIRouter()

@router.post("/preprocess")
async def preprocess_endpoint(
    file_url: str = Form(...),
    background_tasks: BackgroundTasks = None
):
    temp_input = None
    try:
        # 下载文件
        temp_input = download_pcd_to_temp(file_url, settings.max_file_size)
        pcd = o3d.io.read_point_cloud(temp_input)
        if pcd.is_empty():
            raise ValueError("Empty point cloud")

        # 预处理
        preprocessor = PointCloudPreprocessor()
        cleaned = preprocessor.preprocess(pcd)

        output_path = settings.preprocessed_dir / f"{uuid.uuid4()}.pcd"
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