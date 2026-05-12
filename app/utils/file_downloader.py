# app/utils/file_downloader.py
import tempfile
import requests
import os
from pathlib import Path
from app.config import settings
import logging

logger = logging.getLogger(__name__)

def download_pcd_to_temp(url: str, max_size: int = None) -> str:
    """
    从给定 URL 下载 .pcd 文件到临时文件，并返回临时文件路径。
    :param url: 点云文件的 HTTP/HTTPS URL
    :param max_size: 最大允许文件大小（字节），默认使用配置
    :return: 临时文件的绝对路径（str）
    """
    if not url or not isinstance(url, str):
        raise ValueError("Invalid file URL")

    max_size = max_size or settings.max_file_size
    timeout = settings.download_timeout

    try:
        logger.info(f"Downloading PCD from: {url}")
        response = requests.get(url, stream=True, timeout=timeout)
        response.raise_for_status()

        # 检查 Content-Length（如果提供）
        content_length = response.headers.get('content-length')
        if content_length and int(content_length) > max_size:
            raise ValueError(f"File too large: {content_length} bytes > {max_size}")

        # 创建临时文件
        temp_file = tempfile.NamedTemporaryFile(delete=False, suffix='.pcd')
        temp_path = temp_file.name
        temp_file.close()  # 手动关闭以便后续写入

        downloaded = 0
        with open(temp_path, 'wb') as f:
            for chunk in response.iter_content(chunk_size=8192):
                if chunk:
                    f.write(chunk)
                    downloaded += len(chunk)
                    if downloaded > max_size:
                        os.unlink(temp_path)
                        raise ValueError("File exceeds size limit during download")

        logger.info(f"Downloaded to temporary file: {temp_path} ({downloaded} bytes)")
        return temp_path

    except requests.RequestException as e:
        logger.error(f"Download failed: {e}")
        raise ValueError(f"Failed to download file: {str(e)}")
    except Exception as e:
        logger.error(f"Unexpected error during download: {e}")
        raise