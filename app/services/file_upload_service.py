# app/services/file_upload_service.py
import requests
import json
import time
import os
from requests.auth import HTTPBasicAuth
from app.config import settings
import logging

logger = logging.getLogger(__name__)

class FileUploadService:
    def __init__(self, server_url: str = None):
        self.server_url = server_url or settings.file_server_url
        self.username = settings.upload_username
        self.password = settings.upload_password

    def _get_upload_token(self) -> str:
        try:
            url = f'{self.server_url}/provider/v1/file/getUploadTaskToken?fileProperties=0'
            response = requests.get(url, auth=HTTPBasicAuth(self.username, self.password), timeout=5, verify=False)
            data = response.json()
            if data.get('code') == 200:
                return data.get('data', {}).get('uploadToken', '')
            return ''
        except Exception as e:
            logger.error(f"Failed to get upload token: {e}")
            return ''

    def upload_file(self, file_path: str) -> str:
        token = self._get_upload_token()
        if not token:
            logger.error("No upload token")
            return ""
        upload_url = f"{self.server_url}/file/upload/{token}"
        timestamp = int(time.time() * 1000)
        filename = f'point_cloud_{timestamp}.pcd'
        with open(file_path, 'rb') as f:
            files = {'file': (filename, f, 'application/octet-stream')}
            response = requests.post(upload_url, files=files, timeout=300, verify=False)
            if response.status_code == 200:
                result = response.json()
                return result.get('data', {}).get('id', '')
        logger.error(f"Upload failed: {response.text}")
        return ""