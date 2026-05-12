import os
import tempfile
import requests
from urllib.parse import urlparse
from pathlib import Path
from typing import Tuple, Optional
from config import MAX_FILE_SIZE
import logging

logger = logging.getLogger(__name__)

class FileService:
    @staticmethod
    def is_local_path(url: str) -> bool:
        parsed = urlparse(url)
        return parsed.scheme in ('', 'file')

    @staticmethod
    def resolve_local_path(url: str) -> str:
        parsed = urlparse(url)
        if parsed.scheme == 'file':
            return parsed.path
        return url

    @staticmethod
    def download_pcd_to_temp(url: str) -> str:
        try:
            with requests.get(url, stream=True, timeout=(10, 300)) as r:
                r.raise_for_status()
                logger.info(f"Downloading PCD from {url}")
                with tempfile.NamedTemporaryFile(delete=False, suffix=".pcd") as tmp:
                    header = b""
                    for chunk in r.iter_content(chunk_size=256):
                        header += chunk
                        if len(header) >= 256 or not chunk:
                            break

                    header_text = header.decode('utf-8', errors='ignore')
                    if not (header_text.startswith('# .PCD') or 'VERSION' in header_text.split('\n')[0]):
                        raise ValueError("Invalid PCD file format")

                    tmp.write(header)
                    total = len(header)
                    for chunk in r.iter_content(8192):
                        if chunk:
                            tmp.write(chunk)
                            total += len(chunk)
                            if total > MAX_FILE_SIZE:
                                raise ValueError(f"File exceeds size limit ({MAX_FILE_SIZE / 1e6:.1f} MB)")
                    return tmp.name
        except Exception as e:
            raise ValueError(f"Download failed: {e}") from e

    @staticmethod
    def cleanup_temp_file(path: Optional[str]):
        if path and os.path.exists(path):
            os.unlink(path)