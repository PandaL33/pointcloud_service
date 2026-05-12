# app/config.py
# from pydantic import BaseSettings
from pydantic_settings import BaseSettings
from pathlib import Path

class Settings(BaseSettings):
    max_file_size: int = 500 * 1024 * 1024  # 500 MB
    download_timeout: int = 120
    preprocessed_dir: Path = Path("data/preprocessed")
    file_server_url: str = "http://127.0.0.1:10103"
    upload_username: str = "robot-manage"
    upload_password: str = "123456"

    class Config:
        env_file = ".env"

settings = Settings()
