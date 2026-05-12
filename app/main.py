# app/main.py
# Author: hzc
# Created: 2026-01-27
# Description: 点云预处理与体积估算服务

from fastapi import FastAPI
from app.routes import preprocess, volume
from app.config import settings
import logging.config

# ==============================
# 日志配置：控制台 + 轮转文件日志
# ==============================
LOGGING_CONFIG = {
    "version": 1,
    "disable_existing_loggers": False,
    "formatters": {"default": {"format": "%(asctime)s - %(name)s - %(levelname)s - %(message)s"}},
    "handlers": {
        "console": {"class": "logging.StreamHandler", "formatter": "default"},
        "file": {
            "class": "logging.handlers.RotatingFileHandler",
            "filename": "app.log",
            "maxBytes": 10_000_000,
            "backupCount": 3,
            "formatter": "default",
            "encoding": "utf-8"
        }
    },
    "root": {"handlers": ["console", "file"], "level": "INFO"}
}

# 应用日志配置
logging.config.dictConfig(LOGGING_CONFIG)

# 确保预处理输出目录存在
settings.preprocessed_dir.mkdir(parents=True, exist_ok=True)

# ==============================
# FastAPI 应用实例
# ==============================
app = FastAPI(
    title="Point Cloud Volume Estimation Service",
    description="提供点云预处理与体积估算的高性能 API 服务",
    version="1.0.0"
)

# 注册子路由
app.include_router(preprocess.router, prefix="/pcdapi", tags=["Preprocessing"])
app.include_router(volume.router, prefix="/pcdapi", tags=["Volume Estimation"])

@app.get("/")
def root():
    return {"message": "Point Cloud Service", "endpoints": ["/pcdapi/preprocess", "/pcdapi/estimate_volume"]}