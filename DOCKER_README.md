# 点云服务 Docker 部署指南

## 项目概述

这是一个基于Python的点云数据处理服务系统，提供点云文件上传、预处理和体积计算等功能。

## 构建和运行 Docker 镜像

### 方法一：使用 docker-compose（推荐）

1. 确保已安装 Docker 和 docker-compose
2. 在项目根目录下运行以下命令：

```bash
docker-compose up -d
```

3. 服务将在 http://localhost:10080 上可用

### 方法二：使用纯 Docker 命令

1. 构建镜像：
```bash
docker build -t pointcloud-service .
```

2. 运行容器：
```bash
docker run -d -p 10080:10080 -v $(pwd)/data:/app/data -v $(pwd)/logs:/app/logs --name pointcloud-service pointcloud-service
```

## 配置外部文件服务器

如果您的应用需要连接到外部文件服务器，请通过环境变量覆盖默认配置：

```bash
# 如果文件服务器运行在宿主机上
docker run -d -p 10080:10080 \
  -e FILE_SERVER_URL="http://host.docker.internal:10103" \
  -v $(pwd)/data:/app/data -v $(pwd)/logs:/app/logs \
  --name pointcloud-service pointcloud-service
```

或在 docker-compose.yml 中添加环境变量：

```yaml
version: '3.8'

services:
  pointcloud-service:
    build: .
    ports:
      - "10080:10080"
    volumes:
      - ./data:/app/data
      - ./logs:/app/logs
    environment:
      - PYTHONPATH=/app
      - FILE_SERVER_URL=http://host.docker.internal:10103
    restart: unless-stopped
    networks:
      - pointcloud-network

networks:
  pointcloud-network:
    driver: bridge
```

## 访问服务

启动后，可以通过以下地址访问服务：
- 主页: http://localhost:10080
- API 文档: http://localhost:10080/docs

## 数据持久化

- `/app/data` 目录用于存储预处理后的数据
- `/app/logs` 目录用于存储应用日志

## 配置

服务的配置可以通过修改 `app/config.py` 文件进行调整，或者在运行容器时设置环境变量。

## 停止服务

### 使用 docker-compose:
```bash
docker-compose down
```

### 使用 Docker:
```bash
docker stop pointcloud-service
docker rm pointcloud-service
```