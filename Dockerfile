# 使用官方Python运行时作为父镜像
FROM python:3.10.12-slim

# 安装系统依赖，解决libgomp.so.1缺失的问题
RUN apt-get update && apt-get install -y \
    gcc \
    g++ \
    libgomp1 \
    libgl1-mesa-glx \
    libglib2.0-0 \
    libsm6 \
    libxext6 \
    libxrender-dev \
    libgomp1 \
    wget \
    curl \
    && rm -rf /var/lib/apt/lists/*

# 设置工作目录
WORKDIR /app

# 将当前目录内容复制到容器中的/app目录
COPY . /app

# 更换pip源并安装依赖包
RUN pip install --no-cache-dir --upgrade pip && \
    pip config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple/ && \
    pip config set global.trusted-host https://pypi.tuna.tsinghua.edu.cn && \
    pip install --no-cache-dir -r requirements.txt

# 暴露应用运行的端口
EXPOSE 10080

# 设置配置文件服务器URL环境变量（默认值可为空或在运行时覆盖）
ENV CONFIG_SERVER_URL=""

# 运行应用程序
CMD ["uvicorn", "app.main:app", "--host", "0.0.0.0", "--port", "10080"]