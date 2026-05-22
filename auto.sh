#!/bin/bash
# 设置环境变量，限制 Open3D 底层线程数，防止多进程资源竞争导致卡死
export OMP_NUM_THREADS=1
export OPEN3D_NUM_THREADS=1

# 替换为 Gunicorn 启动命令，-w 4 表示开启 4 个工作进程
APP="gunicorn app.main:app -w 4 -k uvicorn.workers.UvicornWorker --bind 0.0.0.0:10080 --timeout 120"
LOG="gunicorn.log"
PIDFILE="gunicorn.pid"

start() {
  if [ -f "$PIDFILE" ] && kill -0 $(cat $PIDFILE) 2>/dev/null; then
    echo "Already running"
    exit 1
  fi
  nohup $APP > $LOG 2>&1 &
  echo $! > $PIDFILE
  echo "Started with PID $(cat $PIDFILE)"
}

stop() {
  if [ -f "$PIDFILE" ]; then
    # 停止 Gunicorn 主进程及其所有子进程
    kill -9 $(cat $PIDFILE) 2>/dev/null
    # 尝试清理可能残留的子进程
    pkill -f "gunicorn app.main:app" 2>/dev/null
    rm -f $PIDFILE
    echo "Stopped"
  else
    echo "Not running"
  fi
}

case "$1" in
  start) start ;;
  stop) stop ;;
  restart) stop; sleep 2; start ;;
  *) echo "Usage: $0 {start|stop|restart}" ;;
esac