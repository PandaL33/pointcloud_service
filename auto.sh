#!/bin/bash
APP="uvicorn app.main:app --host 0.0.0.0 --port 10080 --workers 4"
LOG="uvicorn.log"
PIDFILE="uvicorn.pid"

start() {
  if [ -f "$PIDFILE" ] && kill -0 $(cat $PIDFILE) 2>/dev/null; then
    echo "Already running"
    exit 1
  fi
  nohup $APP > $LOG 2>&1
  echo $! > $PIDFILE
  echo "Started with PID $(cat $PIDFILE)"
}

stop() {
  if [ -f "$PIDFILE" ]; then
    kill $(cat $PIDFILE)
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
