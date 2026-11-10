#!/bin/sh
# 시연용: 앱이 꺼지면 자동으로 다시 켬, 실행마다 로그를 따로 남김
# 끄기: touch /tmp/stop_app 후 killall app_wayland_npu
cd /mnt/user_data/applications
rm -f /tmp/stop_app
n=0
while [ $n -lt 20 ]; do
    n=$((n+1))
    echo "[WATCH] run $n start, uptime $(cut -d' ' -f1 /proc/uptime)" >> /tmp/m_watch.txt
    ./run_long.sh > /tmp/m_run$n.txt 2>&1 < /dev/null
    echo "[WATCH] run $n exit,  uptime $(cut -d' ' -f1 /proc/uptime)" >> /tmp/m_watch.txt
    [ -f /tmp/stop_app ] && break
    sleep 2
done