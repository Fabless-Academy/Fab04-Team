#!/bin/sh
# 32번 장시간 테스트용: 앱을 앞에서 실행해서 종료 코드를 남김
APP_PATH=.
export XDG_RUNTIME_DIR=/var/run
export WAYLAND_DISPLAY=wayland-1
export LD_LIBRARY_PATH=${APP_PATH}/misc/lib
insmod ${APP_PATH}/../modules/vpu.ko
insmod ${APP_PATH}/../modules/aiware_drv_ng.ko
insmod ${APP_PATH}/../modules/aiware_drv_ng-apache6.ko
insmod ${APP_PATH}/../modules/nc_dmabuf_ctrl.ko
insmod ${APP_PATH}/../modules/nc_dsr_drv.ko

echo "[START] uptime $(cut -d' ' -f1 /proc/uptime)"
${APP_PATH}/app_wayland_npu
echo "[EXIT] code $? uptime $(cut -d' ' -f1 /proc/uptime)"