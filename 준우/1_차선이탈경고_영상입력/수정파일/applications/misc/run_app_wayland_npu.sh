#!/bin/sh
#
#   ./run_app_wayland_npu.sh [video-file]
#   e.g. ./run_app_wayland_npu.sh misc/image_npu/1105_kki.mov
#   (no file: misc/image_npu/1920_1080_0001.mp4)

if [ "$#" -gt 1 ]; then
    echo "Usage: $0 [video-file]" >&2
    exit 2
fi

# video file: absolute, relative to the current folder, or to the app folder
VIDEO_FILE=${1:-}
case "$VIDEO_FILE" in
    ""|/*) ;;
    *) [ -e "$VIDEO_FILE" ] && VIDEO_FILE="$PWD/$VIDEO_FILE" ;;
esac

SCRIPT_DIR=$(CDPATH= cd -P "$(dirname "$0")" && pwd) || exit 1

if [ -n "${APP_PATH:-}" ]; then
    APP_DIR=$APP_PATH
elif [ -x "$SCRIPT_DIR/app_wayland_npu" ]; then
    APP_DIR=$SCRIPT_DIR
else
    APP_DIR=$(dirname "$SCRIPT_DIR")
fi

APP_DIR=$(CDPATH= cd -P "$APP_DIR" && pwd) || {
    echo "Application directory not found: $APP_DIR" >&2
    exit 1
}

if [ ! -x "$APP_DIR/app_wayland_npu" ]; then
    echo "NPU app not found or not executable: $APP_DIR/app_wayland_npu" >&2
    exit 1
fi

if [ -x "$APP_DIR/run_v4l2loopback.sh" ]; then
    LOOPBACK_SCRIPT="$APP_DIR/run_v4l2loopback.sh"
else
    LOOPBACK_SCRIPT="$APP_DIR/misc/run_v4l2loopback.sh"
fi

if [ ! -x "$LOOPBACK_SCRIPT" ]; then
    echo "V4L2 loopback script not found or not executable: $LOOPBACK_SCRIPT" >&2
    exit 1
fi

load_module()
{
    module_file=$1
    module_name=$2

    if [ -e "/sys/module/$module_name" ]; then
        return 0
    fi

    if ! insmod "$APP_DIR/../modules/$module_file"; then
        echo "Failed to load kernel module: $module_file" >&2
        exit 1
    fi
}

load_module aiware_drv_ng.ko aiware_drv_ng
load_module aiware_drv_ng-apache6.ko aiware_drv_ng_apache6
load_module nc_dmabuf_ctrl.ko nc_dmabuf_ctrl
load_module nc_dsr_drv.ko nc_dsr_drv

cd "$APP_DIR" || exit 1
if [ -n "$VIDEO_FILE" ] && [ ! -r "$VIDEO_FILE" ]; then
    echo "Video file not found or unreadable: $VIDEO_FILE" >&2
    exit 1
fi
export XDG_RUNTIME_DIR=/var/run
export WAYLAND_DISPLAY=wayland-1
export LD_LIBRARY_PATH="$APP_DIR/misc/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

COMMON_SH="$SCRIPT_DIR/run_app_common.sh"
[ -r "$COMMON_SH" ] || COMMON_SH="$APP_DIR/misc/run_app_common.sh"
if [ ! -r "$COMMON_SH" ]; then
    echo "Helper script not found: run_app_common.sh" >&2
    exit 1
fi
. "$COMMON_SH"

check_wayland || exit 1

# clean up anything left from an earlier run (otherwise two feeders write the
# same device and the video overlaps)
"$LOOPBACK_SCRIPT" --stop
stop_stale_app app_wayland_npu

# own session: Ctrl+C must not stop the video feeder before the app has exited
APP_PATH="$APP_DIR" setsid "$LOOPBACK_SCRIPT" ${VIDEO_FILE:+"$VIDEO_FILE"} &
LOOPBACK_PID=$!
APP_PID=
GOT_INT=0

cleanup()
{
    stop_app "$APP_PID" "$GOT_INT"
    "$LOOPBACK_SCRIPT" --stop        # finds ffmpeg by command line, not by PID
    wait "$LOOPBACK_PID" 2>/dev/null
}

trap cleanup 0
trap 'GOT_INT=1; exit 130' INT
trap 'exit 129' HUP                  # terminal / SSH session closed
trap 'exit 131' QUIT
trap 'exit 143' TERM

wait_feeder "$LOOPBACK_PID" || exit 1

# give ffmpeg time to write the first frame (the app cannot set the
# capture format on v4l2loopback before that)
sleep 1
if ! is_running "$(cat "$FEEDER_PIDFILE" 2>/dev/null)"; then
    echo "V4L2 loopback video feeder stopped right after start" >&2
    exit 1
fi

for n in 11 12 13 14; do
    if [ ! -e "/dev/video$n" ]; then
        echo "V4L2 loopback device /dev/video$n is missing" >&2
        exit 1
    fi
done

"$APP_DIR/app_wayland_npu" &
APP_PID=$!
wait "$APP_PID"
APP_STATUS=$?
APP_PID=
exit "$APP_STATUS"
