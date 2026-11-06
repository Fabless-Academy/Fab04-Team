#!/bin/sh

SCRIPT_DIR=$(CDPATH= cd -P "$(dirname "$0")" && pwd) || exit 1

if [ -n "${APP_PATH:-}" ]; then
    APP_DIR=$APP_PATH
elif [ -d "$SCRIPT_DIR/misc" ]; then
    APP_DIR=$SCRIPT_DIR
else
    APP_DIR=$(dirname "$SCRIPT_DIR")
fi

APP_DIR=$(CDPATH= cd -P "$APP_DIR" && pwd) || {
    echo "Application directory not found: $APP_DIR" >&2
    exit 1
}

if [ "$#" -gt 1 ]; then
    echo "Usage: $0 [video-file]" >&2
    exit 2
fi

VIDEO_FILE=
if [ "$#" -eq 1 ]; then
    case "$1" in
        /*) VIDEO_FILE=$1 ;;
        *) VIDEO_FILE=$(pwd)/$1 ;;
    esac
fi

if [ ! -x "$APP_DIR/app_wayland_cam" ]; then
    echo "Camera app not found or not executable: $APP_DIR/app_wayland_cam" >&2
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

cd "$APP_DIR" || exit 1

export XDG_RUNTIME_DIR=/var/run
export WAYLAND_DISPLAY=wayland-1

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
stop_stale_app app_wayland_cam

# own session: Ctrl+C must not stop the video feeder before the app has exited
if [ "$#" -eq 1 ]; then
    APP_PATH="$APP_DIR" setsid "$LOOPBACK_SCRIPT" "$VIDEO_FILE" &
else
    APP_PATH="$APP_DIR" setsid "$LOOPBACK_SCRIPT" &
fi
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

for n in 11; do
    if [ ! -e "/dev/video$n" ]; then
        echo "V4L2 loopback device /dev/video$n is missing" >&2
        exit 1
    fi
done

"$APP_DIR/app_wayland_cam" &
APP_PID=$!
wait "$APP_PID"
APP_STATUS=$?
APP_PID=
exit "$APP_STATUS"
