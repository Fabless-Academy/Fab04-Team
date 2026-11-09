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

VIDEO_NR=11
FEEDER_PIDFILE=${FEEDER_PIDFILE:-/var/run/nc_v4l2loopback.pid}

# Feed settings (override with environment variables)
#  FEED_FPS            : frames per second written to the device. The app shows
#                        about 9-10 fps; writing much faster only overwrites
#                        frames the app is still reading (video jumps/tears).
#  FEED_DEVICES        : how many devices to feed, from /dev/video11 (1-4).
#                        app_wayland_npu/cam use one channel (/dev/video11).
#  LOOPBACK_MAX_BUFFERS: v4l2loopback ring size (module default is 2).
FEED_FPS=${FEED_FPS:-15}
FEED_DEVICES=${FEED_DEVICES:-1}
LOOPBACK_MAX_BUFFERS=${LOOPBACK_MAX_BUFFERS:-4}

# ---------------------------------------------------------------------------
# Feeder process control. Feeders are found by their command line
# ("ffmpeg ... -f v4l2 /dev/video11"), not by a saved PID, so a feeder is
# found even when it was started by hand, by an older script, or when
# setsid forked and the launcher PID is not the ffmpeg PID.
# ---------------------------------------------------------------------------
is_running()
{
    [ -n "$1" ] && [ -r "/proc/$1/stat" ] || return 1
    st=$(sed 's/^.*) //' "/proc/$1/stat" 2>/dev/null | cut -c1)
    [ -n "$st" ] && [ "$st" != Z ]
}

find_feeders()
{
    for d in /proc/[0-9]*; do
        p=${d#/proc/}
        [ "$p" = "$$" ] && continue
        cmd=$( (tr '\0' ' ' < "$d/cmdline") 2>/dev/null ) || continue   # process may exit meanwhile
        case "$cmd" in
            *ffmpeg*"-f v4l2 /dev/video$VIDEO_NR"*) echo "$p" ;;
        esac
    done
}

stop_feeders()
{
    pids=$(find_feeders)
    for p in $pids; do
        echo "Stopping video feeder (pid $p)" >&2
        kill -TERM "$p" 2>/dev/null
    done
    for p in $pids; do
        n=0
        while is_running "$p" && [ "$n" -lt 20 ]; do
            sleep 0.1
            n=$((n + 1))
        done
        is_running "$p" && kill -KILL "$p" 2>/dev/null
    done
    rm -f "$FEEDER_PIDFILE"
}

if [ "${1:-}" = "--stop" ]; then
    stop_feeders
    exit 0
fi

INPUT_FILE_PATH=${1:-"$APP_DIR/misc/image_npu/1920_1080_0001.mp4"}
FFMPEG="$APP_DIR/misc/ffmpeg"
LOOPBACK_MODULE="$APP_DIR/../modules/v4l2loopback_ko"

if [ "$#" -gt 1 ]; then
    echo "Usage: $0 [video-file] | --stop" >&2
    exit 2
fi

if [ ! -r "$INPUT_FILE_PATH" ]; then
    echo "Video file not found or unreadable: $INPUT_FILE_PATH" >&2
    exit 1
fi

if [ ! -x "$FFMPEG" ]; then
    echo "FFmpeg not found or not executable: $FFMPEG" >&2
    exit 1
fi

case "$FEED_DEVICES" in
    1|2|3|4) ;;
    *) echo "FEED_DEVICES must be 1-4 (got $FEED_DEVICES)" >&2; exit 2 ;;
esac

# Only one feeder per device: two writers interleave frames (overlapping video).
# Stop old feeders first, so the module is free to be reloaded below.
stop_feeders

# Module loaded earlier with a smaller ring (e.g. the default 2): reload it.
# This only works when nothing uses the devices; otherwise keep the old one.
MAXBUF_PARAM=/sys/module/v4l2loopback/parameters/max_buffers
if [ -r "$MAXBUF_PARAM" ] && [ "$(cat "$MAXBUF_PARAM")" -lt "$LOOPBACK_MAX_BUFFERS" ]; then
    if rmmod v4l2loopback 2>/dev/null; then
        echo "Reloading v4l2loopback with max_buffers=$LOOPBACK_MAX_BUFFERS" >&2
    else
        echo "v4l2loopback is in use, keeping max_buffers=$(cat "$MAXBUF_PARAM")" >&2
    fi
fi

if [ ! -e /sys/module/v4l2loopback ]; then
    if [ ! -r "$LOOPBACK_MODULE" ]; then
        SOURCE_MODULE="$APP_DIR/../modules/v4l2loopback_drv_prebuild/v4l2loopback_ko"
        if [ -r "$SOURCE_MODULE" ]; then
            LOOPBACK_MODULE=$SOURCE_MODULE
        else
            echo "V4L2 loopback module not found: $LOOPBACK_MODULE" >&2
            exit 1
        fi
    fi

    insmod "$LOOPBACK_MODULE" \
        video_nr=$((VIDEO_NR+0)),$((VIDEO_NR+1)),$((VIDEO_NR+2)),$((VIDEO_NR+3)) \
        max_buffers="$LOOPBACK_MAX_BUFFERS" || {
        echo "Failed to load the V4L2 loopback module" >&2
        exit 1
    }
fi

attempt=0
while [ ! -e /dev/video11 ] || [ ! -e /dev/video12 ] || \
      [ ! -e /dev/video13 ] || [ ! -e /dev/video14 ]; do
    if [ "$attempt" -ge 5 ]; then
        echo "Expected loopback devices /dev/video11 through /dev/video14 are unavailable." >&2
        echo "If v4l2loopback is already loaded with different device numbers, unload it before retrying." >&2
        exit 1
    fi
    sleep 1
    attempt=$((attempt + 1))
done

export LD_LIBRARY_PATH="$APP_DIR/misc/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

# ffmpeg arguments: one output per fed device, always 1920x1080 RGB24
# (-s scales other video sizes to what the apps expect)
set -- -stream_loop -1 -re -i "$INPUT_FILE_PATH"
n=0
while [ "$n" -lt "$FEED_DEVICES" ]; do
    # FEED_VF (optional): ffmpeg filter for test videos whose framing differs
    # from our camera, e.g. horizon too low:
    #   FEED_VF="crop=1067:600:426:480" ./run_app_wayland_npu.sh video.mp4
    if [ -n "$FEED_VF" ]; then
        set -- "$@" -vf "$FEED_VF"
    fi
    set -- "$@" -s 1920x1080 -pix_fmt rgb24 -r "$FEED_FPS" -f v4l2 "/dev/video$((VIDEO_NR + n))"
    n=$((n + 1))
done

echo "Feeding $INPUT_FILE_PATH -> /dev/video$VIDEO_NR (x$FEED_DEVICES) at ${FEED_FPS} fps${FEED_VF:+ (filter: $FEED_VF)}" >&2

# exec keeps this PID, so the pid file holds the ffmpeg PID
echo "$$" > "$FEEDER_PIDFILE" 2>/dev/null || true

exec "$FFMPEG" "$@"
