#!/bin/sh
# Shared helpers for run_app_wayland_*.sh (sourced, not executed).

APP_STOP_TIMEOUT=${APP_STOP_TIMEOUT:-5}

# weston must be running, otherwise the app fails in nc_wayland_display_init()
check_wayland()
{
    if ! pidof weston >/dev/null 2>&1 || \
       [ ! -S "${XDG_RUNTIME_DIR:-/var/run}/${WAYLAND_DISPLAY:-wayland-1}" ]; then
        echo "Wayland compositor is not available (${XDG_RUNTIME_DIR:-/var/run}/${WAYLAND_DISPLAY:-wayland-1})." >&2
        echo "Restart it with: /etc/init.d/S99weston restart" >&2
        return 1
    fi
    return 0
}

# stop_app <pid> <got_int>
# Let the app shut down by itself (it releases its Wayland surface first).
# Ctrl+C already delivered SIGINT to the app, so only send it when it did not:
# a second SIGINT would kill the app at once (its handler uses SA_RESETHAND).
# A watchdog forces the stop after APP_STOP_TIMEOUT seconds.
stop_app()
{
    pid=$1
    got_int=$2
    [ -n "$pid" ] || return 0

    [ "$got_int" = 1 ] || kill -INT "$pid" 2>/dev/null

    (
        sleep "$APP_STOP_TIMEOUT"
        if kill -0 "$pid" 2>/dev/null; then
            echo "App did not exit within ${APP_STOP_TIMEOUT}s, forcing it to stop" >&2
            kill -TERM "$pid" 2>/dev/null
            sleep 1
            kill -KILL "$pid" 2>/dev/null
        fi
    ) &
    watchdog=$!

    wait "$pid" 2>/dev/null
    kill "$watchdog" 2>/dev/null
    wait "$watchdog" 2>/dev/null
}

# is_running <pid> : process exists and is not a zombie
is_running()
{
    [ -n "$1" ] && [ -r "/proc/$1/stat" ] || return 1
    st=$(sed 's/^.*) //' "/proc/$1/stat" 2>/dev/null | cut -c1)
    [ -n "$st" ] && [ "$st" != Z ]
}

FEEDER_PIDFILE=${FEEDER_PIDFILE:-/var/run/nc_v4l2loopback.pid}

# wait_feeder <launcher_pid>
# Wait until ffmpeg is running (run_v4l2loopback.sh writes its PID file right
# before exec). The launcher PID alone is not enough: when setsid has to fork,
# the launcher exits at once with status 0 while ffmpeg keeps running.
wait_feeder()
{
    launcher=$1
    n=0
    while [ "$n" -lt 100 ]; do
        if [ -r "$FEEDER_PIDFILE" ] && is_running "$(cat "$FEEDER_PIDFILE" 2>/dev/null)"; then
            return 0
        fi
        if [ -n "$launcher" ] && ! is_running "$launcher"; then
            wait "$launcher"
            if [ "$?" -ne 0 ]; then
                echo "V4L2 loopback video feeder failed to start" >&2
                return 1
            fi
            launcher=      # setsid forked: keep waiting for the PID file
        fi
        sleep 0.1
        n=$((n + 1))
    done
    echo "V4L2 loopback video feeder did not start within 10s" >&2
    return 1
}

# stop_stale_app <name> : stop an app instance left from an earlier run
stop_stale_app()
{
    pids=$(pidof "$1" 2>/dev/null)
    [ -n "$pids" ] || return 0
    for p in $pids; do
        echo "Stopping previous $1 (pid $p)" >&2
        kill -INT "$p" 2>/dev/null
    done
    for p in $pids; do
        n=0
        while is_running "$p" && [ "$n" -lt $((APP_STOP_TIMEOUT * 10)) ]; do
            sleep 0.1
            n=$((n + 1))
        done
        is_running "$p" && kill -KILL "$p" 2>/dev/null
    done
}
