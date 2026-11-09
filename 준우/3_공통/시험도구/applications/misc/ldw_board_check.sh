#!/bin/sh
# Board check for the video input + LDW.
#
#   ./ldw_board_check.sh [seconds] [video-file]   (default 50 s, default video)
#   e.g. ./ldw_board_check.sh 50 misc/image_npu/1105_kki.mov   (cut-in check)
#
# Runs run_app_wayland_npu.sh twice (long run, then a short restart check),
# stops it the same way Ctrl+C does, and writes everything that has to be
# checked into ldw_check_<date>.txt (+ the full app logs next to it).
# Watch the screen while it runs: measurement line, vehicle marker, warnings.

SCRIPT_DIR=$(CDPATH= cd -P "$(dirname "$0")" && pwd) || exit 1
RUN="$SCRIPT_DIR/run_app_wayland_npu.sh"
DUR=${1:-50}
VIDEO=${2:-}
case "$VIDEO" in
    ""|/*) ;;
    *) VIDEO="$PWD/$VIDEO" ;;
esac
if [ -n "$VIDEO" ] && [ ! -r "$VIDEO" ]; then
    echo "Video file not found: $VIDEO" >&2
    exit 1
fi
RESTART_DUR=${RESTART_DUR:-15}
STAMP=$(date +%m%d_%H%M%S)
OUT="$SCRIPT_DIR/ldw_check_$STAMP.txt"
LOG1="$SCRIPT_DIR/ldw_check_${STAMP}_run1.log"
LOG2="$SCRIPT_DIR/ldw_check_${STAMP}_run2.log"

if [ ! -x "$RUN" ]; then
    echo "Not found or not executable: $RUN" >&2
    exit 1
fi

section() { printf '\n===== %s\n' "$*"; }
count_procs() { ps | grep -E "$1" | grep -v grep | wc -l; }
mali_faults() { dmesg | grep -ci "mali.*fault"; }

CUR_PID=
RESULT=

# Ctrl+C on this script: the app script runs in the background (it does not
# get Ctrl+C itself), so stop it explicitly - it then stops the app and ffmpeg.
abort()
{
    trap - INT TERM HUP
    if [ -n "$CUR_PID" ]; then
        echo "Stopping the app..." >&2
        kill -TERM "$CUR_PID" 2>/dev/null
        wait "$CUR_PID" 2>/dev/null
    fi
    echo "Aborted. Partial results: $OUT" >&2
    exit 130
}
trap abort INT TERM HUP

# run_once <log> <seconds>: start the app script, stop it like Ctrl+C would.
# Sets RESULT.
run_once()
{
    "$RUN" ${VIDEO:+"$VIDEO"} > "$1" 2>&1 &
    CUR_PID=$!
    n=0
    while [ "$n" -lt "$2" ] && kill -0 "$CUR_PID" 2>/dev/null; do
        sleep 1
        n=$((n + 1))
    done
    if kill -0 "$CUR_PID" 2>/dev/null; then
        kill -TERM "$CUR_PID"        # script sends SIGINT to the app, then stops ffmpeg
        wait "$CUR_PID"
        RESULT="stopped after ${n}s (script exit $?)"
    else
        wait "$CUR_PID"
        RESULT="!! script ended by itself after ${n}s (exit $?) - see log"
    fi
    CUR_PID=
}

MALI_BEFORE=$(mali_faults)

{
    section "board"
    date
    echo "video: ${VIDEO:-default (run_v4l2loopback.sh)}"
    uname -a
    cat /proc/loadavg

    section "before start"
    echo "weston running     : $(pidof weston >/dev/null && echo yes || echo NO)"
    echo "ffmpeg processes   : $(count_procs 'ffmpeg')"
    echo "app processes      : $(count_procs '(^|[ /])app_wayland_(npu|cam)( |$)')"
    echo "mali faults (dmesg): $MALI_BEFORE"
} > "$OUT"

echo "Run 1: ${DUR}s - watch the screen (measurement line, vehicle marker, warnings)"
run_once "$LOG1" "$DUR"; R1=$RESULT
sleep 1
AFTER1_FF=$(count_procs 'ffmpeg')
AFTER1_APP=$(count_procs '(^|[ /])app_wayland_npu( |$)')

echo "Run 2: ${RESTART_DUR}s - restart check"
run_once "$LOG2" "$RESTART_DUR"; R2=$RESULT
sleep 1

{
    section "run 1 (${DUR}s)"
    echo "$R1"
    echo "left after run 1   : ffmpeg=$AFTER1_FF app=$AFTER1_APP   (expect 0 / 0)"

    section "run 2 (restart, ${RESTART_DUR}s)"
    echo "$R2"
    echo "left after run 2   : ffmpeg=$(count_procs 'ffmpeg') app=$(count_procs '(^|[ /])app_wayland_npu( |$)')   (expect 0 / 0)"
    echo "weston still alive : $(pidof weston >/dev/null && echo yes || echo NO)"

    section "GPU (Mali) faults"
    MALI_AFTER=$(mali_faults)
    echo "before=$MALI_BEFORE after=$MALI_AFTER new=$((MALI_AFTER - MALI_BEFORE))   (expect new=0)"
    dmesg | grep -i "mali.*fault" | tail -5

    section "video input"
    if [ -r /sys/module/v4l2loopback/parameters/max_buffers ]; then
        echo "v4l2loopback max_buffers: $(cat /sys/module/v4l2loopback/parameters/max_buffers)"
    fi
    grep -hE "Feeding|Reloading|in use, keeping|Stopping|\[v4l2\]" "$LOG1" "$LOG2"

    section "errors in logs"
    grep -hiE "assert|failure|failed|error|segmentation|not available|missing" "$LOG1" "$LOG2" | sort | uniq -c | head -20

    section "LDW state changes (run 1)"
    grep "\[LDW\]" "$LOG1" | grep -v "lane_class" | head -60
    echo "warnings: strong=$(grep -c 'WARN_[A-Z]*(strong)' "$LOG1")  weak=$(grep -c 'WARN_[A-Z]*(weak)' "$LOG1")"
    echo "nc_rec_trigger calls: $(grep -c 'nc_rec_trigger(0)' "$LOG1")   (expect = number of warnings that turned on from NORMAL)"

    section "lane_class per slot (run 1) - count of 'slot:class'"
    grep "lane_class:" "$LOG1" | awk '{
        for (i = 1; i <= NF; i++) if ($i ~ /^\[[0-9]\]class=/) {
            split($i, a, "class="); split(a[2], b, "("); cnt[substr($i, 2, 1) ":" b[1]]++
        }
    } END { for (k in cnt) printf "  slot %s -> %d\n", k, cnt[k] }' | sort
    echo "slot1 left of slot2 (x1 < x2): $(grep 'lane_class:' "$LOG1" | awk '{
        x1 = ""; x2 = "";
        for (i = 1; i <= NF; i++) {
            if ($i ~ /^\[1\]class=/) { split($i, a, /x=|\)/); x1 = a[2] }
            if ($i ~ /^\[2\]class=/) { split($i, a, /x=|\)/); x2 = a[2] }
        }
        if (x1 != "" && x2 != "") { n++; if (x1 + 0 < x2 + 0) l++ }
    } END { printf "%d of %d", l, n }')"
    echo "sample:"
    grep "lane_class:" "$LOG1" | head -3

    section "cut-in (run 1)"
    grep -h "\[nc_cutin\] config" "$LOG1" | head -2
    grep -h "\[LDW\] \(ego lane for\|TEST\)" "$LOG1" | head -2
    echo "ego lane source changes: $(grep -c 'ego lane source' "$LOG1")   (held: $(grep -c 'source: .* -> held' "$LOG1"), virtual: $(grep -c 'source: .* -> virtual' "$LOG1"))"
    echo "cut-in ON by lane: measured $(grep -c 'lane measured)' "$LOG1"), held $(grep -c 'lane held)' "$LOG1"), virtual $(grep -c 'lane virtual)' "$LOG1")"
    grep -hE "\[nc_cutin\].*(cut-in O|ego lane change)" "$LOG1" | head -40
    echo "cut-in ON: $(grep -c 'cut-in ON' "$LOG1")   nc_rec_trigger(1) calls: $(grep -c 'nc_rec_trigger(1)' "$LOG1")   (expect equal)"
    grep "\[nc_cutin\] ch0 stats" "$LOG1" | awk '{
        for (i = 1; i <= NF; i++) { split($i, kv, ":"); if (kv[2] != "") s[kv[1]] += kv[2] }
        n++
    } END { if (n) printf "stats total over %d s: eval=%d hold=%d no_lane=%d (far=%d lost=%d stale=%d) inside=%d not_enter=%d cand=%d\n", n, s["eval"], s["hold"], s["no_lane"], s["far"], s["lost"], s["stale"], s["inside"], s["not_enter"], s["cand"] }'
    echo "per second (eval far lost stale inside not_enter cand on fps):"
    grep "\[nc_cutin\] ch0 stats" "$LOG1" | awk '{
        line = ""
        for (i = 1; i <= NF; i++) { split($i, kv, ":"); if (kv[1] ~ /^(eval|far|lost|stale|inside|not_enter|cand|on|fps)$/) line = line " " kv[2] }
        printf "  %2d s:%s\n", ++n, line
    }' | head -60
    grep -h "\[nc_traj\]" "$LOG1" | head -3

    section "app shutdown messages"
    grep -hE "\[app\]|did not exit" "$LOG1" "$LOG2"

    section "files"
    echo "summary : $OUT"
    echo "run logs: $LOG1"
    echo "          $LOG2"
} >> "$OUT"

echo
echo "Done. Send these files:"
echo "  $OUT"
echo "  $LOG1"
echo "  $LOG2"
