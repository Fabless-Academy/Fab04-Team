#ifndef NC_CUTIN_H
#define NC_CUTIN_H

/*
 * nc_cutin : cut-in warning from predicted trajectories + ego lane
 *
 * Render thread only (no lock). Per displayed frame of a channel:
 *   nc_cutin_begin_frame(ch, disp_ts)
 *   for each predicted vehicle: nc_cutin_eval(...)   -> 1: draw as cut-in
 *   nc_cutin_end_frame(ch)                            -> 1: a warning just turned on
 *
 * Decision is made in lane coordinates, u = (x - x_left) / (x_right - x_left),
 * u in [0,1] = inside the ego lane. Curves / ego yaw shift cars and lanes
 * together, so u cancels them out; only real lateral motion changes u.
 *
 * Cut-in candidate (one frame):
 *   bottom-center now    : outside the ego lane (u < 0 or u > 1)
 *   bottom-center future : inside with margin (MARGIN <= u <= 1 - MARGIN)
 * Warning ON when the track stays a candidate for NC_CUTIN_ON_MS (and at least
 * NC_CUTIN_MIN_ON_HITS frames), OFF after NC_CUTIN_OFF_MS without being one.
 * Times use the displayed-frame timestamps, so the behavior does not change
 * with the frame rate (the app runs ~14 fps, not 30). (junwoo 11/06: was
 * 3 / 10 / 15 frames, i.e. ON 100 ms, OFF 333 ms, hold 500 ms at 30 fps)
 */

#include <stdint.h>
#include "nc_cnn_common.h"

#ifdef __cplusplus
extern "C" {
#endif

#define NC_CUTIN_MAX_CH          (8)
#define NC_CUTIN_MAX_TRACKS      (64)     /* per channel */
#define NC_CUTIN_LANE_MARGIN     (0.10f)  /* future point must be this far inside (lane width ratio) */
#define NC_CUTIN_HORIZON_MS      (1000.0f) /* prediction time for the decision. 500 ms missed a
                                           * slow (6 s) cut-in in the PC test; 1000 ms caught it with
                                           * no false warning (junwoo 11/05). Line drawing uses it too. */
/* Point of the car used for the decision (env CUTIN_POINT=0/1/2):
 *  CENTER : bottom-center of the box (junyi's original)
 *  EDGE   : bottom corner on our lane's side only (test; misses a car that is
 *           first seen with its near side already in our lane)
 *  BOTH   : center decides as before, AND the near corner must already be in
 *           our lane or be predicted to enter. Big cars beside us show more of
 *           their side as we close in, which pulls the box center toward our
 *           lane (bus false warning, board 11/06); the corner stays on the
 *           car's side line, so it rejects that case. */
#define NC_CUTIN_POINT_CENTER    (0)
#define NC_CUTIN_POINT_EDGE      (1)
#define NC_CUTIN_POINT_BOTH      (2)
#define NC_CUTIN_POINT_DEFAULT   NC_CUTIN_POINT_CENTER   /* BOTH gave the same result on the 11/06 board videos */
#define NC_CUTIN_MIN_ON_MS       (1000.0f) /* a warning stays on at least this long (was ~0.5 s) */
#define NC_CUTIN_DUP_MS          (3000.0f) /* same place within this time = same vehicle, no 2nd trigger */
#define NC_CUTIN_ON_MS           (100.0f) /* candidate this long in a row -> ON  (was 3 frames @30fps) */
#define NC_CUTIN_MIN_ON_HITS     (3)      /* ...and at least this many candidate frames (junyi's 3) */
#define NC_CUTIN_OFF_MS          (333.0f) /* not a candidate this long -> OFF  (was 10 frames @30fps) */
#define NC_CUTIN_LANE_STALE_US   (200000ULL) /* ignore lane result older than this ... */
#define NC_CUTIN_STALE_FRAMES    (3.0f)   /* ... or than 3 frame intervals, whichever is longer
                                           * (cut-in uses the previous frame's lane: detection is
                                           * drawn before the lane block in render()) */
#define NC_CUTIN_MIN_LANE_W_PX   (200.0f) /* narrower lane at the car's row (far car, roughly > 17 m
                                           * with a ~1000 px focal length) -> no decision. 120 gave a
                                           * false warning for a far bus beside us; 200 removed it and
                                           * kept both real cut-ins (board 11/06, junwoo) */
#define NC_CUTIN_REF_Y           (1000.0f) /* ego lane-change detection height (near bottom) */
#define NC_CUTIN_LC_JUMP_RATIO   (0.40f)  /* lane center jump > ratio * lane width = ego lane change */
#define NC_CUTIN_LC_HOLD_MS      (500.0f) /* pause decisions after ego lane change (was 15 frames @30fps) */

/* Board bring-up: print decision counts once per second per channel.
 * Uncomment NC_CUTIN_STATS to enable (ldw_board_check.sh then shows the cut-in stats).
 * no_lane = far + lost + stale:
 *   far   : lane narrower than MIN_LANE_W at that height (far car)
 *   lost  : no ego lane at that height (LDW has no lane / above the horizon)
 *   stale : lane result older than the stale limit */
/* #define NC_CUTIN_STATS */   /* uncomment for board tuning: stats + candidate debug log */
#define NC_CUTIN_STATS_MS        (1000.0f)

/*
 * Ego lane boundary, 1: ok, 0: no lane at this y (our convention).
 * y: screen px (same space as bbox, 1920x1080).
 * lane_ts_us: capture ts of the lane result (monotonic us), 0 = unknown.
 * Defined in nc_cutin.c:
 *   USE_LDW    : adapter to Junwoo's nc_ldw_get_ego_lane() (0 = ok, -1 = fail)
 *   otherwise  : fixed straight-road trapezoid stub (testing only)
 */
int nc_ldw_get_ego_lane_x(int ch, float y, float *x_left, float *x_right, uint64_t *lane_ts_us);

void nc_cutin_reset(void);

/* Tuning values in use (macro defaults, overridable by environment variables
 * on the board, e.g.  CUTIN_HORIZON_MS=1000 CUTIN_MARGIN=0.05 ./run_app_wayland_npu.sh
 *   CUTIN_POINT        NC_CUTIN_POINT_DEFAULT (0 center / 1 edge / 2 both)
 *   CUTIN_MIN_ON_MS    NC_CUTIN_MIN_ON_MS
 *   CUTIN_DUP_MS       NC_CUTIN_DUP_MS
 *   CUTIN_HORIZON_MS   NC_CUTIN_HORIZON_MS
 *   CUTIN_MARGIN       NC_CUTIN_LANE_MARGIN
 *   CUTIN_ON_MS        NC_CUTIN_ON_MS
 *   CUTIN_OFF_MS       NC_CUTIN_OFF_MS
 *   CUTIN_LC_HOLD_MS   NC_CUTIN_LC_HOLD_MS
 *   CUTIN_MIN_LANE_W   NC_CUTIN_MIN_LANE_W_PX
 *   CUTIN_LANE_STALE_MS NC_CUTIN_LANE_STALE_US / 1000 (minimum; 3 frame intervals if longer)
 * Read once, at the first frame (printed as "[nc_cutin] config: ..."). */
float nc_cutin_get_horizon_ms(void);

/* [render] start of a displayed frame of ch */
void nc_cutin_begin_frame(int ch, uint64_t disp_ts_us);

/* [render] now_box: predicted box at the displayed frame (top-left x,y,w,h).
 * fut_cx/fut_cy: predicted center after the horizon (nc_traj_get_future_center).
 * Returns 1 while the warning for this track is ON. */
int nc_cutin_eval(int ch, int class_id, int track_id, const stBBox *now_box,
                  float fut_cx, float fut_cy, uint64_t disp_ts_us);

/* [render] end of the frame. Tracks not evaluated this frame count as
 * non-candidates. Returns 1 if any warning turned OFF->ON this frame
 * (edge, for the recording trigger). */
int nc_cutin_end_frame(int ch);

/* [render] number of tracks with warning ON in ch (after end_frame) */
int nc_cutin_active_count(int ch);

#ifdef __cplusplus
}
#endif

#endif /* NC_CUTIN_H */
