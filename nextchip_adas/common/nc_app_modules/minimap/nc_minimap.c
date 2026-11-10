#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <math.h>

#include "nc_minimap.h"
#include "nc_ldw.h"
#include "nc_bevlog.h"

/*
 * Road coordinates (m): x = lateral from vehicle centre (+: right),
 *                       z = forward from the camera.
 * Panel coordinates   : canvas px, y grows downward (same as other overlays).
 *
 * Position pipeline is the same as the first minimap (14:34 build):
 * drawn box bottom-centre -> LDW road model with the calibrated horizon ->
 * EMA 0.35 per displayed frame. On top of it only display logic was added
 * (track joining over class / id changes, class vote, range hysteresis,
 * short hold, opaque panel, lanes from the network points).
 * Vehicle lateral position and the lanes use the geometry of the lane frame itself.
 * Drawing uses only the existing rectangle / line / quad primitives.
 */

#define MM_MAX_OBS          (64)        // detections per frame
#define MM_MAX_TRACK        (48)        // minimap tracks per channel
#define MM_MAX_ALIAS        (4)         // (class, tracker id) keys per minimap track
#define MM_BEHIND_M         (3.0f)      // road shown behind the camera (ego body)

#define MM_LANE_LINES       (4)         // network lane slots: 0, 3 outer lines, 1, 2 ego lines
#define MM_PRED_LANE_MAX    (2)         // lanes each side of the ego lane that get an arrow (the lanes the panel draws)
#define MM_SMOOTH_ALPHA     (0.35f)     // position EMA per displayed frame (as before)
// [11.09 서영] 먼 차(z > FAR_Z)는 지평선 가까이라 박스 바닥이나 LDW 지평선(vp)이 몇 px만 흔들려도
//   거리(z)와 옆 위치(x)가 함께 크게 튐 (px 다리: 버스 박스 h 112->188->114 한 번에 z 8 / 46 / 100)
//   -> 이번 프레임 z 변화가 "초당 max(DZ_MIN, 현재 z x DZ_FRAC)"를 넘으면 그 프레임은 그만큼만 믿고
//      x, z를 같은 비율(w)로만 따라감. 천천히 바뀌는 vp(실제 기울기 변화)는 그대로 따라감
#define MM_FAR_Z_M          (12.0f)
#define MM_FAR_DZ_MIN       (6.0f)      // 미니맵 z 단위/초
#define MM_FAR_DZ_FRAC      (1.0f)      // 현재 z의 이 비율/초 (z 20 -> 초당 20까지)
#define MM_TRACK_GAP_US     (500000ULL) // longer gap -> restart smoothing / no re-association
#define MM_HOLD_US          (300000ULL) // keep drawing a vehicle this long after its last detection
#define MM_DROP_US          (1000000ULL)// forget a track not detected this long

/* Vehicle leaving through the side of the image: once the box touches the edge the rear
 * is out of view and the box bottom moves to the front wheels, so z stalls and x bends
 * inwards (0001: about 1 s at z = 2.2). The measurement is dropped there and the vehicle
 * goes on with the closing speed and x from before the edge, faded out, drawn dashed.
 * A long vehicle in the next lane fills the view before it reaches the edge: its bottom
 * goes behind the dashboard and z runs away (0003: bus, z 4 -> 18 at the end), so a tall
 * box beside the ego lane counts as well. Never in the ego lane: that is a vehicle ahead.
 * Only for vehicles closing slower than V_MAX (faster ones are crossing or oncoming)
 * and with an x that moved less than X_MAX. Speeds in minimap z units per second. */
#define MM_COAST_HIST       (96)        // position history per track (2.5 s at up to 38 fps)
#define MM_COAST_EDGE_PX    (20.0f)     // box this close to the image side: touching
#define MM_COAST_TALL_PX    (500.0f)    // box this tall counts as touching
#define MM_COAST_TALL_X_M   (2.0f)      // ... when at least this far from the ego lane centre
#define MM_COAST_CANCEL_PX  (60.0f)     // box this far inside again and
#define MM_COAST_CANCEL_H_PX (400.0f)   // lower than this: measured position is back
#define MM_COAST_MAX_Z_M    (12.0f)     // farther vehicles at the edge are not passing
#define MM_COAST_V_MIN      (0.5f)
#define MM_COAST_V_MAX      (6.0f)
#define MM_COAST_X_MAX_M    (1.5f)
#define MM_COAST_V_NEW_S    (0.2f)      // speed from samples aged V_NEW..V_OLD (newest are bent)
#define MM_COAST_V_OLD_S    (2.5f)      // long: boxes of near vehicles shake by about 1 m
#define MM_COAST_V_MIN_PTS  (8)
#define MM_COAST_X_NEW_S    (0.5f)      // x from samples aged X_NEW..X_OLD
#define MM_COAST_X_OLD_S    (1.5f)
#define MM_COAST_X_BLEND_S  (0.3f)      // edge x -> x from before the edge
#define MM_COAST_MAX_S      (5.0f)      // 0003: a bus takes about 4.9 s to pass
#define MM_COAST_FADE_S     (1.2f)      // fade over the last part of MAX_S
#define MM_COAST_FADE_Z_M   (3.0f)      // fade over the last part of the panel
#define MM_COAST_FILL_ALPHA (0.22f)
#define MM_COAST_DASH_M     (0.6f)      // outline dash and gap

enum { MM_COAST_NONE = 0, MM_COAST_RUN, MM_COAST_DONE };

/* Oncoming vehicle: left of the ego lane (lane position < POS_MAX, 0 = ego lane centre,
 * as for the arrows) and its smoothed z falling faster than V over the last WIN_S
 * (>= MIN_PTS samples spanning >= SPAN_S). Kept until the track ends; it then gets no
 * arrow. review1109 logs (offline): no same-direction vehicle flagged on 0001-0003.
 * Speed alone flags far vehicles (z jitter: 0001 half the rows), lane alone flags
 * 0001 left-lane traffic. The speed is relative to the other vehicles (see
 * mm_flag_oncoming): a road crest in 0001 makes every far vehicle close at once.
 * The first ~0.2 s of an oncoming vehicle is not flagged yet. */
#define MM_ONC_HIST         (16)        // smoothed z history (0.3 s at up to 50 fps)
#define MM_ONC_POS_MAX      (-0.5f)
#define MM_ONC_V            (10.0f)     // minimap z units per second
#define MM_ONC_WIN_S        (0.3f)
#define MM_ONC_SPAN_S       (0.15f)
#define MM_ONC_MIN_PTS      (4)

/* Arrow lean (NC_MINIMAP_PRED_LANE_REL): least-squares lateral speed of the box bottom over
 * the last WIN_S (>= MIN_PTS samples spanning >= SPAN_S). Every sample is converted from
 * its pixels with the current frame geometry, so a geometry change does not look like a
 * move (0001 f418: x jumped 1.7 m with the box still), and the move of the lane model
 * between the sample and now is removed (ego steering). The arrow leans after the speed
 * stays above V_ON for HOLD_S and goes straight again below V_OFF; lean = speed x HZ_S,
 * at most LEAN_MAX_M. Buses and trucks nearer than BIG_Z_M stay straight: their box takes
 * in the side as they come close and the bottom centre moves sideways (0003 bus).
 * oncrun logs (offline): 0001 lane-keeping cars leaning > 0.25 m 26% -> 7%, wrong way in
 * lane changes 12% -> 6% (0003 32% -> 17%), but lane changes show later (right way 76% ->
 * 55%). Curves (0001 curve end, 0002) stay wrong. */
#define MM_PRED_HIST        (48)        // 1.2 s at up to 40 fps
#define MM_PRED_WIN_S       (1.0f)
#define MM_PRED_MIN_PTS     (8)
#define MM_PRED_SPAN_S      (0.6f)
#define MM_PRED_V_ON        (0.5f)      // minimap x units per second
#define MM_PRED_V_OFF       (0.3f)
#define MM_PRED_HOLD_S      (0.2f)
#define MM_PRED_HZ_S        (0.5f)
#define MM_PRED_LEAN_MAX_M  (0.6f)
#define MM_PRED_BIG_Z_M     (12.0f)

/* Yellow centre line (NC_MINIMAP_YELLOW_CENTRE): pixels of the camera frame within
 * HALF_PX (canvas px) left and right of each point of the lines left of the ego vehicle.
 * A line is yellow when FRAC of them are yellow (HSV hue 36-68 deg, s and v >= 0.35),
 * held HOLD_US after the last yellow measurement (lines are often not detected), and
 * dropped after WHITE_US of non-yellow measurements (lane change). The detected points
 * lag the paint by a few px, hence the wide window. Sampled frames of 0001-0003:
 * 0002 centre line 0.1-0.8 in every frame where it was detected, other lines <= 0.05. */
#define MM_YEL_HALF_PX      (24.0f)
#define MM_YEL_MAX_ROW_PX   (1004.0f)   // rows below this are the bonnet / dashboard
#define MM_YEL_MIN_SAMPLES  (20)
#define MM_YEL_FRAC         (0.1f)
#define MM_YEL_HOLD_US      (2000000ULL)
#define MM_YEL_WHITE_US     (500000ULL)
#define MM_YEL_GAP_M        (0.26f)     // distance between the two drawn lines

/* association gates (m) */
#define MM_GATE_Z_MIN       (2.0f)
#define MM_GATE_Z_RATIO     (0.15f)
#define MM_GATE_X           (1.5f)
#define MM_DUP_Z_MIN        (1.0f)      // same vehicle detected twice (two classes) in one frame
#define MM_DUP_Z_RATIO      (0.06f)
#define MM_DUP_X            (0.8f)

/* class vote (once per detection frame) */
#define MM_CLS_DECAY        (0.8f)
#define MM_CLS_SWITCH       (1.0f)      // other class must lead by this much to switch

/* lanes: one centre curve x = c0 + h*z + c*z^2 from the ego-line points, with this
 * frame's geometry (k_inst, vp_inst); lines are drawn at fixed offsets from it, so the
 * lane width never changes. The ego-line points end near 13 m, and in the board logs
 * c from them is wrong by up to about 3 m of bend at 25 m on straight roads, so c is
 * only used above CURVE_ON_M: gentle curves are drawn straight on purpose. */
#define MM_LANE_MAX_Z_M      (40.0f)    // ignore points farther than this (noisy)
#define MM_LANE_MIN_ROW_PX   (8.0f)     // ignore points this close to the horizon
#define MM_LANE_MIN_PTS      (4)        // fewer points in a frame: keep the last model
#define MM_LANE_QUAD_SPAN_M  (5.0f)     // need this z span to measure c
#define MM_LANE_CURVE_REF_M  (25.0f)    // CURVE_ON/OFF are the bend at this distance
#define MM_LANE_CURVE_ON_M   (4.5f)     // straight -> curve
#define MM_LANE_CURVE_OFF_M  (3.2f)     // curve -> straight
#define MM_LANE_C_TAU_S      (1.0f)     // low-pass of the measured c
#define MM_LANE_SHOW_TAU_S   (0.4f)     // fade between straight and curve
#define MM_LANE_POS_TAU_S    (0.09f)    // low-pass of c0, h (same as EMA 0.35 at 26 fps)
#define MM_LANE_INST_MAX_PX  (30.0f)    // frame horizon this far from the calibrated one: bad frame
#define MM_LANE_GEOM_HOLD_US (1000000ULL) // keep the last frame geometry this long (one line lost)
#define MM_LANE_HOLD_US      (500000ULL)  // no fit this long: no lateral correction from the lines
/* Lanes lost (no fit), same steps as LDW: held = the last model drawn as it was for
 * HELD_US, then virtual = the last offset kept, heading and curve fading to straight
 * (VIRT_TAU_S), all lines dashed and faint, until lines are fitted again. The age is
 * taken from the displayed frame, so it also runs when no lane result comes (SKIP). */
#define MM_LANE_HELD_US      (1000000ULL)
#define MM_LANE_VIRT_TAU_S   (1.0f)
#define MM_LANE_VIRT_ALPHA   (0.45f)    // alpha factor of the virtual lines
#define MM_LANE_OUTER_HOLD_US (1000000ULL)// outer line not detected this long: not drawn
#define MM_LANE_SEG_M        (4.0f)     // drawn segment length
#define MM_LANE_EXTRAP_ALPHA (0.55f)    // alpha factor of the extrapolated part
#define MM_LAT_MIN_W_PX      (10.0f)    // ego lines closer than this on the vehicle row: no correction
#define MM_LAT_TAU_S         (0.5f)     // low-pass of the lateral correction of a track

/* Side face: the box of a vehicle beside the ego lane holds its rear and the side facing
 * the ego vehicle (0001: trucks 3.2-3.7 m wide boxes), so the box centre is inside the
 * vehicle centre. Beside the ego lane the centre is taken from the outer box edge (rear
 * outer corner) minus half the class width, never further in than the box centre. Not when
 * that edge touches the image side (cut there). Full from |x| >= SIDE_X1_M, none below SIDE_X0_M. */
#define MM_SIDE_CORR         (1)
#define MM_SIDE_X0_M         (1.2f)
#define MM_SIDE_X1_M         (2.4f)
#define MM_SIDE_EDGE_PX      (8.0f)

/* Drawing only: of two vehicles side by side (lateral distance > SEP_MIN_M) whose boxes
 * overlap, the outer one is pushed outwards until they are SEP_GAP_M apart, at most SEP_MAX_M.
 * Far lanes are drawn too far in, more the farther out (0001: trucks in lanes +2 and +3, 3.5 m
 * apart, drawn 1.9 m apart), so moving the outer one is the smaller error. Ramps in over
 * SEP_RAMP_M of overlap along the lane so that it does not jump. */
#define MM_SEP_MIN_M         (0.8f)
#define MM_SEP_GAP_M         (0.2f)
#define MM_SEP_MAX_M         (1.2f)
#define MM_SEP_RAMP_M        (1.0f)
#define MM_SEP_ITER          (3)

/* ego vehicle body relative to the camera (m) */
#define MM_EGO_W_M          (1.85f)
#define MM_EGO_FRONT_M      (2.0f)      // bumper ahead of the camera
#define MM_EGO_REAR_M       (2.5f)      // body behind the camera

enum { MM_CLS_CAR = 0, MM_CLS_TRUCK, MM_CLS_BUS, MM_CLS_MOTO, MM_CLS_N };
static const float k_cls_w[MM_CLS_N] = { 1.85f, 2.3f,  2.5f, 0.8f };
static const float k_cls_l[MM_CLS_N] = { 4.5f,  7.0f, 11.0f, 2.0f };

typedef struct {
    const char *class_name;     // pointer from the network ini (stable)
    int         track_id;
} stMmAlias;

typedef struct {
    int       used;
    uint32_t  id;
    stMmAlias alias[MM_MAX_ALIAS];
    int       alias_next;
    float     x, z;             // smoothed position
    float     lat_dx;           // smoothed lateral correction (stMmObs.lat_dx), held while not measured
    uint64_t  lat_us;           // displayed frame of the last lat_dx measurement (0 = none)
    int       n_upd;
    uint64_t  last_upd_disp_us; // displayed frame of the last position update
    uint64_t  last_det_us;      // detection frame of the last class vote
    uint64_t  last_dup_det_us;  // detection frame of the last duplicate (half) vote
    uint64_t  last_seen_disp_us;
    int       in_range;         // range hysteresis state
    float     cls_score[MM_CLS_N];
    int       cls;
    float     color[MM_CLS_N][4];
    int       has_color[MM_CLS_N];
    int       highlight;
    float     v_prev;           // raw box bottom (px) of the last update (pitch estimate)
    uint64_t  v_prev_us;        // displayed frame of v_prev
    int       fut_n;            // smoothed predicted offset available (0 = none)
    float     fut_dx, fut_dz;   // predicted position - position (m), smoothed
    uint64_t  hist_us[MM_COAST_HIST];   // road positions of the last updates, newest at hist_n - 1
    float     hist_x[MM_COAST_HIST], hist_z[MM_COAST_HIST];
    uint32_t  hist_n;
    int       coast;            // MM_COAST_*
    uint64_t  coast_us;         // displayed frame where it started
    float     coast_x0, coast_z0;   // position there
    float     coast_xh, coast_v;    // x and z speed from before the edge
    float     coast_a;          // opacity
    uint64_t  onc_us[MM_ONC_HIST];  // smoothed z of the last updates, newest at onc_n - 1
    float     onc_z[MM_ONC_HIST];
    uint32_t  onc_n;
    int       oncoming;         // 1: oncoming vehicle (no arrow), kept for the track's life
    uint64_t  pred_us[MM_PRED_HIST];    // arrow lean input (MM_PRED_*), newest at pred_n - 1:
    float     pred_u[MM_PRED_HIST], pred_v[MM_PRED_HIST];   // box bottom (px, pitch compensated)
    float     pred_lane[MM_PRED_HIST][3];                    // lane model c0, h, c at that frame
    uint32_t  pred_n;
    int       pred_st;          // -1 / 0 / +1: leaning left / straight / right
    int       pred_cand;        // direction waiting for MM_PRED_HOLD_S
    uint64_t  pred_cand_us;
    int       pred_ok;          // pred_vel measured
    float     pred_vel;         // lateral speed relative to the lane (m/s)
    float     pred_lean;        // drawn lean (m)
} stMmTrack;

typedef struct {
    stNcMinimapObj in;
    int    ok;                  // road position available
    int    cls;
    float  xm, zm;              // road position of the drawn box bottom-centre (pitch compensated)
    int    lat_ok;              // lat_dx measured
    float  lat_dx;              // x correction to the image place between the ego lines (m)
    float  u, v_raw, v_used;    // box bottom-centre (px): raw / after pitch compensation
    int    fut_ok;              // predicted road position available
    float  fxm, fzm;            // road position of the predicted bottom-centre (pitch compensated)
    stNcLdwGeom geom;           // geometry used for the road position
    float  vp_used;
    uint64_t geom_t_us;
    int    track;               // index into tracks, -1 = none
    int    pos_upd;             // 1: this box updated the track position this frame
} stMmObs;

typedef struct {
    uint64_t  t_us;             // lane frame of the last update
    uint64_t  fit_us;           // lane frame of the last fit (0 = none)
    float     k_hold, vp_hold;  // last good frame geometry
    uint64_t  hold_us;          // its lane frame (0 = none)
    float     c0, h;            // offset, heading of the centre curve
    int       c_init;
    float     c_filt, c_shown;  // measured / drawn curvature
    int       curve_on;
    float     zmin, zmax;       // range of the real points
    int       cls[MM_LANE_LINES];
    uint64_t  seen_us[MM_LANE_LINES];
    float     yel_frac[MM_LANE_LINES];  // yellow share at the last frame (-1: not measured)
    uint64_t  yel_us[MM_LANE_LINES];    // lane frame of the last yellow measurement (0 = none)
    uint64_t  white_us[MM_LANE_LINES];  // start of the current non-yellow run (0 = none)
} stMmLaneModel;

typedef struct {
    uint64_t  t_disp_us;
    int       obs_cnt;
    stMmObs   obs[MM_MAX_OBS];
    uint32_t  next_id;
    stMmTrack track[MM_MAX_TRACK];
    uint64_t  prev_disp_us;     // previous displayed frame
    float     pitch_px;         // pitch offset of the box bottoms (px, + = down)
    uint64_t  pitch_us;         // displayed frame of pitch_px
    uint64_t  lane_log_us;      // lane frame already written to bev_lanepts.csv
    stMmLaneModel lane;
} stMmChannel;

static stMmChannel  g_mm[NC_LDW_MAX_CH];    // render thread only
static stNcLdwLanes g_mm_lanes;             // scratch for drawing (render thread only)

/* ------------------------------------------------------------------------- */
/* classes                                                                    */

static int mm_class_of(const char *name)
{
    if (name == NULL) return MM_CLS_CAR;
    if (!strcasecmp(name, "bus")) return MM_CLS_BUS;
    if (!strcasecmp(name, "tru") || !strcasecmp(name, "truck")) return MM_CLS_TRUCK;
    if (!strcasecmp(name, "mot") || !strcasecmp(name, "motorcycle") || !strcasecmp(name, "motorbike") ||
        !strcasecmp(name, "bik") || !strcasecmp(name, "bike") || !strcasecmp(name, "bicycle") ||
        !strcasecmp(name, "cycle"))
        return MM_CLS_MOTO;
    return MM_CLS_CAR;
}

/* one vote per detection frame (the same detections are reported every displayed frame) */
static void mm_vote(stMmTrack *t, const stMmObs *o, float weight)
{
    uint64_t *last = weight >= 1.0f ? &t->last_det_us : &t->last_dup_det_us;
    if (o->in.t_det_us == *last) return;
    *last = o->in.t_det_us;
    for (int i = 0; i < MM_CLS_N; i++) t->cls_score[i] *= MM_CLS_DECAY;
    t->cls_score[o->cls] += weight;
    if (o->cls != t->cls && t->cls_score[o->cls] > t->cls_score[t->cls] + MM_CLS_SWITCH) t->cls = o->cls;
}

static void mm_take_color(stMmTrack *t, const stMmObs *o)
{
    if (o->in.highlight) return;                 // cut-in colour is not the class colour
    memcpy(t->color[o->cls], o->in.color, sizeof(t->color[0]));
    t->has_color[o->cls] = 1;
}

/* ------------------------------------------------------------------------- */
/* tracks                                                                     */

static stMmTrack *mm_find_alias(stMmChannel *c, const char *cls_name, int track_id)
{
    if (track_id < 0) return NULL;
    for (int i = 0; i < MM_MAX_TRACK; i++) {
        stMmTrack *t = &c->track[i];
        if (!t->used) continue;
        for (int a = 0; a < MM_MAX_ALIAS; a++)
            if (t->alias[a].class_name == cls_name && t->alias[a].track_id == track_id) return t;
    }
    return NULL;
}

static void mm_add_alias(stMmChannel *c, stMmTrack *t, const char *cls_name, int track_id)
{
    if (track_id < 0) return;
    for (int i = 0; i < MM_MAX_TRACK; i++) {            // a key belongs to one track only
        stMmTrack *o = &c->track[i];
        if (!o->used || o == t) continue;
        for (int a = 0; a < MM_MAX_ALIAS; a++)
            if (o->alias[a].class_name == cls_name && o->alias[a].track_id == track_id)
                o->alias[a].track_id = -1;
    }
    for (int a = 0; a < MM_MAX_ALIAS; a++)
        if (t->alias[a].class_name == cls_name && t->alias[a].track_id == track_id) return;
    t->alias[t->alias_next].class_name = cls_name;
    t->alias[t->alias_next].track_id = track_id;
    t->alias_next = (t->alias_next + 1) % MM_MAX_ALIAS;
}

static stMmTrack *mm_new_track(stMmChannel *c, const stMmObs *o)
{
    stMmTrack *t = NULL;
    for (int i = 0; i < MM_MAX_TRACK && t == NULL; i++)
        if (!c->track[i].used) t = &c->track[i];
    if (t == NULL) {                                  // full: reuse the longest unseen one
        t = &c->track[0];
        for (int i = 1; i < MM_MAX_TRACK; i++)
            if (c->track[i].last_seen_disp_us < t->last_seen_disp_us) t = &c->track[i];
    }
    memset(t, 0, sizeof(*t));
    t->used = 1;
    t->id = ++c->next_id;
    for (int a = 0; a < MM_MAX_ALIAS; a++) t->alias[a].track_id = -1;
    t->cls = o->cls;
    return t;
}

static int mm_at_side(const stBBox *b, float margin)
{
    return b->x <= margin || b->x + b->w >= (float)VIEWPORT_WIDTH - margin;
}

/* Start coasting if t is a vehicle passing through the image side (see MM_COAST_*) */
static void mm_coast_try(stMmTrack *t, uint64_t now)
{
    float sa = 0.0f, sz = 0.0f, saa = 0.0f, saz = 0.0f, xs[MM_COAST_HIST];
    int nv = 0, nx = 0;
    uint32_t cnt = t->hist_n < MM_COAST_HIST ? t->hist_n : MM_COAST_HIST;

    for (uint32_t i = 0; i < cnt; i++) {
        uint32_t k = (t->hist_n - 1 - i) % MM_COAST_HIST;
        float age = (float)(now - t->hist_us[k]) * 1e-6f;
        if (age > MM_COAST_V_NEW_S && age <= MM_COAST_V_OLD_S) {
            sa += age; sz += t->hist_z[k]; saa += age * age; saz += age * t->hist_z[k];
            nv++;
        }
        if (age > MM_COAST_X_NEW_S && age <= MM_COAST_X_OLD_S) {
            int j = nx++;                                               // insertion sort
            while (j > 0 && xs[j - 1] > t->hist_x[k]) { xs[j] = xs[j - 1]; j--; }
            xs[j] = t->hist_x[k];
        }
    }
    if (nv < MM_COAST_V_MIN_PTS) return;
    float den = (float)nv * saa - sa * sa;
    if (den <= 0.0f) return;
    float v = -((float)nv * saz - sa * sz) / den;                       // age runs backwards in time
    float xh = nx > 0 ? xs[nx / 2] : t->x;
    if (v >= -MM_COAST_V_MIN || v <= -MM_COAST_V_MAX || fabsf(xh - t->x) > MM_COAST_X_MAX_M) return;

    t->coast = MM_COAST_RUN;
    t->coast_us = now;
    t->coast_x0 = t->x;
    t->coast_z0 = t->z;
    t->coast_xh = xh;
    t->coast_v = v;
    t->coast_a = 1.0f;
    t->fut_n = 0;
}

/* EMA per displayed frame, as in the first minimap. Returns 1 if o updated the position. */
static int mm_smooth(stMmTrack *t, const stMmObs *o, uint64_t now)
{
    if (t->n_upd > 0 && t->last_upd_disp_us == now) return 0;        // already updated this frame
    int restart = (t->n_upd == 0 || now < t->last_upd_disp_us || now - t->last_upd_disp_us > MM_TRACK_GAP_US);

    // back inside the image: the measurement is good again
    if (t->coast != MM_COAST_NONE && !mm_at_side(&o->in.meas, MM_COAST_CANCEL_PX) &&
        o->in.meas.h < MM_COAST_CANCEL_H_PX) {
        t->coast = MM_COAST_NONE;
        restart = 1;
    }

    // lateral correction to the image place between the lines (mm_lateral): slow, and held
    // while it is not measured, so that line drop-outs do not shake the box
    if (o->lat_ok) {
        if (restart || t->lat_us == 0 || now < t->lat_us || now - t->lat_us > MM_TRACK_GAP_US) {
            t->lat_dx = o->lat_dx;
        } else {
            float dt = (float)(now - t->lat_us) * 1e-6f;
            t->lat_dx += (1.0f - expf(-dt / MM_LAT_TAU_S)) * (o->lat_dx - t->lat_dx);
        }
        t->lat_us = now;
    } else if (restart) {
        t->lat_dx = 0.0f;
        t->lat_us = 0;
    }
    const float xm = o->xm + t->lat_dx;

    // predicted offset: same EMA as the position; dropped when this frame has none
    if (!o->fut_ok || t->coast != MM_COAST_NONE) {
        t->fut_n = 0;
    } else {
        float dx = o->fxm - o->xm, dz = o->fzm - o->zm;
        if (restart || t->fut_n == 0) { t->fut_dx = dx; t->fut_dz = dz; t->fut_n = 1; }
        else {
            t->fut_dx += MM_SMOOTH_ALPHA * (dx - t->fut_dx);
            t->fut_dz += MM_SMOOTH_ALPHA * (dz - t->fut_dz);
            t->fut_n++;
        }
    }

    if (t->coast == MM_COAST_NONE) {
        if (restart) {
            t->x = xm; t->z = o->zm;
            t->n_upd = 1;
        } else {
            float dz = MM_SMOOTH_ALPHA * (o->zm - t->z);
            float w = 1.0f;   // [11.09 서영] 이번 프레임을 믿는 정도 (1 = 그대로)
            if (t->z > MM_FAR_Z_M) {
                float lim = fmaxf(MM_FAR_DZ_MIN, MM_FAR_DZ_FRAC * t->z) * (float)(now - t->last_upd_disp_us) * 1e-6f;
                if (fabsf(dz) > lim) w = lim / fabsf(dz);
            }
            t->x += w * MM_SMOOTH_ALPHA * (xm - t->x);
            t->z += w * dz;
            t->n_upd++;
        }
        if (restart) t->onc_n = 0;
        uint32_t k = t->onc_n++ % MM_ONC_HIST;
        t->onc_us[k] = now;
        t->onc_z[k] = t->z;
#if NC_MINIMAP_COAST
        if (t->in_range && o->zm < MM_COAST_MAX_Z_M &&
            (mm_at_side(&o->in.meas, MM_COAST_EDGE_PX) ||
             (o->in.meas.h >= MM_COAST_TALL_PX && fabsf(t->x) >= MM_COAST_TALL_X_M)))
            mm_coast_try(t, now);
#endif
    }
    uint32_t h = t->hist_n++ % MM_COAST_HIST;
    t->hist_us[h] = now;
    t->hist_x[h] = xm;
    t->hist_z[h] = o->zm;
    t->last_upd_disp_us = now;
    t->v_prev = o->v_raw;
    t->v_prev_us = now;
    mm_vote(t, o, 1.0f);
    return 1;
}

/* ------------------------------------------------------------------------- */
/* frame input                                                                */

void nc_minimap_begin_frame(uint32_t ch, uint64_t t_disp_us)
{
    if (ch >= NC_LDW_MAX_CH) return;
    g_mm[ch].obs_cnt = 0;
    g_mm[ch].t_disp_us = t_disp_us;
}

void nc_minimap_add_obj(uint32_t ch, const stNcMinimapObj *in)
{
    if (ch >= NC_LDW_MAX_CH || in == NULL) return;
    stMmChannel *c = &g_mm[ch];
    if (c->obs_cnt >= MM_MAX_OBS) return;

    stMmObs *o = &c->obs[c->obs_cnt++];
    memset(o, 0, sizeof(*o));
    o->in = *in;
    o->cls = mm_class_of(in->class_name);
    o->track = -1;

    // ground contact point of the drawn box with the newest (calibrated) geometry.
    // Road position is computed in mm_process() after the pitch compensation.
    o->u = in->shown.x + in->shown.w * 0.5f;
    o->v_raw = o->v_used = in->shown.y + in->shown.h;
    if (nc_ldw_app_get_geom(ch, 0, &o->geom) >= 0) {
        o->ok = 1;
        o->vp_used = o->geom.vp_y;
        o->geom_t_us = o->geom.t_us;
    }
}

/* 1 if g has a good frame geometry of its own */
static int mm_frame_ok(const stNcLdwGeom *g)
{
    return g->k_inst > 0.0f && fabsf(g->vp_inst - g->vp_y) <= MM_LANE_INST_MAX_PX;
}

/* Geometry for the lanes and the vehicle lateral position: the frame geometry, or the
 * last good one while a line is lost (one line can drop out every other frame, and
 * switching to the calibrated geometry in those frames shakes everything).
 * Returns 0 if there is none: gi is then the calibrated geometry. */
static int mm_frame_geom(uint32_t ch, const stNcLdwGeom *g, stNcLdwGeom *gi)
{
    stMmLaneModel *M = &g_mm[ch].lane;
    *gi = *g;
    if (mm_frame_ok(g)) {
        M->k_hold = g->k_inst;
        M->vp_hold = g->vp_inst;
        M->hold_us = g->t_us;
    }
    // either side: the lane model takes the newest lane frame before the vehicles take theirs
    if (M->hold_us == 0 ||
        (g->t_us > M->hold_us ? g->t_us - M->hold_us : M->hold_us - g->t_us) > MM_LANE_GEOM_HOLD_US) return 0;
    gi->k = M->k_hold;
    gi->vp_y = M->vp_hold;
    return 1;
}

static void mm_log_lanes(uint32_t ch, const stNcLdwLanes *L, const stNcLdwGeom *g);
static void mm_lane_update(uint32_t ch, const stNcLdwLanes *L, const stNcLdwGeom *g);

/* u (px) of a network lane line on image row v: straight between the points around v,
 * the top / bottom segment extended outside the points. Returns -1 with fewer than 2 points. */
static int mm_lane_u_at(const stLaneDet *ln, float v, float *u)
{
    float pu[MAX_POINT_CNT_OF_LANE], pv[MAX_POINT_CNT_OF_LANE];
    int n = ln->point_cnt, m = 0;
    if (n > MAX_POINT_CNT_OF_LANE) n = MAX_POINT_CNT_OF_LANE;
    for (int j = 0; j < n; j++) {                    // valid points, sorted by row
        float x = ln->point[j].x, y = ln->point[j].y;
        if (!isfinite(x) || !isfinite(y) || (x <= 0.0f && y <= 0.0f)) continue;
        int k = m++;
        while (k > 0 && pv[k - 1] > y) { pu[k] = pu[k - 1]; pv[k] = pv[k - 1]; k--; }
        pu[k] = x; pv[k] = y;
    }
    if (m < 2) return -1;
    int i = 0;                                       // segment i .. i + 1
    while (i < m - 2 && pv[i + 1] < v) i++;
    float dv = pv[i + 1] - pv[i];
    if (dv < 1.0f) return -1;
    *u = pu[i] + (pu[i + 1] - pu[i]) * (v - pv[i]) / dv;
    return 0;
}

/* Lateral position with the geometry of the lanes at row v (pitch compensated): the vehicle
 * keeps its place between the lines. Distance stays on the calibrated geometry.
 *
 * dx (optional): correction to the place of the vehicle between the two ego lines in the
 * image, on the row of the box bottom without pitch compensation (as the lane points), put
 * on the drawn centre curve at distance z; above the top lane point the top segment is
 * extended. Returns 1 if dx was measured. The geometry alone pulls far vehicles towards the
 * centre: k, vp come from straight fits of the near rows while the lines converge faster
 * farther up, and the drawn centre is straight on gentle curves (0001: about 0.16 lane in
 * the next lanes beyond the points). The image place is noisier than the geometry and
 * missing whenever a line drops out, so the track applies dx slowly (mm_smooth). */
static int mm_lateral(uint32_t ch, const stNcLdwGeom *g, float u, float v_img, float v, float z,
                      float *x_m, float *dx)
{
    stNcLdwGeom gi;
    float x;
    if (mm_frame_geom(ch, g, &gi) && nc_ldw_geom_to_road(ch, &gi, u, v, &x, NULL) == 0) *x_m = x;
    if (dx == NULL) return 0;

    const stMmLaneModel *M = &g_mm[ch].lane;
    const stNcLdwLanes *L = &g_mm_lanes;
    const int li = (int)NC_LDW_EGO_LANE.left_boundary_index, ri = (int)NC_LDW_EGO_LANE.right_boundary_index;
    float ul, ur;
    if (M->fit_us == 0 || L->t_us < M->fit_us || L->t_us - M->fit_us > MM_LANE_HOLD_US) return 0;
    if ((g->t_us > L->t_us ? g->t_us - L->t_us : L->t_us - g->t_us) > MM_LANE_HOLD_US) return 0;
    if (L->lane_num <= li || L->lane_num <= ri) return 0;
    if (mm_lane_u_at(&L->lane[li], v_img, &ul) != 0 || mm_lane_u_at(&L->lane[ri], v_img, &ur) != 0) return 0;
    if (ur - ul <= MM_LAT_MIN_W_PX) return 0;
    float pos = (u - 0.5f * (ul + ur)) / (ur - ul);
    *dx = M->c0 + M->h * z + M->c_shown * z * z + pos * NC_LDW_DEFAULT_LANE_WIDTH_M - *x_m;
    return 1;
}

#if MM_SIDE_CORR
/* Move o->u from the box centre to the vehicle centre beside the ego lane (see MM_SIDE_*)
 * and recompute o->xm. The image place between the lines (mm_lateral) then uses it too. */
static void mm_side_corr(uint32_t ch, stMmObs *o)
{
    const float ax = fabsf(o->xm);
    if (ax <= MM_SIDE_X0_M) return;
    const stBBox *b = &o->in.shown;
    const int right = o->xm > 0.0f;
    const float ue = right ? b->x + b->w : b->x;                 // outer edge
    if (right ? ue >= (float)VIEWPORT_WIDTH - MM_SIDE_EDGE_PX : ue <= MM_SIDE_EDGE_PX) return;
    float xe;
    if (fabsf(ue - o->u) < 1.0f || nc_ldw_geom_to_road(ch, &o->geom, ue, o->v_used, &xe, NULL) != 0) return;
    const float m_per_px = (xe - o->xm) / (ue - o->u);
    if (!(m_per_px > 0.0f)) return;
    const float half = 0.5f * k_cls_w[o->cls];
    float shift = right ? (xe - half) - o->xm : (xe + half) - o->xm;
    if (right ? shift <= 0.0f : shift >= 0.0f) return;   // only outwards (box narrower than the class)
    float a = (ax - MM_SIDE_X0_M) / (MM_SIDE_X1_M - MM_SIDE_X0_M);
    if (a > 1.0f) a = 1.0f;
    shift *= a;
    o->u += shift / m_per_px;
    o->xm += shift;
}
#endif

/* pitch offset from the common bottom move of the known vehicles, then the road
 * position of every detection with bottom - offset */
static void mm_pitch_and_road(uint32_t ch)
{
    stMmChannel *c = &g_mm[ch];
    const uint64_t now = c->t_disp_us;

    // newest lane frame and lane model first: the lateral position uses both
    {
        stNcLdwGeom gl;
        if (nc_ldw_app_get_lanes(ch, &g_mm_lanes) != 0) g_mm_lanes.t_us = 0;
        else if (nc_ldw_app_get_geom(ch, 0, &gl) >= 0) {
            mm_log_lanes(ch, &g_mm_lanes, &gl);
            mm_lane_update(ch, &g_mm_lanes, &gl);
        }
    }

#if NC_MINIMAP_PITCH_COMP
    if (now != c->pitch_us) {
        float d[MM_MAX_OBS];
        int nd = 0;
        for (int n = 0; n < c->obs_cnt; n++) {
            const stMmObs *o = &c->obs[n];
            if (!o->ok) continue;
            const stMmTrack *t = mm_find_alias(c, o->in.class_name, o->in.track_id);
            if (t == NULL || t->v_prev_us == 0 || t->v_prev_us != c->prev_disp_us) continue;
            float dv = o->v_raw - t->v_prev;
            if (fabsf(dv) > NC_MINIMAP_PITCH_OUTLIER_PX) continue;      // re-association / jump
            int k = nd++;                                              // insertion sort
            while (k > 0 && d[k - 1] > dv) { d[k] = d[k - 1]; k--; }
            d[k] = dv;
        }
        if (c->pitch_us == 0 || now < c->pitch_us || now - c->pitch_us > MM_TRACK_GAP_US) {
            c->pitch_px = 0.0f;                                        // long gap: start again
        } else {
            float dt = (float)(now - c->pitch_us) * 1e-6f;
            c->pitch_px *= expf(-dt / NC_MINIMAP_PITCH_TAU_S);
            if (nd >= NC_MINIMAP_PITCH_MIN_VEH) {
                float med = (nd % 2) ? d[nd / 2] : 0.5f * (d[nd / 2 - 1] + d[nd / 2]);
                // pitch moves every bottom by the same amount: add it only when at least
                // half of the vehicles (and 2) agree with the median, so that one car
                // closing in fast is not taken for pitch (0002 t 5.7 s, 2 vehicles)
                float tol = fmaxf(NC_MINIMAP_PITCH_AGREE_PX, NC_MINIMAP_PITCH_AGREE_FRAC * fabsf(med));
                int agree = 0;
                for (int k = 0; k < nd; k++) if (fabsf(d[k] - med) <= tol) agree++;
                if (agree >= 2 && 2 * agree >= nd) c->pitch_px += med;
            }
            c->pitch_px = fmaxf(-NC_MINIMAP_PITCH_MAX_PX, fminf(NC_MINIMAP_PITCH_MAX_PX, c->pitch_px));
        }
        c->pitch_us = now;
    }
#endif

    for (int n = 0; n < c->obs_cnt; n++) {
        stMmObs *o = &c->obs[n];
        if (!o->ok) continue;
#if NC_MINIMAP_PITCH_COMP
        o->v_used = o->v_raw - c->pitch_px;
#endif
        if (nc_ldw_geom_to_road(ch, &o->geom, o->u, o->v_used, &o->xm, &o->zm) != 0 || o->zm <= 0.0f) {
            o->ok = 0;
            continue;
        }
#if MM_SIDE_CORR
        mm_side_corr(ch, o);
#endif
        o->lat_ok = mm_lateral(ch, &o->geom, o->u, o->v_raw, o->v_used, o->zm, &o->xm, &o->lat_dx);
#if NC_MINIMAP_SHOW_PRED
        // predicted point: same geometry and pitch offset as the current point
        if (o->in.fut_valid) {
            float fv = o->in.fut_v - (o->v_raw - o->v_used);
            if (nc_ldw_geom_to_road(ch, &o->geom, o->in.fut_u, fv, &o->fxm, &o->fzm) == 0 && o->fzm > 0.0f) {
                mm_lateral(ch, &o->geom, o->in.fut_u, o->in.fut_v, fv, o->fzm, &o->fxm, NULL);
                o->fut_ok = 1;
            }
        }
#endif
    }
}

#if NC_MINIMAP_SHOW_PRED && NC_MINIMAP_PRED_LANE_REL
/* Arrow lean of track t from its detection o of this frame (see MM_PRED_*) */
static void mm_pred_update(uint32_t ch, stMmTrack *t, const stMmObs *o, uint64_t now)
{
    const stMmLaneModel *M = &g_mm[ch].lane;
    if (t->pred_n > 0) {
        uint64_t last = t->pred_us[(t->pred_n - 1) % MM_PRED_HIST];
        if (last == now) return;                                    // already added this frame
        if (now < last || now - last > MM_TRACK_GAP_US) { t->pred_n = 0; t->pred_st = 0; t->pred_cand = 0; }
    }
    uint32_t k = t->pred_n++ % MM_PRED_HIST;
    t->pred_us[k] = now;
    t->pred_u[k] = o->u;
    t->pred_v[k] = o->v_used;
    t->pred_lane[k][0] = M->fit_us ? M->c0 : 0.0f;
    t->pred_lane[k][1] = M->fit_us ? M->h : 0.0f;
    t->pred_lane[k][2] = M->fit_us ? M->c_shown : 0.0f;

    // lane-relative x of every sample with the geometry of this frame
    stNcLdwGeom gi;
    mm_frame_geom(ch, &o->geom, &gi);                               // calibrated one if none
    const float *ln = t->pred_lane[k];
    double sa = 0.0, sq = 0.0, saa = 0.0, saq = 0.0;
    float oldest = 0.0f;
    int n = 0;
    uint32_t cnt = t->pred_n < MM_PRED_HIST ? t->pred_n : MM_PRED_HIST;
    for (uint32_t i = 0; i < cnt; i++) {
        uint32_t j = (t->pred_n - 1 - i) % MM_PRED_HIST;
        float age = (float)(now - t->pred_us[j]) * 1e-6f;
        if (age > MM_PRED_WIN_S) break;
        float x, z;
        if (nc_ldw_geom_to_road(ch, &gi, t->pred_u[j], t->pred_v[j], &x, &z) != 0 || z <= 0.0f) continue;
        const float *lj = t->pred_lane[j];
        float q = x - ((lj[0] + lj[1] * z + lj[2] * z * z) - (ln[0] + ln[1] * z + ln[2] * z * z));
        sa += -age; sq += q; saa += (double)age * age; saq += -(double)age * q;
        oldest = age;
        n++;
    }
    double d = saa - sa * sa / (double)n;
    t->pred_ok = (n >= MM_PRED_MIN_PTS && oldest >= MM_PRED_SPAN_S && d > 0.0);
    if (!t->pred_ok) {
        t->pred_st = 0; t->pred_cand = 0; t->pred_lean = 0.0f;
        return;
    }
    const float v = (float)((saq - sa * sq / (double)n) / d);
    t->pred_vel = v;

    // dead band with hysteresis: lean after V_ON held for HOLD_S, straight below V_OFF
    int want = v > MM_PRED_V_ON ? 1 : (v < -MM_PRED_V_ON ? -1 : 0);
    if (t->pred_st != 0 && v * (float)t->pred_st < MM_PRED_V_OFF) t->pred_st = 0;
    if (t->pred_st == 0 && want != 0) {
        if (t->pred_cand != want) { t->pred_cand = want; t->pred_cand_us = now; }
        if (now - t->pred_cand_us >= (uint64_t)(MM_PRED_HOLD_S * 1e6f)) t->pred_st = want;
    } else if (want == 0) {
        t->pred_cand = 0;
    }
    t->pred_lean = t->pred_st != 0 ? fmaxf(-MM_PRED_LEAN_MAX_M, fminf(MM_PRED_LEAN_MAX_M, v * MM_PRED_HZ_S)) : 0.0f;
    if ((t->cls == MM_CLS_BUS || t->cls == MM_CLS_TRUCK) && o->zm < MM_PRED_BIG_Z_M) t->pred_lean = 0.0f;
}
#endif

/* move the coasting vehicles of this frame */
static void mm_coast_step(stMmChannel *c)
{
    const uint64_t now = c->t_disp_us;

    for (int i = 0; i < MM_MAX_TRACK; i++) {
        stMmTrack *t = &c->track[i];
        if (!t->used || t->coast != MM_COAST_RUN) continue;

        float d = now >= t->coast_us ? (float)(now - t->coast_us) * 1e-6f : MM_COAST_MAX_S + 1.0f;
        float z = t->coast_z0 + t->coast_v * d;
        if (d > MM_COAST_MAX_S || z < -MM_BEHIND_M) {
            t->coast = MM_COAST_DONE;           // stays hidden while the box is still at the edge
            continue;
        }
        float blend = fminf(1.0f, d / MM_COAST_X_BLEND_S);
        t->x = t->coast_x0 + (t->coast_xh - t->coast_x0) * blend;
        t->z = z;
        t->coast_a = fminf(1.0f, fminf((z + MM_BEHIND_M) / MM_COAST_FADE_Z_M,
                                       (MM_COAST_MAX_S - d) / MM_COAST_FADE_S));
    }
}

/* match this frame's detections to minimap tracks and update them */
static void mm_process(uint32_t ch)
{
    stMmChannel *c = &g_mm[ch];
    const uint64_t now = c->t_disp_us;

    for (int i = 0; i < MM_MAX_TRACK; i++) {
        stMmTrack *t = &c->track[i];
        if (!t->used) continue;
        t->highlight = 0;
        if (t->coast != MM_COAST_RUN && now > t->last_seen_disp_us && now - t->last_seen_disp_us > MM_DROP_US)
            t->used = 0;
    }

    mm_pitch_and_road(ch);

    // 1. known (class, tracker id)
    for (int n = 0; n < c->obs_cnt; n++) {
        stMmObs *o = &c->obs[n];
        if (!o->ok) continue;
        stMmTrack *t = mm_find_alias(c, o->in.class_name, o->in.track_id);
        if (t == NULL) continue;
        o->track = (int)(t - c->track);
        o->pos_upd = mm_smooth(t, o, now);
    }

    // 2. new key: same vehicle with another class / tracker id, matched by position
    for (int n = 0; n < c->obs_cnt; n++) {
        stMmObs *o = &c->obs[n];
        if (!o->ok || o->track >= 0) continue;

        int best = -1, dup = 0;
        float best_cost = 1.0f;
        for (int i = 0; i < MM_MAX_TRACK; i++) {
            stMmTrack *t = &c->track[i];
            if (!t->used || t->n_upd == 0) continue;
            int is_dup = (t->last_upd_disp_us == now);   // already updated by another box of this frame
            if (!is_dup && (now < t->last_upd_disp_us || now - t->last_upd_disp_us > MM_TRACK_GAP_US)) continue;
            float gz = is_dup ? fmaxf(MM_DUP_Z_MIN, MM_DUP_Z_RATIO * o->zm) : fmaxf(MM_GATE_Z_MIN, MM_GATE_Z_RATIO * o->zm);
            float gx = is_dup ? MM_DUP_X : MM_GATE_X;
            float dz = o->zm - t->z, dx = o->xm + t->lat_dx - t->x;
            float cost = (dz / gz) * (dz / gz) + (dx / gx) * (dx / gx);
            if (cost < best_cost) { best_cost = cost; best = i; dup = is_dup; }
        }
        if (best < 0) continue;

        stMmTrack *t = &c->track[best];
        mm_add_alias(c, t, o->in.class_name, o->in.track_id);
        o->track = best;
        if (dup) mm_vote(t, o, 0.5f);                // two classes for one vehicle: half a vote
        else     o->pos_upd = mm_smooth(t, o, now);
    }

    // 3. new vehicles
    for (int n = 0; n < c->obs_cnt; n++) {
        stMmObs *o = &c->obs[n];
        if (!o->ok || o->track >= 0) continue;
        stMmTrack *t = mm_new_track(c, o);
        mm_add_alias(c, t, o->in.class_name, o->in.track_id);
        o->track = (int)(t - c->track);
        o->pos_upd = mm_smooth(t, o, now);
    }

    // seen / colour / cut-in / arrow lean
    for (int n = 0; n < c->obs_cnt; n++) {
        stMmObs *o = &c->obs[n];
        if (o->track < 0) continue;
        stMmTrack *t = &c->track[o->track];
        t->last_seen_disp_us = now;
        t->highlight |= o->in.highlight;
        mm_take_color(t, o);
#if NC_MINIMAP_SHOW_PRED && NC_MINIMAP_PRED_LANE_REL
        if (o->pos_upd) mm_pred_update(ch, t, o, now);   // not a second-class duplicate
#endif
    }

    mm_coast_step(c);

    // range hysteresis
    for (int i = 0; i < MM_MAX_TRACK; i++) {
        stMmTrack *t = &c->track[i];
        if (!t->used) continue;
        if (!t->in_range && t->z <= NC_MINIMAP_RANGE_M - NC_MINIMAP_RANGE_HYST_M) t->in_range = 1;
        else if (t->in_range && t->z > NC_MINIMAP_RANGE_M) t->in_range = 0;
    }
    c->prev_disp_us = now;
}

#if NC_MINIMAP_SHOW_PRED
/* z speed of the smoothed position over the last MM_ONC_WIN_S (least squares).
 * Returns 0 and *v when there are enough samples (see MM_ONC_*). */
static int mm_onc_speed(const stMmTrack *t, float *v)
{
    uint32_t cnt = t->onc_n < MM_ONC_HIST ? t->onc_n : MM_ONC_HIST;
    if (cnt < MM_ONC_MIN_PTS) return -1;
    const uint64_t t0 = t->onc_us[(t->onc_n - 1) % MM_ONC_HIST];
    float sa = 0.0f, sz = 0.0f, saa = 0.0f, saz = 0.0f, oldest = 0.0f;
    int n = 0;
    for (uint32_t i = 0; i < cnt; i++) {
        uint32_t k = (t->onc_n - 1 - i) % MM_ONC_HIST;
        float age = t0 >= t->onc_us[k] ? (float)(t0 - t->onc_us[k]) * 1e-6f : MM_ONC_WIN_S + 1.0f;
        if (age > MM_ONC_WIN_S) break;
        sa += -age; sz += t->onc_z[k]; saa += age * age; saz += -age * t->onc_z[k];
        oldest = age;
        n++;
    }
    if (n < MM_ONC_MIN_PTS || oldest < MM_ONC_SPAN_S) return -1;
    float d = saa - sa * sa / (float)n;
    if (d <= 0.0f) return -1;
    *v = (saz - sa * sz / (float)n) / d;
    return 0;
}
#endif

static int mm_track_drawn(const stMmChannel *c, const stMmTrack *t)
{
    if (!t->used || !t->in_range || t->coast == MM_COAST_DONE) return 0;
    if (t->coast == MM_COAST_RUN) return 1;
    return c->t_disp_us >= t->last_seen_disp_us && c->t_disp_us - t->last_seen_disp_us <= MM_HOLD_US;
}

/* diagnostics (NC_BEV_LOG). Columns: see nc_bevlog.c */
static void mm_log(uint32_t ch)
{
    if (!nc_bevlog_enabled()) return;
    stMmChannel *c = &g_mm[ch];
    float k = 0.0f, vp = 0.0f;
    nc_ldw_app_get_calib(ch, &k, &vp, NULL);

    static const char *const cls_name[MM_CLS_N] = { "car", "truck", "bus", "moto" };
    for (int n = 0; n < c->obs_cnt; n++) {
        const stMmObs *o = &c->obs[n];
        const stMmTrack *t = o->track >= 0 ? &c->track[o->track] : NULL;
        nc_bevlog_row(NC_BEVLOG_OBJ,
                      "%llu,%llu,%llu,%u,%d,%s,%.1f,%.1f,%.1f,%.1f,%d,%.1f,%.1f,%.1f,%.1f,"
                      "%.1f,%.1f,%.2f,%.5f,%.3f,%.3f,%.3f,%.3f,%d,"
                      "%.2f,%d,%u,%s,%d,%d,%.3f,%.3f",
                      (unsigned long long)o->in.t_disp_us, (unsigned long long)o->in.t_det_us,
                      (unsigned long long)o->geom_t_us, ch, o->in.track_id,
                      o->in.class_name ? o->in.class_name : "?",
                      (double)o->in.meas.x, (double)o->in.meas.y, (double)o->in.meas.w, (double)o->in.meas.h,
                      o->in.pred_valid,
                      (double)o->in.shown.x, (double)o->in.shown.y, (double)o->in.shown.w, (double)o->in.shown.h,
                      (double)o->u, (double)o->v_used,          // v_used: after pitch compensation
                      (double)vp, (double)k,
                      (double)o->xm, (double)(o->ok ? o->zm : 0.0f),
                      (double)(t ? t->x : 0.0f), (double)(t ? t->z : 0.0f),
                      t ? mm_track_drawn(c, t) : 0,
                      (double)o->vp_used, (o->geom_t_us == o->in.t_det_us) ? 1 : 0, t ? t->id : 0u,
                      t ? cls_name[t->cls] : "-", t ? t->oncoming : 0,
                      t ? t->pred_ok : 0, (double)(t ? t->pred_vel : 0.0f), (double)(t ? t->pred_lean : 0.0f));
    }
    for (int i = 0; i < MM_MAX_TRACK; i++) {
        const stMmTrack *t = &c->track[i];
        if (!t->used || t->coast != MM_COAST_RUN) continue;
        nc_bevlog_row(NC_BEVLOG_COAST, "%llu,%llu,%u,%u,%.3f,%.3f,%.3f,%.3f",
                      (unsigned long long)c->t_disp_us, (unsigned long long)t->coast_us, ch, t->id,
                      (double)t->x, (double)t->z, (double)t->coast_v, (double)t->coast_a);
    }
}

/* ------------------------------------------------------------------------- */
/* drawing helpers                                                            */

static const float k_px = NC_MINIMAP_PANEL_X;
static float k_py = NC_MINIMAP_PANEL_Y;   // [merge 11.06] const 제거: HUD 줄 수에 맞춰 바뀜
void nc_minimap_set_panel_y(float y) { k_py = y; }
static const float k_pw = NC_MINIMAP_PANEL_W;
static const float k_ph = NC_MINIMAP_PANEL_H;

static float mm_sx(void) { return k_pw / (2.0f * NC_MINIMAP_LAT_RANGE_M); }
static float mm_sz(void) { return k_ph / (NC_MINIMAP_RANGE_M + MM_BEHIND_M); }

static float mm_px(float x_m) { return k_px + k_pw * 0.5f + x_m * mm_sx(); }
static float mm_py(float z_m) { return k_py + k_ph - (z_m + MM_BEHIND_M) * mm_sz(); }

/* filled quad: same shader / primitive as the first minimap background */
static void mm_fill_rect(float x, float y, float w, float h, const float color[4],
                         const struct gl_npu_program *p)
{
    float v[12];
    nc_opengl_create_rect_vertex(x, y, w, h, v);     // LT, LB, RB, RT -> fan

    glUseProgram(p->program);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glVertexAttribPointer(p->pos_handle, 3, GL_FLOAT, GL_FALSE, 0, v);
    glEnableVertexAttribArray(p->pos_handle);
    glUniform1i((GLint)p->drawing_type_handle, DRAW_NPU_TYPE_SOLID);
    glUniform4fv((GLint)p->color_handle, 1, color);
    glDrawArrays(GL_TRIANGLE_FAN, 0, 4);
    glDisable(GL_BLEND);
    glDisableVertexAttribArray(p->pos_handle);
    glUseProgram(0);
}

static void mm_rect(float x, float y, float w, float h, const float color[4],
                    const struct gl_npu_program *p)
{
    float c[4] = {color[0], color[1], color[2], color[3]};
    nc_opengl_draw_rectangle(x, y, w, h, c, *p);
}

/* road rectangle (x centre, z from z0 to z1) clipped to the panel */
static void mm_road_rect(float xc, float w_m, float z0, float z1, const float color[4], int fill,
                         const struct gl_npu_program *p)
{
    float left = mm_px(xc - w_m * 0.5f), right = mm_px(xc + w_m * 0.5f);
    float top  = mm_py(z1),              bot   = mm_py(z0);

    if (left < k_px) left = k_px;
    if (right > k_px + k_pw) right = k_px + k_pw;
    if (top < k_py) top = k_py;
    if (bot > k_py + k_ph) bot = k_py + k_ph;
    if (right - left < 1.0f || bot - top < 1.0f) return;

    if (fill) mm_fill_rect(left, top, right - left, bot - top, color, p);
    else      mm_rect(left, top, right - left, bot - top, color, p);
}

/* filled convex or star-shaped polygon (triangle fan), canvas px, at most MM_POLY_MAX points */
#define MM_POLY_MAX (10)
static void mm_fill_poly(const float (*xy)[2], int n, const float color[4],
                         const struct gl_npu_program *p)
{
    float v[MM_POLY_MAX * 3];
    for (int i = 0; i < n; i++) {
        nc_opengl_pixel_to_ndc(xy[i][0], xy[i][1], &v[i * 3], &v[i * 3 + 1]);
        v[i * 3 + 2] = 0.0f;
    }

    glUseProgram(p->program);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glVertexAttribPointer(p->pos_handle, 3, GL_FLOAT, GL_FALSE, 0, v);
    glEnableVertexAttribArray(p->pos_handle);
    glUniform1i((GLint)p->drawing_type_handle, DRAW_NPU_TYPE_SOLID);
    glUniform4fv((GLint)p->color_handle, 1, color);
    glDrawArrays(GL_TRIANGLE_FAN, 0, n);
    glDisable(GL_BLEND);
    glDisableVertexAttribArray(p->pos_handle);
    glUseProgram(0);
}

/* Part t0..t1 of a road segment inside the panel (Liang-Barsky). Returns 0 if none. */
static int mm_clip_seg(float x0, float z0, float x1, float z1, float *t0_out, float *t1_out)
{
    const float lat = NC_MINIMAP_LAT_RANGE_M;
    float t0 = 0.0f, t1 = 1.0f, dx = x1 - x0, dz = z1 - z0;
    const float pq[4][2] = {
        { -dx, x0 + lat }, { dx, lat - x0 },
        { -dz, z0 + MM_BEHIND_M }, { dz, NC_MINIMAP_RANGE_M - z0 },
    };
    for (int i = 0; i < 4; i++) {
        float pp = pq[i][0], q = pq[i][1];
        if (fabsf(pp) < 1e-9f) { if (q < 0.0f) return 0; continue; }
        float r = q / pp;
        if (pp < 0.0f) { if (r > t1) return 0; if (r > t0) t0 = r; }
        else           { if (r < t0) return 0; if (r < t1) t1 = r; }
    }
    *t0_out = t0;
    *t1_out = t1;
    return 1;
}

/* road segment clipped to the panel, drawn with nc_opengl_draw_line */
static void mm_road_seg(float x0, float z0, float x1, float z1, int dashed, const float color[4],
                        const struct gl_npu_program *p)
{
    float t0, t1, dx = x1 - x0, dz = z1 - z0;
    if (!mm_clip_seg(x0, z0, x1, z1, &t0, &t1)) return;
    float c[4] = {color[0], color[1], color[2], color[3]};
    nc_opengl_draw_line(mm_px(x0 + t0 * dx), mm_py(z0 + t0 * dz), mm_px(x0 + t1 * dx), mm_py(z0 + t1 * dz),
                        dashed ? 2 : 1, c, *p);
}

/* Vehicle box on the road, turned to follow the lane: rear edge centred on (x, z), length l
 * along the direction (s, 1), i.e. x changes by s per metre of z. s is the slope of the drawn
 * lane lines over the box, so the box stays parallel to them. Turning about the rear centre
 * keeps the measured point (box bottom-centre in the image) where it is.
 * Corners: rear-left, rear-right, front-right, front-left. *fx, *fz = front edge centre. */
static void mm_box_corners(float x, float z, float w, float l, float s, float q[4][2],
                           float *fx, float *fz)
{
    const float n = 1.0f / sqrtf(1.0f + s * s);
    const float ux = s * n, uz = n;                  // forward
    const float rx = uz * 0.5f * w, rz = -ux * 0.5f * w;   // half width to the right
    q[0][0] = x - rx;           q[0][1] = z - rz;
    q[1][0] = x + rx;           q[1][1] = z + rz;
    q[2][0] = x + rx + ux * l;  q[2][1] = z + rz + uz * l;
    q[3][0] = x - rx + ux * l;  q[3][1] = z - rz + uz * l;
    *fx = x + ux * l;
    *fz = z + uz * l;
}

/* filled road quad clipped to the panel (Sutherland-Hodgman in road metres: the
 * road -> canvas mapping only scales and shifts each axis, so the result is the same) */
static void mm_road_quad(const float q[4][2], const float color[4], const struct gl_npu_program *p)
{
    const float lat = NC_MINIMAP_LAT_RANGE_M;
    // edges: x >= -lat, x <= lat, z >= -behind, z <= range  as  sign * coord <= lim
    const float clip[4][3] = {
        { 0, -1.0f, lat }, { 0, 1.0f, lat }, { 1, -1.0f, MM_BEHIND_M }, { 1, 1.0f, NC_MINIMAP_RANGE_M },
    };
    float a[8][2], b[8][2];
    int n = 4;
    memcpy(a, q, sizeof(float) * 8);
    for (int e = 0; e < 4 && n > 0; e++) {
        const int ax = (int)clip[e][0];
        const float sg = clip[e][1], lim = clip[e][2];
        int m = 0;
        for (int i = 0; i < n; i++) {
            const float *P = a[i], *Q = a[(i + 1) % n];
            float dp = sg * P[ax] - lim, dq = sg * Q[ax] - lim;
            if (dp <= 0.0f && m < 8) { b[m][0] = P[0]; b[m][1] = P[1]; m++; }
            if ((dp < 0.0f) != (dq < 0.0f) && dp != dq && m < 8) {
                float t = dp / (dp - dq);
                b[m][0] = P[0] + t * (Q[0] - P[0]);
                b[m][1] = P[1] + t * (Q[1] - P[1]);
                m++;
            }
        }
        memcpy(a, b, sizeof(float) * 2 * (size_t)m);
        n = m;
    }
    if (n < 3) return;

    float xy[8][2];
    for (int i = 0; i < n; i++) { xy[i][0] = mm_px(a[i][0]); xy[i][1] = mm_py(a[i][1]); }
    mm_fill_poly((const float (*)[2])xy, n, color, p);
}

/* road quad outline: solid, or made of short solid pieces */
static void mm_road_quad_outline(const float q[4][2], int dashed, const float color[4],
                                 const struct gl_npu_program *p)
{
    for (int s = 0; s < 4; s++) {
        const float *A = q[s], *B = q[(s + 1) % 4];
        if (!dashed) { mm_road_seg(A[0], A[1], B[0], B[1], 0, color, p); continue; }
        float dx = B[0] - A[0], dz = B[1] - A[1];
        float len = sqrtf(dx * dx + dz * dz);
        for (float a = 0.0f; a < len; a += 2.0f * MM_COAST_DASH_M) {
            float b = fminf(a + MM_COAST_DASH_M, len);
            mm_road_seg(A[0] + dx * a / len, A[1] + dz * a / len,
                        A[0] + dx * b / len, A[1] + dz * b / len, 0, color, p);
        }
    }
}

/* weighted least squares x = a + b z (+ c z^2). Returns 0 on success. */
static int mm_lane_fit(const float *xs, const float *zs, const float *ws, int m, int quad, float coef[3])
{
    double S[5] = {0}, T[3] = {0};               // sum w z^k (k 0..4), sum w x z^k (k 0..2)
    for (int j = 0; j < m; j++) {
        double w = ws[j], zk = 1.0;
        for (int k = 0; k < 5; k++) { S[k] += w * zk; if (k < 3) T[k] += w * xs[j] * zk; zk *= zs[j]; }
    }
    coef[0] = coef[1] = coef[2] = 0.0f;
    if (quad) {                                   // 3x3 normal equations (Cramer)
        double A[3][3] = { {S[0], S[1], S[2]}, {S[1], S[2], S[3]}, {S[2], S[3], S[4]} };
        double det = A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1])
                   - A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0])
                   + A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
        if (fabs(det) < 1e-12) return -1;
        for (int col = 0; col < 3; col++) {
            double B[3][3];
            memcpy(B, A, sizeof(B));
            for (int row = 0; row < 3; row++) B[row][col] = T[row];
            coef[col] = (float)((B[0][0] * (B[1][1] * B[2][2] - B[1][2] * B[2][1])
                               - B[0][1] * (B[1][0] * B[2][2] - B[1][2] * B[2][0])
                               + B[0][2] * (B[1][0] * B[2][1] - B[1][1] * B[2][0])) / det);
        }
        return 0;
    }
    double det = S[0] * S[2] - S[1] * S[1];
    if (fabs(det) < 1e-12) return -1;
    coef[1] = (float)((S[0] * T[1] - S[1] * T[0]) / det);
    coef[0] = (float)((T[0] - coef[1] * S[1]) / S[0]);
    return 0;
}

/* Fitted ego-lane boundaries of the frame being drawn ([0] left, [1] right) */
typedef struct {
    int   ok[2];
    float cf[2][3];
} stMmEgoFit;

/* 1 if lane slot s is drawn yellow at lane frame now (see MM_YEL_*) */
static int mm_line_yellow(const stMmLaneModel *M, int s, uint64_t now)
{
    if (s < 0 || s >= MM_LANE_LINES || M->yel_us[s] == 0 || now < M->yel_us[s]) return 0;
    if (now - M->yel_us[s] > MM_YEL_HOLD_US) return 0;
    if (M->white_us[s] != 0 && now >= M->white_us[s] && now - M->white_us[s] > MM_YEL_WHITE_US) return 0;
    return 1;
}

#if NC_MINIMAP_YELLOW_CENTRE
static int mm_is_yellow(int r, int g, int b)
{
    int mx = r > g ? (r > b ? r : b) : (g > b ? g : b);
    int mn = r < g ? (r < b ? r : b) : (g < b ? g : b);
    if (mx < 90 || (mx - mn) * 100 < 35 * mx) return 0;        // v >= 0.35, s >= 0.35
    float d = (float)(mx - mn), hue;
    if (mx == r)      hue = 60.0f * (float)(g - b) / d;
    else if (mx == g) hue = 60.0f * (2.0f + (float)(b - r) / d);
    else return 0;
    return hue >= 36.0f && hue <= 68.4f;
}

/* yellow share of the frame pixels along lane line ln (-1: too few pixels) */
static float mm_yellow_frac(const stLaneDet *ln, const uint8_t *rgb, int w, int h)
{
    const float sx = (float)w / (float)VIEWPORT_WIDTH, sy = (float)h / (float)VIEWPORT_HEIGHT;
    int half = (int)lroundf(MM_YEL_HALF_PX * sx);
    if (half < 1) half = 1;
    int n = ln->point_cnt, cnt = 0, yel = 0;
    if (n > MAX_POINT_CNT_OF_LANE) n = MAX_POINT_CNT_OF_LANE;
    for (int j = 0; j < n; j++) {
        float u = ln->point[j].x, v = ln->point[j].y;
        if (!isfinite(u) || !isfinite(v) || (u <= 0.0f && v <= 0.0f) || v > MM_YEL_MAX_ROW_PX) continue;
        int y = (int)(v * sy), xc = (int)(u * sx);
        if (y < 0 || y >= h) continue;
        const uint8_t *row = rgb + (size_t)y * (size_t)w * 3;
        for (int x = xc - half; x <= xc + half; x++) {
            if (x < 0 || x >= w) continue;
            const uint8_t *px = row + (size_t)x * 3;
            cnt++;
            yel += mm_is_yellow(px[0], px[1], px[2]);
        }
    }
    return cnt >= MM_YEL_MIN_SAMPLES ? (float)yel / (float)cnt : -1.0f;
}
#endif

void nc_minimap_set_frame(uint32_t ch, const uint8_t *rgb, int w, int h)
{
#if NC_MINIMAP_YELLOW_CENTRE
    if (ch >= NC_LDW_MAX_CH || rgb == NULL || w <= 0 || h <= 0) return;
    stNcLdwLanes *L = &g_mm_lanes;
    if (nc_ldw_app_get_lanes(ch, L) != 0) return;
    stMmLaneModel *M = &g_mm[ch].lane;
    const int li = (int)NC_LDW_EGO_LANE.left_boundary_index;
    for (int s = 0; s < MM_LANE_LINES; s++) M->yel_frac[s] = -1.0f;
    for (int s = li - 1; s <= li; s++) {              // lines left of the ego vehicle only
        if (s < 0 || s >= MM_LANE_LINES || s >= L->lane_num || L->lane[s].point_cnt <= 0) continue;
        float f = mm_yellow_frac(&L->lane[s], rgb, w, h);
        M->yel_frac[s] = f;
        if (f < 0.0f) continue;
        if (f >= MM_YEL_FRAC) {
            M->yel_us[s] = L->t_us;
            M->white_us[s] = 0;
        } else if (M->white_us[s] == 0) {
            M->white_us[s] = L->t_us;
        }
    }
#else
    (void)ch; (void)rgb; (void)w; (void)h;
#endif
}

/* diagnostics (NC_BEV_LOG): lane points of a new lane frame, one row per lane line.
 * Empty lines are written too, so that lost lines show up in the log. */
static void mm_log_lanes(uint32_t ch, const stNcLdwLanes *L, const stNcLdwGeom *g)
{
    if (!nc_bevlog_enabled() || L->t_us == g_mm[ch].lane_log_us) return;
    g_mm[ch].lane_log_us = L->t_us;

    for (int i = 0; i < L->lane_num; i++) {
        const stLaneDet *ln = &L->lane[i];
        int n = ln->point_cnt;
        if (n < 0) n = 0;
        if (n > MAX_POINT_CNT_OF_LANE) n = MAX_POINT_CNT_OF_LANE;

        char pts[MAX_POINT_CNT_OF_LANE * 16 + 1];
        size_t len = 0;
        pts[0] = '\0';
        for (int j = 0; j < n; j++) {
            int w = snprintf(pts + len, sizeof(pts) - len, "%s%.1f:%.1f", j ? ";" : "",
                             (double)ln->point[j].x, (double)ln->point[j].y);
            if (w < 0 || (size_t)w >= sizeof(pts) - len) { pts[len] = '\0'; break; }   // keep whole points only
            len += (size_t)w;
        }
        const stMmLaneModel *M = &g_mm[ch].lane;
        nc_bevlog_row(NC_BEVLOG_LANEPTS, "%llu,%u,%d,%d,%d,%.5f,%.2f,%s,%.2f,%d",
                      (unsigned long long)L->t_us, ch, i, ln->lane_class, n,
                      (double)g->k, (double)g->vp_y, pts,
                      (double)(i < MM_LANE_LINES ? M->yel_frac[i] : -1.0f), mm_line_yellow(M, i, L->t_us));
    }
}

/* road points of lane slot s with geometry gi. Returns the count, *mean_x = their mean. */
static int mm_lane_points(uint32_t ch, const stNcLdwGeom *gi, const stLaneDet *ln,
                          float *xs, float *zs, float *mean_x)
{
    int n = ln->point_cnt, m = 0;
    float sum = 0.0f;
    if (n > MAX_POINT_CNT_OF_LANE) n = MAX_POINT_CNT_OF_LANE;
    for (int j = 0; j < n; j++) {
        float u = ln->point[j].x, v = ln->point[j].y, x, z;
        if (!isfinite(u) || !isfinite(v) || (u <= 0.0f && v <= 0.0f)) continue;
        if (v - gi->vp_y < MM_LANE_MIN_ROW_PX) continue;
        if (nc_ldw_geom_to_road(ch, gi, u, v, &x, &z) != 0 || z <= 0.0f || z > MM_LANE_MAX_Z_M) continue;
        xs[m] = x; zs[m] = z; m++;
        sum += x;
    }
    *mean_x = m > 0 ? sum / (float)m : 0.0f;
    return m;
}

/* Update the lane model of channel ch from a new lane frame. Both ego lines give one
 * centre curve, a single line gives it through the fixed lane width. */
static void mm_lane_update(uint32_t ch, const stNcLdwLanes *L, const stNcLdwGeom *g)
{
    stMmLaneModel *M = &g_mm[ch].lane;
    if (L->t_us == M->t_us) return;
    float dt = M->t_us != 0 && L->t_us > M->t_us ? (float)(L->t_us - M->t_us) * 1e-6f : 0.04f;
    M->t_us = L->t_us;

    for (int i = 0; i < MM_LANE_LINES && i < L->lane_num; i++) {
        if (L->lane[i].point_cnt > 0) {
            M->seen_us[i] = L->t_us;
            M->cls[i] = L->lane[i].lane_class;
        }
    }

    const int li = (int)NC_LDW_EGO_LANE.left_boundary_index, ri = (int)NC_LDW_EGO_LANE.right_boundary_index;
    if (L->lane_num <= li || L->lane_num <= ri) return;

    const int both = mm_frame_ok(g);
    stNcLdwGeom gi;
    mm_frame_geom(ch, g, &gi);

    // points of the two lines; left and right by position, as in LDW
    float px[2][MAX_POINT_CNT_OF_LANE], pz[2][MAX_POINT_CNT_OF_LANE], mean[2];
    int cnt[2];
    cnt[0] = mm_lane_points(ch, &gi, &L->lane[li], px[0], pz[0], &mean[0]);
    cnt[1] = mm_lane_points(ch, &gi, &L->lane[ri], px[1], pz[1], &mean[1]);

    float xs[2 * MAX_POINT_CNT_OF_LANE], zs[2 * MAX_POINT_CNT_OF_LANE], ws[2 * MAX_POINT_CNT_OF_LANE];
    float zmin = 1e9f, zmax = -1e9f;
    int m = 0;
    const float half = NC_LDW_DEFAULT_LANE_WIDTH_M * 0.5f;
    for (int s = 0; s < 2; s++) {
        if (cnt[s] == 0) continue;
        int is_left = (both && cnt[0] > 0 && cnt[1] > 0) ? mean[s] < mean[1 - s] : mean[s] < 0.0f;
        if (!(is_left ? g->left_ok : g->right_ok)) continue;
        for (int j = 0; j < cnt[s]; j++) {
            float z = pz[s][j];
            xs[m] = px[s][j] + (is_left ? half : -half);
            zs[m] = z;
            ws[m] = 1.0f / (z * z);               // equal noise in pixels
            m++;
            if (z < zmin) zmin = z;
            if (z > zmax) zmax = z;
        }
    }
    if (m < MM_LANE_MIN_PTS) return;

    float cf[3], c_raw = 0.0f;
    if (both && zmax - zmin > MM_LANE_QUAD_SPAN_M && mm_lane_fit(xs, zs, ws, m, 1, cf) == 0) {
        c_raw = cf[2];
        if (!M->c_init) { M->c_filt = c_raw; M->c_init = 1; }
        M->c_filt += (1.0f - expf(-dt / MM_LANE_C_TAU_S)) * (c_raw - M->c_filt);
    }
    const float ref2 = MM_LANE_CURVE_REF_M * MM_LANE_CURVE_REF_M;
    float bend = fabsf(M->c_filt) * ref2;
    if (!M->curve_on && bend > MM_LANE_CURVE_ON_M) M->curve_on = 1;
    else if (M->curve_on && bend < MM_LANE_CURVE_OFF_M) M->curve_on = 0;
    M->c_shown += (1.0f - expf(-dt / MM_LANE_SHOW_TAU_S)) * ((M->curve_on ? M->c_filt : 0.0f) - M->c_shown);

    // offset and heading with the drawn curvature
    for (int j = 0; j < m; j++) xs[j] -= M->c_shown * zs[j] * zs[j];
    if (mm_lane_fit(xs, zs, ws, m, 0, cf) != 0) return;
    if (M->fit_us == 0) {
        M->c0 = cf[0]; M->h = cf[1];
    } else {
        float a = 1.0f - expf(-dt / MM_LANE_POS_TAU_S);
        M->c0 += a * (cf[0] - M->c0);
        M->h  += a * (cf[1] - M->h);
    }
    M->fit_us = L->t_us;
    M->zmin = zmin;
    M->zmax = zmax;

    if (nc_bevlog_enabled())
        nc_bevlog_row(NC_BEVLOG_LANEFIT, "%llu,%u,%d,%d,%.5f,%.2f,%.6f,%.6f,%.6f,%d,%.4f,%.5f",
                      (unsigned long long)L->t_us, ch, both, m, (double)gi.k, (double)gi.vp_y,
                      (double)c_raw, (double)M->c_filt, (double)M->c_shown, M->curve_on,
                      (double)M->c0, (double)M->h);
}

enum { MM_LANES_NONE = 0, MM_LANES_MEASURED, MM_LANES_HELD, MM_LANES_VIRTUAL };

/* Lane lines of the lane model at fixed offsets from the centre curve, over the whole
 * panel height. Ego lines are drawn while the model is valid, outer lines only while
 * their network line is detected. The part outside the real points is fainter.
 * Lanes lost: held, then virtual (MM_LANE_HELD_US). Returns MM_LANES_*.
 * Style: lane class 1 = solid, 2 = dashed, other = dashed (never guessed solid);
 * NC_MINIMAP_USE_LANE_CLASS 0 = all dashed. Ego-lane boundaries white, others grey.
 * A line measured yellow (NC_MINIMAP_YELLOW_CENTRE) is a yellow double solid line. */
static int mm_draw_lanes(uint32_t ch, stMmEgoFit *ego_fit, const struct gl_npu_program *p)
{
    stNcLdwLanes *L = &g_mm_lanes;
    stNcLdwGeom g;
    if (nc_ldw_app_get_geom(ch, 0, &g) < 0) return MM_LANES_NONE;
    if (nc_ldw_app_get_lanes(ch, L) == 0) {
        mm_log_lanes(ch, L, &g);
        mm_lane_update(ch, L, &g);
    } else {
        memset(L, 0, sizeof(*L));                 // no lane result yet: virtual lanes
    }

    const stMmLaneModel *M = &g_mm[ch].lane;
    uint64_t now = g_mm[ch].t_disp_us > L->t_us ? g_mm[ch].t_disp_us : L->t_us;
    if (M->t_us > now) now = M->t_us;
    uint64_t age = (M->fit_us != 0 && now > M->fit_us) ? now - M->fit_us : 0;

    // drawn model: the fitted one, or the virtual one
    int state = MM_LANES_MEASURED;
    float c0 = M->c0, h = M->h, cs = M->c_shown;
    if (M->fit_us == 0) {
        state = MM_LANES_VIRTUAL;
        c0 = h = cs = 0.0f;
    } else if (age > MM_LANE_HELD_US) {
        state = MM_LANES_VIRTUAL;
        float e = expf(-(float)(age - MM_LANE_HELD_US) * 1e-6f / MM_LANE_VIRT_TAU_S);
        h *= e;
        cs *= e;
    } else if (M->t_us - M->fit_us > 0 || age > MM_LANE_HOLD_US) {
        state = MM_LANES_HELD;                    // a lane frame without a fit, or none for a while
    }
    const int virt = (state == MM_LANES_VIRTUAL);
    static int s_virt[NC_LDW_MAX_CH];
    if (virt != s_virt[ch]) {
        printf("[MINIMAP] ch%u lanes %s (last fit %.2f s ago)\n", ch,
               virt ? "lost: virtual lanes" : "back: measured", (double)age * 1e-6);
        fflush(stdout);
        s_virt[ch] = virt;
    }

    const float ego_col[4] = {1.0f, 1.0f, 1.0f, 0.95f};
    const float adj_col[4] = {0.70f, 0.70f, 0.70f, 0.90f};
    const float yel_col[4] = {1.0f, 0.80f, 0.0f, 0.95f};
    const float w = NC_LDW_DEFAULT_LANE_WIDTH_M;
    const int li = (int)NC_LDW_EGO_LANE.left_boundary_index, ri = (int)NC_LDW_EGO_LANE.right_boundary_index;
    const struct { int slot; float offset; } lines[MM_LANE_LINES] = {
        { li - 1, -1.5f * w }, { li, -0.5f * w }, { ri, 0.5f * w }, { ri + 1, 1.5f * w },
    };

    for (int s = 0; s < 2; s++) {
        ego_fit->ok[s] = 1;
        ego_fit->cf[s][0] = c0 + (s ? 0.5f : -0.5f) * w;
        ego_fit->cf[s][1] = h;
        ego_fit->cf[s][2] = cs;
    }

    for (int i = 0; i < MM_LANE_LINES; i++) {
        int slot = lines[i].slot;
        int ego = (slot == li || slot == ri);
        // virtual: ego lines and the next lines on both sides, all dashed
        if (!ego && !virt && (M->seen_us[slot] == 0 || now - M->seen_us[slot] > MM_LANE_OUTER_HOLD_US)) continue;

        int dashed = virt ? 1 : NC_MINIMAP_USE_LANE_CLASS ? (M->cls[slot] != 1) : 1;
        const float *col = ego ? ego_col : adj_col;
        // centre line: yellow double solid line (only lines left of the ego vehicle are measured)
        const int yellow = !virt && mm_line_yellow(M, slot, L->t_us);
        if (yellow) { col = yel_col; dashed = 0; }
        const float vcol[4] = {col[0], col[1], col[2], col[3] * MM_LANE_VIRT_ALPHA};
        const float fcol[4] = {col[0], col[1], col[2], col[3] * MM_LANE_EXTRAP_ALPHA};
        if (virt) col = vcol;

        // segment ends: panel bottom .. panel top, split at the real-point range
        float zb[16];
        int nb = 0;
        const float z_lo = -MM_BEHIND_M, z_hi = NC_MINIMAP_RANGE_M;
        const float cuts[2] = { virt ? z_lo : M->zmin, virt ? z_lo : M->zmax };
        zb[nb++] = z_lo;
        for (float z = z_lo + MM_LANE_SEG_M; ; z += MM_LANE_SEG_M) {
            for (int k = 0; k < 2; k++)                   // insert the cuts in order
                if (cuts[k] > zb[nb - 1] && cuts[k] < fminf(z, z_hi) && nb < 15) zb[nb++] = cuts[k];
            if (z >= z_hi || nb >= 15) { zb[nb++] = z_hi; break; }
            zb[nb++] = z;
        }
        for (int e = 0; e < (yellow ? 2 : 1); e++) {
            const float a0 = c0 + lines[i].offset + (yellow ? (e ? 0.5f : -0.5f) * MM_YEL_GAP_M : 0.0f);
            for (int k = 0; k + 1 < nb; k++) {
                float z0 = zb[k], z1 = zb[k + 1];
                if (z1 - z0 < 0.05f) continue;
                float x0 = a0 + h * z0 + cs * z0 * z0;
                float x1 = a0 + h * z1 + cs * z1 * z1;
                float zm = 0.5f * (z0 + z1);
                mm_road_seg(x0, z0, x1, z1, dashed, (virt || (zm >= M->zmin && zm <= M->zmax)) ? col : fcol, p);
            }
        }
    }
    return state;
}

/* Slope dx/dz of the drawn lane lines between z and z + l (chord of the centre curve).
 * 0 (upright box) when no lane model is drawn. The ego box is not turned: the panel is in
 * ego coordinates, so its heading is always straight up. */
static float mm_box_slope(const stMmEgoFit *e, float z, float l)
{
    for (int s = 0; s < 2; s++)
        if (e->ok[s]) return e->cf[s][1] + e->cf[s][2] * (2.0f * z + l);
    return 0.0f;
}

#if NC_MINIMAP_SHOW_PRED
/* Lateral position of a vehicle in lanes relative to the ego-lane centre at the
 * vehicle's distance (0 = ego lane centre, +: right), so that it follows curves and
 * the ego offset. Rounded, it is the lane the box is drawn in. */
static float mm_track_lane_pos(const stMmTrack *t, const stMmEgoFit *e)
{
    const float w = NC_LDW_DEFAULT_LANE_WIDTH_M;
    float xs[2] = {0.0f, 0.0f}, centre = 0.0f;
    for (int s = 0; s < 2; s++)
        if (e->ok[s]) xs[s] = e->cf[s][0] + e->cf[s][1] * t->z + e->cf[s][2] * t->z * t->z;
    if (e->ok[0] && e->ok[1]) centre = 0.5f * (xs[0] + xs[1]);
    else if (e->ok[0])        centre = xs[0] + 0.5f * w;
    else if (e->ok[1])        centre = xs[1] - 0.5f * w;
    return (t->x - centre) / w;
}

/* Flag oncoming vehicles among the drawn ones that were measured this frame (see
 * MM_ONC_*). The closing speed is taken relative to the other vehicles of the frame:
 * when the road geometry is off (0001 board run, t about 19.5 s, a crest) every far
 * vehicle closes at once by an amount that grows with z, and two left-lane cars reached
 * -13 m/s. So z times the median v/z of the others is removed first; both the raw and
 * the relative speed must be below -V. With no other vehicle the raw speed is used. */
static void mm_flag_oncoming(stMmChannel *c, const stMmEgoFit *e)
{
    const uint64_t now = c->t_disp_us;
    int idx[MM_MAX_TRACK], ref[MM_MAX_TRACK], n = 0;
    float v[MM_MAX_TRACK], ratio[MM_MAX_TRACK];
    for (int i = 0; i < MM_MAX_TRACK; i++) {
        const stMmTrack *t = &c->track[i];
        float s;
        if (!t->used || t->coast != MM_COAST_NONE || t->last_upd_disp_us != now) continue;
        if (mm_onc_speed(t, &s) != 0) continue;
        idx[n] = i;
        v[n] = s;
        ref[n] = !t->oncoming && t->z > 1.0f;
        ratio[n] = ref[n] ? s / t->z : 0.0f;
        n++;
    }
    for (int k = 0; k < n; k++) {
        stMmTrack *t = &c->track[idx[k]];
        if (t->oncoming || v[k] >= -MM_ONC_V || !mm_track_drawn(c, t)) continue;
        if (mm_track_lane_pos(t, e) >= MM_ONC_POS_MAX) continue;
        float o[MM_MAX_TRACK];
        int m = 0;
        for (int j = 0; j < n; j++) {                 // ratios of the others, sorted
            if (j == k || !ref[j]) continue;
            int q = m++;
            while (q > 0 && o[q - 1] > ratio[j]) { o[q] = o[q - 1]; q--; }
            o[q] = ratio[j];
        }
        float vr = v[k];
        if (m > 0) vr -= t->z * ((m % 2) ? o[m / 2] : 0.5f * (o[m / 2 - 1] + o[m / 2]));
        if (vr < -MM_ONC_V) t->oncoming = 1;
    }
}

/* Arrow from road point (x0, z0) along (dx, dz): dark outline, body that widens
 * and fades in towards the head, filled head. The head is left out when the
 * arrow leaves the panel. */
static void mm_draw_pred(float x0, float z0, float dx, float dz, const float color[4],
                         const struct gl_npu_program *p)
{
    const float bw = NC_MINIMAP_PRED_BODY_W * 0.5f, e = NC_MINIMAP_PRED_OUTLINE_PX;
    const float taper = 0.55f;                       // start width / head-end width
    const int   nseg = 6;
    float t0, t1;
    if (!mm_clip_seg(x0, z0, x0 + dx, z0 + dz, &t0, &t1)) return;

    float ax = mm_px(x0), ay = mm_py(z0);
    float vx = mm_px(x0 + dx) - ax, vy = mm_py(z0 + dz) - ay;
    float len = sqrtf(vx * vx + vy * vy);
    if (len < 1.0f) return;
    float ux = vx / len, uy = vy / len, nx = -uy, ny = ux;

    float hl = fminf(NC_MINIMAP_PRED_HEAD_LEN, 0.4f * len), hw = fmaxf(hl * 0.6f, NC_MINIMAP_PRED_BODY_W);
    float tipx = ax + vx, tipy = ay + vy;
    float neckx = tipx - ux * hl, necky = tipy - uy * hl;
    float wing = hw + 2.0f * e;
    int head = (t1 >= 1.0f);
    for (float s = -1.0f; head && s <= 1.0f; s += 2.0f) {
        float wx = neckx + (float)s * nx * wing, wy = necky + (float)s * ny * wing;
        if (wx < k_px || wx > k_px + k_pw || wy < k_py || wy > k_py + k_ph) head = 0;
    }
    float te = head ? 1.0f - hl / len : t1;          // body end
    if (te <= t0) return;

    float sx = ax + t0 * vx, sy = ay + t0 * vy, ex = ax + te * vx, ey = ay + te * vy;
    float ws = bw * taper + e, we = bw + e;

    // outline: one polygon, so that it is not blended twice
    const float dark[4] = {0.0f, 0.0f, 0.0f, 0.6f};
    if (head) {
        const float o[9][2] = {
            { ex, ey },
            { sx + nx * ws, sy + ny * ws }, { ex + nx * we, ey + ny * we },
            { neckx + nx * wing, necky + ny * wing }, { tipx + ux * 2.0f * e, tipy + uy * 2.0f * e },
            { neckx - nx * wing, necky - ny * wing }, { ex - nx * we, ey - ny * we },
            { sx - nx * ws, sy - ny * ws }, { sx + nx * ws, sy + ny * ws },
        };
        mm_fill_poly(o, 9, dark, p);
    } else {
        const float o[4][2] = {
            { sx + nx * ws, sy + ny * ws }, { ex + nx * we, ey + ny * we },
            { ex - nx * we, ey - ny * we }, { sx - nx * ws, sy - ny * ws },
        };
        mm_fill_poly(o, 4, dark, p);
    }

    // body
    for (int k = 0; k < nseg; k++) {
        float f0 = (float)k / (float)nseg, f1 = (float)(k + 1) / (float)nseg;
        float w0 = bw * (taper + (1.0f - taper) * f0), w1 = bw * (taper + (1.0f - taper) * f1);
        float px0 = sx + (ex - sx) * f0, py0 = sy + (ey - sy) * f0;
        float px1 = sx + (ex - sx) * f1, py1 = sy + (ey - sy) * f1;
        const float q[4][2] = {
            { px0 + nx * w0, py0 + ny * w0 }, { px1 + nx * w1, py1 + ny * w1 },
            { px1 - nx * w1, py1 - ny * w1 }, { px0 - nx * w0, py0 - ny * w0 },
        };
        float col[4] = {color[0], color[1], color[2], color[3] * (0.35f + 0.65f * f1)};
        mm_fill_poly(q, 4, col, p);
    }

    if (head) {
        const float h[3][2] = {
            { neckx + nx * hw, necky + ny * hw }, { tipx, tipy }, { neckx - nx * hw, necky - ny * hw },
        };
        mm_fill_poly(h, 3, color, p);
    }
}
#endif

static void mm_text(const stNcMinimapDrawRes *res, const char *s, float x, float y_canvas,
                    float scale, float color[3])
{
    // text y is measured from the bottom
    nc_opengl_draw_text(res->font, s, x, (float)VIEWPORT_HEIGHT - y_canvas, scale, color,
                        VIEWPORT_WIDTH, VIEWPORT_HEIGHT, *res->font_prog);
}

/* ------------------------------------------------------------------------- */

void nc_minimap_draw(uint32_t ch, struct viewport vp, const stNcMinimapDrawRes *res)
{
    if (ch >= NC_LDW_MAX_CH || res == NULL || vp.height == 0) return;
    stMmChannel *c = &g_mm[ch];
    const struct gl_npu_program *p = res->npu_prog;
    float text_scale = (float)VIEWPORT_HEIGHT / (float)vp.height * 0.8f;

    mm_process(ch);
    mm_log(ch);

    glViewport((GLint)vp.x, (GLint)vp.y, (GLsizei)vp.width, (GLsizei)vp.height);

    // background + frame
    const float bg[4]    = {0.05f, 0.05f, 0.07f, NC_MINIMAP_BG_ALPHA};
    const float frame[4] = {0.7f, 0.7f, 0.7f, 1.0f};
    mm_fill_rect(k_px, k_py, k_pw, k_ph, bg, p);
    mm_rect(k_px, k_py, k_pw, k_ph, frame, p);

    stNcLdwGeom g;
    if (nc_ldw_app_get_geom(ch, 0, &g) < 0) {
        float gray[3] = {0.8f, 0.8f, 0.8f};
        mm_text(res, "MINIMAP: no lane", k_px + 10.0f, k_py + 30.0f, text_scale, gray);
        return;
    }

    // distance grid every 10 m. No labels: z uses the 672 px focal default and the
    // measured value (about 1900 px) is not applied, so real distances are about 2.8x z.
    for (int d = 10; d <= (int)NC_MINIMAP_RANGE_M; d += 10) {
        float y = mm_py((float)d);
        float gcol[4] = {0.5f, 0.5f, 0.5f, 0.5f};
        nc_opengl_draw_line(k_px, y, k_px + k_pw, y, 1, gcol, *p);
    }

    stMmEgoFit ego_fit;
    memset(&ego_fit, 0, sizeof(ego_fit));
    if (mm_draw_lanes(ch, &ego_fit, p) == MM_LANES_VIRTUAL) {
        float gray[3] = {0.6f, 0.6f, 0.6f};
        mm_text(res, "virtual lane", k_px + 10.0f, k_py + 30.0f, text_scale, gray);
    }

    // vehicles, far first so that near ones are drawn on top
    int order[MM_MAX_TRACK], cnt = 0;
    for (int i = 0; i < MM_MAX_TRACK; i++)
        if (mm_track_drawn(c, &c->track[i])) order[cnt++] = i;
    for (int i = 1; i < cnt; i++) {
        int t = order[i], j = i;
        while (j > 0 && c->track[order[j - 1]].z < c->track[t].z) { order[j] = order[j - 1]; j--; }
        order[j] = t;
    }

    // drawn x: side-by-side boxes that overlap pushed apart (see MM_SEP_*)
    float draw_x[MM_MAX_TRACK];
    {
        float pos[MM_MAX_TRACK], push[MM_MAX_TRACK];
        for (int i = 0; i < cnt; i++) {
            pos[i] = mm_track_lane_pos(&c->track[order[i]], &ego_fit) * NC_LDW_DEFAULT_LANE_WIDTH_M;
            push[i] = 0.0f;
        }
        for (int it = 0; it < MM_SEP_ITER; it++)              // again: a pushed box can meet the next one
        for (int i = 0; i < cnt; i++) {
            const stMmTrack *a = &c->track[order[i]];
            for (int j = i + 1; j < cnt; j++) {
                const stMmTrack *b = &c->track[order[j]];
                float pi = pos[i] + push[i], pj = pos[j] + push[j];
                float d = pj - pi, ad = fabsf(d);
                if (ad <= MM_SEP_MIN_M) continue;                       // same column
                float need = 0.5f * (k_cls_w[a->cls] + k_cls_w[b->cls]) + MM_SEP_GAP_M - ad;
                float oz = 0.5f * (k_cls_l[a->cls] + k_cls_l[b->cls]) - fabsf(a->z - b->z);
                if (need <= 0.0f || oz <= 0.0f) continue;
                float r = oz / MM_SEP_RAMP_M;
                int out = fabsf(pj) > fabsf(pi) ? j : i;                // outer one moves out
                push[out] += (pos[out] > 0.0f ? 1.0f : -1.0f) * need * (r < 1.0f ? r : 1.0f);
                push[out] = fmaxf(-MM_SEP_MAX_M, fminf(MM_SEP_MAX_M, push[out]));
            }
        }
        for (int i = 0; i < cnt; i++)
            draw_x[i] = c->track[order[i]].x + push[i];
    }

    const float cutin[4] = {1.0f, 0.5f, 0.0f, 1.0f};   // [11.06] 앱과 같은 주황 (노랑은 LDW 약한 경고)
    const float edge[4]  = {1.0f, 1.0f, 1.0f, 0.9f};
    float front[MM_MAX_TRACK][2];                // front edge centre of each drawn box
    for (int i = 0; i < cnt; i++) {
        const stMmTrack *t = &c->track[order[i]];
        float fc[4] = {0.8f, 0.2f, 0.2f, 1.0f};
        if (t->has_color[t->cls]) memcpy(fc, t->color[t->cls], sizeof(fc));
        else for (int k = 0; k < MM_CLS_N; k++) if (t->has_color[k]) { memcpy(fc, t->color[k], sizeof(fc)); break; }
        if (t->highlight) memcpy(fc, cutin, sizeof(fc));
        fc[3] = 0.95f;

        float w = k_cls_w[t->cls], l = k_cls_l[t->cls];
        float q[4][2];
        mm_box_corners(draw_x[i], t->z, w, l, mm_box_slope(&ego_fit, t->z, l), q,
                       &front[order[i]][0], &front[order[i]][1]);
        if (t->coast == MM_COAST_RUN) {          // not measured: dashed, fading
            const float dash[4] = {1.0f, 1.0f, 1.0f, t->coast_a};
            fc[3] = MM_COAST_FILL_ALPHA * t->coast_a;
            mm_road_quad(q, fc, p);
            mm_road_quad_outline(q, 1, dash, p);
            continue;
        }
        mm_road_quad(q, fc, p);
        mm_road_quad_outline(q, 0, t->highlight ? cutin : edge, p);
    }

#if NC_MINIMAP_SHOW_PRED
    // predicted trajectory of the nearest vehicle of each lane, from its front edge:
    // forward, leaning by the predicted lateral motion. Drawn over the boxes so that it
    // stays visible on the vehicle ahead. order[] is far first, so walk it from the end
    // (nearest first). The lane is the one the box is drawn in. A vehicle whose box
    // overlaps sideways with a nearer chosen one is behind it in the same column: no arrow.
    // Oncoming vehicles are not chosen (the next one of their lane is).
    const float pred_col[4] = NC_MINIMAP_PRED_COLOR;
    const float pred_cutin[4] = NC_MINIMAP_PRED_CUTIN_COLOR;
    const float lane_w = NC_LDW_DEFAULT_LANE_WIDTH_M;
    int nearest[2 * MM_PRED_LANE_MAX + 1];
    float near_pos[2 * MM_PRED_LANE_MAX + 1];
    for (int k = 0; k < 2 * MM_PRED_LANE_MAX + 1; k++) nearest[k] = -1;
    mm_flag_oncoming(c, &ego_fit);
    for (int i = cnt - 1; i >= 0; i--) {
        const stMmTrack *t = &c->track[order[i]];
        if (t->coast == MM_COAST_RUN || t->oncoming) continue;
        float pos = mm_track_lane_pos(t, &ego_fit);
        int lane = (int)lroundf(pos);
        if (lane < -MM_PRED_LANE_MAX || lane > MM_PRED_LANE_MAX || nearest[lane + MM_PRED_LANE_MAX] >= 0) continue;
        int hidden = 0;
        for (int k = 0; k < 2 * MM_PRED_LANE_MAX + 1 && !hidden; k++)
            if (nearest[k] >= 0 && fabsf(pos - near_pos[k]) * lane_w
                < 0.5f * (k_cls_w[t->cls] + k_cls_w[c->track[nearest[k]].cls])) hidden = 1;
        if (hidden) continue;
        nearest[lane + MM_PRED_LANE_MAX] = order[i];
        near_pos[lane + MM_PRED_LANE_MAX] = pos;
    }
    for (int k = 0; k < 2 * MM_PRED_LANE_MAX + 1; k++) {
        if (nearest[k] < 0) continue;
        const stMmTrack *t = &c->track[nearest[k]];
        if (t->fut_n == 0) continue;
        float len = sqrtf(t->fut_dx * t->fut_dx + t->fut_dz * t->fut_dz);
        if (len > NC_MINIMAP_PRED_MAX_M) continue;
        const float fx = front[nearest[k]][0], fz = front[nearest[k]][1];
#if NC_MINIMAP_PRED_LANE_REL
        // along the drawn lane (chord over the arrow length), leaning by the lane-relative speed
        float dx = mm_box_slope(&ego_fit, fz, NC_MINIMAP_PRED_FWD_M) * NC_MINIMAP_PRED_FWD_M + t->pred_lean;
#else
        float dx = t->fut_dx;
#endif
        mm_draw_pred(fx, fz, dx, NC_MINIMAP_PRED_FWD_M, t->highlight ? pred_cutin : pred_col, p);
    }
#endif

    // ego vehicle
    const float ego[4] = {1.0f, 1.0f, 1.0f, 0.95f};
    const float ego_win[4] = {0.2f, 0.4f, 0.8f, 1.0f};
    mm_road_rect(0.0f, MM_EGO_W_M, -MM_EGO_REAR_M, MM_EGO_FRONT_M, ego, 1, p);
    mm_road_rect(0.0f, MM_EGO_W_M * 0.7f, 0.0f, 0.8f, ego_win, 1, p);   // windshield = front
}