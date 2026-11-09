#include "nc_cutin.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "nc_traj.h"   /* NC_TRAJ_FUTURE_MS_MAX */

/* ---------------- tuning (macro defaults, env overrides; junwoo) ---------------- */

static int      g_cfg_loaded = 0;
static float    g_horizon_ms = NC_CUTIN_HORIZON_MS;
static float    g_margin     = NC_CUTIN_LANE_MARGIN;
static uint64_t g_on_us      = (uint64_t)(NC_CUTIN_ON_MS * 1000.0f);
static uint64_t g_off_us     = (uint64_t)(NC_CUTIN_OFF_MS * 1000.0f);
static uint64_t g_hold_us    = (uint64_t)(NC_CUTIN_LC_HOLD_MS * 1000.0f);
static float    g_min_lane_w = NC_CUTIN_MIN_LANE_W_PX;
static uint64_t g_lane_stale_us = NC_CUTIN_LANE_STALE_US;
static int      g_point      = NC_CUTIN_POINT_DEFAULT;
static uint64_t g_min_on_us  = (uint64_t)(NC_CUTIN_MIN_ON_MS * 1000.0f);
static uint64_t g_dup_us     = (uint64_t)(NC_CUTIN_DUP_MS * 1000.0f);
static int      g_lane_src_max = NC_CUTIN_LANE_SRC_MAX;
static float    g_virt_margin  = NC_CUTIN_VIRT_MARGIN;
static uint64_t g_virt_on_us   = (uint64_t)(NC_CUTIN_VIRT_ON_MS * 1000.0f);
static uint64_t g_lead_on_us   = (uint64_t)(NC_CUTIN_LEAD_ON_MS * 1000.0f);
static float    g_est_edge     = NC_CUTIN_EST_EDGE;
static float    g_est_move     = NC_CUTIN_EST_MOVE;
static float    g_away_ratio   = NC_CUTIN_AWAY_RATIO;

static void env_float(const char *name, float *v)
{
    const char *e = getenv(name);
    char *end = NULL;
    if (!e || !*e) return;
    float f = strtof(e, &end);
    if (end != e) *v = f;
    else printf("[nc_cutin] ignoring %s=%s (not a number)\n", name, e);
}

/* ms value in the environment -> us */
static void env_ms(const char *name, uint64_t *us)
{
    float ms = (float)*us * 0.001f;
    env_float(name, &ms);
    if (ms < 0.0f) ms = 0.0f;
    *us = (uint64_t)(ms * 1000.0f);
}

static void cutin_load_config(void)
{
    if (g_cfg_loaded) return;
    g_cfg_loaded = 1;
    env_float("CUTIN_HORIZON_MS", &g_horizon_ms);
    env_float("CUTIN_MARGIN",     &g_margin);
    env_ms   ("CUTIN_ON_MS",      &g_on_us);
    env_ms   ("CUTIN_OFF_MS",     &g_off_us);
    env_ms   ("CUTIN_LC_HOLD_MS", &g_hold_us);
    env_float("CUTIN_MIN_LANE_W", &g_min_lane_w);
    env_ms   ("CUTIN_LANE_STALE_MS", &g_lane_stale_us);
    env_ms   ("CUTIN_MIN_ON_MS",  &g_min_on_us);
    env_ms   ("CUTIN_DUP_MS",     &g_dup_us);
    env_float("CUTIN_VIRT_MARGIN", &g_virt_margin);
    env_ms   ("CUTIN_VIRT_ON_MS", &g_virt_on_us);
    env_ms   ("CUTIN_LEAD_ON_MS", &g_lead_on_us);
    env_float("CUTIN_EST_EDGE",   &g_est_edge);
    env_float("CUTIN_EST_MOVE",   &g_est_move);
    env_float("CUTIN_AWAY_RATIO", &g_away_ratio);
    {
        float v = (float)g_lane_src_max;
        env_float("CUTIN_LANE_SRC", &v);
        int iv = (int)(v + 0.5f);
        g_lane_src_max = iv < NC_CUTIN_LANE_MEASURED ? NC_CUTIN_LANE_MEASURED
                       : iv > NC_CUTIN_LANE_VIRTUAL ? NC_CUTIN_LANE_VIRTUAL : iv;
    }
    {
        float pt = (float)g_point;
        env_float("CUTIN_POINT", &pt);
        g_point = pt >= 1.5f ? NC_CUTIN_POINT_BOTH : pt >= 0.5f ? NC_CUTIN_POINT_EDGE : NC_CUTIN_POINT_CENTER;
    }
    if (g_horizon_ms < 0.0f) g_horizon_ms = 0.0f;
    if (g_horizon_ms > (float)NC_TRAJ_FUTURE_MS_MAX) g_horizon_ms = (float)NC_TRAJ_FUTURE_MS_MAX;
    printf("[nc_cutin] config: point=%s min_on=%.0fms dup=%.0fms horizon=%.0fms margin=%.2f on=%.0fms(>=%d hits) off=%.0fms "
           "lc_hold=%.0fms min_lane_w=%.0fpx lane_stale=max(%.0fms, %.0f frames)\n",
           g_point == NC_CUTIN_POINT_BOTH ? "both" : g_point == NC_CUTIN_POINT_EDGE ? "edge" : "center",
           (double)g_min_on_us * 0.001, (double)g_dup_us * 0.001,
           (double)g_horizon_ms, (double)g_margin, (double)g_on_us * 0.001, NC_CUTIN_MIN_ON_HITS,
           (double)g_off_us * 0.001, (double)g_hold_us * 0.001, (double)g_min_lane_w,
           (double)g_lane_stale_us * 0.001, (double)NC_CUTIN_STALE_FRAMES);
    printf("[nc_cutin] config: lane source=%s (lead: on=%.0fms, virtual: margin=%.2f on=%.0fms, "
           "moving-in edge=%.2f move=%.2f, pulling away < %.2f)\n",
           g_lane_src_max == NC_CUTIN_LANE_VIRTUAL ? "measured+held+lead+virtual"
           : g_lane_src_max == NC_CUTIN_LANE_LEAD ? "measured+held+lead"
           : g_lane_src_max == NC_CUTIN_LANE_HELD ? "measured+held" : "measured",
           (double)g_lead_on_us * 0.001, (double)g_virt_margin, (double)g_virt_on_us * 0.001,
           (double)g_est_edge, (double)g_est_move, (double)g_away_ratio);
}

float nc_cutin_get_horizon_ms(void)
{
    cutin_load_config();
    return g_horizon_ms;
}

/* ---------------- ego lane source ---------------- */

#ifdef USE_LDW
#include "nc_ldw.h"

/* Junwoo's function returns 0 = ok, -1 = fail; convert to 1 = ok, 0 = fail.
 * His code is used as is (no change on his side). */
int nc_ldw_get_ego_lane_x(int ch, float y, float *x_left, float *x_right, uint64_t *lane_ts_us)
{
    if (ch < 0) return 0;
    return nc_ldw_get_ego_lane((uint32_t)ch, y, x_left, x_right, lane_ts_us) == 0 ? 1 : 0;
}

int nc_ldw_get_ego_lane_src_x(int ch, float y, float *x_left, float *x_right,
                              uint64_t *lane_ts_us, int *src)
{
    eNcLdwEgoSrc s = NC_LDW_EGO_SRC_NONE;
    if (src) *src = NC_CUTIN_LANE_MEASURED;
    if (ch < 0) return 0;
    if (nc_ldw_get_ego_lane_src((uint32_t)ch, y, x_left, x_right, lane_ts_us, &s) != 0) return 0;
    if (src) *src = s == NC_LDW_EGO_SRC_HELD    ? NC_CUTIN_LANE_HELD
                  : s == NC_LDW_EGO_SRC_LEAD    ? NC_CUTIN_LANE_LEAD
                  : s == NC_LDW_EGO_SRC_VIRTUAL ? NC_CUTIN_LANE_VIRTUAL
                  :                               NC_CUTIN_LANE_MEASURED;
    return 1;
}

#else /* !USE_LDW */

/* Fixed straight-road trapezoid in 1920x1080 screen px:
 * lane edges meet at the vanishing point, half width STUB_HALF_W at the bottom.
 * Only for testing the cut-in path; the real nc_ldw_get_ego_lane_x overrides it. */
#define STUB_SCREEN_W   (1920.0f)
#define STUB_SCREEN_H   (1080.0f)
#define STUB_VP_X       (STUB_SCREEN_W * 0.5f)
#define STUB_VP_Y       (STUB_SCREEN_H * 0.42f)
#define STUB_HALF_W     (STUB_SCREEN_W * 0.30f)

int nc_ldw_get_ego_lane_x(int ch, float y, float *x_left, float *x_right, uint64_t *lane_ts_us)
{
    (void)ch;
    if (x_left == NULL || x_right == NULL) return 0;
    if (y <= STUB_VP_Y || y > STUB_SCREEN_H) return 0;

    float r = (y - STUB_VP_Y) / (STUB_SCREEN_H - STUB_VP_Y);  /* 0 at VP, 1 at bottom */
    *x_left  = STUB_VP_X - STUB_HALF_W * r;
    *x_right = STUB_VP_X + STUB_HALF_W * r;
    if (lane_ts_us) *lane_ts_us = 0; /* unknown -> treated as fresh */
    return 1;
}

int nc_ldw_get_ego_lane_src_x(int ch, float y, float *x_left, float *x_right,
                              uint64_t *lane_ts_us, int *src)
{
    if (src) *src = NC_CUTIN_LANE_MEASURED;
    return nc_ldw_get_ego_lane_x(ch, y, x_left, x_right, lane_ts_us);
}
#endif /* USE_LDW */

/* ---------------- per-track hysteresis state (render thread only) ---------------- */

typedef struct {
    int used;
    int class_id, track_id;
    int on;               /* warning ON */
    int hit_cnt;          /* consecutive candidate frames */
    uint64_t hit_since;   /* time of the first frame of the current candidate run */
    uint64_t miss_since;  /* time of the first frame of the current non-candidate run (0: none) */
    int seen;             /* evaluated in the current frame */
    int candidate;        /* candidate in the current frame */
    uint64_t on_since;    /* time the warning turned on */
    float bx, by, bw;     /* last evaluated box: bottom-center x, bottom y, width */
    int lane_src;         /* lane source of the current frame's decision */
    int need_on_ms;       /* longest ON time (ms) needed by the lanes of the current candidate run */
    /* box height growth (px/s): an approaching car's box grows, so its bottom
     * moves down faster than its center. Used for the future bottom point. */
    float    h_ref, vh;
    uint64_t h_ref_us;
    uint64_t away_until;  /* pulling away fast was seen: not a candidate until this time */
} stCutinTrack;

/* recent warnings (any class): a track that turns ON near one of them within
 * CUTIN_DUP_MS is the same vehicle seen again (ByteTrack keys tracks by class,
 * so a car re-detected as a truck gets a new track) -> no second trigger */
#define NC_CUTIN_RECENT  (8)
typedef struct {
    int      used, class_id, track_id;
    uint64_t t;           /* last time this warning was ON */
    float    bx, by, bw;
} stCutinRecent;

/* lane_u() result */
enum { LANE_OK = 0, LANE_LOST, LANE_FAR, LANE_STALE };

typedef struct {
    stCutinTrack t[NC_CUTIN_MAX_TRACKS];
    int rose;             /* some track turned ON in this frame */
    uint64_t now_us;      /* displayed-frame time of the current frame */
    uint64_t last_disp_us;
    float    frame_dt_us; /* smoothed displayed-frame interval (0 = unknown yet) */
    /* ego lane-change detection */
    int   ref_valid;      /* ref_cx/ref_w hold the last valid lane at NC_CUTIN_REF_Y */
    float ref_cx, ref_w;
    uint64_t hold_until;  /* decisions paused until this time */
    stCutinRecent recent[NC_CUTIN_RECENT];
#ifdef NC_CUTIN_STATS
    /* why no warning: counts over NC_CUTIN_STATS_MS */
    uint64_t st_start;
    int st_frames, st_eval, st_hold, st_far, st_lost, st_stale, st_inside_now, st_not_enter, st_cand;
    int st_held, st_lead, st_virt; /* judged on a HELD / LEAD / VIRTUAL lane */
    int st_movein;        /* candidates from the moving-in rule */
    int st_away;          /* rejected: pulling away fast */
#endif
} stCutinCh;

static stCutinCh g_cutin[NC_CUTIN_MAX_CH];

static stCutinTrack *get_track(int ch, int class_id, int track_id)
{
    stCutinTrack *free_slot = NULL;
    for (int i = 0; i < NC_CUTIN_MAX_TRACKS; i++) {
        stCutinTrack *t = &g_cutin[ch].t[i];
        if (t->used) {
            if (t->class_id == class_id && t->track_id == track_id) return t;
        } else if (free_slot == NULL) {
            free_slot = t;
        }
    }
    if (free_slot) {
        memset(free_slot, 0, sizeof(*free_slot));
        free_slot->used = 1;
        free_slot->class_id = class_id;
        free_slot->track_id = track_id;
    }
    return free_slot;
}

/* lane result older than max(CUTIN_LANE_STALE_MS, NC_CUTIN_STALE_FRAMES frame intervals) */
static int lane_is_stale(int ch, uint64_t disp_ts_us, uint64_t lts)
{
    if (lts == 0 || disp_ts_us <= lts) return 0;
    uint64_t limit = g_lane_stale_us;
    uint64_t by_fps = (uint64_t)(g_cutin[ch].frame_dt_us * NC_CUTIN_STALE_FRAMES);
    if (by_fps > limit) limit = by_fps;
    return disp_ts_us - lts > limit;
}

/* ego lane at row y within the allowed sources (CUTIN_LANE_SRC). 1 = ok */
static int ego_lane(int ch, float y, float *xl, float *xr, uint64_t *lts, int *src)
{
    int s = NC_CUTIN_LANE_MEASURED;
    if (!nc_ldw_get_ego_lane_src_x(ch, y, xl, xr, lts, &s)) return 0;
    if (s > g_lane_src_max) return 0;
    if (src) *src = s;
    return 1;
}

/* lane coordinate u at (x, y). LANE_OK or the reason it failed.
 * src (optional): lane source used */
static int lane_u(int ch, float x, float y, uint64_t disp_ts_us, float *u, int *src)
{
    float xl, xr;
    uint64_t lts = 0;
    if (!ego_lane(ch, y, &xl, &xr, &lts, src)) return LANE_LOST;
    if (lane_is_stale(ch, disp_ts_us, lts)) return LANE_STALE;
    float w = xr - xl;
    if (w < g_min_lane_w) return LANE_FAR; /* too far / unstable ratio */
    *u = (x - xl) / w;
    return LANE_OK;
}

/* ego lane width at row y (0 = none). Width ~ 1 / distance on a flat road */
static float lane_w_at(int ch, float y)
{
    float xl, xr;
    uint64_t lts = 0;
    if (!ego_lane(ch, y, &xl, &xr, &lts, NULL)) return 0.0f;
    return xr - xl;
}

/* ---------------- API ---------------- */

void nc_cutin_reset(void)
{
    memset(g_cutin, 0, sizeof(g_cutin));
}

/* Ego lane change: when our car crosses a line, the lane module switches
 * lanes and x_left/x_right jump by about one lane width in one frame.
 * Compare the lane center near the bottom with the last valid one. */
static void check_ego_lane_change(int ch, uint64_t disp_ts_us)
{
    stCutinCh *c = &g_cutin[ch];
    float xl, xr;
    uint64_t lts = 0;

    int src = NC_CUTIN_LANE_MEASURED;

    if (!ego_lane(ch, NC_CUTIN_REF_Y, &xl, &xr, &lts, &src)) return;
    if (src != NC_CUTIN_LANE_MEASURED) {
        /* held / virtual lanes do not jump; forget the reference so the
         * first measured lane after the gap is not taken as a lane change */
        c->ref_valid = 0;
        return;
    }
    if (lane_is_stale(ch, disp_ts_us, lts)) return;
    float w = xr - xl;
    if (w < g_min_lane_w) return;
    float cx = (xl + xr) * 0.5f;

    if (c->ref_valid) {
        float d = cx - c->ref_cx;
        if (d < 0.0f) d = -d;
        if (d > NC_CUTIN_LC_JUMP_RATIO * c->ref_w) {
            c->hold_until = disp_ts_us + g_hold_us;
            for (int i = 0; i < NC_CUTIN_MAX_TRACKS; i++) c->t[i].hit_cnt = 0;
            printf("[nc_cutin] ch%d ego lane change (center %.0f -> %.0f px), hold %.0f ms\n",
                   ch, (double)c->ref_cx, (double)cx, (double)g_hold_us * 0.001);
        }
    }
    c->ref_valid = 1;
    c->ref_cx = cx;
    c->ref_w = w;
}

void nc_cutin_begin_frame(int ch, uint64_t disp_ts_us)
{
    if (ch < 0 || ch >= NC_CUTIN_MAX_CH) return;
    cutin_load_config();
    stCutinCh *c = &g_cutin[ch];

    /* displayed-frame interval (same frame shown again: no update) */
    if (c->last_disp_us != 0 && disp_ts_us > c->last_disp_us) {
        float dt = (float)(disp_ts_us - c->last_disp_us);
        if (dt < 2.0e6f) /* ignore pauses (> 2 s) */
            c->frame_dt_us = c->frame_dt_us > 0.0f ? c->frame_dt_us + 0.2f * (dt - c->frame_dt_us) : dt;
    }
    if (disp_ts_us > c->last_disp_us) c->last_disp_us = disp_ts_us;
    c->now_us = disp_ts_us;

    c->rose = 0;
    for (int i = 0; i < NC_CUTIN_MAX_TRACKS; i++) {
        c->t[i].seen = 0;
        c->t[i].candidate = 0;
    }
    check_ego_lane_change(ch, disp_ts_us);
}

int nc_cutin_eval(int ch, int class_id, int track_id, const stBBox *now_box,
                  float fut_cx, float fut_cy, uint64_t disp_ts_us)
{
    if (ch < 0 || ch >= NC_CUTIN_MAX_CH || now_box == NULL || track_id < 0) return 0;
    stCutinTrack *t = get_track(ch, class_id, track_id);
    if (t == NULL) return 0; /* table full */

    /* box height growth, measured over >= NC_CUTIN_VH_MIN_DT_US (the box only
     * changes when a new detection comes in) */
    if (t->h_ref_us == 0) {
        t->h_ref = now_box->h;
        t->h_ref_us = disp_ts_us;
    } else if (disp_ts_us > t->h_ref_us && disp_ts_us - t->h_ref_us >= NC_CUTIN_VH_MIN_DT_US) {
        float dt = (float)(disp_ts_us - t->h_ref_us) * 1e-6f;
        float v = (now_box->h - t->h_ref) / dt;
        t->vh += (t->vh == 0.0f ? 1.0f : NC_CUTIN_VH_ALPHA) * (v - t->vh);
        t->h_ref = now_box->h;
        t->h_ref_us = disp_ts_us;
    }
    float fh = now_box->h + t->vh * g_horizon_ms * 1e-3f;
    if (fh < now_box->h * 0.5f) fh = now_box->h * 0.5f;
    if (fh > now_box->h * 2.0f) fh = now_box->h * 2.0f;

    /* bottom-center (where the car touches the road) now and after the horizon */
    float nx = now_box->x + now_box->w * 0.5f;
    float ny = now_box->y + now_box->h;
    float fx = fut_cx;
    float fy = fut_cy + fh * 0.5f;   /* future height: was now_box->h (bottom too high when approaching) */

    t->bx = nx; t->by = ny; t->bw = now_box->w;
#ifdef USE_LDW
    /* for the lane from the car ahead (nc_ldw picks the car ahead itself) */
    nc_ldw_feed_vehicle((uint32_t)ch, class_id, track_id, nx, ny, now_box->w,
                        class_id == 1 ? NC_CUTIN_CAR_WIDTH_M : NC_CUTIN_BIG_WIDTH_M,
                        t->on || t->hit_cnt > 0);
#endif

    float u_now = 0.0f, u_fut = 0.0f;
    int cand = 0;
#ifdef NC_CUTIN_STATS
    stCutinCh *sc = &g_cutin[ch];
    sc->st_eval++;
#endif
    if (disp_ts_us < g_cutin[ch].hold_until) {
        /* paused right after our own lane change */
#ifdef NC_CUTIN_STATS
        sc->st_hold++;
#endif
    } else {
        int src = NC_CUTIN_LANE_MEASURED;
        int r = lane_u(ch, nx, ny, disp_ts_us, &u_now, &src);
        float margin = src == NC_CUTIN_LANE_VIRTUAL ? g_virt_margin : g_margin;
        t->lane_src = src;
        /* inner bottom corner: the side of the car that faces our lane.
         * A car beside us shows its side more as we get closer, which pulls
         * the box center toward our lane; the near corner stays on the side
         * line (same perspective ray), so only lateral motion moves it. */
        float half = now_box->w * 0.5f;
        float dx = (r == LANE_OK && u_now >= 0.5f) ? -half : half;   /* right car: left corner */
        float e_now = 0.0f, e_fut = 0.0f;
        int edge_ok = 1;
        if (r == LANE_OK && g_point != NC_CUTIN_POINT_CENTER) {
            int re = lane_u(ch, nx + dx, ny, disp_ts_us, &e_now, NULL);
            if (re == LANE_OK) re = lane_u(ch, fx + dx, fy, disp_ts_us, &e_fut, NULL);
            if (re == LANE_OK) {
                int e_in_now = (e_now >= 0.0f && e_now <= 1.0f);
                int e_in_fut = (e_fut >= margin && e_fut <= 1.0f - margin);
                edge_ok = e_in_now || e_in_fut;
            } else {
                edge_ok = 0;
            }
            if (g_point == NC_CUTIN_POINT_EDGE) {   /* corner only (test) */
                nx += dx; fx += dx;
                u_now = e_now;
                edge_ok = 1;
                if (re != LANE_OK) r = re;
            }
        }
        if (r == LANE_OK) r = lane_u(ch, fx, fy, disp_ts_us, &u_fut, NULL);
        if (r == LANE_OK) {
            int outside_now = (u_now < 0.0f || u_now > 1.0f);
            int inside_fut  = (u_fut >= margin && u_fut <= 1.0f - margin);
            int est = (src == NC_CUTIN_LANE_LEAD || src == NC_CUTIN_LANE_VIRTUAL);
            int movein = 0;
            if (est && !outside_now && inside_fut) {
                /* moving-in rule (see nc_cutin.h): near a side, moving to the centre */
                movein = (u_now > 1.0f - g_est_edge && u_now - u_fut >= g_est_move) ||
                         (u_now < g_est_edge        && u_fut - u_now >= g_est_move);
            }
#ifdef NC_CUTIN_STATS
            if (src == NC_CUTIN_LANE_HELD)    sc->st_held++;
            if (src == NC_CUTIN_LANE_LEAD)    sc->st_lead++;
            if (src == NC_CUTIN_LANE_VIRTUAL) sc->st_virt++;
            if (movein && edge_ok)            sc->st_movein++;
#endif
            /* BOTH: the center says "entering" and the near corner is already
             * in our lane or will enter (a car beside us that only looks like it
             * is coming in because its side shows more is rejected) */
            /* two signs of pulling away: the box shrinks (measured over >= 150 ms,
             * so it is late at the start) and the predicted bottom row is much
             * nearer the horizon (lane there narrower; same prediction as the
             * position, so no lag) */
            if (g_away_ratio > 0.0f) {
                float wn = lane_w_at(ch, ny), wf = lane_w_at(ch, fy);
                if (fh < g_away_ratio * now_box->h || (wn > 0.0f && wf > 0.0f && wf < NC_CUTIN_AWAY_ROW_RATIO * wn)) {
                    t->away_until = disp_ts_us + NC_CUTIN_AWAY_HOLD_US;
#ifdef NC_CUTIN_STATS
                    printf("[nc_cutin] ch%d away class:%d id:%d box h %.0f -> %.0f (%.2f, vh %.0f px/s) row lane %.0f -> %.0f (%.2f) u_now %.2f u_fut %.2f\n",
                           ch, class_id, track_id, (double)now_box->h, (double)fh, (double)(fh / now_box->h),
                           (double)t->vh, (double)wn, (double)wf, wn > 0.0f ? (double)(wf / wn) : 0.0,
                           (double)u_now, (double)u_fut);
#endif
                }
            }
            /* kept for a while: the position prediction lags the box size */
            int away = disp_ts_us < t->away_until;
            cand = (outside_now || movein) && inside_fut && edge_ok && !away;
#ifdef NC_CUTIN_STATS
            if ((outside_now || movein) && inside_fut && edge_ok && away) sc->st_away++;
#endif
#ifdef NC_CUTIN_STATS
            if (cand) {
                /* debug: why this track is a candidate (lane at the car's bottom row) */
                float dxl = 0.0f, dxr = 0.0f;
                uint64_t dts = 0;
                ego_lane(ch, ny, &dxl, &dxr, &dts, NULL);
                printf("[nc_cutin] ch%d cand class:%d id:%d u_now:%.2f u_fut:%.2f edge:%.2f>%.2f box:%.0f,%.0f,%.0f,%.0f "
                       "fut:%.0f,%.0f lane@%.0f:%.0f-%.0f\n", ch, class_id, track_id,
                       (double)u_now, (double)u_fut, (double)e_now, (double)e_fut, (double)now_box->x, (double)now_box->y,
                       (double)now_box->w, (double)now_box->h, (double)fx, (double)fy,
                       (double)ny, (double)dxl, (double)dxr);
            }
            if (!outside_now && !movein) sc->st_inside_now++;
            else if (!inside_fut || !edge_ok) sc->st_not_enter++;
            else                  sc->st_cand++;
#endif
        }
#ifdef NC_CUTIN_STATS
        else if (r == LANE_FAR)   sc->st_far++;
        else if (r == LANE_STALE) sc->st_stale++;
        else                      sc->st_lost++;
#endif
        /* no lane at this height (far car, lane lost) -> not a candidate */
    }

    t->seen = 1;
    t->candidate = cand;
    return t->on;  /* state from previous frames; updated in end_frame */
}

static int near_box(float ax, float ay, float aw, float bx, float by, float bw)
{
    float d = aw > bw ? aw : bw;          /* about one vehicle width */
    float dx = ax - bx, dy = ay - by;
    return dx * dx + dy * dy < d * d;
}

static stCutinRecent *find_recent(stCutinCh *c, const stCutinTrack *t, uint64_t now)
{
    for (int i = 0; i < NC_CUTIN_RECENT; i++) {
        stCutinRecent *r = &c->recent[i];
        if (!r->used || now - r->t > g_dup_us) continue;
        /* the same track turning ON again soon is the same vehicle too
         * (board 11/06: virtual lane, same car ON twice 1.5 s apart) */
        if (r->class_id == t->class_id && r->track_id == t->track_id) return r;
        if (near_box(t->bx, t->by, t->bw, r->bx, r->by, r->bw)) return r;
    }
    return NULL;
}

static void remember_recent(stCutinCh *c, const stCutinTrack *t, uint64_t now)
{
    stCutinRecent *slot = NULL, *oldest = &c->recent[0];
    for (int i = 0; i < NC_CUTIN_RECENT; i++) {
        stCutinRecent *r = &c->recent[i];
        if (r->used && r->class_id == t->class_id && r->track_id == t->track_id) { slot = r; break; }
        if (!r->used || r->t < oldest->t) oldest = r;
    }
    if (slot == NULL) slot = oldest;
    slot->used = 1;
    slot->class_id = t->class_id;
    slot->track_id = t->track_id;
    slot->t = now;
    slot->bx = t->bx; slot->by = t->by; slot->bw = t->bw;
}

int nc_cutin_end_frame(int ch)
{
    if (ch < 0 || ch >= NC_CUTIN_MAX_CH) return 0;
    stCutinCh *c = &g_cutin[ch];
    uint64_t now = c->now_us;

    for (int i = 0; i < NC_CUTIN_MAX_TRACKS; i++) {
        stCutinTrack *t = &c->t[i];
        if (!t->used) continue;

        if (t->candidate) {
            if (t->hit_cnt == 0) { t->hit_since = now; t->need_on_ms = 0; }
            {
                uint64_t need = t->lane_src == NC_CUTIN_LANE_VIRTUAL ? g_virt_on_us
                              : t->lane_src == NC_CUTIN_LANE_LEAD    ? g_lead_on_us : g_on_us;
                if ((int)(need / 1000u) > t->need_on_ms) t->need_on_ms = (int)(need / 1000u);
            }
            t->hit_cnt++;
            t->miss_since = 0;
            uint64_t on_us = (uint64_t)t->need_on_ms * 1000u;
            if (on_us < g_on_us) on_us = g_on_us;
            if (!t->on && t->hit_cnt >= NC_CUTIN_MIN_ON_HITS && now - t->hit_since >= on_us) {
                t->on = 1;
                t->on_since = now;
                stCutinRecent *dup = find_recent(c, t, now);
                if (dup) {
                    printf("[nc_cutin] ch%d cut-in ON  class:%d id:%d (same vehicle as class:%d id:%d, no new trigger)\n",
                           ch, t->class_id, t->track_id, dup->class_id, dup->track_id);
                } else {
                    c->rose = 1;
                    printf("[nc_cutin] ch%d cut-in ON  class:%d id:%d (candidate %.0f ms, %d frames, lane %s)\n",
                           ch, t->class_id, t->track_id, (double)(now - t->hit_since) * 0.001, t->hit_cnt,
                           t->lane_src == NC_CUTIN_LANE_VIRTUAL ? "virtual"
                           : t->lane_src == NC_CUTIN_LANE_LEAD ? "lead"
                           : t->lane_src == NC_CUTIN_LANE_HELD ? "held" : "measured");
                }
            }
        } else {
            t->hit_cnt = 0;
            if (t->miss_since == 0) t->miss_since = now;
            if (t->on && now - t->miss_since >= g_off_us && now - t->on_since >= g_min_on_us) {
                t->on = 0;
                printf("[nc_cutin] ch%d cut-in OFF class:%d id:%d\n", ch, t->class_id, t->track_id);
            }
            /* forget tracks that are gone or long quiet (unseen tracks also count as misses) */
            if (!t->on && now - t->miss_since >= 4 * g_off_us) {
                memset(t, 0, sizeof(*t));
            }
        }
    }
    for (int i = 0; i < NC_CUTIN_MAX_TRACKS; i++) {
        if (c->t[i].used && c->t[i].on) remember_recent(c, &c->t[i], now);
    }

#ifdef NC_CUTIN_STATS
    c->st_frames++;
    if (c->st_start == 0) c->st_start = now;
    if (now - c->st_start >= (uint64_t)(NC_CUTIN_STATS_MS * 1000.0f)) {
        /* eval: predicted vehicles judged. no_lane = far + lost + stale (see nc_cutin.h),
         * inside: already in ego lane, not_enter: outside and stays outside, cand: cut-in candidate */
        printf("[nc_cutin] ch%d stats/%dfr eval:%d hold:%d no_lane:%d far:%d lost:%d stale:%d "
               "inside:%d not_enter:%d cand:%d on:%d held:%d lead:%d virtual:%d movein:%d away:%d fps:%.1f\n",
               ch, c->st_frames, c->st_eval, c->st_hold, c->st_far + c->st_lost + c->st_stale,
               c->st_far, c->st_lost, c->st_stale, c->st_inside_now, c->st_not_enter, c->st_cand,
               nc_cutin_active_count(ch), c->st_held, c->st_lead, c->st_virt, c->st_movein, c->st_away,
               c->frame_dt_us > 0.0f ? 1.0e6 / (double)c->frame_dt_us : 0.0);
        c->st_start = now;
        c->st_frames = c->st_eval = c->st_hold = 0;
        c->st_far = c->st_lost = c->st_stale = 0;
        c->st_inside_now = c->st_not_enter = c->st_cand = 0;
        c->st_held = c->st_lead = c->st_virt = c->st_movein = c->st_away = 0;
    }
#endif
    return c->rose;
}

int nc_cutin_active_count(int ch)
{
    if (ch < 0 || ch >= NC_CUTIN_MAX_CH) return 0;
    int n = 0;
    for (int i = 0; i < NC_CUTIN_MAX_TRACKS; i++) {
        if (g_cutin[ch].t[i].used && g_cutin[ch].t[i].on) n++;
    }
    return n;
}
