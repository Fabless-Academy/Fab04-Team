#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

#include "nc_ldw.h"

/*
 * Shared functions from teammates (lines marked "// [TEAM]").
 * Until the real nc_meas.c / nc_model.c / nc_rec.c are merged, nc_c_stub.c
 * in wayland_npu_app provides them.
 *   nc_meas_get_latency_ms() : capture -> NPU result latency (ms)
 *   nc_rec_trigger(type)     : start recording, 0 = lane departure
 *   nc_model_get_mode()      : MODE_NORMAL / MODE_LIGHT / MODE_SKIP
 */
#include "nc_meas.h"     // [TEAM]
#include "nc_rec.h"      // [TEAM]
#include "nc_model.h"    // [TEAM]

#define NC_REC_TYPE_LANE_DEPARTURE  (0)

/* Print lane_class of every lane once a second (to confirm 0/1/2 meaning
 * on the board). Set to 0 when done. */
#define NC_LDW_LOG_LANE_CLASS       (0)

const stNcLdwEgoLane NC_LDW_EGO_LANE = {
    NC_LDW_LEFT_EGO_LANE_BOUNDARY_INDEX,
    NC_LDW_RIGHT_EGO_LANE_BOUNDARY_INDEX
};

#define LDW_MIN_FIT_POINTS          (3)
#define LDW_MAX_FIT_RMS_RATIO       (0.02f)   // max fit residual, ratio of img_w
#define LDW_MIN_LANE_PX_RATIO       (0.15f)   // min lane width at ref row, ratio of img_w
#define LDW_MAX_LANE_PX_RATIO       (1.50f)
#define LDW_CALIB_ALPHA             (0.10f)   // EMA weight for k / vp_y
#define LDW_VEL_ALPHA               (0.30f)   // EMA weight for lateral velocity
#define LDW_MAX_DT_S                (1.0f)
#define LDW_CLASS_ALPHA             (0.15f)   // EMA weight for the dashed/solid vote (~0.5 s)
#define LDW_CENTER_ALPHA            (0.01f)   // EMA weight for the usual lane centre (~7 s at 15 fps)
#define LDW_CENTER_MAX_VEL_MPS      (0.20f)   // auto_center learns only below this lateral speed
#define LDW_CENTER_DEADBAND_M       (0.20f)   // auto_center corrects only the part beyond this
#define LDW_NO_LANE_HINT_S          (10.0f)   // no lane this long from start -> camera hint
#define LDW_HELD_MAX_VEL_MPS        (1.0f)    // lateral speed used to move a held lane (clamped)
#define LDW_HELD_VEL_DEADBAND_MPS   (0.2f)    // ... minus this (the speed estimate is noisy)

/* LEAD lane: the car ahead in our lane */
#define LDW_LEAD_MAX_VEH            (32)      // vehicles per frame looked at
#define LDW_LEAD_U_MIN              (0.25f)   // pick: bottom-center inside this part of the lane
#define LDW_LEAD_U_MAX              (0.75f)
#define LDW_LEAD_EST_U_MIN          (0.40f)   // pick on a held / virtual lane: well centred only
#define LDW_LEAD_EST_U_MAX          (0.60f)
#define LDW_LEAD_DROP_U_MIN         (0.10f)   // drop: it left our lane (on the virtual lane)
#define LDW_LEAD_DROP_U_MAX         (0.90f)
#define LDW_LEAD_MIN_ROW_RATIO      (0.5f)    // close enough: its row at least this far below the
                                              // vanishing row, relative to ref_y (lane at least half
                                              // as wide as on ref_y; any camera FOV). A far car (86 px
                                              // box) gave a lane 40% off; 350 px fixed failed on a
                                              // wide-angle camera (lane 325 px on the bottom row)
#define LDW_LEAD_LOST_S             (0.5f)    // not seen this long -> no lead
#define LDW_LEAD_WIDTH_JUMP         (0.30f)   // box width change > 30% in one step -> another car
#define LDW_LEAD_ALPHA              (0.10f)   // EMA for its place and width in the lane
#define LDW_LEAD_MIN_LEARN          (5)       // learned samples before they are used
#define LDW_LEAD_K_MIN_RATIO        (0.75f)   // lane from the lead must be within this ratio of the
#define LDW_LEAD_K_MAX_RATIO        (1.30f)   // learned lane width, else it is not trusted
#define LDW_LEAD_RATIO_MIN          (0.40f)   // learned box width / lane width must look like a car
#define LDW_LEAD_RATIO_MAX          (0.85f)   // (board: 0.49 - 0.74; boxes include mirrors and margin,
                                              // city lanes are ~3.0 m), else only its row is used

static float us_to_s(uint64_t us)
{
    return (float)((double)us / 1000000.0);
}

static uint64_t s_to_us(float s)
{
    return (uint64_t)((double)s * 1000000.0);
}

/* Least-squares fit x = a*y + b using points with y_min <= y <= y_max. */
static int fit_lane_line(const stLaneDet *ln, float y_min, float y_max, float max_rms,
                         float *a, float *b)
{
    double sy = 0.0, syy = 0.0, sx = 0.0, sxy = 0.0;
    int n = 0;
    int cnt = ln->point_cnt;

    if (cnt > MAX_POINT_CNT_OF_LANE) cnt = MAX_POINT_CNT_OF_LANE;

    for (int i = 0; i < cnt; i++) {
        double x = (double)ln->point[i].x;
        double y = (double)ln->point[i].y;
        if (y < (double)y_min || y > (double)y_max) continue;
        sy += y; syy += y * y; sx += x; sxy += x * y;
        n++;
    }
    if (n < LDW_MIN_FIT_POINTS) return -1;

    double dn = (double)n;
    double denom = dn * syy - sy * sy;
    if (fabs(denom) < 1e-6) return -1;

    double fa = (dn * sxy - sy * sx) / denom;
    double fb = (sx - fa * sy) / dn;

    double err = 0.0;
    for (int i = 0; i < cnt; i++) {
        double x = (double)ln->point[i].x;
        double y = (double)ln->point[i].y;
        if (y < (double)y_min || y > (double)y_max) continue;
        double r = x - (fa * y + fb);
        err += r * r;
    }
    if (sqrt(err / dn) > (double)max_rms) return -1;

    *a = (float)fa;
    *b = (float)fb;
    return 0;
}

/* side 0 = left, 1 = right. Unknown class (-1) keeps the previous vote. */
static void ldw_vote_dashed(stNcLdwContext *ctx, int side, int lane_class)
{
    float *v = &ctx->dashed_score[side];
    if (lane_class < 0) return;
    float d = lane_class == ctx->cfg.dashed_class ? 1.0f : 0.0f;
    if (*v < 0.0f) *v = d;
    else           *v += LDW_CLASS_ALPHA * (d - *v);
}

void nc_ldw_default_config(stNcLdwConfig *cfg, float img_w, float img_h)
{
    memset(cfg, 0, sizeof(*cfg));
    cfg->img_w           = img_w;
    cfg->img_h           = img_h;
    cfg->lane_width_m    = NC_LDW_DEFAULT_LANE_WIDTH_M;
    cfg->vehicle_width_m = 1.85f;
    cfg->cam_x_px        = img_w * 0.5f;
    cfg->cam_offset_m    = 0.0f;
    cfg->ref_y_px        = img_h * 0.80f;
    cfg->fit_y_min_px    = img_h * 0.55f;
    cfg->focal_px        = 0.0f;
    cfg->warn_margin_m   = 0.50f;   // warn when the car edge is closer than this to a line
    cfg->reaction_time_s = 0.8f;    // TLC threshold = 0.8 + latency + 0.7
    cfg->tlc_margin_s    = 0.7f;
    cfg->tlc_max_dist_m  = 1.5f;
    cfg->min_lat_vel_mps = 0.10f;
    cfg->dashed_class    = 2;       // ufld: 0 = none, 1 = solid, 2 = dashed (confirmed on board 11/05)
    cfg->trigger_frames  = 2;
    cfg->hold_s          = 1.0f;
    cfg->lost_timeout_s  = 0.5f;
    cfg->ego_fresh_s     = 0.2f;
    cfg->ego_hold_s      = 1.0f;    // board 11/06: a 1 s old held lane was ~10% off
    cfg->ego_virtual     = 1;
    cfg->ego_lead        = 1;
    cfg->lead_min_s      = 1.0f;
    cfg->auto_center       = 1;
    cfg->auto_center_max_m = 0.6f;
    cfg->auto_center_tau_s = 20.0f;
    cfg->auto_center_min_s = 10.0f;
}

void nc_ldw_init(stNcLdwContext *ctx, const stNcLdwConfig *cfg)
{
    memset(ctx, 0, sizeof(*ctx));
    ctx->cfg = *cfg;
    ctx->res.state = NC_LDW_STATE_INVALID;
    ctx->res.tlc_s = -1.0f;
    ctx->dashed_score[0] = -1.0f;
    ctx->dashed_score[1] = -1.0f;
}

void nc_ldw_set_turn_signal(stNcLdwContext *ctx, int left_on, int right_on)
{
    ctx->turn_left = left_on ? 1 : 0;
    ctx->turn_right = right_on ? 1 : 0;
}

static void ldw_reset_tracking(stNcLdwContext *ctx)
{
    ctx->has_last = 0;
    ctx->left_cnt = 0;
    ctx->right_cnt = 0;
    ctx->warn_until_us = 0;
    ctx->res.lat_vel_mps = 0.0f;
    ctx->res.tlc_s = -1.0f;
}

static void ldw_check_lost(stNcLdwContext *ctx, uint64_t now_us)
{
    if (ctx->res.state == NC_LDW_STATE_INVALID) return;
    if (now_us > ctx->last_wall_us &&
        us_to_s(now_us - ctx->last_wall_us) > ctx->cfg.lost_timeout_s) {
        ctx->res.state = NC_LDW_STATE_INVALID;
        ctx->res.level = NC_LDW_LEVEL_NONE;
        ctx->res.valid = 0;
        ldw_reset_tracking(ctx);
    }
}

void nc_ldw_suspend(stNcLdwContext *ctx)
{
    ctx->res.state = NC_LDW_STATE_INVALID;
    ctx->res.level = NC_LDW_LEVEL_NONE;
    ctx->res.valid = 0;
    ldw_reset_tracking(ctx);
}

static void ldw_update_calibration(stNcLdwContext *ctx, float aL, float bL, float aR, float bR)
{
    const stNcLdwConfig *c = &ctx->cfg;
    float k = aR - aL;
    if (k <= 1e-4f) return;

    float vp_y = (bL - bR) / k;
    if (vp_y < c->img_h * 0.05f || vp_y > c->ref_y_px - c->img_h * 0.05f) return;

    float w_ref = k * (c->ref_y_px - vp_y);
    if (w_ref < c->img_w * LDW_MIN_LANE_PX_RATIO || w_ref > c->img_w * LDW_MAX_LANE_PX_RATIO) return;

    float vp_x = aL * vp_y + bL;                                    // where both lines meet
    float cx   = 0.5f * ((aL + aR) * c->ref_y_px + bL + bR);        // lane centre on ref row

    if (!ctx->calib_valid) {
        ctx->calib_k = k;
        ctx->calib_vp_y = vp_y;
        ctx->calib_vp_x = vp_x;
        ctx->calib_cx_ref = cx;
        ctx->calib_valid = 1;
    } else {
        ctx->calib_k    += LDW_CALIB_ALPHA * (k - ctx->calib_k);
        ctx->calib_vp_y += LDW_CALIB_ALPHA * (vp_y - ctx->calib_vp_y);
        ctx->calib_vp_x += LDW_CALIB_ALPHA * (vp_x - ctx->calib_vp_x);
        /* slow: the usual place of the car in its lane (camera mount offset),
         * not the current one. A lane change jumps cx for a moment only. */
        ctx->calib_cx_ref += LDW_CENTER_ALPHA * (cx - ctx->calib_cx_ref);
    }
}

int nc_ldw_update(stNcLdwContext *ctx, const stLaneDet *lanes, int lane_num,
                  uint64_t data_time_us, uint64_t now_us)
{
    const stNcLdwConfig *c = &ctx->cfg;
    stNcLdwResult *r = &ctx->res;
    float aL = 0.0f, bL = 0.0f, aR = 0.0f, bR = 0.0f;
    int left_ok = 0, right_ok = 0;
    const uint64_t t = data_time_us ? data_time_us : now_us;   // time of this lane data

    /* NULL = no new lane result since the last call: only check the timeout.
     * (Identical points are still a new result: UFLD x is quantised, so a
     *  steady drive often gives exactly the same points twice.) */
    if (lanes == NULL) {
        ldw_check_lost(ctx, now_us);
        return 0;
    }
    ctx->last_update_us = now_us;

    float max_rms = c->img_w * LDW_MAX_FIT_RMS_RATIO;
    float y = c->ref_y_px;

    /* Ego-lane boundaries are lane slots 1 and 2, but left/right is decided by
     * position, not by slot number: the line with the smaller x on the
     * reference row is the left one. A single line is left when it is left
     * of the camera column. */
    {
        const int slot[2] = { (int)NC_LDW_EGO_LANE.left_boundary_index,
                              (int)NC_LDW_EGO_LANE.right_boundary_index };
        int   ok[2] = {0, 0}, cls[2] = {-1, -1};
        float fa[2] = {0.0f, 0.0f}, fb[2] = {0.0f, 0.0f};

        for (int i = 0; i < 2; i++) {
            if (lane_num > slot[i] &&
                fit_lane_line(&lanes[slot[i]], c->fit_y_min_px, c->img_h, max_rms, &fa[i], &fb[i]) == 0) {
                ok[i] = 1;
                cls[i] = lanes[slot[i]].lane_class;
            }
        }

        int l = -1, rgt = -1;     // index (0/1) of the left / right line
        if (ok[0] && ok[1]) {
            int first_is_left = (fa[0] * y + fb[0]) <= (fa[1] * y + fb[1]);
            l   = first_is_left ? 0 : 1;
            rgt = first_is_left ? 1 : 0;
        } else if (ok[0] || ok[1]) {
            int i = ok[0] ? 0 : 1;
            if (fa[i] * y + fb[i] < c->cam_x_px) l = i;
            else                                  rgt = i;
        }

        if (l >= 0)   { left_ok = 1;  aL = fa[l];   bL = fb[l]; }
        if (rgt >= 0) { right_ok = 1; aR = fa[rgt]; bR = fb[rgt]; }
        r->left_class  = l >= 0 ? cls[l] : -1;
        r->right_class = rgt >= 0 ? cls[rgt] : -1;
    }

    /* lane_class flickers between solid and dashed on the same line, so the
     * warning level uses a vote over recent frames, not the current frame. */
    ldw_vote_dashed(ctx, 0, r->left_class);
    ldw_vote_dashed(ctx, 1, r->right_class);

    float xl = aL * y + bL;
    float xr = aR * y + bR;

    if (left_ok && right_ok) {
        float w = xr - xl;
        if (w < c->img_w * LDW_MIN_LANE_PX_RATIO || w > c->img_w * LDW_MAX_LANE_PX_RATIO) {
            left_ok = right_ok = 0;   // inconsistent pair
        } else {
            ldw_update_calibration(ctx, aL, bL, aR, bR);
        }
    }

    /* One line only: infer the other from the calibrated lane width
     * w(y) = k*(y - vp_y), i.e. x_other = x_known +/- k*(y - vp_y). */
    if (left_ok != right_ok) {
        if (!ctx->calib_valid) {
            left_ok = right_ok = 0;
        } else {
            float k = ctx->calib_k, vp = ctx->calib_vp_y;
            if (left_ok) { aR = aL + k; bR = bL - k * vp; }
            else         { aL = aR - k; bL = bR + k * vp; }
            xl = aL * y + bL;
            xr = aR * y + bR;
        }
    }

    if (!left_ok && !right_ok) {
        ldw_check_lost(ctx, now_us);
        return 1;
    }

    /* Both lines are 'lane_width_m' apart on this row -> metres per pixel. */
    float s = c->lane_width_m / (xr - xl);
    float half_vw = c->vehicle_width_m * 0.5f;
    float veh_x_raw = c->cam_x_px - c->cam_offset_m / s;
    float raw_off = (veh_x_raw - 0.5f * (xl + xr)) * s;

    /* Usual place in the lane: a camera that is not on the car's centre line
     * (phone on the driver side) makes the car look off-centre all the time and
     * LDW warns on that side again and again (board 11/06, night-curve video:
     * 13 warnings, offset -0.4 ~ -0.9 m). Drivers keep to the centre on
     * average, so the slow average of the offset during steady driving (both
     * lines, small lateral speed, not over a line) is the camera offset. A real
     * departure takes seconds, the learning ~20 s, so it is not learned away. */
    if (c->auto_center) {
        float dt_c = ctx->has_last && t > ctx->last_valid_us ? us_to_s(t - ctx->last_valid_us) : 0.0f;
        int steady = left_ok && right_ok && dt_c > 0.0f && dt_c <= LDW_MAX_DT_S &&
                     fabsf(r->lat_vel_mps) < LDW_CENTER_MAX_VEL_MPS &&
                     fabsf(raw_off) < 0.5f * c->lane_width_m;
        /* (not "no warning": an offset camera warns all the time and would never learn) */
        if (steady) {
            float a = dt_c / c->auto_center_tau_s;
            if (ctx->center_learn_s <= 0.0f) ctx->center_off_m = raw_off;
            else ctx->center_off_m += (a > 1.0f ? 1.0f : a) * (raw_off - ctx->center_off_m);
            ctx->center_learn_s += dt_c;
        }
        if (ctx->center_learn_s >= c->auto_center_min_s) {
            /* up to LDW_CENTER_DEADBAND_M is a driving habit, not the camera:
             * keep it (board 11/06: default video +0.21 m; removing it hid the
             * right solid-line warnings at 20-28 s that were checked correct) */
            float m = ctx->center_off_m;
            m = m >  LDW_CENTER_DEADBAND_M ? m - LDW_CENTER_DEADBAND_M
              : m < -LDW_CENTER_DEADBAND_M ? m + LDW_CENTER_DEADBAND_M : 0.0f;
            if (m >  c->auto_center_max_m) m =  c->auto_center_max_m;
            if (m < -c->auto_center_max_m) m = -c->auto_center_max_m;
            ctx->center_used_m = m;
            if (fabsf(m - ctx->center_logged_m) >= 0.1f) {
                printf("[LDW] usual place in lane %+.2fm, correction %+.2fm (camera offset)\n",
                       (double)ctx->center_off_m, (double)m);
                ctx->center_logged_m = m;
            }
        }
    }
    float veh_x = veh_x_raw - ctx->center_used_m / s;

    r->valid          = 1;
    r->left_detected  = left_ok;
    r->right_detected = right_ok;
    r->x_left_px      = xl;
    r->x_right_px     = xr;
    r->vehicle_x_px   = veh_x;
    r->m_per_px       = s;
    r->offset_m       = (veh_x - 0.5f * (xl + xr)) * s;
    r->center_corr_m  = ctx->center_used_m;
    r->dist_left_m    = (veh_x - xl) * s - half_vw;
    r->dist_right_m   = (xr - veh_x) * s - half_vw;
    r->vp_y_px        = ctx->calib_valid ? ctx->calib_vp_y : 0.0f;
    r->cam_height_m   = ctx->calib_valid ? c->lane_width_m / ctx->calib_k : 0.0f;
    r->left_a         = aL;
    r->left_b         = bL;
    r->right_a        = aR;
    r->right_b        = bR;
    r->data_time_us   = t;

    /* Lateral velocity from the offset change. A jump of about one lane width
     * means the network re-indexed the lanes (a line was crossed): remove the
     * jump so the velocity stays continuous, and restart the warning logic. */
    int lane_changed = 0;
    float dt = ctx->has_last && t > ctx->last_valid_us ? us_to_s(t - ctx->last_valid_us) : 0.0f;
    if (ctx->has_last && dt > 0.0f && dt <= LDW_MAX_DT_S) {
        /* raw offset (before the usual-place correction): a correction step
         * is not a movement (board 11/09: each step gave a 0.9-1.2 m/s spike
         * and a false TLC warning) */
        float d = raw_off - ctx->last_offset_m;
        if (fabsf(d) > c->lane_width_m * 0.5f) {
            lane_changed = 1;
            d -= copysignf(c->lane_width_m, d);
            ctx->left_cnt = 0;
            ctx->right_cnt = 0;
            ctx->warn_until_us = 0;
            ctx->dashed_score[0] = -1.0f;   // the lines changed sides
            ctx->dashed_score[1] = -1.0f;
        }
        r->lat_vel_mps += LDW_VEL_ALPHA * (d / dt - r->lat_vel_mps);
    } else {
        r->lat_vel_mps = 0.0f;
    }
    ctx->has_last = 1;
    ctx->last_offset_m = raw_off;
    ctx->last_valid_us = t;
    ctx->last_wall_us = now_us;

    /* TLC threshold = reaction time + measured latency + margin.
     * The lane result is already 'latency' old when we see it, so the
     * driver has that much less time left: it is added to the threshold. */
    float latency_ms = 0.0f;
    latency_ms = nc_meas_get_latency_ms();     // [TEAM]
    if (latency_ms < 0.0f) latency_ms = 0.0f;
    r->latency_ms   = latency_ms;
    r->tlc_thresh_s = c->reaction_time_s + latency_ms * 0.001f + c->tlc_margin_s;

    float dl = r->dist_left_m;
    float dr = r->dist_right_m;

    r->tlc_s = -1.0f;
    if (r->lat_vel_mps > c->min_lat_vel_mps)
        r->tlc_s = dr > 0.0f ? dr / r->lat_vel_mps : 0.0f;
    else if (r->lat_vel_mps < -c->min_lat_vel_mps)
        r->tlc_s = dl > 0.0f ? dl / -r->lat_vel_mps : 0.0f;

    /* Warning decision.
     * A side warns when the vehicle is close to (or over) that line and not
     * moving away from it, or when TLC < tlc_thresh_s while approaching it.
     * trigger_frames debounces the start; hold_s is the minimum display time. */
    int moving_left  = r->lat_vel_mps < -c->min_lat_vel_mps;
    int moving_right = r->lat_vel_mps >  c->min_lat_vel_mps;
    int cond_l = !ctx->turn_left && !lane_changed &&
                 ((dl < c->warn_margin_m && !moving_right) ||
                  (moving_left && r->tlc_s >= 0.0f && r->tlc_s < r->tlc_thresh_s &&
                   dl < c->tlc_max_dist_m));
    int cond_r = !ctx->turn_right && !lane_changed &&
                 ((dr < c->warn_margin_m && !moving_left) ||
                  (moving_right && r->tlc_s >= 0.0f && r->tlc_s < r->tlc_thresh_s &&
                   dr < c->tlc_max_dist_m));

    ctx->left_cnt  = cond_l ? ctx->left_cnt + 1 : 0;
    ctx->right_cnt = cond_r ? ctx->right_cnt + 1 : 0;

    int fire_l = ctx->left_cnt  >= c->trigger_frames;
    int fire_r = ctx->right_cnt >= c->trigger_frames;
    eNcLdwState next = NC_LDW_STATE_NORMAL;

    if (fire_l && fire_r)
        next = dl <= dr ? NC_LDW_STATE_WARN_LEFT : NC_LDW_STATE_WARN_RIGHT;
    else if (fire_l)
        next = NC_LDW_STATE_WARN_LEFT;
    else if (fire_r)
        next = NC_LDW_STATE_WARN_RIGHT;
    else if (!lane_changed && t < ctx->warn_until_us &&
             (r->state == NC_LDW_STATE_WARN_LEFT || r->state == NC_LDW_STATE_WARN_RIGHT))
        next = r->state;   /* minimum display time */

    int prev_warn = (r->state == NC_LDW_STATE_WARN_LEFT || r->state == NC_LDW_STATE_WARN_RIGHT);
    int warn      = (next == NC_LDW_STATE_WARN_LEFT || next == NC_LDW_STATE_WARN_RIGHT);

    if (warn && next != r->state)
        ctx->warn_until_us = t + s_to_us(c->hold_s);   // new warning or side switch

    /* Recording: only when the warning turns on (off -> on), not every frame
     * and not on a left <-> right switch. */
    if (!prev_warn && warn) {
        nc_rec_trigger(NC_REC_TYPE_LANE_DEPARTURE);   // [TEAM]
    }
    r->state = next;

    /* Strength: crossing a dashed line is allowed (lane change) -> weak alert,
     * solid or unknown line -> strong warning. */
    if (next == NC_LDW_STATE_WARN_LEFT)
        r->level = ctx->dashed_score[0] >= 0.5f ? NC_LDW_LEVEL_WEAK : NC_LDW_LEVEL_STRONG;
    else if (next == NC_LDW_STATE_WARN_RIGHT)
        r->level = ctx->dashed_score[1] >= 0.5f ? NC_LDW_LEVEL_WEAK : NC_LDW_LEVEL_STRONG;
    else
        r->level = NC_LDW_LEVEL_NONE;

    return 1;
}

int nc_ldw_pixel_to_road(const stNcLdwContext *ctx, float u, float v, float *x_m, float *z_m)
{
    const stNcLdwConfig *c = &ctx->cfg;
    if (!ctx->calib_valid) return -1;

    float w = ctx->calib_k * (v - ctx->calib_vp_y);   // lane width in px on row v
    if (w <= 1.0f) return -1;

    float s = c->lane_width_m / w;                     // metres per pixel on row v
    if (x_m) *x_m = (u - c->cam_x_px) * s + c->cam_offset_m;
    if (z_m) *z_m = c->focal_px > 0.0f ? c->focal_px * s : -1.0f;
    return 0;
}

const char *nc_ldw_state_name(eNcLdwState state)
{
    switch (state) {
    case NC_LDW_STATE_NORMAL:     return "NORMAL";
    case NC_LDW_STATE_WARN_LEFT:  return "WARN_LEFT";
    case NC_LDW_STATE_WARN_RIGHT: return "WARN_RIGHT";
    default:                      return "INVALID";
    }
}

/*
********************************************************************************
*               APPLICATION LAYER (app_wayland_npu)
********************************************************************************
*/
#ifdef USE_LDW
#include <pthread.h>

static stNcLdwContext g_ldw_ctx[NC_LDW_MAX_CH];     // render thread only
static int g_ldw_ch_num = 0;
static float g_ldw_canvas_w = 0.0f;
static float g_ldw_canvas_h = 0.0f;
static int g_ldw_suspended[NC_LDW_MAX_CH];

/* New-result detection: the lane result buffer is a flip-flop (2 buffers),
 * so a new result moves the readable pointer. The timestamp also changes once
 * the input timestamp is passed through. */
static const void *g_ldw_last_buf[NC_LDW_MAX_CH];
static uint64_t g_ldw_last_ts[NC_LDW_MAX_CH];

/* Ego-lane snapshot shared with other threads (cut-in prediction). */
typedef struct {
    int      valid;
    eNcLdwEgoSrc src;
    float    left_a, left_b, right_a, right_b;
    float    vp_y;
    uint64_t time_us;
} stLdwEgoSnapshot;

static stLdwEgoSnapshot g_ldw_ego[NC_LDW_MAX_CH];
static pthread_mutex_t g_ldw_ego_lock = PTHREAD_MUTEX_INITIALIZER;

/* Last measured ego lane (render thread only): base of the HELD lane. */
typedef struct {
    int      ok;
    int      both;              // both lines detected
    float    left_a, left_b, right_a, right_b;
    float    lat_vel_mps;       // our lateral speed then (+: moving right)
    uint64_t wall_us;           // when it was measured
    uint64_t data_us;           // data time of the lane result
} stLdwEgoMeas;

static stLdwEgoMeas g_ldw_meas[NC_LDW_MAX_CH];

/* Car ahead (render thread only).
 * While the lines are seen, learn where it drives in our lane and how wide it
 * is compared to the lane. When the lines are gone, our lane is rebuilt around
 * it from those two values: this follows the real lane (curves, our position,
 * road shape changes) better than the averaged virtual lane. */
typedef struct {
    int   cls, id;
    float x, y, w, width_m;
    int   cutting_in;
} stLdwVeh;

typedef struct {
    stLdwVeh v[LDW_LEAD_MAX_VEH];      // this frame (nc_ldw_feed_vehicle)
    int      n;
    int      valid;
    stLdwVeh car;                      // current lead
    uint64_t since_us, last_us;
    int      learn_n;                  // samples learned on measured lanes
    float    off_m;                    // its offset from the lane centre (m, +: right)
    float    ratio;                    // its box width / lane width on its row
} stLdwLead;

static stLdwLead g_ldw_lead[NC_LDW_MAX_CH];
static eNcLdwEgoSrc g_ldw_src_shown[NC_LDW_MAX_CH];  // for the source-change log

/* Lane-loss test (no rebuild): ignore lane results to see what cut-in does
 * without lines. Seconds from the first frame of the channel.
 *   LDW_DROP_AFTER_S=5                       : no lanes from 5 s on
 *   LDW_DROP_AFTER_S=3 LDW_DROP_EVERY_S=4 LDW_DROP_LEN_S=1 : 1 s gap every 4 s */
static float g_ldw_drop_after_s = -1.0f;
static float g_ldw_drop_every_s = 0.0f;
static float g_ldw_drop_len_s   = 0.0f;
static uint64_t g_ldw_t0_us[NC_LDW_MAX_CH];

static uint64_t ldw_now_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000u + (uint64_t)ts.tv_nsec / 1000u;
}

/* SKIP mode: the lane network is not running, so there are no new lanes. */
static int ldw_model_skip(void)
{
    return nc_model_get_mode() == MODE_SKIP;   // [TEAM]
}

/* lines x = a*y + b of a lane whose centre passes through (vx, vy) and
 * (cx, y0) and whose width is w0 on row y0 (0 at the vanishing row) */
static void ldw_lane_through(float vx, float vy, float cx, float y0, float w0, stLdwEgoSnapshot *e)
{
    float ca = (cx - vx) / (y0 - vy);
    float cb = vx - ca * vy;
    float hk = 0.5f * w0 / (y0 - vy);
    e->left_a = ca - hk;  e->left_b = cb + hk * vy;
    e->right_a = ca + hk; e->right_b = cb - hk * vy;
}

static void ldw_lane_at(const stLdwEgoSnapshot *e, float y, float *xl, float *xr)
{
    *xl = e->left_a * y + e->left_b;
    *xr = e->right_a * y + e->right_b;
}

static void ldw_lead_reset(stLdwLead *L)
{
    L->valid = 0;
    L->learn_n = 0;
    L->off_m = 0.0f;
    L->ratio = 0.0f;
}

/* Pick / follow / learn the car ahead. ref: our lane now (measured, held, or virtual) */
static void ldw_lead_update(uint32_t ch, const stNcLdwContext *ctx, const stLdwEgoSnapshot *ref,
                            int ref_measured, uint64_t now_us)
{
    const stNcLdwConfig *c = &ctx->cfg;
    stLdwLead *L = &g_ldw_lead[ch];
    const stLdwVeh *best = NULL;
    float xl, xr;

    if (!c->ego_lead || !ref->valid) { ldw_lead_reset(L); L->n = 0; return; }

    /* the closest car (lowest bottom) well inside our lane. On a held / virtual
     * lane (may be off) only a well centred one, and never a car cutting in */
    float umin = ref_measured ? LDW_LEAD_U_MIN : LDW_LEAD_EST_U_MIN;
    float umax = ref_measured ? LDW_LEAD_U_MAX : LDW_LEAD_EST_U_MAX;
    for (int i = 0; i < L->n; i++) {
        const stLdwVeh *v = &L->v[i];
        if (v->cutting_in) continue;
        if (v->y <= ref->vp_y + 1.0f) continue;
        if (v->y - ref->vp_y < LDW_LEAD_MIN_ROW_RATIO * (c->ref_y_px - ref->vp_y)) continue;
        ldw_lane_at(ref, v->y, &xl, &xr);
        float wl = xr - xl;
        if (wl <= 1.0f) continue;
        float u = (v->x - xl) / wl;
        if (u < umin || u > umax) continue;
        if (best == NULL || v->y > best->y) best = v;
    }

    /* the current lead seen again (same track), even if it is not "best" now */
    const stLdwVeh *same = NULL;
    if (L->valid) {
        for (int i = 0; i < L->n; i++)
            if (L->v[i].cls == L->car.cls && L->v[i].id == L->car.id) { same = &L->v[i]; break; }
    }

    if (same) {
        if (fabsf(same->w - L->car.w) > LDW_LEAD_WIDTH_JUMP * L->car.w) {
            printf("[LDW] ch%u lead car class:%d id:%d dropped (box width %.0f -> %.0f)\n",
                   ch, L->car.cls, L->car.id, (double)L->car.w, (double)same->w);
            ldw_lead_reset(L);                        // box jumped: not the same car
        } else {
            L->car = *same;
            L->last_us = now_us;
        }
    } else if (L->valid && us_to_s(now_us - L->last_us) > LDW_LEAD_LOST_S) {
        printf("[LDW] ch%u lead car class:%d id:%d lost\n", ch, L->car.cls, L->car.id);
        ldw_lead_reset(L);
    }

    if ((!L->valid && best) || (L->valid && best && !same && ref_measured)) {
        /* new car ahead; while the lines are seen a closer car can take the place */
        printf("[LDW] ch%u lead car class:%d id:%d picked (%s lane, bottom %.0f,%.0f)\n",
               ch, best->cls, best->id, ref_measured ? "measured" : nc_ldw_ego_src_name(ref->src),
               (double)best->x, (double)best->y);
        L->valid = 1;
        L->car = *best;
        L->since_us = now_us;
        L->last_us = now_us;
        L->learn_n = 0;
    }

    if (L->valid && same) {
        ldw_lane_at(ref, L->car.y, &xl, &xr);
        float wl = xr - xl;
        if (ref_measured && wl > 1.0f) {
            /* learn its place and width in the lane */
            float off = (L->car.x - 0.5f * (xl + xr)) * c->lane_width_m / wl;
            float rat = L->car.w / wl;
            if (L->learn_n == 0) { L->off_m = off; L->ratio = rat; }
            else { L->off_m += LDW_LEAD_ALPHA * (off - L->off_m); L->ratio += LDW_LEAD_ALPHA * (rat - L->ratio); }
            L->learn_n++;
            if (L->learn_n == LDW_LEAD_MIN_LEARN)
                printf("[LDW] ch%u lead car class:%d id:%d learned (offset %+.2fm, width %.2f of lane)\n",
                       ch, L->car.cls, L->car.id, (double)L->off_m, (double)L->ratio);
        } else if (!ref_measured && wl > 1.0f) {
            /* lines are gone: drop it when it leaves our lane (checked on the virtual lane) */
            float u = (L->car.x - xl) / wl;
            if (u < LDW_LEAD_DROP_U_MIN || u > LDW_LEAD_DROP_U_MAX) {
                printf("[LDW] ch%u lead car class:%d id:%d left our lane (u %.2f)\n",
                       ch, L->car.cls, L->car.id, (double)u);
                ldw_lead_reset(L);
            }
        }
    }
    L->n = 0;     // next frame
}

/* our lane around the lead car. 0: not usable */
static int ldw_lead_lane(uint32_t ch, const stNcLdwContext *ctx, uint64_t now_us, stLdwEgoSnapshot *e)
{
    const stNcLdwConfig *c = &ctx->cfg;
    const stLdwLead *L = &g_ldw_lead[ch];
    if (!c->ego_lead || !L->valid || L->last_us != now_us) return 0;   // must be seen in this frame
    if (us_to_s(now_us - L->since_us) < c->lead_min_s) return 0;
    if (L->car.y <= ctx->calib_vp_y + 1.0f) return 0;

    /* learned: width and centre from the car ahead.
     * not learned (lines were gone too soon): only the width, from an assumed
     * vehicle width; the centre stays the virtual one (where it drives in its
     * lane is unknown, up to +-0.5 m) */
    int learned = L->learn_n >= LDW_LEAD_MIN_LEARN &&
                  L->ratio >= LDW_LEAD_RATIO_MIN && L->ratio <= LDW_LEAD_RATIO_MAX;
    float w0 = learned ? L->car.w / L->ratio : L->car.w * c->lane_width_m / L->car.width_m;
    float k0 = w0 / (L->car.y - ctx->calib_vp_y);
    if (k0 < LDW_LEAD_K_MIN_RATIO * ctx->calib_k || k0 > LDW_LEAD_K_MAX_RATIO * ctx->calib_k) return 0;

    float cx;
    if (learned) {
        cx = L->car.x - L->off_m * w0 / c->lane_width_m;
    } else {
        float ca = (ctx->calib_cx_ref - ctx->calib_vp_x) / (c->ref_y_px - ctx->calib_vp_y);
        cx = ctx->calib_vp_x + ca * (L->car.y - ctx->calib_vp_y);   // virtual centre on its row
    }
    ldw_lane_through(ctx->calib_vp_x, ctx->calib_vp_y, cx, L->car.y, w0, e);
    return 1;
}

void nc_ldw_feed_vehicle(uint32_t ch, int class_id, int track_id, float x, float y, float w,
                         float width_m, int cutting_in)
{
    if (ch >= NC_LDW_MAX_CH) return;
    stLdwLead *L = &g_ldw_lead[ch];
    if (L->n >= LDW_LEAD_MAX_VEH || w <= 1.0f) return;
    stLdwVeh *v = &L->v[L->n++];
    v->cls = class_id; v->id = track_id;
    v->x = x; v->y = y; v->w = w;
    v->width_m = width_m > 0.5f ? width_m : 1.8f;
    v->cutting_in = cutting_in;
}

/* Ego lane for other modules, by age of the last measured lane:
 *   <= ego_fresh_s : DETECTED / ONE_SIDE (the measured lines)
 *   <= ego_hold_s  : HELD    - the last lines, moved by how far our car has
 *                    moved sideways since (lateral speed x age)
 *   later          : LEAD    - lane around the car ahead (if one is followed)
 *                    VIRTUAL - otherwise a 3.5 m lane at our usual place in the
 *                    lane, from the learned road shape (vanishing point, width/row)
 * Flat road: a lateral move of d metres shifts a line on row y by
 * d * k * (y - vp_y) / W pixels, i.e. x = a*y + b gets a -= d*k/W, b += d*k*vp_y/W. */
static void ldw_publish_ego(uint32_t ch, const stNcLdwContext *ctx, uint64_t now_us)
{
    const stNcLdwConfig *c = &ctx->cfg;
    const stLdwEgoMeas *m = &g_ldw_meas[ch];
    stLdwEgoSnapshot e, virt;
    float age_s = m->ok && now_us > m->wall_us ? us_to_s(now_us - m->wall_us) : 0.0f;

    memset(&e, 0, sizeof(e));
    e.vp_y = ctx->calib_vp_y;
    memset(&virt, 0, sizeof(virt));
    virt.vp_y = ctx->calib_vp_y;
    if (ctx->calib_valid) {
        /* centre line through the vanishing point and the usual lane centre on
         * the ref row, width k * (ref_y - vp_y) there */
        ldw_lane_through(ctx->calib_vp_x, ctx->calib_vp_y, ctx->calib_cx_ref, c->ref_y_px,
                         ctx->calib_k * (c->ref_y_px - ctx->calib_vp_y), &virt);
        virt.valid = 1;
        virt.src = NC_LDW_EGO_SRC_VIRTUAL;
        virt.time_us = now_us;
    }

    if (!ctx->calib_valid) {
        e.src = NC_LDW_EGO_SRC_NONE;
    } else if (m->ok && age_s <= c->ego_fresh_s) {
        e.src = m->both ? NC_LDW_EGO_SRC_DETECTED : NC_LDW_EGO_SRC_ONE_SIDE;
        e.left_a = m->left_a;   e.left_b = m->left_b;
        e.right_a = m->right_a; e.right_b = m->right_b;
        e.time_us = m->data_us;
    } else if (m->ok && age_s <= c->ego_hold_s) {
        float v = m->lat_vel_mps;
        v = v > LDW_HELD_VEL_DEADBAND_MPS ? v - LDW_HELD_VEL_DEADBAND_MPS
          : v < -LDW_HELD_VEL_DEADBAND_MPS ? v + LDW_HELD_VEL_DEADBAND_MPS : 0.0f;
        if (v >  LDW_HELD_MAX_VEL_MPS) v =  LDW_HELD_MAX_VEL_MPS;
        if (v < -LDW_HELD_MAX_VEL_MPS) v = -LDW_HELD_MAX_VEL_MPS;
        float d  = v * age_s;                              // metres we moved right
        float da = -d * ctx->calib_k / c->lane_width_m;
        float db =  d * ctx->calib_k * ctx->calib_vp_y / c->lane_width_m;
        e.src = NC_LDW_EGO_SRC_HELD;
        e.left_a = m->left_a + da;   e.left_b = m->left_b + db;
        e.right_a = m->right_a + da; e.right_b = m->right_b + db;
        e.time_us = now_us;
    }
    e.valid = e.src != NC_LDW_EGO_SRC_NONE;

    /* car ahead: picked and learned on the measured / held lane, checked on the
     * virtual lane once the lines are gone */
    int measured = e.src == NC_LDW_EGO_SRC_DETECTED || e.src == NC_LDW_EGO_SRC_ONE_SIDE;
    ldw_lead_update(ch, ctx, e.valid ? &e : &virt, measured, now_us);

    if (ctx->calib_valid && !e.valid) {
        if (ldw_lead_lane(ch, ctx, now_us, &e)) {
            e.src = NC_LDW_EGO_SRC_LEAD;
            e.time_us = now_us;
        } else if (c->ego_virtual) {
            e = virt;
        }
        e.vp_y = ctx->calib_vp_y;
        e.valid = e.src != NC_LDW_EGO_SRC_NONE;
    }

    /* log when the kind changes (detected <-> one-side flips often: one group) */
    {
        eNcLdwEgoSrc g_new = e.src == NC_LDW_EGO_SRC_ONE_SIDE ? NC_LDW_EGO_SRC_DETECTED : e.src;
        eNcLdwEgoSrc g_old = g_ldw_src_shown[ch] == NC_LDW_EGO_SRC_ONE_SIDE ? NC_LDW_EGO_SRC_DETECTED
                                                                            : g_ldw_src_shown[ch];
        if (g_new != g_old) {
            float y = c->ref_y_px;
            const stLdwLead *L = &g_ldw_lead[ch];
            printf("[LDW] ch%u ego lane source: %s -> %s (y=%.0f: %.0f-%.0f px, last measured %.0f-%.0f, "
                   "vp %.0f,%.0f k %.3f, lead %s)\n", ch,
                   nc_ldw_ego_src_name(g_ldw_src_shown[ch]), nc_ldw_ego_src_name(e.src), (double)y,
                   (double)(e.left_a * y + e.left_b), (double)(e.right_a * y + e.right_b),
                   (double)(m->left_a * y + m->left_b), (double)(m->right_a * y + m->right_b),
                   (double)ctx->calib_vp_x, (double)ctx->calib_vp_y, (double)ctx->calib_k,
                   !L->valid ? "none" : L->learn_n >= LDW_LEAD_MIN_LEARN ? "learned" : "seen");
        }
        g_ldw_src_shown[ch] = e.src;
    }

    pthread_mutex_lock(&g_ldw_ego_lock);
    g_ldw_ego[ch] = e;
    pthread_mutex_unlock(&g_ldw_ego_lock);
}

/* lane-loss test: 1 = ignore lane results now */
static int ldw_drop_lane(uint32_t ch, uint64_t now_us)
{
    if (g_ldw_drop_after_s < 0.0f) return 0;
    if (g_ldw_t0_us[ch] == 0) g_ldw_t0_us[ch] = now_us;
    float t = us_to_s(now_us - g_ldw_t0_us[ch]) - g_ldw_drop_after_s;
    if (t < 0.0f) return 0;
    if (g_ldw_drop_every_s <= 0.0f) return 1;
    return fmodf(t, g_ldw_drop_every_s) < g_ldw_drop_len_s;
}

#if NC_LDW_LOG_LANE_CLASS
static void ldw_log_lane_class(uint32_t ch, const pp_result_buf *lane_buf, uint64_t now_us)
{
    static uint64_t last_us[NC_LDW_MAX_CH];
    if (now_us - last_us[ch] < 1000000u) return;
    last_us[ch] = now_us;

    const stLaneDet *ln = lane_buf->cnn_result.lane_det;
    int n = lane_buf->lane_draw_info.max_lane_num;
    if (n > MAX_LANE_DET_CNT) n = MAX_LANE_DET_CNT;

    printf("[LDW] ch%u lane_class:", ch);
    for (int i = 0; i < n; i++) {
        if (ln[i].point_cnt > 0) {
            // bottom-most point x, to check which slot is left/right
            int b = 0;
            for (int j = 1; j < ln[i].point_cnt && j < MAX_POINT_CNT_OF_LANE; j++)
                if (ln[i].point[j].y > ln[i].point[b].y) b = j;
            printf(" [%d]class=%d(x=%.0f)", i, ln[i].lane_class, (double)ln[i].point[b].x);
        } else {
            printf(" [%d]-", i);
        }
    }
    printf("\n");
}
#endif

/* Board tuning without a rebuild: environment variables override the
 * defaults, e.g.  LDW_WARN_MARGIN=0.6 ./run_app_wayland_npu.sh  */
static void ldw_env_float(const char *name, float *v)
{
    const char *e = getenv(name);
    char *end = NULL;
    if (!e || !*e) return;
    float f = strtof(e, &end);
    if (end != e) *v = f;
    else printf("[LDW] ignoring %s=%s (not a number)\n", name, e);
}

static void ldw_env_int(const char *name, int *v)
{
    float f = (float)*v;
    ldw_env_float(name, &f);
    *v = (int)lroundf(f);
}

static void ldw_config_from_env(stNcLdwConfig *cfg)
{
    ldw_env_float("LDW_WARN_MARGIN",  &cfg->warn_margin_m);
    ldw_env_float("LDW_REACTION",     &cfg->reaction_time_s);
    ldw_env_float("LDW_TLC_MARGIN",   &cfg->tlc_margin_s);
    ldw_env_float("LDW_TLC_MAX_DIST", &cfg->tlc_max_dist_m);
    ldw_env_float("LDW_MIN_VEL",      &cfg->min_lat_vel_mps);
    ldw_env_float("LDW_VEHICLE_WIDTH",&cfg->vehicle_width_m);
    ldw_env_float("LDW_REF_Y",        &cfg->ref_y_px);
    ldw_env_float("LDW_CAM_X",        &cfg->cam_x_px);
    ldw_env_float("LDW_HOLD",         &cfg->hold_s);
    ldw_env_int  ("LDW_TRIGGER_FRAMES", &cfg->trigger_frames);
    ldw_env_int  ("LDW_DASHED_CLASS",   &cfg->dashed_class);
    ldw_env_float("LDW_EGO_HOLD_S",   &cfg->ego_hold_s);
    ldw_env_int  ("LDW_EGO_VIRTUAL",  &cfg->ego_virtual);
    ldw_env_int  ("LDW_EGO_LEAD",     &cfg->ego_lead);
    ldw_env_float("LDW_CAM_OFFSET",   &cfg->cam_offset_m);
    ldw_env_float("LDW_LANE_WIDTH",   &cfg->lane_width_m);
    ldw_env_int  ("LDW_AUTO_CENTER",  &cfg->auto_center);
    ldw_env_float("LDW_AUTO_CENTER_MAX", &cfg->auto_center_max_m);
    ldw_env_float("LDW_LEAD_MIN_S",   &cfg->lead_min_s);
    ldw_env_float("LDW_DROP_AFTER_S", &g_ldw_drop_after_s);
    ldw_env_float("LDW_DROP_EVERY_S", &g_ldw_drop_every_s);
    ldw_env_float("LDW_DROP_LEN_S",   &g_ldw_drop_len_s);
    if (cfg->trigger_frames < 1) cfg->trigger_frames = 1;
    if (cfg->ego_hold_s < cfg->ego_fresh_s) cfg->ego_hold_s = cfg->ego_fresh_s;
}

void nc_ldw_app_init(int ch_num, float canvas_w, float canvas_h)
{
    stNcLdwConfig cfg;

    if (ch_num > NC_LDW_MAX_CH) ch_num = NC_LDW_MAX_CH;
    if (ch_num < 0) ch_num = 0;
    g_ldw_ch_num = ch_num;
    g_ldw_canvas_w = canvas_w;
    g_ldw_canvas_h = canvas_h;

    nc_ldw_default_config(&cfg, canvas_w, canvas_h);
    ldw_config_from_env(&cfg);
    printf("[LDW] config: warn_margin=%.2fm tlc<%.2f+lat+%.2fs (dist<%.2fm, vel>%.2fm/s) "
           "trigger=%d frames ref_y=%.0f cam_x=%.0f dashed_class=%d\n",
           (double)cfg.warn_margin_m, (double)cfg.reaction_time_s, (double)cfg.tlc_margin_s,
           (double)cfg.tlc_max_dist_m, (double)cfg.min_lat_vel_mps, cfg.trigger_frames,
           (double)cfg.ref_y_px, (double)cfg.cam_x_px, cfg.dashed_class);
    printf("[LDW] camera: offset %+.2fm, lane width %.2fm, usual place %s (max %.2fm after %.0fs)\n",
           (double)cfg.cam_offset_m, (double)cfg.lane_width_m,
           cfg.auto_center ? "learned" : "off", (double)cfg.auto_center_max_m, (double)cfg.auto_center_min_s);
    printf("[LDW] ego lane for cut-in: measured <=%.1fs, held <=%.1fs, then %s%s\n",
           (double)cfg.ego_fresh_s, (double)cfg.ego_hold_s, cfg.ego_lead ? "lead car (>=1 s) / " : "",
           cfg.ego_virtual ? "virtual" : "none");
    if (g_ldw_drop_after_s >= 0.0f)
        printf("[LDW] TEST: lane results ignored from %.1fs%s (every %.1fs for %.1fs)\n",
               (double)g_ldw_drop_after_s, g_ldw_drop_every_s > 0.0f ? "" : " on",
               (double)g_ldw_drop_every_s, (double)g_ldw_drop_len_s);
    for (int i = 0; i < ch_num; i++) {
        nc_ldw_init(&g_ldw_ctx[i], &cfg);
        g_ldw_suspended[i] = 0;
        g_ldw_last_buf[i] = NULL;
        g_ldw_last_ts[i] = 0;
        g_ldw_t0_us[i] = 0;
        g_ldw_src_shown[i] = NC_LDW_EGO_SRC_NONE;
    }
    memset(g_ldw_meas, 0, sizeof(g_ldw_meas));
    memset(g_ldw_lead, 0, sizeof(g_ldw_lead));

    pthread_mutex_lock(&g_ldw_ego_lock);
    memset(g_ldw_ego, 0, sizeof(g_ldw_ego));
    pthread_mutex_unlock(&g_ldw_ego_lock);
}

void nc_ldw_app_process(uint32_t ch, const pp_result_buf *lane_buf)
{
    if ((int)ch >= g_ldw_ch_num) return;

    stNcLdwContext *ctx = &g_ldw_ctx[ch];
    eNcLdwState prev = ctx->res.state;
    uint64_t now_us = ldw_now_us();

    /* camera hint: the lane network finds lanes when the horizon is at about
     * 40-50% of the image height (board 11/06: videos with the horizon at ~70%
     * never gave a lane; the same videos cropped lower did) */
    {
        static uint64_t t0[NC_LDW_MAX_CH];
        static int hinted[NC_LDW_MAX_CH];
        if (t0[ch] == 0) t0[ch] = now_us;
        if (!hinted[ch] && !ctx->calib_valid && us_to_s(now_us - t0[ch]) > LDW_NO_LANE_HINT_S) {
            printf("[LDW] ch%u no lane for %.0f s: check the camera - horizon at about 40-50%% of the "
                   "image height, road in the lower half, lane lines visible\n", ch, (double)LDW_NO_LANE_HINT_S);
            hinted[ch] = 1;
        }
        /* 70%: test3 / px / city work with 62-67%; videos at ~70% gave no lane */
        if (hinted[ch] < 2 && ctx->calib_valid && ctx->calib_vp_y > 0.70f * ctx->cfg.img_h) {
            printf("[LDW] ch%u horizon at %.0f%% of the image height (lanes are found up to ~65%%): camera tilted down?\n",
                   ch, (double)(100.0f * ctx->calib_vp_y / ctx->cfg.img_h));
            hinted[ch] = 2;
        }
    }

    /* SKIP mode: no lane results -> stop LDW (no warning, no recording).
     * The "LDW OFF" text is drawn by the model-switch module. */
    if (ldw_model_skip()) {
        if (!g_ldw_suspended[ch]) {
            nc_ldw_suspend(ctx);
            g_ldw_suspended[ch] = 1;
            printf("[LDW] ch%u suspended (model SKIP mode)\n", ch);
        }
        ldw_publish_ego(ch, ctx, now_us);   // held, then virtual lane for cut-in
        return;
    }
    if (g_ldw_suspended[ch]) {
        /* back from SKIP: the buffer may still hold the lane result from before
         * SKIP; its pointer was already seen, so it is ignored until new data comes */
        g_ldw_suspended[ch] = 0;
        printf("[LDW] ch%u resumed\n", ch);
    }

    int is_new = lane_buf != NULL &&
                 ((const void *)lane_buf != g_ldw_last_buf[ch] || lane_buf->time_stamp != g_ldw_last_ts[ch]);

    if (is_new) {
        g_ldw_last_buf[ch] = lane_buf;
        g_ldw_last_ts[ch] = lane_buf->time_stamp;
#if NC_LDW_LOG_LANE_CLASS
        ldw_log_lane_class(ch, lane_buf, now_us);
#endif
        if (ldw_drop_lane(ch, now_us)) {
            /* test: a new result with no lines */
            nc_ldw_update(ctx, lane_buf->cnn_result.lane_det, 0, lane_buf->time_stamp, now_us);
        } else {
            // time_stamp stays 0 until the input timestamp is passed through (then it is used)
            nc_ldw_update(ctx, lane_buf->cnn_result.lane_det, lane_buf->lane_draw_info.max_lane_num,
                          lane_buf->time_stamp, now_us);
        }
        if (ctx->res.valid && ctx->last_wall_us == now_us) {
            /* lines found in this result: remember them for the HELD lane */
            stLdwEgoMeas *m = &g_ldw_meas[ch];
            m->ok = 1;
            m->both = ctx->res.left_detected && ctx->res.right_detected;
            m->left_a = ctx->res.left_a;   m->left_b = ctx->res.left_b;
            m->right_a = ctx->res.right_a; m->right_b = ctx->res.right_b;
            m->lat_vel_mps = ctx->res.lat_vel_mps;
            m->wall_us = now_us;
            m->data_us = ctx->res.data_time_us;
        }
    } else {
        nc_ldw_update(ctx, NULL, 0, 0, now_us);    // no new result: timeout check only
    }
    ldw_publish_ego(ch, ctx, now_us);

    const stNcLdwResult *r = &ctx->res;
    if (r->state != prev) {
        printf("[LDW] ch%u %s -> %s%s (left %.2fm, right %.2fm, offset %+.2fm, vel %+.2fm/s, "
               "tlc %.2fs < %.2fs?, latency %.0fms, class L%d R%d, dashed vote L%.2f R%.2f)\n",
               ch, nc_ldw_state_name(prev), nc_ldw_state_name(r->state),
               r->level == NC_LDW_LEVEL_WEAK ? "(weak)" : r->level == NC_LDW_LEVEL_STRONG ? "(strong)" : "",
               (double)r->dist_left_m, (double)r->dist_right_m, (double)r->offset_m,
               (double)r->lat_vel_mps, (double)r->tlc_s, (double)r->tlc_thresh_s,
               (double)r->latency_ms, r->left_class, r->right_class,
               (double)ctx->dashed_score[0], (double)ctx->dashed_score[1]);
    }
}

const char *nc_ldw_ego_src_name(eNcLdwEgoSrc src)
{
    switch (src) {
    case NC_LDW_EGO_SRC_DETECTED: return "detected";
    case NC_LDW_EGO_SRC_ONE_SIDE: return "one-side";
    case NC_LDW_EGO_SRC_HELD:     return "held";
    case NC_LDW_EGO_SRC_LEAD:     return "lead";
    case NC_LDW_EGO_SRC_VIRTUAL:  return "virtual";
    default:                      return "none";
    }
}

int nc_ldw_get_ego_lane_src(uint32_t ch, float y, float *x_left, float *x_right,
                            uint64_t *time_us, eNcLdwEgoSrc *src)
{
    stLdwEgoSnapshot e;

    if (src) *src = NC_LDW_EGO_SRC_NONE;
    if (ch >= NC_LDW_MAX_CH) return -1;

    pthread_mutex_lock(&g_ldw_ego_lock);
    e = g_ldw_ego[ch];
    pthread_mutex_unlock(&g_ldw_ego_lock);

    if (!e.valid) return -1;
    if (y <= e.vp_y + 1.0f) return -1;          // at/above the horizon: no lane width

    float xl = e.left_a * y + e.left_b;
    float xr = e.right_a * y + e.right_b;
    if (xr <= xl) return -1;

    if (x_left)  *x_left = xl;
    if (x_right) *x_right = xr;
    if (time_us) *time_us = e.time_us;
    if (src)     *src = e.src;
    return 0;
}

int nc_ldw_get_ego_lane(uint32_t ch, float y, float *x_left, float *x_right, uint64_t *time_us)
{
    float xl, xr;
    uint64_t t;
    eNcLdwEgoSrc src;

    if (nc_ldw_get_ego_lane_src(ch, y, &xl, &xr, &t, &src) != 0) return -1;
    if (src != NC_LDW_EGO_SRC_DETECTED && src != NC_LDW_EGO_SRC_ONE_SIDE) return -1;
    if (x_left)  *x_left = xl;
    if (x_right) *x_right = xr;
    if (time_us) *time_us = t;
    return 0;
}

int nc_ldw_is_in_ego_lane(uint32_t ch, float x, float y)
{
    float xl, xr;
    if (nc_ldw_get_ego_lane(ch, y, &xl, &xr, NULL) != 0) return -1;
    return (x >= xl && x <= xr) ? 1 : 0;
}

void nc_ldw_app_draw(uint32_t ch, struct viewport vp, const stNcLdwDrawRes *res)
{
    if ((int)ch >= g_ldw_ch_num || res == NULL || vp.height <= 0) return;
    if (g_ldw_suspended[ch]) return;     // SKIP mode: "LDW OFF" is drawn elsewhere

    const stNcLdwContext *ctx = &g_ldw_ctx[ch];
    const stNcLdwResult *r = &ctx->res;
    const float cw = g_ldw_canvas_w;
    const float chh = g_ldw_canvas_h;
    const int sw = (int)cw;
    const int sh = (int)chh;
    float text_scale = chh / (float)vp.height;
    char buftext[160];

    glViewport(vp.x, vp.y, vp.width, vp.height);

    /* lines not seen: show the HELD / VIRTUAL lane that cut-in is using
     * (render thread writes the snapshot, so no lock is needed here) */
    const stLdwEgoSnapshot *eg = &g_ldw_ego[ch];
    int est = eg->valid && (eg->src == NC_LDW_EGO_SRC_HELD || eg->src == NC_LDW_EGO_SRC_LEAD ||
                            eg->src == NC_LDW_EGO_SRC_VIRTUAL);
    if (est) {
        float col[4] = {0.3f, 0.8f, 1.0f, 1.0f};      // light blue
        float y0 = ctx->cfg.ref_y_px;
        float y1 = eg->vp_y + (y0 - eg->vp_y) * 0.35f;
        nc_opengl_draw_line(eg->left_a * y0 + eg->left_b, y0, eg->left_a * y1 + eg->left_b, y1,
                            2, col, *res->npu_prog);
        nc_opengl_draw_line(eg->right_a * y0 + eg->right_b, y0, eg->right_a * y1 + eg->right_b, y1,
                            2, col, *res->npu_prog);
    }

    if (!r->valid) {
        float gray[3] = {0.7f, 0.7f, 0.7f};
        const char *txt = !est ? "LDW: no lane"
                        : eg->src == NC_LDW_EGO_SRC_HELD ? "LDW: no lane (cut-in: held lane)"
                        : eg->src == NC_LDW_EGO_SRC_LEAD ? "LDW: no lane (cut-in: lane from car ahead)"
                                                         : "LDW: no lane (cut-in: virtual lane)";
        nc_opengl_draw_text(res->font_small, txt, 10.0f, chh - 110.0f,
                            text_scale, gray, sw, sh, *res->font_prog);
        return;
    }

    int warn = (r->state == NC_LDW_STATE_WARN_LEFT || r->state == NC_LDW_STATE_WARN_RIGHT);
    float y = ctx->cfg.ref_y_px;

    // measurement row between both lines and the vehicle centre marker
    float row_color[4] = {1.0f, 1.0f, 0.0f, 1.0f};
    float veh_color[4] = {1.0f, 1.0f, 1.0f, 1.0f};
    nc_opengl_draw_line(r->x_left_px, y, r->x_right_px, y, 1, row_color, *res->npu_prog);
    nc_opengl_draw_rectangle(r->vehicle_x_px - 8.0f, y - 8.0f, 16.0f, 16.0f, veh_color, *res->npu_prog);

    // distances
    float info_color[3] = {1.0f, 1.0f, 1.0f};
    if (r->tlc_s >= 0.0f)
        snprintf(buftext, sizeof(buftext), "LDW  L %.2fm  R %.2fm  v %+.2fm/s  TLC %.1fs (warn<%.1fs, lat %.0fms)",
                 (double)r->dist_left_m, (double)r->dist_right_m, (double)r->lat_vel_mps,
                 (double)r->tlc_s, (double)r->tlc_thresh_s, (double)r->latency_ms);
    else
        snprintf(buftext, sizeof(buftext), "LDW  L %.2fm  R %.2fm  v %+.2fm/s  TLC -  (warn<%.1fs, lat %.0fms)",
                 (double)r->dist_left_m, (double)r->dist_right_m, (double)r->lat_vel_mps,
                 (double)r->tlc_thresh_s, (double)r->latency_ms);
    nc_opengl_draw_text(res->font_small, buftext, 10.0f, chh - 110.0f,
                        text_scale, info_color, sw, sh, *res->font_prog);

    if (warn) {
        /* strong (solid / unknown line): thick red frame
         * weak   (dashed line)         : thin yellow frame */
        int strong = (r->level != NC_LDW_LEVEL_WEAK);
        float col4[4] = {1.0f, strong ? 0.0f : 0.85f, 0.0f, 1.0f};
        float col3[3] = {col4[0], col4[1], col4[2]};
        int frames = strong ? 4 : 2;

        for (int i = 0; i < frames; i++) {
            float m = 4.0f + (float)i * 4.0f;
            nc_opengl_draw_rectangle(m, m, cw - 2.0f * m, chh - 2.0f * m, col4, *res->npu_prog);
        }

        const char *msg;
        if (r->state == NC_LDW_STATE_WARN_LEFT)
            msg = strong ? "<<< LANE DEPARTURE LEFT" : "<<< LEFT (dashed)";
        else
            msg = strong ? "LANE DEPARTURE RIGHT >>>" : "RIGHT (dashed) >>>";
        nc_opengl_draw_text(res->font_large, msg, cw * 0.5f - 300.0f, chh - 180.0f,
                            text_scale, col3, sw, sh, *res->font_prog);
    }
}

#endif /* USE_LDW */
