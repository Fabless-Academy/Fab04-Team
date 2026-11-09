/* PC test: cut-in with lane lines lost (held / virtual lane). Simulated clock. */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include "nc_ldw.h"
#include "nc_traj.h"
#include "nc_cutin.h"
void glViewport(int a, int b, int c, int d) { (void)a; (void)b; (void)c; (void)d; }
void nc_opengl_draw_rectangle(float a, float b, float c, float d, float *e, struct gl_npu_program f) { (void)a;(void)b;(void)c;(void)d;(void)e;(void)f; }
void nc_opengl_draw_line(float a, float b, float c, float d, int w, float *e, struct gl_npu_program f) { (void)a;(void)b;(void)c;(void)d;(void)w;(void)e;(void)f; }
void nc_opengl_draw_text(Font *a, const char *b, float c, float d, float e, float *f, int g, int h, struct gl_font_program i) { (void)a;(void)b;(void)c;(void)d;(void)e;(void)f;(void)g;(void)h;(void)i; }

/* simulated monotonic clock for nc_ldw.c (linked with --wrap=clock_gettime) */
static uint64_t g_sim_us = 1000000000ULL;
int __wrap_clock_gettime(clockid_t id, struct timespec *ts) { (void)id; ts->tv_sec = g_sim_us / 1000000u; ts->tv_nsec = (g_sim_us % 1000000u) * 1000u; return 0; }

static const float VP_X = 960.0f, VP_Y = 460.0f, K = 2.746f, W = 3.5f;
static float px(float X, float y) { return VP_X + K * (y - VP_Y) * X / W; }
/* object world, after lines are gone: road shape scale g_ks from g_kt, our drift g_egodx (m, +right) from g_et */
static float g_ks = 1.f, g_kt = 99.f, g_egodx = 0.f, g_et = 99.f;
static int   g_lead = 0; static float g_lx0 = 0.3f, g_lx1 = 0.3f, g_lt0 = 99.f, g_lt1 = 99.f, g_lyb = 650.f;
static float kobj(float t) { float r = t < g_kt ? 0.f : (t - g_kt > 1.f ? 1.f : t - g_kt); return K * (1.f + (g_ks - 1.f) * r); }  /* ramp over 1 s */
static float pxo(float X, float y, float t) {
    float d = t >= g_et ? g_egodx * (t - g_et > 1.f ? 1.f : t - g_et) : 0.f;
    return VP_X + kobj(t) * (y - VP_Y) * (X - d) / W;
}
static void add_obj(stCnnPostprocessingResults *det, int cls, int id, float X, float yb, float wm, float t) {
    float bw = kobj(t) * (yb - VP_Y) * wm / W, bh = bw * 0.9f;
    stObjInfo *o = &det->class_objs[cls].objs[det->class_objs[cls].obj_cnt++];
    o->track_id = id; o->prob = 0.9f;
    o->bbox.x = pxo(X, yb, t) - bw * 0.5f; o->bbox.y = yb - bh; o->bbox.w = bw; o->bbox.h = bh;
}
static void make_lane(stLaneDet *ln, float X, int cls) {
    memset(ln, 0, sizeof(*ln));
    for (int i = 0; i < 18; i++) {
        float y = (0.2f + 0.8f * (float)i / 17.0f) * 1080.0f;
        if (y <= VP_Y + 5.0f) continue;
        ln->point[ln->point_cnt].x = px(X, y); ln->point[ln->point_cnt].y = y; ln->point_cnt++;
    }
    ln->lane_class = cls;
}
static float g_t0 = 1.f, g_t1 = 7.f, g_jit = 0.f, g_yb0 = 760.f, g_yb1 = 760.f;
static float g_drop0 = 99.f, g_drop1 = 99.f;
static int   g_ybsync = 0;   /* 1: bottom row moves with the lateral motion (t0..t1) */   /* no lane lines in [drop0, drop1) */
static int run(const char *name, float x0, float x1, float *first_on) {
    static pp_result_buf bufs[2]; pp_result_buf *bp = &bufs[0];
    static stCnnPostprocessingResults det;
    nc_ldw_app_init(1, 1920.0f, 1080.0f); nc_traj_reset(); nc_cutin_reset();
    int on_frames = 0; *first_on = -1.f;
    const uint64_t T0 = 1000000000ULL;
    const char *fe = getenv("SIM_FPS"); const float DT = 1.0f / (fe ? (float)atof(fe) : 14.0f);
    int nf = (int)(10.5f / DT);
    for (int f = 0; f < nf; f++) {
        float t = (float)f * DT; uint64_t ts = T0 + (uint64_t)(t * 1e6f);
        g_sim_us = ts;
        float a = t < g_t0 ? 0.f : (t > g_t1 ? 1.f : (t - g_t0) / (g_t1 - g_t0));
        float X = x0 + (x1 - x0) * a + g_jit * sinf((float)f * 1.7f) * (((f * 7919) % 13) / 13.f);
        memset(bp, 0, sizeof(*bp)); bp->time_stamp = ts > (uint64_t)(DT * 1e6f) ? ts - (uint64_t)(DT * 1e6f) : ts;
        int drop = t >= g_drop0 && t < g_drop1;
        bp->lane_draw_info.max_lane_num = drop ? 0 : 4;
        if (!drop) {
            make_lane(&bp->cnn_result.lane_det[0], -5.25f, 2); make_lane(&bp->cnn_result.lane_det[1], -1.75f, 1);
            make_lane(&bp->cnn_result.lane_det[2], 1.75f, 2); make_lane(&bp->cnn_result.lane_det[3], 5.25f, 1);
        }
        nc_ldw_app_process(0, bp);
        bp = (bp == &bufs[0]) ? &bufs[1] : &bufs[0];
        memset(&det, 0, sizeof(det));
        float yb = g_yb0 + (g_yb1 - g_yb0) * (g_ybsync ? a : t / 10.5f);
        add_obj(&det, 2, 7, X, yb, 0.9f * 2.5f, t);
        if (g_lead) {
            float la = t < g_lt0 ? 0.f : (t > g_lt1 ? 1.f : (t - g_lt0) / (g_lt1 - g_lt0));
            add_obj(&det, 1, 3, g_lx0 + (g_lx1 - g_lx0) * la + 0.1f * sinf(t * 2.3f), g_lyb, 1.8f, t);
        }
        nc_traj_update(0, NETWORK_TRI_CHIMERA, &det, ts);
        nc_traj_predict_frame(0, ts);
        nc_cutin_begin_frame(0, ts);
        stBBox pb; float fx, fy;
        if (g_lead && nc_traj_get_predicted_box(0, 1, 3, ts, &pb) && nc_traj_get_future_center(0, 1, 3, ts, nc_cutin_get_horizon_ms(), &fx, &fy))
            nc_cutin_eval(0, 1, 3, &pb, fx, fy, ts);
        if (nc_traj_get_predicted_box(0, 2, 7, ts, &pb) && nc_traj_get_future_center(0, 2, 7, ts, nc_cutin_get_horizon_ms(), &fx, &fy))
        {
            nc_cutin_eval(0, 2, 7, &pb, fx, fy, ts);
            if (getenv("DBG") && t > 2.5f && t < 4.5f) {
                float l0, r0, l1, r1; float ny = pb.y + pb.h, nx = pb.x + pb.w * 0.5f, fyy = fy + pb.h * 0.5f;
                nc_ldw_get_ego_lane_src(0, ny, &l0, &r0, NULL, NULL); nc_ldw_get_ego_lane_src(0, fyy, &l1, &r1, NULL, NULL);
                printf("t=%.2f X=%.2f true_by=%.0f pred_by=%.0f u_now=%.2f fut_y=%.0f u_fut=%.2f\n", (double)t, (double)X, (double)yb,
                       (double)ny, (double)((nx - l0) / (r0 - l0)), (double)fyy, (double)((fx - l1) / (r1 - l1)));
            }
        }
        nc_cutin_end_frame(0);
        if (nc_cutin_active_count(0) > 0) { on_frames++; if (*first_on < 0) *first_on = t; }
    }
    (void)name;
    return on_frames;
}
static int fails = 0;
static void check(const char *name, float x0, float x1, int expect_on) {
    float first; int on = run(name, x0, x1, &first);
    int ok = expect_on == 2 ? 1 : expect_on ? on > 0 : on == 0;   /* 2: record only */
    if (!ok) fails++;
    printf("%-46s on=%3d first=%5.2fs  %s\n", name, on, (double)first, expect_on == 2 ? "(record)" : ok ? "ok" : "FAIL");
}
int main(void) {
    const char *only_meas = getenv("CUTIN_LANE_SRC");
    printf("== lanes always seen\n");
    check("A  slow cut-in 1-7s", 3.5f, 0.0f, 1);
    g_t1 = 3.f; check("F  fast cut-in 1-3s", 3.5f, 0.0f, 1); g_t1 = 7.f;
    check("B  stays in right lane", 3.5f, 3.5f, 0);
    g_yb0 = 560.f; g_yb1 = 1000.f; check("J  cut-in while approaching", 3.5f, 0.0f, 1); g_yb0 = g_yb1 = 760.f;
    printf("== lanes lost 1.0-2.0s (held lane)\n");
    g_drop0 = 1.0f; g_drop1 = 2.0f;
    g_t1 = 3.f; check("F1 fast cut-in, gap 1s", 3.5f, 0.0f, 1); g_t1 = 7.f;
    check("B1 stays right, gap 1s", 3.5f, 3.5f, 0);
    printf("== lanes lost from 0.5s on (virtual lane)\n");
    if (!getenv("NODROP")) { g_drop0 = 0.5f; g_drop1 = 99.f; } else { g_drop0 = g_drop1 = 99.f; }
    int want = !(only_meas && atoi(only_meas) < 2);
    check("A2 slow cut-in", 3.5f, 0.0f, want);
    g_t1 = 3.f; check("F2 fast cut-in", 3.5f, 0.0f, want); g_t1 = 7.f;
    check("B2 stays in right lane", 3.5f, 3.5f, 0);
    check("D2 drifts 3.5->2.8m, stays out", 3.5f, 2.8f, 0);
    check("D3 drifts 3.5->2.6m (ego 0.9m off-centre)", 3.5f, 2.6f, 0);
    g_jit = 0.25f; check("E2 next lane 2.6m, jitter", 2.6f, 2.6f, 0);
    check("G2 ahead at lane edge 1.2m, jitter", 1.2f, 1.2f, 0); g_jit = 0.f;
    check("C2 ahead in lane", 0.0f, 0.0f, 0);
    g_yb0 = 560.f; g_yb1 = 1000.f;
    check("H2 overtaking right-lane car", 3.5f, 3.5f, 0);
    check("I2 overtaking left-lane car", -3.5f, -3.5f, 0);
    check("J2 cut-in while approaching", 3.5f, 0.0f, want);
    g_yb0 = g_yb1 = 760.f;

    /* step 4: road shape changes after the lines are gone (test3 case: lane on the car's
     * row 11% narrower than learned), close fast cut-in near the bottom */
    for (int lead = 0; lead <= 1; lead++) {
        g_lead = lead; g_lx0 = g_lx1 = 0.3f; g_lt0 = g_lt1 = 99.f;
        printf("== lanes lost from 0.5s, road shape x0.88 from 1s, %s\n", lead ? "car ahead" : "no car ahead");
        g_drop0 = 0.5f; g_drop1 = 99.f; g_ks = 0.88f; g_kt = 1.0f;
        g_yb0 = g_yb1 = 940.f; g_t0 = 3.f; g_t1 = 4.3f;
        check(lead ? "K1L close fast cut-in" : "K1  close fast cut-in", 3.0f, 0.3f, want);
        g_jit = 0.25f; check(lead ? "K2L next lane car 3.2m, jitter" : "K2  next lane car 3.2m, jitter", 3.2f, 3.2f, 0);
        check(lead ? "K3L in lane at 1.1m, jitter" : "K3  in lane at 1.1m, jitter", 1.1f, 1.1f, 0); g_jit = 0.f;
        g_yb0 = g_yb1 = 760.f; g_t0 = 1.f; g_t1 = 7.f;
        check(lead ? "K4L slow cut-in" : "K4  slow cut-in", 3.5f, 0.0f, want);
        g_ks = 1.f; g_kt = 99.f;
        /* our car drifts 0.8 m right in its lane while lines are gone */
        g_egodx = 0.8f; g_et = 1.0f;
        g_jit = 0.25f; check(lead ? "E1L we drift 0.8m right, next-lane car, jitter" : "E1  we drift 0.8m right, next-lane car, jitter", 3.5f, 3.5f, 0); g_jit = 0.f;
        g_t0 = 3.f; g_t1 = 5.f; check(lead ? "E2L we drift 0.8m right, cut-in" : "E2  we drift 0.8m right, cut-in", 3.5f, 0.0f, want); g_t0 = 1.f; g_t1 = 7.f;
        g_egodx = 0.f; g_et = 99.f;
    }
    /* car ahead changes to the right lane while lines are gone; next-lane car stays */
    g_lead = 1; g_lx0 = 0.3f; g_lx1 = 3.5f; g_lt0 = 3.f; g_lt1 = 5.f;
    printf("== lanes lost, car ahead leaves to the right lane\n");
    g_jit = 0.25f; check("L1  car ahead leaves, next-lane car stays", 3.5f, 3.5f, 0); g_jit = 0.f;
    g_lead = 0; g_drop0 = g_drop1 = 99.f;
    /* pulling away (board 11/06 city video) */
    printf("== pulling away, lanes seen\n");
    g_ybsync = 1; g_t0 = 1.f; g_t1 = 3.f;
    g_yb0 = 1000.f; g_yb1 = 600.f; check("M   next-lane car pulls away fast", 3.2f, 3.2f, 0);
    g_yb0 = 950.f;  g_yb1 = 850.f; check("M2  overtakes and cuts in, pulling away slowly", 3.2f, 0.3f, 1);
    g_yb0 = 1000.f; g_yb1 = 600.f; check("M3  cuts in while pulling away fast", 3.2f, 0.3f, 2);
    g_ybsync = 0; g_yb0 = g_yb1 = 760.f; g_t1 = 7.f;
    /* car right at our line, we close in (board 11/09, px bus: u_now ~1.15-1.2) */
    printf("== next-lane car right at our line, we close in, lanes seen\n");
    g_yb0 = 600.f; g_yb1 = 1000.f;
    g_jit = 0.25f; check("N   at the line (u 1.15), jitter", 2.3f, 2.3f, 0);
    check("N2  at the line, box pulled in 0.1 lane", 2.3f, 1.95f, 0); g_jit = 0.f;
    g_t1 = 3.f; check("N3  at the line, cuts in over 2 s", 2.3f, 0.0f, 1); g_t1 = 7.f;
    g_yb0 = g_yb1 = 760.f;
    /* far cars (lane on the car's row ~330 px < CUTIN_SIDE_W): sideways check */
    printf("== far cars (lane ~330 px on their row), lanes seen\n");
    g_yb0 = g_yb1 = 580.f;
    check("P1  far slow cut-in 0.58 m/s", 3.5f, 0.0f, 1);
    g_t1 = 3.f; check("P2  far fast cut-in", 3.5f, 0.0f, 1); g_t1 = 7.f;
    g_yb0 = 570.f; g_yb1 = 620.f; g_jit = 0.25f;
    check("P3  far car at the line, box pulled in", 2.3f, 1.95f, 0); g_jit = 0.f;
    g_yb0 = g_yb1 = 760.f;
    puts(fails ? "RESULT: FAIL" : "RESULT: PASS");
    return fails;
}
