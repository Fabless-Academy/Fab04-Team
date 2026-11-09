/* PC test: camera 0.6 m left of the car centre (phone on the driver side) */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "nc_ldw.h"
static const float VP_X = 960.0f, VP_Y = 460.0f, K = 2.746f, W = 3.5f;
static void make_lane(stLaneDet *ln, float X, int cls) {
    memset(ln, 0, sizeof(*ln));
    for (int i = 0; i < 18; i++) {
        float y = (0.2f + 0.8f * (float)i / 17.0f) * 1080.0f;
        if (y <= VP_Y + 5.0f) continue;
        ln->point[ln->point_cnt].x = VP_X + K * (y - VP_Y) * X / W; ln->point[ln->point_cnt].y = y; ln->point_cnt++;
    }
    ln->lane_class = cls;
}
int main(int argc, char **argv) {
    float cam = argc > 1 ? (float)atof(argv[1]) : -0.6f;   /* camera position from car centre (m, +right) */
    int autoc = argc > 2 ? atoi(argv[2]) : 1;
    stNcLdwConfig cfg; stNcLdwContext ctx; static stLaneDet lanes[4];
    nc_ldw_default_config(&cfg, 1920.f, 1080.f); cfg.auto_center = autoc;
    nc_ldw_init(&ctx, &cfg);
    int warn_early = 0, warn_mid = 0, warn_dep = 0, warn_dep_r = 0; eNcLdwState prev = NC_LDW_STATE_INVALID;
    float first_dep = -1;
    for (int f = 0; f < 15 * 45; f++) {
        float t = f / 15.0f; uint64_t ts = 1000000000ULL + (uint64_t)(t * 1e6f);
        /* car position in lane (m, +right): centre, small weave; 30-33 s drift left 0.5 m/s; 38-41 s drift right */
        float car = 0.1f * sinf(t * 1.26f) + 0.02f * (float)((f * 7919) % 5 - 2) / 2.0f;   /* smooth weave + 2 cm noise */
        if (t >= 30 && t < 33) car += -0.5f * (t - 30); else if (t >= 33 && t < 35) car += -1.5f + 0.75f * (t - 33);
        if (t >= 38 && t < 41) car += 0.5f * (t - 38); else if (t >= 41 && t < 43) car += 1.5f - 0.75f * (t - 41);
        float c = car + cam;                       /* camera position in lane */
        make_lane(&lanes[1], -1.75f - c, 1); make_lane(&lanes[2], 1.75f - c, 2);
        nc_ldw_update(&ctx, lanes, 4, ts, ts);
        eNcLdwState s = ctx.res.state;
        int w = s == NC_LDW_STATE_WARN_LEFT || s == NC_LDW_STATE_WARN_RIGHT;
        int pw = prev == NC_LDW_STATE_WARN_LEFT || prev == NC_LDW_STATE_WARN_RIGHT;
        if (w && !pw) {
            if (t < 10) warn_early++; else if (t < 30) warn_mid++;
            else if (t < 37) { warn_dep++; if (first_dep < 0) first_dep = t; } else warn_dep_r++;
        }
        if (getenv("DBG") && f % 30 == 0) printf("t=%.0f learn=%.1fs off=%+.2f used=%+.2f vel=%+.2f state=%d\n", (double)t, (double)ctx.center_learn_s, (double)ctx.center_off_m, (double)ctx.center_used_m, (double)ctx.res.lat_vel_mps, (int)s);
        prev = s;
    }
    printf("cam %+.1fm auto %d: warnings 0-10s %d, 10-30s %d (steady, want 0), left departure 30-35s %d (first %.2fs), right departure 38-43s %d, used corr %+.2f\n",
           (double)cam, autoc, warn_early, warn_mid, warn_dep, (double)first_dep, warn_dep_r, (double)ctx.center_used_m);
    return 0;
}
