#include <stdio.h>
#include "nc_model.h"
#include "nc_meas.h"
#include "nc_utils.h"
#include "nc_cnn_aiware_runtime.h"   // NETWORK_TRI_CHIMERA, NETWORK_PELEE_DET
#include "nc_traj.h"

// 온도 기준 (°C): 올라갈 때 / 내려갈 때 (히스테리시스)
#define T_LIGHT_UP     (80.0f)
#define T_LIGHT_DOWN   (75.0f)
#define T_SKIP_UP      (90.0f)
#define T_SKIP_DOWN    (85.0f)
// NPU 점유율 기준 (%)
#define LOAD_UP        (50.0f)
#define LOAD_DOWN      (20.0f)
// 조건이 N초 연속이어야 전환 (디바운스)
#define HOLD_SEC       (3)

static int      g_mode = MODE_NORMAL;
static int      g_up_cnt, g_down_cnt;
static uint64_t g_last_us;
static const char *k_names[] = { "NORMAL", "LIGHT", "SKIP" };

// // 준이의 진짜 nc_traj_reset()이 들어오면 자동으로 그쪽이 쓰임 (weak)
// __attribute__((weak)) void nc_traj_reset(void)
// {
//     printf("[FSM] nc_traj_reset (stub)\n");
//     fflush(stdout);
// }

void nc_model_update(void)
{
    uint64_t now = nc_get_mono_us_time();
    if (g_last_us != 0 && now - g_last_us < 1000000ULL) return;   // 1초마다 판단
    g_last_us = now;

    float t    = nc_meas_get_temp_c();
    float load = nc_meas_get_npu_ms() * nc_meas_get_npu_fps() / 10.0f;   // ms × fps ÷ 1000 × 100
    int go_up = 0, go_down = 0;

    switch (g_mode) {
    case MODE_NORMAL:
        go_up   = (t >= T_LIGHT_UP) || (load >= LOAD_UP);
        break;
    case MODE_LIGHT:
        go_up   = (t >= T_SKIP_UP)  || (load >= LOAD_UP);
        go_down = (t <  T_LIGHT_DOWN) && (load < LOAD_DOWN);
        break;
    default:   // MODE_SKIP
        go_down = (t <  T_SKIP_DOWN)  && (load < LOAD_DOWN);
        break;
    }

    g_up_cnt   = go_up   ? g_up_cnt + 1   : 0;
    g_down_cnt = go_down ? g_down_cnt + 1 : 0;

    int prev = g_mode;
    if (g_up_cnt >= HOLD_SEC)        g_mode++;
    else if (g_down_cnt >= HOLD_SEC) g_mode--;

    if (g_mode != prev) {
        g_up_cnt = g_down_cnt = 0;
        printf("[FSM] %s -> %s (temp %.1f C, load %.0f%%)\n",
               k_names[prev], k_names[g_mode], t, load);
        fflush(stdout);

        // TRICHIMERA ↔ PeleeDet으로 모델 자체가 바뀔 때만 (NORMAL ↔ LIGHT는 같은 모델)
        if ((prev == MODE_SKIP) != (g_mode == MODE_SKIP)) {
            printf("[FSM] model changed -> nc_traj_reset()\n");
            fflush(stdout);
            nc_traj_reset();
        }
    }
}

int nc_model_select(uint64_t frame)
{
    if (g_mode == MODE_NORMAL) return (int)NETWORK_TRI_CHIMERA;
    if (frame % 2 != 0)        return -1;                 // LIGHT·SKIP: 2프레임에 1번
#ifdef SHOW_PELEE_DETECT
    if (g_mode == MODE_SKIP)   return (int)NETWORK_PELEE_DET;
#endif
    return (int)NETWORK_TRI_CHIMERA;
}

int         nc_model_get_mode(void)  { return g_mode; }
const char *nc_model_mode_name(void) { return k_names[g_mode]; }

int nc_model_is_vehicle(int class_idx)
{
#ifdef SHOW_PELEE_DETECT
    if (g_mode == MODE_SKIP)   // PeleeDet: bik0 bus1 car2 mot3 per4 rid5 t-l6 t-s7 tra8 tru9
        return (class_idx == 1 || class_idx == 2 || class_idx == 9);
#endif
    // TRICHIMERA: per0 car1 bus2 tru3 cycle4 mot5
    return (class_idx == 1 || class_idx == 2 || class_idx == 3);
}