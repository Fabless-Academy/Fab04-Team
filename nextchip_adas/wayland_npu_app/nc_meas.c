#include <stdio.h>
#include <time.h>
#include "nc_meas.h"
#include "nc_utils.h"

#define TEMP_PATH       "/sys/class/hwmon/hwmon0/temp1_input"
#define FAKE_TEMP_PATH  "/tmp/fake_temp"
#define NPU_CLK_MHZ     (1500.0f)    // clk_summary: CLKDIV_VAL_NPU_CORE = 1.5 GHz
#define MEAS_LOG        (1)          // 1이면 1초마다 [MEAS] 로그 출력

// 최근 1초 평균 (화면·다른 모듈이 읽는 값)
static float g_up_ms, g_exec_ms, g_down_ms, g_post_ms, g_hw_ms, g_max_ms;
static float g_npu_fps, g_mhz, g_temp_c;
static int   g_temp_fake = 0;

static int g_temp_stuck = 0;   // PVT 센서 raw 0x7FF 고정 감지

static float g_temp_sensor_last = 0.0f;   // 마지막으로 읽은 진짜 센서 값 (STUCK일 때 대신 씀, fake는 안 들어감)

// PVT raw가 0x7FF(2047)로 고정되면 som 보정으로 31609 m°C가 나옴
// (2047-1931)*60/(2984-1931)+25 = 31.61 °C, 1코드 = 약 57 m°C라 정상 센서가 계속 이 값일 확률은 거의 없음
#define TEMP_STUCK_MILLI   31609L
#define TEMP_STUCK_TOL     2L

// 1초 동안 모으는 합계
static float    s_up, s_exec, s_down, s_post, s_hw, s_max;
static uint32_t s_cnt;
static uint64_t s_win_start_us;

// 지연 측정용 (render 스레드에서만 기록)
static float    g_lat_ms, g_lat_max_ms;      // 최근 1초 평균·최댓값 (다른 모듈이 읽는 값)
static float    s_lat_sum, s_lat_max;
static uint32_t s_lat_cnt;
static uint64_t s_lat_win_us;
static uint64_t s_lat_last_ts;               // 같은 결과를 두 번 세지 않으려고

void nc_meas_npu_hw(uint64_t cycles, uint32_t ms)
{
    s_hw += (float)cycles / (NPU_CLK_MHZ * 1000.0f);             // cycles → ms
    if (ms > 0) g_mhz = (float)cycles / ((float)ms * 1000.0f);   // 클럭 검증용
}

void nc_meas_npu_record(uint64_t up_us, uint64_t exec_us, uint64_t down_us, uint64_t post_us)
{
    float up   = (float)up_us   / 1000.0f;
    float exec = (float)exec_us / 1000.0f;
    float down = (float)down_us / 1000.0f;
    float post = (float)post_us / 1000.0f;
    float total = up + exec + down + post;

    s_up += up;  s_exec += exec;  s_down += down;  s_post += post;
    if (total > s_max) s_max = total;
    s_cnt++;

    uint64_t now = nc_get_mono_us_time();
    if (s_win_start_us == 0) s_win_start_us = now;

    if (now - s_win_start_us >= 1000000) {          // 1초마다 평균 계산
        float n = (float)s_cnt;
        g_up_ms   = s_up   / n;
        g_exec_ms = s_exec / n;
        g_down_ms = s_down / n;
        g_post_ms = s_post / n;
        g_hw_ms   = s_hw   / n;
        g_max_ms  = s_max;
        g_npu_fps = n * 1000000.0f / (float)(now - s_win_start_us);

#if MEAS_LOG
        printf("[MEAS] up %.2f / exec %.2f / down %.2f / post %.2f = %.2f ms (max %.2f) | HW %.2f ms | NPU %.1f fps | %.1f C%s | lat %.1f ms\n",
               g_up_ms, g_exec_ms, g_down_ms, g_post_ms, nc_meas_get_npu_ms(), g_max_ms,
               g_hw_ms, g_npu_fps, g_temp_c, nc_meas_temp_tag(), g_lat_ms);
        fflush(stdout);   // 파일로 저장할 때도 1초마다 바로 기록
#endif
        s_up = s_exec = s_down = s_post = s_hw = s_max = 0.0f;
        s_cnt = 0;
        s_win_start_us = now;
    }
}

void nc_meas_get_npu_parts(float *up, float *exec, float *down, float *post)
{
    *up = g_up_ms;  *exec = g_exec_ms;  *down = g_down_ms;  *post = g_post_ms;
}

float nc_meas_get_npu_ms(void)     { return g_up_ms + g_exec_ms + g_down_ms + g_post_ms; }
float nc_meas_get_npu_max_ms(void) { return g_max_ms; }
float nc_meas_get_npu_fps(void)    { return g_npu_fps; }
float nc_meas_get_npu_hw_ms(void)  { return g_hw_ms; }
float nc_meas_get_npu_mhz(void)    { return g_mhz; }

static int read_milli_c(const char *path, long *milli)
{
    FILE *fp = fopen(path, "r");
    if (fp == NULL) return 0;
    int ok = (fscanf(fp, "%ld", milli) == 1);
    fclose(fp);
    return ok;
}

float nc_meas_get_temp_c(void)
{
    static uint64_t last_us = 0;   // 1초마다 한 번만 읽으려고

    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    uint64_t now = (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;

    if (last_us != 0 && now - last_us < 1000000ULL) return g_temp_c;
    last_us = now;

    long milli = 0;
    int ok;
    if (read_milli_c(FAKE_TEMP_PATH, &milli)) {
        g_temp_fake = 1;
        ok = 1;
    } else {
        g_temp_fake = 0;
        ok = read_milli_c(TEMP_PATH, &milli) && milli > 0 && milli < 125000;
    }
    if (!ok) return g_temp_c;

    // 센서 고정 감지: 고정 값 대신 마지막 진짜 센서 값을 씀
    // (직전 값을 쓰면 fake 값이 남아서 FSM이 그 모드에 갇힘 → 11.05 데모에서 LIGHT 고정)
    long diff = milli - TEMP_STUCK_MILLI;
    if (diff >= -TEMP_STUCK_TOL && diff <= TEMP_STUCK_TOL) {
        g_temp_stuck = 1;
        g_temp_c = g_temp_sensor_last;
        return g_temp_c;
    }
    g_temp_stuck = 0;
    g_temp_c = (float)milli / 1000.0f;
    if (!g_temp_fake) g_temp_sensor_last = g_temp_c;   // 진짜 센서 값만 기억
    return g_temp_c;
}

// 온도 뒤에 붙일 표시 (로그·화면 공용)
const char *nc_meas_temp_tag(void)
{
    if (g_temp_stuck) return " (STUCK)";
    if (g_temp_fake)  return " (FAKE)";
    return "";
}

int nc_meas_temp_is_fake(void) { return g_temp_fake; }

// 지연 기록: render가 결과를 읽을 때 그 결과의 time_stamp를 넘겨줌
void nc_meas_latency_record(uint64_t time_stamp_us)
{
    if (time_stamp_us == 0) return;                   // 시각이 안 들어온 결과는 무시
    if (time_stamp_us == s_lat_last_ts) return;       // 같은 결과를 다시 읽은 경우 무시
    s_lat_last_ts = time_stamp_us;

    uint64_t now = nc_get_mono_us_time();
    if (now < time_stamp_us) return;                  // 시각이 거꾸로면 무시 (안전장치)

    float lat = (float)(now - time_stamp_us) / 1000.0f;
    s_lat_sum += lat;
    if (lat > s_lat_max) s_lat_max = lat;
    s_lat_cnt++;

    if (s_lat_win_us == 0) s_lat_win_us = now;
    if (now - s_lat_win_us >= 1000000ULL) {           // 1초마다 평균 갱신
        g_lat_ms     = s_lat_sum / (float)s_lat_cnt;
        g_lat_max_ms = s_lat_max;
        s_lat_sum = s_lat_max = 0.0f;
        s_lat_cnt = 0;
        s_lat_win_us = now;
    }
}

float nc_meas_get_latency_ms(void)     { return g_lat_ms; }
float nc_meas_get_latency_max_ms(void) { return g_lat_max_ms; }