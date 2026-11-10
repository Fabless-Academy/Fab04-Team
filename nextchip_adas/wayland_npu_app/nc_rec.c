#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/stat.h>                            // [27-3] mkdir
#include "nc_rec.h"
#include "nc_video_codec.h"
#include "nc_utils.h"

#define REC_W         (640)
#define REC_H         (384)
#define REC_NV12_SZ   ((size_t)(REC_W * REC_H * 3 / 2))
#define REC_RGB_SZ    ((size_t)(REC_W * REC_H * 3))
#define REC_FPS       (30)
#define REC_PRE_SEC   (5)                        // 경고 이전 저장 시간
#define REC_POST_SEC  (5)                        // 경고 이후 저장 시간
#define RING_N        (REC_FPS * REC_PRE_SEC)    // 링버퍼 칸 수 (150)
#define REC_TMP_DIR   "/tmp"                     // [27-3] 녹화 중에 쓰는 곳 (RAM, 빠름)
#define REC_SAVE_DIR  "/mnt/user_data/rec"       // [27-3] 최종 저장 위치 (SD카드, 재부팅해도 남음)
#define REC_MAX_FILES (30)                       // [11.06] SD카드에 남길 최대 녹화 개수 (약 2.8MB × 30 ≈ 85MB)
#define REC_NEXT_FILE REC_SAVE_DIR "/next_no.txt" // [11.06] 다음 번호 기록 (보드에 시계가 없어 파일 시간으로 최신을 못 찾음)

// 압축된 프레임 1개
typedef struct {
    uint8_t *data;
    uint32_t size;
} rec_frame_t;

static volatile int g_enc_ok = 0;      // 인코더 준비 완료 여부
static uint8_t *g_hdr    = NULL;       // 첫 콜백으로 받은 헤더(SPS/PPS)
static uint32_t g_hdr_sz = 0;

// 링버퍼: 인코더 콜백 스레드만 만짐 → 잠금 필요 없음
static rec_frame_t g_ring[RING_N];
static int g_ring_head = 0;            // 다음에 쓸 칸
static int g_ring_cnt  = 0;            // 채워진 칸 수 (최대 RING_N)

// 녹화 요청: 다른 스레드(경고 코드)가 세우고 콜백 스레드가 처리
static volatile int g_req      = 0;
static volatile int g_req_type = 0;
static volatile int g_busy     = 0;    // 파일 쓰는 중이면 1

// 녹화 파일: 콜백 스레드만 만짐
static FILE *g_fp        = NULL;
static int   g_post_left = 0;          // 경고 이후 더 쓸 프레임 수
static int   g_file_no   = 0;          // 다음 파일 번호 (시작할 때 SD카드를 보고 정함)
static char  g_path[96];               // [27-3] 지금 쓰고 있는 /tmp 파일 경로

// render → 변환 스레드로 넘기는 최신 프레임 1장
static uint8_t         g_slot[REC_W * REC_H * 3];
static int             g_slot_full = 0;
static pthread_mutex_t g_slot_mtx  = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  g_slot_cv   = PTHREAD_COND_INITIALIZER;

// RGB888(packed) → NV12 변환 (BT.601)
// NV12 = Y 평면(w*h) 다음에 U,V가 번갈아 들어간 평면(w*h/2)
static void rgb_to_nv12(const uint8_t *rgb, uint8_t *nv12, int w, int h)
{
    uint8_t *y_plane  = nv12;
    uint8_t *uv_plane = nv12 + w * h;

    for (int j = 0; j < h; j++) {
        for (int i = 0; i < w; i++) {
            const uint8_t *p = rgb + (j * w + i) * 3;
            int r = p[0], g = p[1], b = p[2];

            y_plane[j * w + i] = (uint8_t)(((66 * r + 129 * g + 25 * b + 128) >> 8) + 16);

            if ((j % 2 == 0) && (i % 2 == 0)) {   // 2x2 블록의 왼쪽 위 픽셀에서만 색 계산
                int u = ((-38 * r - 74 * g + 112 * b + 128) >> 8) + 128;
                int v = ((112 * r - 94 * g - 18 * b + 128) >> 8) + 128;
                uv_plane[(j / 2) * w + i]     = (uint8_t)u;
                uv_plane[(j / 2) * w + i + 1] = (uint8_t)v;
            }
        }
    }
}

// 링버퍼에 압축 프레임 1개 넣기 (가장 오래된 칸을 덮어씀)
static void ring_push(const uint8_t *buf, uint32_t size)
{
    rec_frame_t *f = &g_ring[g_ring_head];
    free(f->data);                                // 덮어쓸 칸의 옛 프레임 해제 (NULL이어도 안전)
    f->data = (uint8_t *)malloc(size);
    memcpy(f->data, buf, size);
    f->size = size;
    g_ring_head = (g_ring_head + 1) % RING_N;
    if (g_ring_cnt < RING_N) g_ring_cnt++;
}

// 링버퍼 내용을 오래된 순서대로 파일에 쓰기
static void ring_dump(FILE *fp)
{
    int start = (g_ring_head - g_ring_cnt + RING_N) % RING_N;   // 가장 오래된 칸
    for (int i = 0; i < g_ring_cnt; i++) {
        rec_frame_t *f = &g_ring[(start + i) % RING_N];
        fwrite(f->data, 1, f->size, fp);
    }
}

// [11.06] 다음 파일 번호 정하기: 0 ~ REC_MAX_FILES-1 을 돌려 씀
// 1) 지난번에 기록해 둔 번호가 있으면 그 번호부터 (앱을 다시 켜도 이어서 씀)
// 2) 기록이 없으면 비어 있는 첫 번호, 다 차 있으면 0
static int next_file_no(void)
{
    FILE *fp = fopen(REC_NEXT_FILE, "r");
    if (fp != NULL) {
        int n = -1;
        int ok = fscanf(fp, "%d", &n);
        fclose(fp);
        if (ok == 1 && n >= 0 && n < REC_MAX_FILES) return n;
    }

    char p[96];
    for (int n = 0; n < REC_MAX_FILES; n++) {
        int used = 0;
        for (int t = 0; t <= 9; t++) {
            snprintf(p, sizeof(p), "%s/rec_%03d_type%d.h264", REC_SAVE_DIR, n, t);
            if (access(p, F_OK) == 0) { used = 1; break; }
        }
        if (!used) return n;
    }
    return 0;
}

// [27-3] /tmp 파일을 SD카드로 복사하고 /tmp 쪽은 지움 (느린 SD 쓰기가 콜백을 막지 않게 별도 스레드)
static void *save_thread(void *arg)
{
    char *src = (char *)arg;
    const char *name = strrchr(src, '/') + 1;
    char dst[128];
    snprintf(dst, sizeof(dst), "%s/%s", REC_SAVE_DIR, name);

    // [11.06] 같은 번호의 옛 파일은 타입과 상관없이 지움 (번호를 돌려 쓰니까)
    int no = atoi(name + 4);                      // "rec_005_type0.h264" -> 5
    char old[128];
    for (int t = 0; t <= 9; t++) {
        snprintf(old, sizeof(old), "%s/rec_%03d_type%d.h264", REC_SAVE_DIR, no, t);
        remove(old);                              // 없으면 실패하지만 상관없음
    }

    FILE *in  = fopen(src, "rb");
    FILE *out = fopen(dst, "wb");
    int ok = (in != NULL && out != NULL);
    if (ok) {
        static uint8_t buf[65536];
        size_t n;
        while ((n = fread(buf, 1, sizeof(buf), in)) > 0) fwrite(buf, 1, n, out);
        fflush(out);
        fsync(fileno(out));                       // SD카드에 실제로 기록될 때까지 기다림
    }
    if (in)  fclose(in);
    if (out) fclose(out);

    if (ok) {
        remove(src);
        printf("[REC] saved %s\n", dst);

        // [11.06] 다음 번호 기록 (앱을 다시 켜도 이어서 씀)
        FILE *nf = fopen(REC_NEXT_FILE, "w");
        if (nf != NULL) {
            fprintf(nf, "%d\n", (no + 1) % REC_MAX_FILES);
            fflush(nf);
            fsync(fileno(nf));
            fclose(nf);
        }
    } else {
        printf("[REC] save fail %s\n", dst);
    }
    fflush(stdout);
    free(src);
    return NULL;
}

// 인코더가 1프레임 압축을 끝낼 때마다 불림 (인코더 스레드)
static void enc_cb(Uint8 *buf, Uint32 size)
{
    // 첫 콜백은 헤더 → 저장만 하고 끝
    if (g_hdr == NULL) {
        g_hdr = (uint8_t *)malloc(size);
        memcpy(g_hdr, buf, size);
        g_hdr_sz = size;
        printf("[REC] header %u bytes\n", size);
        fflush(stdout);
        nc_unlock_encoding_done();
        return;
    }

    // 1) 녹화 요청이 왔으면: /tmp에 파일 열고 헤더 + 경고 이전 5초(링버퍼) 쓰기
    if (g_req && g_fp == NULL) {
        snprintf(g_path, sizeof(g_path), "%s/rec_%03d_type%d.h264", REC_TMP_DIR, g_file_no, g_req_type);
        g_file_no = (g_file_no + 1) % REC_MAX_FILES;   // [11.06] 0 ~ 29 돌려 씀
        g_fp = fopen(g_path, "wb");
        if (g_fp != NULL) {
            fwrite(g_hdr, 1, g_hdr_sz, g_fp);
            ring_dump(g_fp);
            g_post_left = REC_FPS * REC_POST_SEC;
            printf("[REC] start %s (pre %d frames)\n", g_path, g_ring_cnt);
            fflush(stdout);
        } else {
            g_busy = 0;
        }
        g_req = 0;
    }

    // 2) 녹화 중이면 지금 프레임도 파일에 (경고 이후 5초)
    if (g_fp != NULL) {
        fwrite(buf, 1, size, g_fp);
        if (--g_post_left == 0) {
            fclose(g_fp);
            g_fp = NULL;
            g_busy = 0;
            printf("[REC] done %s\n", g_path);
            fflush(stdout);

            // [27-3] SD카드 복사는 별도 스레드에 맡기고 콜백은 바로 진행
            pthread_t th;
            if (pthread_create(&th, NULL, save_thread, strdup(g_path)) == 0)
                pthread_detach(th);
        }
    }

    // 3) 항상 링버퍼에 보관
    ring_push(buf, size);

    nc_unlock_encoding_done();            // 다 썼다고 알려야 다음 프레임 진행
}

// nc_start_encoder()는 인코딩이 끝날 때까지 돌아오지 않아서 별도 스레드에서 실행
static void *enc_thread(void *arg)
{
    (void)arg;
    CODEC_ERR_STATE st = nc_start_encoder();
    printf("[REC] encoder thread end %d\n", (int)st);   // 정상이면 앱 끝날 때까지 안 찍힘
    fflush(stdout);
    g_enc_ok = 0;
    return NULL;
}

// 변환 스레드: 최신 프레임을 NV12로 바꿔 인코더에 보냄 (render 시간을 줄이려고 분리)
static void *conv_thread(void *arg)
{
    (void)arg;
    static uint8_t rgb[REC_W * REC_H * 3];   // 변환 중에 render가 슬롯을 덮어써도 되게 따로 복사
    uint64_t conv_sum = 0;
    int      conv_cnt = 0;

    while (1) {
        pthread_mutex_lock(&g_slot_mtx);
        while (!g_slot_full) pthread_cond_wait(&g_slot_cv, &g_slot_mtx);   // 새 프레임 올 때까지 대기
        memcpy(rgb, g_slot, REC_RGB_SZ);
        g_slot_full = 0;
        pthread_mutex_unlock(&g_slot_mtx);

        if (!g_enc_ok) continue;

        uint64_t t0 = nc_get_mono_us_time();
        uint8_t *nv12 = (uint8_t *)malloc(REC_NV12_SZ);
        rgb_to_nv12(rgb, nv12, REC_W, REC_H);
        uint64_t t1 = nc_get_mono_us_time();
        if (nc_send_buf_to_encode(nv12, 1) != CODEC_SUCCESS)   // 1 = 인코더가 다 쓰고 free
            printf("[REC] send fail\n");

        // 확인용: NV12 변환 시간 (10초마다 1번 출력)
        conv_sum += t1 - t0;
        if (++conv_cnt == REC_FPS * 10) {
            printf("[REC] conv %.2f ms\n", (double)conv_sum / (double)conv_cnt / 1000.0);
            fflush(stdout);
            conv_sum = 0; conv_cnt = 0;
        }
    }
    return NULL;
}

int nc_rec_init(void)
{
    // [27-3] 저장 폴더 만들고 다음 파일 번호 정하기
    mkdir(REC_SAVE_DIR, 0755);            // 이미 있으면 실패하지만 상관없음
    g_file_no = next_file_no();

    ENCParameter p;
    memset(&p, 0, sizeof(p));
    p.enc_width      = REC_W;
    p.enc_height     = REC_H;
    p.enc_mode       = RC_MODE_CBR;     // 예제 ini 값
    p.enc_codec_type = AVC;             // h264: PC 재생 호환성
    p.enc_framerate  = REC_FPS;
    p.enc_bitrate    = 2300000;         // 예제 ini 값 (bps)
    p.enc_gop        = 1;               // 모든 프레임 I-frame → 링버퍼 어디서 잘라도 재생
    p.enc_qp         = 35;
    p.enc_minqp      = 25;
    p.enc_maxqp      = 40;

    CODEC_ERR_STATE st = nc_init_encoder(&p);
    if (st != CODEC_SUCCESS) {
        printf("[REC] init fail %d\n", (int)st); fflush(stdout);
        return -1;
    }

    nc_register_encode_callback(enc_cb);

    pthread_t th;
    if (pthread_create(&th, NULL, enc_thread, NULL) != 0) {
        printf("[REC] thread create fail\n"); fflush(stdout);
        return -1;
    }
    pthread_detach(th);   // join 안 함 (앱 끝날 때까지 돎)

    g_enc_ok = 1;

    // 변환 스레드 시작 (return보다 앞에 있어야 실행됨)
    pthread_t th2;
    if (pthread_create(&th2, NULL, conv_thread, NULL) != 0) {
        printf("[REC] conv thread create fail\n"); fflush(stdout);
        g_enc_ok = 0;
        return -1;
    }
    pthread_detach(th2);

    printf("[REC] encoder ready %dx%d, next file %03d\n", REC_W, REC_H, g_file_no);
    fflush(stdout);
    return 0;
}

// render에서 매 프레임 호출: 슬롯에 복사만 하고 바로 돌아감 (변환은 conv_thread)
void nc_rec_push_frame(const uint8_t *rgb)
{
    if (!g_enc_ok) return;

    pthread_mutex_lock(&g_slot_mtx);
    memcpy(g_slot, rgb, REC_RGB_SZ);
    g_slot_full = 1;
    pthread_cond_signal(&g_slot_cv);       // 변환 스레드 깨우기
    pthread_mutex_unlock(&g_slot_mtx);

    // 테스트용: /tmp/rec_test 파일이 생기면 녹화 요청 (type 9 = 테스트)
    if (access("/tmp/rec_test", F_OK) == 0) {
        remove("/tmp/rec_test");
        nc_rec_trigger(9);
    }
}

// 경고가 켜지는 순간 1번 호출 (0 = 차선 이탈, 1 = 끼어들기). 어느 스레드에서 불러도 됨
void nc_rec_trigger(int type)
{
    if (!g_enc_ok) return;
    if (g_busy) {                         // 이미 녹화 중이면 무시
        printf("[REC] busy, trigger %d ignored\n", type);
        fflush(stdout);
        return;
    }
    g_busy     = 1;
    g_req_type = type;
    g_req      = 1;                       // 마지막에 세워야 콜백이 type을 제대로 읽음
}

// [11.06] 앱 종료 때 1번 호출: 인코더를 멈춰 VPU를 돌려줌
// (안 하면 VPU가 사용 중으로 남아 다음 실행이 nc_init_encoder 근처에서 멈춤)
void nc_rec_deinit(void)
{
    if (!g_enc_ok) return;
    g_enc_ok = 0;                          // 새 프레임을 인코더로 안 보냄 (push_frame, conv_thread)
    printf("[REC] encoder stop\n");
    fflush(stdout);
    nc_stop_encoder();
    printf("[REC] encoder stopped\n");    // 이 줄이 안 찍히면 nc_stop_encoder()에서 멈춘 것
    fflush(stdout);
}

// 녹화 중이면 1 (화면 REC 표시용)
int nc_rec_is_recording(void) { return g_busy; }

// 녹화 남은 시간(초, 올림). 녹화 중이 아니거나 이후 프레임을 다 썼으면 0
int nc_rec_get_remain_sec(void)
{
    int left = g_post_left;
    if (left <= 0) return 0;
    return (left + REC_FPS - 1) / REC_FPS;
}