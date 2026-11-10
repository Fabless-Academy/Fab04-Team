/**
********************************************************************************
* Copyright (C) 2021 NEXTCHIP Inc. All rights reserved.
* This software is the confidential and proprietary information of
* NEXTCHIP, Inc. ("Confidential Information"). You shall not disclose such
* Confidential Information and shall use it only in accordance with
* the terms of the license agreement you entered into with NEXTCHIP.
********************************************************************************

********************************************************************************
* @file    : wayland_npu_app.c
*
* @brief   : wayland_npu application
*
* @author  : Software Development Team.  NextChip Inc.
*
* @date    : 2024.04.26.
*
* @version : 1.0.0
********************************************************************************
* @note
*
********************************************************************************
*/

/*
********************************************************************************
*               INCLUDES
********************************************************************************
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include <assert.h>
#include <signal.h>
#include <linux/input.h>
#include <fcntl.h>
#include <errno.h>
#include <pthread.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <sys/mman.h>
#include <GLES2/gl2.h>
#include <EGL/egl.h>
#include <time.h>
#include <errno.h>
#include <SOIL.h>
#include <sys/eventfd.h>
#include <arm_neon.h>
#include <omp.h>

#include "wayland_egl.h"
#include "nc_opengl_init.h"
#include "v4l2_interface.h"
#include "nc_opengl_shader.h"
#include "nc_opengl_interface.h"
#include "nc_ts_fsync_flipflop_buffers.h"
#include "nc_app_config_parser.h"
#include "nc_cnn_aiware_runtime.h"
#include "nc_cnn_communicator.h"
#include "nc_cnn_worker_for_postprocess.h"
#include "nc_neon.h"
#ifdef AIWARE_DEVICE_SUPPORTED
#include "aiware/runtime/c/aiwaredevice.h"
#endif
#include "nc_opengl_ttf_font.h"
#ifdef USE_BYTETRACK
#include "nc_cnn_tracker.h"
#endif

#include "nc_meas.h"
#include "nc_model.h"
#include "nc_rec.h"
void nc_rec_deinit(void);   // [11.06] nc_rec.c에 추가 (nc_rec.h에도 넣어두면 좋음)
#include "nc_utils.h"
#include "nc_traj.h"       // [준이] 궤적 예측
#include "nc_cutin.h"      // [준이] 끼어들기 판정
#include "nc_bevlog.h"     // [준이] 진단 로그 (NC_BEV_LOG 환경변수 있을 때만 기록)
#ifdef USE_LDW
#include "nc_ldw.h"        // [준우] 차선 이탈 경고
#include "nc_minimap.h"    // [준이] 미니맵
#include "nc_class.h"      // [준이] 모델별 차량 클래스
#endif

#ifdef USE_8MP_VI
#include "nc_dsr_helper.h"
#include "nc_dsr_set.h"
#include "nc_dmabuf_ctrl_helper.h"
#endif

#ifdef USE_ADAS_LD
#include"ADAS_LD_Lib.h"
#endif

/*
********************************************************************************
*               DEFINES
********************************************************************************
*/

#define VIS0_MAX_CH         (1)
#define VIS1_MAX_CH         (0)
#define VIDEO_MAX_CH        (VIS0_MAX_CH + VIS1_MAX_CH)

#define VIDEO_BUFFER_NUM    (3)

#ifdef USE_8MP_VI
#define VIDEO_WIDTH         MAX_WIDTH_FOR_VDMA_CNN_DS
#define VIDEO_HEIGHT        MAX_HEIGHT_FOR_VDMA_CNN_DS
#else
#define VIDEO_WIDTH         NPU_INPUT_WIDTH
#define VIDEO_HEIGHT        NPU_INPUT_HEIGHT
#endif

#define CAP_FPS             (30)

#define MQ_NAME_CNN_BUF     "/cnn_data"
#define DEV_FILE_DSR        "/dev/dsr"

#ifdef SHOW_PELEE_SEG
    #define NETWORK_FILE_PELEE_SEG      "misc/networks/peleeseg/peleeseg_640x384_apache6sr250_aiw4939.aiwbin"
#endif
#ifdef SHOW_PELEE_DETECT
    #define NETWORK_FILE_PELEE_DET      "misc/networks/peleeDet/peleedet_10class_640x384_apache6sr250_aiw4939.aiwbin"
#endif
#ifdef SHOW_YOLOV5_DETECT
    #define NETWORK_FILE_YOLOV5_DET     "misc/networks/Yolov5s/yolov5s_coco_640x384_apache6sr250_aiw4939.aiwbin"
#endif
#ifdef SHOW_YOLOV8_DETECT
    #define NETWORK_FILE_YOLOV8_DET     "misc/networks/Yolov8/yolov8s_coco_640x384_apache6sr250_aiw4939.aiwbin"
#endif
#ifdef SHOW_UFLD_LANE
    #define NETWORK_FILE_UFLD_LANE      "misc/networks/ufld/ufld_a6sr250_aiw4939.aiwbin"
#endif
#ifdef SHOW_TRI_CHIMERA
    #define NETWORK_FILE_TRI_CHIMERA    "misc/networks/trichimera/trichimera_640x384_a6sr_aiw4939.aiwbin"
#endif
#define COLOR_PALETTE_CNT    (10)
typedef enum {
    RGBA_R = 0,
    RGBA_G,
    RGBA_B,
    RGBA_A,
    RGBA_CNT,
} E_RGBA_IDX;

/*    don't modify  */
#define MAX_WIDTH_FOR_VDMA_CNN_DS   (1920)
#define MAX_HEIGHT_FOR_VDMA_CNN_DS  (1080)
/********************/

/*
********************************************************************************
*               VARIABLE DECLARATIONS
********************************************************************************
*/

st_npu_input_info npu_input_info;
struct gl_npu_program g_npu_prog;
GLuint g_seg_texture;
struct gl_font_program g_font_prog;
Font font_38;
Font font_24;

unsigned char* image_data;
static int running = 1;

st_nc_v4l2_config v4l2_config[VIDEO_MAX_CH];

#ifdef USE_FILE_INPUT
#ifdef USE_8MP_VI
#error "USE_FILE_INPUT does not support USE_8MP_VI"
#endif
// [준이] 파일 입력: 카메라 대신 ffmpeg로 mp4를 디코딩해서 파이프로 받음
// ffmpeg 출력: NPU 입력 크기의 interleaved RGB24, 실제 속도(-re), 무한 반복(-stream_loop -1)
// 파이프 fd를 video_fd에 넣어서 기존 "video_fd != -1" 검사가 그대로 통과함
#define FILE_INPUT_FFMPEG_PATH      "misc/ffmpeg"
#define FILE_INPUT_FRAME_SIZE       (NPU_INPUT_WIDTH * NPU_INPUT_HEIGHT * 3)

/* [준이 d10b2d4] 기본: 아래 영상을 순서대로 재생하고 반복 (없는 파일은 건너뜀), 영상마다 ffmpeg를 새로 띄움
 * "-i <file>" 또는 [11.06 서영] INPUT_VIDEO=<file>: 그 영상 하나만 무한 반복 (-stream_loop -1)
 * 그래도 스트림이 끝나면 ffmpeg를 다시 띄워 처음부터 */
#define FILE_INPUT_PATH_MAX         (256)

static const char *const g_file_input_default_videos[] = {
    "misc/image_npu/1920_1080_0001.mp4",
    "misc/image_npu/1920_1080_0002.mp4",
    "misc/image_npu/1920_1080_0003.mp4",
};
#define FILE_INPUT_MAX_VIDEOS       (sizeof(g_file_input_default_videos) / sizeof(g_file_input_default_videos[0]))

static const char *g_file_input_path = NULL;    // set by "-i <file>", NULL -> INPUT_VIDEO or default list
static char g_file_input_videos[FILE_INPUT_MAX_VIDEOS][FILE_INPUT_PATH_MAX];
static int g_file_input_video_cnt = 0;
static int g_file_input_video_idx[VIDEO_MAX_CH];           // video now playing on each channel
static char g_file_input_ffmpeg[FILE_INPUT_PATH_MAX];
static FILE *g_file_input_fp[VIDEO_MAX_CH];
static unsigned char *g_file_input_frame[VIDEO_MAX_CH];   // interleaved RGB24
#define FILE_INPUT_MAX_RESTART      (3)    // consecutive restarts without a frame -> give up

static void file_input_add_video(const char *path)
{
    if (g_file_input_video_cnt >= (int)FILE_INPUT_MAX_VIDEOS) return;
    if (access(path, R_OK) != 0) {
        printf("[error] input video file not found: %s\n", path);
        return;
    }
    snprintf(g_file_input_videos[g_file_input_video_cnt++], FILE_INPUT_PATH_MAX, "%s", path);
}

// start ffmpeg on the channel's current video. returns NULL on failure
static FILE *file_input_open(int ch)
{
    char cmd[1024];
    const char *video = g_file_input_videos[g_file_input_video_idx[ch]];

    snprintf(cmd, sizeof(cmd),
             "%s -loglevel error -nostdin %s-re -i \"%s\" "
             "-map 0:v:0 -an -vf scale=%d:%d -pix_fmt rgb24 -f rawvideo -",
             g_file_input_ffmpeg, (g_file_input_video_cnt == 1) ? "-stream_loop -1 " : "",
             video, NPU_INPUT_WIDTH, NPU_INPUT_HEIGHT);
    printf("[file input] ch%d : %s\n", ch, cmd);
    return popen(cmd, "r");
}

static int file_input_initialize(void)
{
    snprintf(g_file_input_ffmpeg, sizeof(g_file_input_ffmpeg), "%s", nc_localize_path(FILE_INPUT_FFMPEG_PATH));
    const char *env_video = getenv("INPUT_VIDEO");   // [11.06] run_demo.sh를 안 고치고 영상 바꾸기: INPUT_VIDEO=경로 ./run_demo.sh
    if (g_file_input_path) {
        file_input_add_video(g_file_input_path);
    } else if (env_video && *env_video) {
        file_input_add_video(env_video);
    } else {
        for (size_t i = 0; i < FILE_INPUT_MAX_VIDEOS; i++) {
            file_input_add_video(nc_localize_path(g_file_input_default_videos[i]));
        }
    }

    if (g_file_input_video_cnt == 0) {
        return -1;
    }

    for (int i = 0; i < VIDEO_MAX_CH; i++) {
        g_file_input_frame[i] = (unsigned char *)malloc(FILE_INPUT_FRAME_SIZE);
        if (g_file_input_frame[i] == NULL) {
            printf("[error] file input frame buffer alloc failure!\n");
            return -1;
        }

        g_file_input_fp[i] = file_input_open(i);
        if (g_file_input_fp[i] == NULL) {
            perror("[error] popen(ffmpeg) failure");
            return -1;
        }
        v4l2_config[i].video_buf.video_fd = fileno(g_file_input_fp[i]);
    }

    return 0;
}

// read one full frame (blocking). returns 0 on success, -1 on EOF/error
static int file_input_read_frame(int ch)
{
    size_t total = 0;
    static int restart_cnt[VIDEO_MAX_CH];

    while (total < FILE_INPUT_FRAME_SIZE) {
        size_t n = fread(g_file_input_frame[ch] + total, 1, FILE_INPUT_FRAME_SIZE - total, g_file_input_fp[ch]);
        if (n == 0) {
            // [준이 d10b2d4] stream ended: go to the next video (single video: restart it)
            pclose(g_file_input_fp[ch]);
            g_file_input_fp[ch] = NULL;
            g_file_input_video_idx[ch] = (g_file_input_video_idx[ch] + 1) % g_file_input_video_cnt;
            if (!running || restart_cnt[ch] >= FILE_INPUT_MAX_RESTART ||   // [11.06] 종료 중이면 다시 안 띄움
                (g_file_input_fp[ch] = file_input_open(ch)) == NULL) {
                printf("[error] ffmpeg stream ended (ch%d). check ffmpeg / video file\n", ch);
                v4l2_config[ch].video_buf.video_fd = -1;    // fall back to logo image
                return -1;
            }
            restart_cnt[ch]++;
            v4l2_config[ch].video_buf.video_fd = fileno(g_file_input_fp[ch]);
            total = 0;                                  // drop the partial frame
            continue;
        }
        total += n;
    }

    restart_cnt[ch] = 0;
    return 0;
}

static void file_input_deinitialize(void)
{
    for (int i = 0; i < VIDEO_MAX_CH; i++) {
        if (g_file_input_fp[i]) {
            pclose(g_file_input_fp[i]);
            g_file_input_fp[i] = NULL;
        }
        if (g_file_input_frame[i]) {
            free(g_file_input_frame[i]);
            g_file_input_frame[i] = NULL;
        }
    }
}
#endif /* USE_FILE_INPUT */

struct viewport g_viewport[VIDEO_MAX_CH];

#ifdef USE_8MP_VI
size_t BUFF_SIZE;
int dsr_fd;
DSR_Data_t dsr_info;
dma_alloc_info dma_info;
st_nc_dsr_config dsr_config;
img_input_config dsr_input_config;
img_output_config dsr_output_config;
#endif
/*
********************************************************************************
*               FUNCTION DEFINITIONS
********************************************************************************
*/
#ifdef USE_8MP_VI
static int dsr_init(void)
{
    int input_fd = 0, output_fd = 0;
    long page_size = sysconf(_SC_PAGESIZE);

    dsr_input_config.format = IMG_FORMAT_RGB888;
    dsr_input_config.width = MAX_WIDTH_FOR_VDMA_CNN_DS;
    dsr_input_config.height = MAX_HEIGHT_FOR_VDMA_CNN_DS;
    dsr_output_config.format = IMG_FORMAT_RGB888;

    BUFF_SIZE = dsr_input_config.width * dsr_input_config.height * 3;
    BUFF_SIZE = ((BUFF_SIZE + page_size - 1) / page_size) * page_size;


    if(dsr_config_downscale(&dsr_config, 1, NPU_INPUT_WIDTH, NPU_INPUT_HEIGHT) < 0){
        perror("Error: DSR config fail");
        return -1;
    }

    if(open_device_and_dma_buffers(DEV_FILE_DSR, &dsr_fd, &input_fd, &output_fd, BUFF_SIZE) < 0){
        perror("Error: DSR open fail");
        return -1;
    }

    if(dsr_setup_buffer(&dsr_info, dsr_fd, &dma_info, input_fd, output_fd, BUFF_SIZE) < 0){
        perror("Error: DSR setup fail");
        return -1;
    }

    return 0;
}

static int dsr_deinit(void)
{
    nc_dmabuf_ctrl_end_cpu_access(dma_info.dmabuf_fd_in);
    nc_dmabuf_ctrl_end_cpu_access(dma_info.dmabuf_fd_out);

    nc_dmabuf_ctrl_free_dma_fd(dma_info.dmabuf_fd_in);
    nc_dmabuf_ctrl_free_dma_fd(dma_info.dmabuf_fd_out);

    nc_dmabuf_ctrl_close();

    for (int i = 0; i < BUFF_NUM; i++) {
        if (dsr_info.dsr_in_buf[i] != MAP_FAILED) {
            munmap(dsr_info.dsr_in_buf[i], BUFF_SIZE);
        }
        if (dsr_info.dsr_out_buf[i] != MAP_FAILED) {
            munmap(dsr_info.dsr_out_buf[i], BUFF_SIZE);
        }
    }

    dsr_device_deinit(&dsr_fd);

    return 0;
}
#endif

void set_viewport_config(void)
{
#if(VIDEO_MAX_CH == 1)
    // full screen view
    g_viewport[0].x         = 0;
    g_viewport[0].y         = 0;
    g_viewport[0].width     = WINDOW_WIDTH;
    g_viewport[0].height    = WINDOW_HEIGHT;
#elif(VIDEO_MAX_CH > 1)
    // quad view
    for(int i = 0; i < VIDEO_MAX_CH; i++)
    {
        g_viewport[i].width     = WINDOW_WIDTH/2;
        g_viewport[i].height    = WINDOW_HEIGHT/2;

        // Set the view position for each channel
        switch(i)
        {
            case 0:
                g_viewport[i].x = 0;
                g_viewport[i].y = WINDOW_HEIGHT/2;
                break;
            case 1:
                g_viewport[i].x = WINDOW_WIDTH/2;
                g_viewport[i].y = WINDOW_HEIGHT/2;
                break;
            case 2:
                g_viewport[i].x = 0;
                g_viewport[i].y = 0;
                break;
            case 3:
                g_viewport[i].x = WINDOW_WIDTH/2;
                g_viewport[i].y = 0;
                break;
            default:
                break;
        }
    }
#endif
}

void set_v4l2_config(void)
{
    int i = 0, j = 0;

    for(i = 0; i < VIS0_MAX_CH; i++) {
        v4l2_config[i].video_buf.video_device_num = CNN_DEVICE_NUM(VISION0) + i;
        v4l2_config[i].video_buf.video_fd         = -1;
#ifdef USE_8MP_VI
        v4l2_config[i].dma_mode                   = INTERLEAVE;
#else
        v4l2_config[i].dma_mode                   = PLANAR;
#endif
        v4l2_config[i].img_process                = MODE_DS;
        v4l2_config[i].pixformat                  = V4L2_PIX_FMT_RGB24;
        v4l2_config[i].crop_x_start               = 0;
        v4l2_config[i].crop_y_start               = 0;
        v4l2_config[i].crop_width                 = 0;
        v4l2_config[i].crop_height                = 0;
#ifdef USE_8MP_VI
        v4l2_config[i].ds_width                   = MAX_WIDTH_FOR_VDMA_CNN_DS;
        v4l2_config[i].ds_height                  = MAX_HEIGHT_FOR_VDMA_CNN_DS;
#else
        v4l2_config[i].ds_width                   = VIDEO_WIDTH;
        v4l2_config[i].ds_height                  = VIDEO_HEIGHT;
#endif
    }

    for(j = VIS0_MAX_CH; j < VIDEO_MAX_CH; j++) {
        v4l2_config[j].video_buf.video_device_num = CNN_DEVICE_NUM(VISION1) + j;
        v4l2_config[j].video_buf.video_fd         = -1;
#ifdef USE_8MP_VI
        v4l2_config[j].dma_mode                   = INTERLEAVE;
#else
        v4l2_config[j].dma_mode                   = PLANAR;
#endif
        v4l2_config[j].img_process                = MODE_DS;
        v4l2_config[j].pixformat                  = V4L2_PIX_FMT_RGB24;
        v4l2_config[j].crop_x_start               = 0;
        v4l2_config[j].crop_y_start               = 0;
        v4l2_config[j].crop_width                 = 0;
        v4l2_config[j].crop_height                = 0;
#ifdef USE_8MP_VI
        v4l2_config[j].ds_width                   = MAX_WIDTH_FOR_VDMA_CNN_DS;
        v4l2_config[j].ds_height                  = MAX_HEIGHT_FOR_VDMA_CNN_DS;
#else
        v4l2_config[j].ds_width                   = VIDEO_WIDTH;
        v4l2_config[j].ds_height                  = VIDEO_HEIGHT;
#endif
    }
}

int send_cnn_buf (uint8_t *ptr_cnn_buf, uint64_t time_stamp_us, uint32_t cam_ch, E_NETWORK_UID net_id)
{
    int ret = 0;
    stCnnData *cnn_data;
    struct mq_attr attr;
    attr.mq_maxmsg = MAX_MQ_MSG_CNT;
    attr.mq_msgsize = sizeof(stCnnData*);
    int oflag = O_WRONLY | O_CREAT;
    mqd_t mfd = mq_open(MQ_NAME_CNN_BUF, oflag, 0666, &attr);
    if (mfd == -1) {
        perror("mq open error");
        return -1;
    }

    cnn_data = (stCnnData*)malloc(sizeof(stCnnData));
    cnn_data->cam_ch = cam_ch;
    cnn_data->ptr_cnn_buf = ptr_cnn_buf;
    cnn_data->time_stamp_us = time_stamp_us;
    cnn_data->net_id = net_id;

    if ((ret = mq_send(mfd, (const char *)&cnn_data, attr.mq_msgsize, 1)) == -1) {
        printf("errno of mq_send = %d\n", errno);
    }
    mq_close(mfd);

    return ret;
}

int receive_cnn_buf (stCnnData **out_cnn_buf)
{
    int ret = 0;
    struct mq_attr attr;
    attr.mq_maxmsg = MAX_MQ_MSG_CNT;
    attr.mq_msgsize = sizeof(stCnnData *);
    int oflag = O_RDONLY | O_CREAT;
    mqd_t mfd = mq_open(MQ_NAME_CNN_BUF, oflag, 0666, &attr);
    if (mfd == -1) {
        perror("mq open error");
        return -1;
    }

    if ((ret = (int32_t)mq_receive(mfd, (char*)out_cnn_buf, attr.mq_msgsize, NULL)) == -1) {
        printf("errno of mq_receive = %d\n", errno);
    }
    mq_close(mfd);

    return ret;
}

int v4l2_initialize(void)
{
#ifdef USE_FILE_INPUT
    return file_input_initialize();   // [준이] 카메라 대신 파일 입력
#endif
    for(int i = 0; i < VIDEO_MAX_CH; i++)
    {
        v4l2_config[i].video_buf.video_fd = nc_v4l2_open(v4l2_config[i].video_buf.video_device_num, true);
        if(v4l2_config[i].video_buf.video_fd == errno) {
            printf("[error] nc_v4l2_open() failure!\n");
        } else {
            if(nc_v4l2_init_device_and_stream_on(&v4l2_config[i], VIDEO_BUFFER_NUM) < 0) {
                printf("[error] nc_v4l2_init_device_and_stream_on() failure!\n");
                return -1;
            }
        }
    }

    nc_v4l2_show_user_config(&v4l2_config[0], VIDEO_MAX_CH);

    return 0;
}

// [준이] 지금 화면에 띄운 프레임의 캡처 시각 (채널별, render 스레드에서만 읽고 씀)
static uint64_t g_disp_frame_ts_us[VIDEO_MAX_CH];

// [준이] 이 길이(화면 px)보다 짧은 예측선은 안 그림
#define PRED_LINE_MIN_PX    (15.0f)

// [준이] 1: 박스 라벨에 도로 거리 "z m(x)" 붙이기 (디버그, 라벨이 겹칠 수 있음)
#define SHOW_OBJ_DISTANCE   (0)

// [준이] 끼어들기 경고 박스·글자 색 (노랑: 차 클래스는 이미 빨강)
// [11.06] 노랑 -> 주황: 노랑 = LDW 약한 경고, 빨강 = LDW 강한 경고와 구분 (분홍은 차선 선 색이라 피함)
static float g_cutin_color[4] = {1.0f, 0.5f, 0.0f, 1.0f};

// 화면 글자 그림자: 검정으로 2px 비켜 먼저 그리고 위에 색 글자 (밝은 하늘·파란 도로 위에서도 보이게)
// [merge 11.05] HUD 상세 표시 스위치 (/tmp/hud_detail). 미니맵 위치도 이 값으로 정함
static int g_hud_detail = 0;

static void draw_text_shadow(Font *font, const char *text, float x, float y, float r, float g, float b)
{
    float black[3] = {0.0f, 0.0f, 0.0f};
    float color[3] = {r, g, b};
    nc_opengl_draw_text(font, text, x + 2.0f, y - 2.0f, 1.0f, black, WINDOW_WIDTH, WINDOW_HEIGHT, g_font_prog);
    nc_opengl_draw_text(font, text, x, y, 1.0f, color, WINDOW_WIDTH, WINDOW_HEIGHT, g_font_prog);
}

void nc_draw_gl_npu(struct viewport viewport, int network_task, pp_result_buf *net_result, struct gl_npu_program g_npu_prog)
{
    stCnnPostprocessingResults *det_result = &net_result->cnn_result;
    stObjDrawInfo *draw_cnn = &net_result->draw_info;
    stSegDrawInfo *draw_seg = &net_result->seg_info;
    stLaneDrawInfo *draw_lane = &net_result->lane_draw_info;

    int max_class_cnt = draw_cnn->max_class_cnt;
    int max_seg_class_cnt = draw_seg->max_class_cnt;
    float target_view_ratio = 1.f;
    char buftext[128];

    target_view_ratio = (float)WINDOW_HEIGHT / (float)viewport.height;

    glViewport(viewport.x, viewport.y, viewport.width, viewport.height);

    if(network_task == DETECTION)
    {
        float** color = (float**)malloc(max_class_cnt * sizeof(float*));
        for (int i = 0; i < max_class_cnt; ++i) {
            color[i] = (float*)malloc(RGBA_CNT * sizeof(float));
        }

        for (int j = 0; j < max_class_cnt; ++j) {
            color[j][RGBA_R] = (float)(draw_cnn->class_colors[j].r) / 255.0f;
            color[j][RGBA_G] = (float)(draw_cnn->class_colors[j].g) / 255.0f;
            color[j][RGBA_B] = (float)(draw_cnn->class_colors[j].b) / 255.0f;
            color[j][RGBA_A] = 1.0f;
        }

        // cut-in judgement runs once per displayed frame of this channel
        int cut_ch = (net_result->cam_channel < VIDEO_MAX_CH) ? (int)net_result->cam_channel : -1;
        // [11.06] 검출 모델이 바뀌면(SKIP 들어가거나 나올 때) 궤적·끼어들기 기록 초기화
        //         클래스 번호와 추적 번호가 모델마다 달라서, 옛 모델 기록이 새 모델 차에 이어 붙지 않게
        if (cut_ch >= 0) {
            static int last_det_net[VIDEO_MAX_CH];
            static int last_det_net_init = 0;
            if (!last_det_net_init) {
                for (int k = 0; k < VIDEO_MAX_CH; k++) last_det_net[k] = -1;
                last_det_net_init = 1;
            }
            int cur_net = (int)net_result->net_id;
            if (last_det_net[cut_ch] >= 0 && last_det_net[cut_ch] != cur_net) {
                nc_traj_reset();
                nc_cutin_reset();
                printf("[mode] ch%d detection net %d -> %d: traj / cut-in reset\n", cut_ch, last_det_net[cut_ch], cur_net);
            }
            last_det_net[cut_ch] = cur_net;
        }
        if (cut_ch >= 0) nc_cutin_begin_frame(cut_ch, g_disp_frame_ts_us[cut_ch]);
#ifdef USE_LDW
        if (cut_ch >= 0) nc_minimap_begin_frame((uint32_t)cut_ch, g_disp_frame_ts_us[cut_ch]);   // [merge 11.05] 준이 미니맵 헤더에 맞춰 화면 프레임 시각 추가
#endif

        for(int i=0; i<draw_cnn->max_class_cnt; i++){
            for(int bidx = 0; bidx < det_result->class_objs[i].obj_cnt; bidx++) {
                stObjInfo obj_info = det_result->class_objs[i].objs[bidx];
                // predicted box if available (vehicle, enough history), else measured box.
                // Only the drawn copy changes; det_result / tracker input stay measured.
                int   has_future = 0;
                int   pred_valid = 0;
                float fut_cx = 0.f, fut_cy = 0.f;
                if (net_result->cam_channel < VIDEO_MAX_CH) {
                    int      tch    = (int)net_result->cam_channel;
                    uint64_t disp_t = g_disp_frame_ts_us[tch];
                    stBBox pred_box;
                    if (nc_traj_get_predicted_box(tch, i, obj_info.track_id, disp_t, &pred_box)) {
                        obj_info.bbox = pred_box;
                        pred_valid = 1;
                        // future center (same snapshot) for the prediction line
                        has_future = nc_traj_get_future_center_fast(tch, i, obj_info.track_id, disp_t,   // [11.09] 끼어들기 판단은 빠른 필터
                                                               nc_cutin_get_horizon_ms(),   // [merge 11.06] 준우 수정: 끼어들기 예측 시간(기본 1초, CUTIN_HORIZON_MS)과 같은 값
                                                               &fut_cx, &fut_cy);
                    }
                }
                // cut-in: future bottom-center enters the ego lane (lane coordinates)
                int is_cutin = 0;
                if (has_future) {
                    is_cutin = nc_cutin_eval(cut_ch, i, obj_info.track_id, &obj_info.bbox,
                                             fut_cx, fut_cy, g_disp_frame_ts_us[cut_ch]);
                }
                float *dcolor = is_cutin ? g_cutin_color : color[i];

                nc_opengl_draw_rectangle(obj_info.bbox.x, obj_info.bbox.y, obj_info.bbox.w, obj_info.bbox.h, dcolor, g_npu_prog);
                if (is_cutin) { // thicker box
                    nc_opengl_draw_rectangle(obj_info.bbox.x + 3.f, obj_info.bbox.y + 3.f,
                                             obj_info.bbox.w - 6.f, obj_info.bbox.h - 6.f, dcolor, g_npu_prog);
                }

                // prediction line: current (predicted-now) center -> center after FUTURE_MS, dashed
                // Hidden when shorter than PRED_LINE_MIN_PX: cars followed at similar speed barely
                // move on screen, and their short line only shows bbox jitter.
                float cur_cx = obj_info.bbox.x + obj_info.bbox.w * 0.5f;
                float cur_cy = obj_info.bbox.y + obj_info.bbox.h * 0.5f;
                float ldx = fut_cx - cur_cx, ldy = fut_cy - cur_cy;
                // [11.06] 예측 경로(굵은 선, 끝점, 1초 뒤 박스)는 끼어들기 경고 차에만 그림 (일반 차에 붙은 선은 지저분해서 뺌)
                if (is_cutin && has_future && (ldx * ldx + ldy * ldy) >= PRED_LINE_MIN_PX * PRED_LINE_MIN_PX) {
                    // [11.06] 경로를 잘 보이게: ① 점선 굵게 ② 1초 뒤 예측 박스 ③ 끝점 키움
                    // ① 선 방향에 수직으로 -1.5 / 0 / +1.5px 밀어서 3번 그림 -> 약 3px 굵기
                    float llen = sqrtf(ldx * ldx + ldy * ldy);
                    float nx = -ldy / llen, ny = ldx / llen;
                    for (int k = -1; k <= 1; k++) {
                        float ox = nx * 1.5f * (float)k, oy = ny * 1.5f * (float)k;
                        nc_opengl_draw_line(cur_cx + ox, cur_cy + oy, fut_cx + ox, fut_cy + oy, 2 /* dashed */, dcolor, g_npu_prog);
                    }
                    // ② 1초 뒤 예측 박스: 끼어들기 경고 차만 (평소 화면은 깔끔하게, 경고 이유만 보여줌)
                    //    지금 박스 크기 그대로, 예측 중심에 점선으로 (실제 박스는 실선이라 구분됨)
                    {
                        float gx0 = fut_cx - obj_info.bbox.w * 0.5f, gy0 = fut_cy - obj_info.bbox.h * 0.5f;
                        float gx1 = gx0 + obj_info.bbox.w,          gy1 = gy0 + obj_info.bbox.h;
                        nc_opengl_draw_line(gx0, gy0, gx1, gy0, 2, dcolor, g_npu_prog);
                        nc_opengl_draw_line(gx1, gy0, gx1, gy1, 2, dcolor, g_npu_prog);
                        nc_opengl_draw_line(gx1, gy1, gx0, gy1, 2, dcolor, g_npu_prog);
                        nc_opengl_draw_line(gx0, gy1, gx0, gy0, 2, dcolor, g_npu_prog);
                    }
                    // ③ 끝점 8px -> 12px
                    nc_opengl_draw_rectangle(fut_cx - 6.f, fut_cy - 6.f, 12.f, 12.f, dcolor, g_npu_prog);
                }

                // draw bbox label
#ifdef USE_BYTETRACK
                // show track id (not cnn probability)
                if (obj_info.track_id < 0) sprintf(buftext, "%s:%0.2f", draw_cnn->class_names[i], obj_info.prob);
                else sprintf(buftext, "[%d]%s:%0.2f", obj_info.track_id, draw_cnn->class_names[i], obj_info.prob);
#else
                sprintf(buftext, "%s:%0.2f", draw_cnn->class_names[i], obj_info.prob);
#endif
                if (is_cutin) sprintf(buftext, "[%d]CUT-IN", obj_info.track_id);
#ifdef USE_LDW
                // minimap: vehicles only (position from the bbox bottom-center)
                if (cut_ch >= 0 && nc_class_is_vehicle(net_result->net_id, i)) {
                    stNcMinimapObj mo;
                    memset(&mo, 0, sizeof(mo));
                    mo.class_name = draw_cnn->class_names[i];
                    mo.track_id   = obj_info.track_id;
                    mo.meas       = det_result->class_objs[i].objs[bidx].bbox;   // tracker output
                    mo.shown      = obj_info.bbox;                               // drawn (predicted) box
                    mo.pred_valid = pred_valid;
                    mo.t_det_us   = net_result->time_stamp;
                    mo.t_disp_us  = g_disp_frame_ts_us[cut_ch];
                    memcpy(mo.color, dcolor, sizeof(mo.color));
                    mo.highlight  = is_cutin;
                    // [준이 final_1st] 미니맵 화살표: NC_TRAJ_FUTURE_MS_DEFAULT(0.5초) 뒤 박스 아래 가운데 (박스 크기 유지)
                    {
                        float mfx = 0.f, mfy = 0.f;
                        mo.fut_valid = pred_valid &&
                                       nc_traj_get_future_center(cut_ch, i, obj_info.track_id, g_disp_frame_ts_us[cut_ch],
                                                                 (float)NC_TRAJ_FUTURE_MS_DEFAULT, &mfx, &mfy);
                        mo.fut_u     = mfx;
                        mo.fut_v     = mfy + obj_info.bbox.h * 0.5f;
                    }
                    nc_minimap_add_obj((uint32_t)cut_ch, &mo);
                }
#endif
#if defined(USE_LDW) && SHOW_OBJ_DISTANCE
                // distance check: road position of the bbox bottom-center (ground contact point)
                if (cut_ch >= 0) {
                    float x_m, z_m;
                    if (nc_ldw_app_pixel_to_road((uint32_t)cut_ch,
                                                 obj_info.bbox.x + obj_info.bbox.w * 0.5f,
                                                 obj_info.bbox.y + obj_info.bbox.h,
                                                 &x_m, &z_m) == 0 && z_m > 0.f) {
                        size_t n = strlen(buftext);
                        snprintf(buftext + n, sizeof(buftext) - n, " %.1fm(%+.1f)", z_m, x_m);
                    }
                }
#endif
                float textcolor[3] = {dcolor[RGBA_R], dcolor[RGBA_G], dcolor[RGBA_B]};
                nc_opengl_draw_text(&font_24, buftext, obj_info.bbox.x, (float)WINDOW_HEIGHT - (obj_info.bbox.y-12), target_view_ratio, textcolor, WINDOW_WIDTH, WINDOW_HEIGHT, g_font_prog);
            }
        }

        if (cut_ch >= 0) {
            if (nc_cutin_end_frame(cut_ch)) {
                nc_rec_trigger(1);   // [서영] 끼어들기 경고가 꺼짐 -> 켜짐 되는 순간에만 녹화 (type 1)
            }
            if (nc_cutin_active_count(cut_ch) > 0) {
                // [merge 11.05] 원래 y 1020(맨 위)은 측정 1줄과 겹쳐서, LDW 경고 문구(y 780) 아래 y 720으로 + 그림자
                draw_text_shadow(&font_38, "CUT-IN WARNING", (float)WINDOW_WIDTH * 0.5f - 160.f, 720.f,
                                 g_cutin_color[RGBA_R], g_cutin_color[RGBA_G], g_cutin_color[RGBA_B]);
            }
        }

        for (int i = 0; i < max_class_cnt; ++i) {
            free(color[i]);
        }
        free(color);
    }
    else if(network_task == SEGMENTATION)
    {
        glBindTexture(GL_TEXTURE_2D, g_seg_texture);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE, draw_seg->width, draw_seg->height, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, det_result->seg);

        float color[COLOR_PALETTE_CNT][RGBA_CNT];

        for (int j = 0; j < max_seg_class_cnt; ++j) {
            color[j][RGBA_R] = (float)(draw_seg->class_colors[j].r) / 255.0f;
            color[j][RGBA_G] = (float)(draw_seg->class_colors[j].g) / 255.0f;
            color[j][RGBA_B] = (float)(draw_seg->class_colors[j].b) / 255.0f;
            color[j][RGBA_A] = (float)(draw_seg->class_colors[j].a) / 255.0f;
        }
        nc_opengl_draw_segmentation(g_seg_texture, (float**)color, draw_seg->max_class_cnt, g_npu_prog);
    }
    else if(network_task == LANE)
    {
        float color[COLOR_PALETTE_CNT][RGBA_CNT];

        for (int j = 0; j < draw_lane->max_lane_num; ++j) {
            color[j][RGBA_R] = (float)(draw_lane->index_colors[j].r) / 255.0f;
            color[j][RGBA_G] = (float)(draw_lane->index_colors[j].g) / 255.0f;
            color[j][RGBA_B] = (float)(draw_lane->index_colors[j].b) / 255.0f;
            color[j][RGBA_A] = 1.0f;
        }

        for(int i = 0; i < draw_lane->max_lane_num; i++)
        {
            int point_num = det_result->lane_det[i].point_cnt;
            int lane_class = det_result->lane_det[i].lane_class;

            if(point_num == 0) continue;

            for(int j=0; j<point_num-1; j++)
            {
                float st_x = det_result->lane_det[i].point[j].x;
                float st_y = det_result->lane_det[i].point[j].y;
                float end_x = det_result->lane_det[i].point[j+1].x;
                float end_y = det_result->lane_det[i].point[j+1].y;

                nc_opengl_draw_line(st_x, st_y, end_x, end_y, lane_class, color[i], g_npu_prog);
            }
        }

#ifdef USE_UFLD_NETWORK_DEBUGGING
        float color_table[6][RGBA_CNT] = {{1.0f,1.0f,1.0f,0.15f}, // white (grid)
                                          {1.0f,0.0f,0.0f,1.0f},  // red (index 0)
                                          {0.0f,0.0f,1.0f,1.0f},  // blue (index 1)
                                          {1.0f,1.0f,0.0f,1.0f},  // yellow (index 2)
                                          {0.0f,1.0f,0.0f,1.0f},  // green (index 3)
                                          {0.0f,0.0f,0.0f,1.0f}}; // black (final point)
        stUFLD_dbg_info* debug_info = NULL;
        int info_cnt = 0;

#if defined(DBG_ROW_ANCHOR)

        float row_cell_num = (float)(draw_lane->row_cell_num);
        float row_cell_width = WINDOW_WIDTH/(float)(row_cell_num-1);
        float row_cell_height = (draw_lane->row_anchor_max - draw_lane->row_anchor_min) * WINDOW_HEIGHT * (1/ (float)(draw_lane->row_anchor_num - 1));

        //draw row anchor grid
        for(int i=0 ; i<(draw_lane->row_anchor_num); i++)
        {
            nc_opengl_draw_debugging_grid_line(0, draw_lane->row_anchor[i] * WINDOW_HEIGHT, WINDOW_WIDTH-1, draw_lane->row_anchor[i] * WINDOW_HEIGHT, 1, color_table[0], g_npu_prog);
        }

        for(float j=0; j<row_cell_num; j++)
        {
            nc_opengl_draw_debugging_grid_line(j/(row_cell_num-1)*WINDOW_WIDTH, 0, j/(row_cell_num-1)*WINDOW_WIDTH, WINDOW_HEIGHT-1, 1, color_table[0], g_npu_prog);
        }

        for(int i = 0; i < draw_lane->max_lane_num; i++)
        {
            if(draw_lane->lane_anchor_info[i] != 0) continue;

            info_cnt = det_result->lane_det[i].dbg_info_cnt;
            if(info_cnt == 0) continue;

            for(int j=0; j<info_cnt; j++)
            {
                debug_info = &(det_result->lane_det[i].ufld_dbg_info[j]);

                //draw max_indices
                nc_opengl_draw_rectangle(debug_info->x, debug_info->y, row_cell_width, row_cell_height , color_table[i+1], g_npu_prog);

                //draw stddev
                nc_opengl_draw_line(debug_info->index_min, debug_info->y + row_cell_height/2, debug_info->index_max, debug_info->y + row_cell_height/2, 1, color_table[i+1], g_npu_prog);

                //draw final result (point)
                nc_opengl_draw_rectangle(debug_info->final_point.x-3, debug_info->final_point.y-3, 6, 6, color_table[5], g_npu_prog);
            }
        }

#elif defined(DBG_COL_ANCHOR)

        float col_cell_num = (float)(draw_lane->col_cell_num);
        float col_cell_width = (draw_lane->col_anchor_max - draw_lane->col_anchor_min) * WINDOW_WIDTH * (1/ (float)(draw_lane->col_anchor_num - 1));
        float col_cell_height = WINDOW_HEIGHT/(float)(col_cell_num-1);

        //draw column anchor grid
        for(int i=0 ; i<(draw_lane->col_anchor_num); i++)
        {
            nc_opengl_draw_debugging_grid_line(draw_lane->col_anchor[i] * WINDOW_WIDTH, 0, draw_lane->col_anchor[i] * WINDOW_WIDTH, WINDOW_HEIGHT-1, 1, color_table[0], g_npu_prog);
        }

        for(float j=0; j<col_cell_num; j++)
        {
            nc_opengl_draw_debugging_grid_line(0, j/(col_cell_num-1)*WINDOW_HEIGHT, WINDOW_WIDTH-1, j/(col_cell_num-1)*WINDOW_HEIGHT, 1, color_table[0], g_npu_prog);
        }

        for(int i = 0; i < draw_lane->max_lane_num; i++)
        {
            if(draw_lane->lane_anchor_info[i] != 1) continue;

            info_cnt = det_result->lane_det[i].dbg_info_cnt;
            if(info_cnt == 0) continue;

            for(int j=0; j<info_cnt; j++)
            {
                debug_info = &(det_result->lane_det[i].ufld_dbg_info[j]);

                //draw max_indices
                nc_opengl_draw_rectangle(debug_info->x, debug_info->y, col_cell_width, col_cell_height , color_table[i+1], g_npu_prog);

                //draw stddev
                nc_opengl_draw_line(debug_info->x + col_cell_width/2, debug_info->index_min, debug_info->x + col_cell_width/2, debug_info->index_max, 1, color_table[i+1], g_npu_prog);

                //draw final result (point)
                nc_opengl_draw_rectangle(debug_info->final_point.x-3, debug_info->final_point.y-3, 6, 6, color_table[5], g_npu_prog);
            }
        }
#endif
#endif
    }
    else
    {
        // printf("Invalid network %d\n", sel_network);
    }
}

void gl_initialize(struct window *window)
{
    int width, height, channels;
    char buf[128];

    sprintf(buf, "misc/image/nextchip_s.png");
    image_data = SOIL_load_image(buf, &width, &height, &channels, SOIL_LOAD_RGBA);

    // init video shader
    nc_opengl_init_video_shader(window, 0);

    // init texture for video draw
    for(int i=0;i<VIDEO_MAX_CH;i++)
    {
        glGenTextures(1, &window->gl.texture[i]);
        glBindTexture(GL_TEXTURE_2D, window->gl.texture[i]);

        //stbi_set_flip_vertically_on_load(true);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, image_data);
        //glGenerateMipmap(GL_TEXTURE_2D);

        // set texture filtering parameters
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    }

    // Load the font using the freetype library.
    nc_opengl_load_font("misc/font/NotoSans-Regular.ttf", 38, &font_38);
    nc_opengl_load_font("misc/font/NotoSans-Regular.ttf", 24, &font_24);
    // init font shader
    nc_opengl_init_font_shader(&g_font_prog);

    // init npu shader
    nc_opengl_init_npu_shader(&g_npu_prog);

    // init texture for segmentation draw
    glGenTextures(1, &g_seg_texture);
    glBindTexture(GL_TEXTURE_2D, g_seg_texture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glBindTexture(GL_TEXTURE_2D, 0);

    // set viewport
    glViewport(0, 0, WINDOW_WIDTH, WINDOW_HEIGHT);
}

// #define SHOW_FPS
#ifdef SHOW_FPS
float calc_fps_at_loop_ent1(int update_period_fcnt)
{
    static uint64_t s_time = 0;
    static float fps = 0.f;
    static uint64_t fcnt = 0;
    uint64_t elapsed_ms = 0;
    fcnt++;
    if (fcnt % update_period_fcnt == 1) {
        if (s_time == 0) {
            s_time = nc_get_mono_time();
        } else {
            elapsed_ms = nc_elapsed_time(s_time);
            // printf("fcnt(%d) elapsed_ms(%d)\n", fcnt, elapsed_ms);
            if (elapsed_ms > 0) fps = (float)((fcnt-1) / (elapsed_ms/1000.f));

            // re-init
            fcnt = 1;
            s_time = nc_get_mono_time();
        }
    }

    return fps;
}
#endif

void render(void *data, struct wl_callback *callback, uint32_t time)
{
    static struct timespec begin, end, set_time;
    static uint64_t fpstime = 0;
    static uint64_t fpscount = 0;
    static uint64_t fpscount_00 = 0;
    static uint64_t frametime = 0;
    static uint64_t opengl_time = 0;
    static uint64_t framecnt = 0;
    int networkOrder[VIDEO_MAX_CH];

    (void)time;

    clock_gettime(CLOCK_MONOTONIC, &begin);
    struct window *window = (struct window *)data;
    // uint64_t start_time = 0;

    assert(window->callback == callback);
    window->callback = NULL;

    if (callback)
        wl_callback_destroy(callback);

    if (!window->configured)
        return;

    glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    nc_model_update();

    for(int i = 0;i<VIDEO_MAX_CH;i++)
    {
        if (v4l2_config[i].video_buf.video_fd == -1)
        {
            // Upload texture from logo image data
            glBindTexture(GL_TEXTURE_2D, window->gl.texture[i]);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, VIDEO_WIDTH, VIDEO_HEIGHT, 0, GL_RGBA, GL_UNSIGNED_BYTE, image_data);

        }else
        {
#ifdef USE_FILE_INPUT
            // [준이] 파일 입력: ffmpeg에서 interleaved RGB24 한 프레임 읽기 (막히면 다음 프레임까지 대기)
            if (file_input_read_frame(i) == 0)
            {
                // [서영] 카메라 경로와 같은 FSM 연결: 고른 모델로 추론, -1이면 이번 프레임 건너뜀
                uint64_t time_stamp_us = nc_get_mono_us_time();   // 프레임을 꺼낸 시각
                g_disp_frame_ts_us[i] = time_stamp_us;            // [준이] 화면 프레임 시각 (궤적 예측·끼어들기 기준)
                networkOrder[i] = nc_model_select(framecnt);
                if (networkOrder[i] >= 0) {
                    unsigned char* rgbdata_for_cnn = (unsigned char *)malloc(npu_input_info.rgb_size);
                    // 파일 입력은 interleaved라서 NPU용으로 planar 변환 (카메라와 반대 방향)
                    nc_rgb_interleaved_to_planar_neon(g_file_input_frame[i] \
                                                    ,(unsigned char *)rgbdata_for_cnn \
                                                    ,(unsigned char *)rgbdata_for_cnn + NPU_INPUT_DATA_SIZE \
                                                    ,(unsigned char *)rgbdata_for_cnn + NPU_INPUT_DATA_SIZE*2 \
                                                    , NPU_INPUT_WIDTH, NPU_INPUT_HEIGHT);
                    send_cnn_buf (rgbdata_for_cnn, time_stamp_us, (uint32_t)i, (E_NETWORK_UID)networkOrder[i]);
                }

                // 화면: 파일 프레임은 이미 interleaved라서 변환 없이 그대로 올림
                glBindTexture(GL_TEXTURE_2D, window->gl.texture[i]);
                glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, VIDEO_WIDTH, VIDEO_HEIGHT, 0, GL_RGB, GL_UNSIGNED_BYTE, g_file_input_frame[i]);
        #ifdef USE_LDW
                nc_minimap_set_frame((uint32_t)i, g_file_input_frame[i], VIDEO_WIDTH, VIDEO_HEIGHT);   // [11.09 서영] 준이 미니맵 노란 중앙선: 영상 색으로 판단
        #endif

                nc_rec_push_frame(g_file_input_frame[i]);   // [서영] 녹화용 (버퍼는 다음 프레임에 덮어써지니 여기서 넘김)
        #ifdef SHOW_FPS
                printf(" -- fps(%0.1f)\n", calc_fps_at_loop_ent1(20));
        #endif
            }
#else
            struct v4l2_buffer video_buf;
            CLEAR(video_buf);

            // Queue the buffer for capturing a new frame
            if (nc_v4l2_dequeue_buffer(v4l2_config[i].video_buf.video_fd, &video_buf) == -1) {
                //printf("Error VIDIOC_DQBUF buffer %d\n", video_buf.index);
            }else
            {
#if (VIDEO_MAX_CH == 4)
                if (((framecnt % 2 == 0) && (i == 0 || i == 2)) ||
                    ((framecnt % 2 == 1) && (i == 1 || i == 3)))
#endif

/*
                {
                    uint64_t time_stamp_us = 0;
                    networkOrder[i] = nc_get_cnn_networks_id();
                    unsigned char* rgbdata_for_cnn = (unsigned char *)malloc(npu_input_info.rgb_size);
#ifdef USE_8MP_VI
                    memcpy(dsr_info.dsr_in_buf[0], (uint8_t *)v4l2_config[i].video_buf.buffers[video_buf.index].start, MAX_WIDTH_FOR_VDMA_CNN_DS*MAX_HEIGHT_FOR_VDMA_CNN_DS*RGB_CNT);
                    dsr_downscale(dsr_fd, &dsr_info, dsr_input_config, dsr_output_config, dsr_config, 0);
                    nc_rgb_interleaved_to_planar_neon((unsigned char *)dsr_info.dsr_out_buf[0] \
                                                    ,(unsigned char *)rgbdata_for_cnn \
                                                    ,(unsigned char *)rgbdata_for_cnn + NPU_INPUT_WIDTH*NPU_INPUT_HEIGHT \
                                                    ,(unsigned char *)rgbdata_for_cnn + NPU_INPUT_WIDTH*NPU_INPUT_HEIGHT*2 \
                                                    , NPU_INPUT_WIDTH,  NPU_INPUT_HEIGHT);
#else
                    memcpy(rgbdata_for_cnn, (unsigned char *)v4l2_config[i].video_buf.buffers[video_buf.index].start, npu_input_info.rgb_size); // 600us
#endif
                    send_cnn_buf (rgbdata_for_cnn, time_stamp_us, (uint32_t)i, (E_NETWORK_UID)networkOrder[i]);// 50us
                    // printf("send cnn msg : %llu us\n", nc_elapsed_us_time(start_time));
                }
*/

                {
                    uint64_t time_stamp_us = nc_get_mono_us_time();   // 프레임을 꺼낸 시각 (준이 timestamp 패치)
                    g_disp_frame_ts_us[i] = time_stamp_us;            // [준이] 화면 프레임 시각
                    networkOrder[i] = nc_model_select(framecnt);   // FSM이 고른 모델, -1이면 이번 프레임 건너뜀
                    if (networkOrder[i] >= 0) {
                        unsigned char* rgbdata_for_cnn = (unsigned char *)malloc(npu_input_info.rgb_size);
#ifdef USE_8MP_VI
                        memcpy(dsr_info.dsr_in_buf[0], (uint8_t *)v4l2_config[i].video_buf.buffers[video_buf.index].start, MAX_WIDTH_FOR_VDMA_CNN_DS*MAX_HEIGHT_FOR_VDMA_CNN_DS*RGB_CNT);
                        dsr_downscale(dsr_fd, &dsr_info, dsr_input_config, dsr_output_config, dsr_config, 0);
                        nc_rgb_interleaved_to_planar_neon((unsigned char *)dsr_info.dsr_out_buf[0] \
                                                        ,(unsigned char *)rgbdata_for_cnn \
                                                        ,(unsigned char *)rgbdata_for_cnn + NPU_INPUT_WIDTH*NPU_INPUT_HEIGHT \
                                                        ,(unsigned char *)rgbdata_for_cnn + NPU_INPUT_WIDTH*NPU_INPUT_HEIGHT*2 \
                                                        , NPU_INPUT_WIDTH,  NPU_INPUT_HEIGHT);
#else
                        memcpy(rgbdata_for_cnn, (unsigned char *)v4l2_config[i].video_buf.buffers[video_buf.index].start, npu_input_info.rgb_size); // 600us
#endif
                        send_cnn_buf (rgbdata_for_cnn, time_stamp_us, (uint32_t)i, (E_NETWORK_UID)networkOrder[i]);// 50us
                    }
                }

#ifndef USE_8MP_VI
                unsigned char *interleaved_rgb = (unsigned char *)malloc(NPU_INPUT_WIDTH*NPU_INPUT_HEIGHT*3);
                // start_time = nc_get_mono_us_time();
                nc_rgb_planar_to_interleaved_neon((uint8_t*)v4l2_config[i].video_buf.buffers[video_buf.index].start \
                                            ,(uint8_t*)v4l2_config[i].video_buf.buffers[video_buf.index].start + NPU_INPUT_DATA_SIZE \
                                            ,(uint8_t*)v4l2_config[i].video_buf.buffers[video_buf.index].start + (NPU_INPUT_DATA_SIZE*2) \
                                            ,interleaved_rgb, NPU_INPUT_WIDTH, NPU_INPUT_HEIGHT);// NPU ON:3ms, NPU OFF:600us
                // printf("conv interleaved: %llu us\n", nc_elapsed_us_time(start_time));
#endif
                if(nc_v4l2_queue_buffer(v4l2_config[i].video_buf.video_fd, video_buf.index) == -1) {
                    printf("Error VIDIOC_QBUF buffer %d\n", video_buf.index);
                }

                // Upload texture from video buffer
                glBindTexture(GL_TEXTURE_2D, window->gl.texture[i]);
#ifdef USE_8MP_VI
                glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, VIDEO_WIDTH, VIDEO_HEIGHT, 0, GL_RGB, GL_UNSIGNED_BYTE, v4l2_config[i].video_buf.buffers[video_buf.index].start);
#else
                glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, VIDEO_WIDTH, VIDEO_HEIGHT, 0, GL_RGB, GL_UNSIGNED_BYTE, interleaved_rgb);
        #ifdef USE_LDW
                nc_minimap_set_frame((uint32_t)i, interleaved_rgb, VIDEO_WIDTH, VIDEO_HEIGHT);   // [11.09 서영] 준이 미니맵 노란 중앙선: 영상 색으로 판단
        #endif

                nc_rec_push_frame(interleaved_rgb);   // 녹화용 (free 전에 넘겨야 함)                

                if(interleaved_rgb){
                    // printf("free inter\n");
                    free(interleaved_rgb);
                }
#endif
        #ifdef SHOW_FPS
                printf(" -- fps(%0.1f)\n", calc_fps_at_loop_ent1(20));
        #endif
            }
#endif /* USE_FILE_INPUT */
        }

        // draw video texture
        glViewport(g_viewport[i].x, g_viewport[i].y, g_viewport[i].width, g_viewport[i].height);
        nc_opengl_draw_texture(window->gl.texture[i], window);
    }

#ifdef USE_ADAS_LD
    draw_ld_output_opengl();
#endif

    // draw cnn network result
    for(uint32_t ch = 0; ch < VIDEO_MAX_CH; ch++)
    {
        if (v4l2_config[ch].video_buf.video_fd == -1){

        }
        else{
            uint64_t time_stamp = 0;

        #ifdef DETECT_NETWORK
            pp_result_buf *det_buf = NULL;
            det_buf = (pp_result_buf *)nc_tsfs_ff_get_readable_buffer_and_timestamp(ch+DETECT_NETWORK, &time_stamp);
            if (det_buf) {
                nc_meas_latency_record(time_stamp);   // 이 결과가 나온 프레임의 시각으로 지연 계산
                // [준이] 화면 프레임 시각까지 궤적 예측 (프레임당 짧은 잠금 한 번) 후 그리기
                nc_traj_predict_frame((int)ch, g_disp_frame_ts_us[ch]);
                nc_draw_gl_npu(g_viewport[ch], det_buf->net_task, det_buf, g_npu_prog);
            }
            nc_tsfs_ff_finish_read_buf(ch+DETECT_NETWORK);
        #endif

        #ifdef SEGMENT_NETWORK
            pp_result_buf *seg_buf = NULL;
            seg_buf = (pp_result_buf *)nc_tsfs_ff_get_readable_buffer_and_timestamp(ch+SEGMENT_NETWORK, &time_stamp);
            if (seg_buf && nc_model_get_mode() != MODE_SKIP) {
                nc_draw_gl_npu(g_viewport[ch], seg_buf->net_task, seg_buf, g_npu_prog);
            }
            nc_tsfs_ff_finish_read_buf(ch+SEGMENT_NETWORK);
        #endif

        #ifdef LANE_NETWORK
            pp_result_buf *lane_buf = NULL;
            lane_buf = (pp_result_buf *)nc_tsfs_ff_get_readable_buffer_and_timestamp(ch+LANE_NETWORK, &time_stamp);
            if (lane_buf && nc_model_get_mode() != MODE_SKIP) {
                nc_draw_gl_npu(g_viewport[ch], lane_buf->net_task, lane_buf, g_npu_prog);
            }
        #ifdef USE_LDW
            nc_ldw_app_process(ch, lane_buf);   // [준우] 버퍼 잠금 안에서 처리 (SKIP이면 LDW 안에서 멈춤)
        #endif
            nc_tsfs_ff_finish_read_buf(ch+LANE_NETWORK);
        #ifdef USE_LDW
            {
                // [준우] 잠금을 푼 뒤 그리기
                stNcLdwDrawRes ldw_res = { &font_24, &font_38, &g_font_prog, &g_npu_prog };
                nc_ldw_app_draw(ch, g_viewport[ch], &ldw_res);

                // [준이] 미니맵 (차선 기준 도로 좌표라 LDW가 있어야 동작)
                stNcMinimapDrawRes mm_res = { &font_24, &g_font_prog, &g_npu_prog };
                nc_minimap_set_panel_y(g_hud_detail ? 235.0f : 165.0f);   // [merge 11.05] 상세 2줄 켜면 그 아래로 내림
                nc_minimap_draw(ch, g_viewport[ch], &mm_res);
            }
        #endif
        #endif
        }
    }

    framecnt++;
    fpscount+=1;
    char buftext[256];

    // sprintf(buftext,"GL: %lums, Frame: %lums/%lufps", opengl_time, frametime, fpscount_00);

    // glViewport(0,0, WINDOW_WIDTH, WINDOW_HEIGHT);
    // float textcolor[3] = {1.0, 0.0, 0.0}; // R, G, B
    // nc_opengl_draw_text(&font_38, buftext, 10, 1020, 1.0f, textcolor, WINDOW_WIDTH, WINDOW_HEIGHT, g_font_prog);

    
    float up, ex, dn, pp;
    nc_meas_get_npu_parts(&up, &ex, &dn, &pp);

    glViewport(0,0, WINDOW_WIDTH, WINDOW_HEIGHT);
    // 화면 표시 색 역할: 흰색 = 측정 / 하늘색 = LDW 동작 중 / 회색 = LDW 꺼짐 / 노랑·빨강 = 경고 / 빨강 = REC
    // 구성: 1줄 모드·모델 / 2줄 NPU·지연·온도 / 3줄 LDW 상태(nc_ldw_app_draw 또는 LDW OFF) (모두 큰 글씨) / 상세 2줄(작은 글씨, /tmp/hud_detail 있을 때만)

    // 상세 표시 스위치: /tmp/hud_detail 파일이 있으면 켬 (1초에 한 번만 확인)
    static uint64_t hud_check_us = 0;
    uint64_t hud_now_us = nc_get_mono_us_time();
    if (hud_now_us - hud_check_us >= 1000000ULL) {
        g_hud_detail = (access("/tmp/hud_detail", F_OK) == 0);
        hud_check_us = hud_now_us;
    }

    // 1줄 요약: 모드(+ 지금 돌리는 모델과 주기) | NPU 처리 시간·fps | 지연 | 온도
    // MULTI 빌드 기준: NORMAL = TRICHIMERA 매 프레임, LIGHT = TRICHIMERA 2프레임에 1번, SKIP = PeleeDet 2프레임에 1번
    int mode = nc_model_get_mode();
    const char *net_name = (mode == MODE_SKIP)   ? "PeleeDet"    : "TRICHIMERA";
    const char *net_rate = (mode == MODE_NORMAL) ? "every frame" : "1/2 frames";
    char temp_str[32];
    float temp_c = nc_meas_get_temp_c();
    if (temp_c <= 0.0f) snprintf(temp_str, sizeof(temp_str), "--%s", nc_meas_temp_tag());   // 센서가 처음부터 고정이면 --
    else                snprintf(temp_str, sizeof(temp_str), "%.1fC%s", temp_c, nc_meas_temp_tag());
    // 1줄: 모드와 지금 돌리는 모델
    sprintf(buftext, "Mode %s (%s, %s)", nc_model_mode_name(), net_name, net_rate);
    draw_text_shadow(&font_38, buftext, 10.0f, 1020.0f, 1.0f, 1.0f, 1.0f);
    // 2줄: 성능·온도 요약
    sprintf(buftext, "NPU %.1fms %.0ffps | Lat %.0fms | Temp %s",
            nc_meas_get_npu_ms(), nc_meas_get_npu_fps(), nc_meas_get_latency_ms(), temp_str);
    draw_text_shadow(&font_38, buftext, 10.0f, 975.0f, 1.0f, 1.0f, 1.0f);

    // 3줄 LDW 상태: 평소엔 준우 nc_ldw_app_draw()가 같은 자리(y 930)에 LDW 수치를 그림,
    //               SKIP일 땐 차선 모델이 안 돌아서 여기서 LDW OFF
    if (mode == MODE_SKIP) {
        sprintf(buftext, "LDW OFF (lane model not running)");
        draw_text_shadow(&font_38, buftext, 10.0f, 930.0f, 0.5f, 0.5f, 0.5f);   // 회색: LDW 꺼짐 (흰색 측정값과 구분되게 0.5)
    }

    // 상세 2줄 (작은 글씨): 분석·캡처할 때만 켜서 봄
    if (g_hud_detail) {
        sprintf(buftext, "NPU %.2fms (max %.2f) = up %.2f + exec %.2f + down %.2f + post %.2f | HW %.2fms @ 1500MHz",
                nc_meas_get_npu_ms(), nc_meas_get_npu_max_ms(), up, ex, dn, pp, nc_meas_get_npu_hw_ms());
        draw_text_shadow(&font_24, buftext, 10.0f, 890.0f, 1.0f, 1.0f, 1.0f);
        sprintf(buftext, "Display %lufps (GL %lums) | Lat %.1fms (max %.1f)",
                fpscount_00, opengl_time, nc_meas_get_latency_ms(), nc_meas_get_latency_max_ms());
        draw_text_shadow(&font_24, buftext, 10.0f, 860.0f, 1.0f, 1.0f, 1.0f);
    }

    // 녹화 중 표시 (오른쪽 위): 글씨는 계속 보이게(남은 초 읽기용), 빨간 박스만 0.3초마다 깜빡임
    if (nc_rec_is_recording()) {
        uint64_t now_ms = nc_get_mono_us_time() / 1000ULL;
        if ((now_ms / 300ULL) % 2ULL == 0ULL) {
            // 사각형 좌표는 위에서부터 y (글자는 아래에서부터 y라서 기준이 반대)
            // 글씨(x W-175 ~ 약 W-55, 폭 약 120px)가 가운데 오도록 박스 x W-195 ~ W-35, y 38 ~ 92
            // LDW 경고 테두리(가장자리에서 4~16px)와 겹치지 않게 20px 아래·25px 오른쪽으로 옮김
            float red4[4] = {1.0f, 0.0f, 0.0f, 1.0f};
            const float bx = (float)WINDOW_WIDTH - 195.0f;
            nc_opengl_draw_rectangle(bx, 38.0f, 160.0f, 54.0f, red4, g_npu_prog);
            nc_opengl_draw_rectangle(bx + 3.0f, 41.0f, 154.0f, 48.0f, red4, g_npu_prog);
        }
        int remain = nc_rec_get_remain_sec();
        if (remain > 0) sprintf(buftext, "REC %ds", remain);
        else            sprintf(buftext, "REC");
        draw_text_shadow(&font_38, buftext, (float)WINDOW_WIDTH - 175.0f, 1000.0f, 1.0f, 0.0f, 0.0f);
    }


    clock_gettime(CLOCK_MONOTONIC, &end);

    opengl_time = ((end.tv_sec - begin.tv_sec)*1000 + (end.tv_nsec - begin.tv_nsec)/1000000);
    // [준이] 진단 로그 (NC_BEV_LOG 환경변수가 있을 때만 기록)
    nc_bevlog_row(NC_BEVLOG_FRAME, "%llu,%lu,%lu", (unsigned long long)nc_get_mono_us_time(),
                  (unsigned long)opengl_time, (unsigned long)frametime);

    frametime = ((end.tv_sec - set_time.tv_sec)*1000 + (end.tv_nsec - set_time.tv_nsec)/1000000);

    set_time = end;

    fpstime = fpstime + frametime;

    if(fpstime>=1000)
    {
        if(fpscount_00>0)
        {
            fpscount_00 = (fpscount_00 + fpscount)/2;
        }
        else
        {
            fpscount_00=fpscount;
        }

        fpscount=0;
        fpstime=0;
    }

    nc_wayland_display_draw(window,(void *)render);
}

int npu_init(st_npu_input_info *npu_input_info)
{
    /* Initialize CNN */
    if (nc_aiw_init_cnn() < 0 ) {
        fprintf(stderr, "nc_aiw_init_cnn() failure!!\n");
        return -1;
    }
#ifdef SHOW_YOLOV8_DETECT
    if (nc_aiw_add_network_to_builder(nc_localize_path((const char *)NETWORK_FILE_YOLOV8_DET), NETWORK_YOLOV8_DET, nc_postprocess_yolov8_inference_result) < 0) {
        fprintf(stderr, "nc_aiw_add_network_to_builder() failure!!\n");
        return -1;
    }
#endif
#ifdef SHOW_YOLOV5_DETECT
    if (nc_aiw_add_network_to_builder(nc_localize_path((const char *)NETWORK_FILE_YOLOV5_DET), NETWORK_YOLOV5_DET, nc_postprocess_yolov5_inference_result) < 0) {
        fprintf(stderr, "nc_aiw_add_network_to_builder() failure!!\n");
        return -1;
    }
#endif
#ifdef SHOW_PELEE_SEG
    if (nc_aiw_add_network_to_builder(nc_localize_path((const char *)NETWORK_FILE_PELEE_SEG), NETWORK_PELEE_SEG, nc_postprocess_segmentation_inference_result) < 0) {
        fprintf(stderr, "nc_aiw_add_network_to_builder() failure!!\n");
        return -1;
    }
#endif
#ifdef SHOW_PELEE_DETECT
    if (nc_aiw_add_network_to_builder(nc_localize_path((const char *)NETWORK_FILE_PELEE_DET), NETWORK_PELEE_DET, nc_postprocess_pelee_inference_result) < 0) {
        fprintf(stderr, "nc_aiw_add_network_to_builder() failure!!\n");
        return -1;
    }
#endif
#ifdef SHOW_UFLD_LANE
    if (nc_aiw_add_network_to_builder(nc_localize_path((const char *)NETWORK_FILE_UFLD_LANE), NETWORK_UFLD_LANE, nc_postprocess_ufld_inference_result) < 0) {
        fprintf(stderr, "nc_aiw_add_network_to_builder() failure!!\n");
        return -1;
    }
#endif
#ifdef SHOW_TRI_CHIMERA
    if (nc_aiw_add_network_to_builder(nc_localize_path((const char *)NETWORK_FILE_TRI_CHIMERA), NETWORK_TRI_CHIMERA, nc_postprocess_trichimera_inference_result) < 0) {
        fprintf(stderr, "nc_aiw_add_network_to_builder() failure!!\n");
        return -1;
    }
#endif
    if(nc_aiw_finish_network_builder() < 0 ) {
        fprintf(stderr, "nc_aiw_finish_network_builder() failure!!\n");
        return -1;
    }
    // obtain the input resoltution for the CNN network
    // get information of the input tensor
    aiwTensorInfo in_tinfo;
#ifdef SHOW_YOLOV8_DETECT
    if(nc_get_cnn_network_input_resol(NETWORK_YOLOV8_DET, &in_tinfo) < 0)
    {
        printf("failed to get the input resolution for the CNN network\n");
    }
#endif
#ifdef SHOW_YOLOV5_DETECT
    if(nc_get_cnn_network_input_resol(NETWORK_YOLOV5_DET, &in_tinfo) < 0)
    {
        printf("failed to get the input resolution for the CNN network\n");
    }
#endif
#ifdef SHOW_PELEE_SEG
    if(nc_get_cnn_network_input_resol(NETWORK_PELEE_SEG, &in_tinfo) < 0)
    {
        printf("failed to get the input resolution for the CNN network\n");
    }
#endif
#ifdef SHOW_PELEE_DETECT
    if(nc_get_cnn_network_input_resol(NETWORK_PELEE_DET, &in_tinfo) < 0)
    {
        printf("failed to get the input resolution for the CNN network\n");
    }
#endif
#ifdef SHOW_UFLD_LANE
    if(nc_get_cnn_network_input_resol(NETWORK_UFLD_LANE, &in_tinfo) < 0)
    {
        printf("failed to get the input resolution for the CNN network\n");
    }
#endif
#ifdef SHOW_TRI_CHIMERA
    if(nc_get_cnn_network_input_resol(NETWORK_TRI_CHIMERA, &in_tinfo) < 0)
    {
        printf("failed to get the input resolution for the CNN network\n");
    }
#endif

    npu_input_info->w = in_tinfo.dim.w;// network input width
    npu_input_info->h = in_tinfo.dim.h;// network input height
    npu_input_info->rgb_size = in_tinfo.dim.w * in_tinfo.dim.h * RGB_CNT;

#ifdef USE_BYTETRACK
    for (int i = 0; i < VIDEO_MAX_CH; i++) {
        E_NETWORK_UID net_id;
#ifdef SHOW_PELEE_DETECT
        net_id = NETWORK_PELEE_DET;
#endif
#ifdef SHOW_YOLOV5_DETECT
        net_id = NETWORK_YOLOV5_DET;
#endif
#ifdef SHOW_YOLOV8_DETECT
        net_id = NETWORK_YOLOV8_DET;
#endif
// [merge 11.06] MULTI(PeleeDet + TRICHIMERA)면 클래스가 더 많은 PeleeDet(10) 기준으로 초기화
// TRICHIMERA(6) 기준이면 SKIP 때 PeleeDet 결과가 트래커 배열을 넘어 segfault
#if defined(SHOW_TRI_CHIMERA) && !defined(SHOW_PELEE_DETECT)
        net_id = NETWORK_TRI_CHIMERA;
#endif
#ifndef SHOW_PELEE_SEG
        if (nc_init_bytetrackers(CAP_FPS, i, net_id) != 0) {
            perror("nc_init_bytetrackers() error");
            return -1;
        }
#endif
    }
#if defined(SHOW_PELEE_DETECT) && defined(SHOW_TRI_CHIMERA) && !defined(SHOW_PELEE_SEG)
    // [11.06] 모델별 트래커: TRICHIMERA용 트래커를 따로 하나 더 만듦 (위는 PeleeDet용)
    //         모드 전환(SKIP <-> NORMAL/LIGHT) 때 두 모델의 추적 번호·클래스가 한 트래커에 섞이지 않게
    for (int i = 0; i < VIDEO_MAX_CH; i++) {
        if (nc_init_bytetrackers(CAP_FPS, i, NETWORK_TRI_CHIMERA) != 0) {
            perror("nc_init_bytetrackers(TRI_CHIMERA) error");
            return -1;
        }
    }
#endif
#endif

    printf("aiw finish\n");

    return 0;
}

static void signal_int()
{
    running = 0;
    nc_cnn_postprocess_stop();
}

static void usage(int error_code)
{
    fprintf(stderr, "Usage: simple-egl [OPTIONS]\n\n"
        "  -f\tRun in fullscreen mode\n"
        "  -o\tCreate an opaque surface\n"
#ifdef USE_FILE_INPUT
        "  -i <file>\tLoop one video file (default: INPUT_VIDEO, else misc/image_npu/1920_1080_000[1-3].mp4 in turn)\n"
#endif
        "  -h\tThis help text\n\n");

    exit(error_code);
}

void *cnn_task(void *arg)
{
    (void) arg;
    printf("CNN TASK RUN!!\n");

    while (running) {
        stCnnData *cnn_data = NULL;
        if(receive_cnn_buf(&cnn_data) != -1){
            if (cnn_data == NULL) continue;   // [준우] 종료할 때 깨우기용 빈 메시지
            nc_aiw_run_cnn(cnn_data->ptr_cnn_buf, cnn_data->time_stamp_us, cnn_data->cam_ch, cnn_data->net_id);
            if(cnn_data) {
                free(cnn_data->ptr_cnn_buf);
                free(cnn_data);
            }
        }
    }

    printf("EXIT CNN_TASK!!\n");
    return NULL;
}

int main(int argc, char **argv)
{
    struct sigaction sigint;
    struct display display;
    struct window  window;
    int i, ret = 0;
    pthread_t p_thread[MAX_TASK_CNT];
    int task_cnt = 0;
    int thr_id;

    setvbuf(stdout, NULL, _IOLBF, 0);   // [준우] 로그를 줄 단위로 바로 출력 (파일로 보내도 버퍼에 안 쌓임)

    for (i = 1; i < argc; i++) {
        if (strcmp("-f", argv[i]) == 0)
            window.fullscreen = 1;
        else if (strcmp("-o", argv[i]) == 0)
            window.opaque = 1;
        else if (strcmp("-h", argv[i]) == 0)
            usage(EXIT_SUCCESS);
#ifdef USE_FILE_INPUT
        else if (strcmp("-i", argv[i]) == 0 && i + 1 < argc)
            g_file_input_path = argv[++i];   // [준이] 입력 영상 지정
#endif
        else
            usage(EXIT_FAILURE);
    }

    sigint.sa_handler = (sighandler_t)signal_int;
    sigemptyset(&sigint.sa_mask);
    sigint.sa_flags = SA_RESETHAND;
    sigaction(SIGINT, &sigint, NULL);
    sigaction(SIGTERM, &sigint, NULL);   // [11.06] killall(SIGTERM)도 Ctrl+C처럼 정리 순서대로 종료 (안 그러면 인코더가 열린 채로 죽어 다음 실행이 멈춤)

    memset(&display, 0, sizeof(display));
    memset(&window, 0, sizeof(window));
    memset(&v4l2_config, 0, sizeof(v4l2_config));

    set_v4l2_config();

    nc_init_path_localizer();

    window.display = &display;
    display.window = &window;
    window.window_size.width  = WINDOW_WIDTH;
    window.window_size.height = WINDOW_HEIGHT;

    set_viewport_config();
#ifdef USE_LDW
    nc_ldw_app_init(VIDEO_MAX_CH, (float)WINDOW_WIDTH, (float)WINDOW_HEIGHT);   // [준우]
#endif

    nc_wayland_display_init(&display,(void *)render);

    ret = v4l2_initialize();
    if(ret < 0) {
        printf("Error v4l2_initialize\n");
        return -1;
    }

    gl_initialize(&window);
#ifdef USE_8MP_VI
    dsr_init();
#endif
#ifdef USE_ADAS_LD
    ld_opengl_program_set(&g_npu_prog);
    NC_ADAS_OPEN();
#endif

    // create thread-safe flip-flop buffer
    for (int i = 0; i < VIDEO_MAX_CH; i++) {
        if (v4l2_config[i].video_buf.video_fd == -1){

        }
        else{
    #ifdef DETECT_NETWORK
            int det_buf_size = sizeof(pp_result_buf);
            if(nc_tsfs_ff_create_buffers(i+DETECT_NETWORK, det_buf_size) < 0) {
                exit(1);
            }
    #endif
    #ifdef SEGMENT_NETWORK
            int seg_buf_size = sizeof(pp_result_buf);
            if(nc_tsfs_ff_create_buffers(i+SEGMENT_NETWORK, seg_buf_size) < 0) {
                exit(1);
            }
    #endif
    #ifdef LANE_NETWORK
            int lane_buf_size = sizeof(pp_result_buf);
            if(nc_tsfs_ff_create_buffers(i+LANE_NETWORK, lane_buf_size) < 0) {
                exit(1);
            }
    #endif
        }
    }
    mq_unlink(MQ_NAME_CNN_BUF);

    if(npu_init(&npu_input_info) < 0) {
        printf("failed to init NPU\n");
        return -1;
    }

    if (nc_rec_init() < 0) {
        printf("[REC] recording disabled\n");   // 녹화가 안 돼도 앱은 계속
    }    

    printf("create tasks\n");
    thr_id = pthread_create(&p_thread[task_cnt++], NULL, cnn_task, (void *)NULL);
    if (thr_id < 0) {
        perror("thread create error : cnn_task");
        exit(1);
    }

    cnn_postprocess_arg cnn_post_param;
    cnn_post_param.target_width = WINDOW_WIDTH;
    cnn_post_param.target_height = WINDOW_HEIGHT;
    thr_id = pthread_create(&p_thread[task_cnt++], NULL, nc_cnn_postprocess_task, (void *)&cnn_post_param);
    if (thr_id < 0) {
        perror("thread create error : nc_cnn_postprocess_task");
        exit(1);
    }

#ifdef USE_ADAS_LD
    thr_id = pthread_create(&p_thread[task_cnt++], NULL, ld_task, NULL);
    if (thr_id < 0) {
        perror("thread create error : ld_task");
        exit(1);
    }
#endif

    while (running && ret != -1)
    {
        ret = wl_display_dispatch(display.display);
    }

    // 32번: 메인 루프가 끝난 이유 기록 (원래는 아무 로그 없이 종료됨)
    if (ret == -1 && running) {   // [11.06] 신호로 끝날 때도 dispatch가 -1(EINTR)을 줄 수 있어서 running으로 구분
        printf("[EXIT] wl_display_dispatch fail: errno %d (%s), wl err %d\n",
               errno, strerror(errno), wl_display_get_error(display.display));
    } else {
        printf("[EXIT] running = 0 (SIGINT/SIGTERM)\n");
    }
    fflush(stdout);

    // [준우] 종료 순서: 화면 먼저 정리 → cnn_task 깨우기 → 스레드 1초만 기다리기
    printf("[app] shutting down\n");
    running = 0;
    nc_cnn_postprocess_stop();

    // [11.06] 녹화 인코더 닫기: 열린 채로 끝나면 VPU가 사용 중으로 남아 다음 실행이 인코더 초기화에서 멈춤
    nc_rec_deinit();

    // weston이 곧 사라질 앱의 버퍼를 계속 그리지 않게 화면(EGL/Wayland)부터 정리
    nc_wayland_display_destroy(&display);

    // mq_receive()에서 막혀 있는 cnn_task를 빈 메시지(NULL)로 깨움
    {
        struct mq_attr attr;
        attr.mq_maxmsg = MAX_MQ_MSG_CNT;
        attr.mq_msgsize = sizeof(stCnnData*);
        mqd_t mfd = mq_open(MQ_NAME_CNN_BUF, O_WRONLY | O_CREAT | O_NONBLOCK, 0666, &attr);
        if (mfd != -1) {
            stCnnData *wake = NULL;
            (void)mq_send(mfd, (const char *)&wake, sizeof(wake), 1);
            mq_close(mfd);
        }
    }

    // 스레드가 아직 막혀 있으면 1초만 기다리고 그냥 종료 (프로세스가 끝나면 같이 끝남)
    for(int i =0; i< task_cnt; i++) {
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_sec += 1;
        if (pthread_timedjoin_np(p_thread[i], NULL, &ts) != 0) {
            printf("[app] worker thread %d still busy, exiting without it\n", i);
        }
    }

#ifdef USE_ADAS_LD
    NC_ADAS_CLOSE();
#endif

#ifdef USE_BYTETRACK
    for (int i = 0; i < VIDEO_MAX_CH; i++) {
        nc_deInit_bytetrackers(i);
    }
#endif

#ifdef USE_8MP_VI
    dsr_deinit();
#endif
#ifdef USE_FILE_INPUT
    file_input_deinitialize();   // [준이] ffmpeg 닫기
#endif

    printf("[app] exit\n");
    return 0;
}