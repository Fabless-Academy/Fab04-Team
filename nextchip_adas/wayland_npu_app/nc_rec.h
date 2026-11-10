#ifndef NC_REC_H
#define NC_REC_H
#include <stdint.h>

int  nc_rec_init(void);                       // main에서 1번: 인코더 준비
void nc_rec_push_frame(const uint8_t *rgb);   // render에서 매 프레임: 640x384 RGB
void nc_rec_trigger(int type);                // 경고가 켜질 때 1번 (0 = 차선, 1 = 끼어들기)
int  nc_rec_is_recording(void);               // 녹화 중이면 1
int nc_rec_get_remain_sec(void);   // 녹화 남은 시간(초), 화면 REC 표시용
#endif