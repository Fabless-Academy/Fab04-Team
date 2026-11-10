#ifndef NC_MODEL_H
#define NC_MODEL_H
#include <stdint.h>

enum { MODE_NORMAL = 0, MODE_LIGHT = 1, MODE_SKIP = 2 };

void        nc_model_update(void);             // render에서 매 프레임 호출, 내부에서 1초마다 판단
int         nc_model_select(uint64_t frame);   // 이번 프레임에 돌릴 net_id, -1이면 건너뜀
int         nc_model_get_mode(void);
const char *nc_model_mode_name(void);
int         nc_model_is_vehicle(int class_idx);
#endif