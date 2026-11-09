#pragma once
#include "nc_cnn_common.h"
typedef struct { int max_lane_num; } stLaneDrawInfoFake;
typedef struct { uint64_t time_stamp; stLaneDrawInfoFake lane_draw_info; stCnnPostprocessingResults cnn_result; } pp_result_buf;
extern int nc_get_cnn_networks_id(void);
