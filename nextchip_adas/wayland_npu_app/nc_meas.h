#ifndef NC_MEAS_H
#define NC_MEAS_H
#include <stdint.h>

// 기록 (run_cnn_program에서 호출)
void  nc_meas_npu_record(uint64_t up_us, uint64_t exec_us, uint64_t down_us, uint64_t post_us);
void  nc_meas_npu_hw(uint64_t cycles, uint32_t ms);

// 읽기 (render, 팀원 코드에서 호출)
void  nc_meas_get_npu_parts(float *up, float *exec, float *down, float *post);  // ms
float nc_meas_get_npu_ms(void);       // 네 구간 합 (ms)
float nc_meas_get_npu_max_ms(void);   // 최근 1초 동안 가장 오래 걸린 처리 시간 (ms)
float nc_meas_get_npu_fps(void);      // NPU가 1초에 몇 번 실행됐는지
float nc_meas_get_npu_hw_ms(void);    // NPU 하드웨어가 실제 계산한 시간 (ms)
float nc_meas_get_npu_mhz(void);      // cycles ÷ ms로 역산한 클럭 (검증용)
float nc_meas_get_temp_c(void);       // NPU 온도 (°C)
int   nc_meas_temp_is_fake(void);     // /tmp/fake_temp 값을 쓰고 있으면 1

// 지연 = render가 프레임을 꺼낸 시각(time_stamp) → render가 그 결과를 읽은 시각 (1초 평균, ms)
// 앱 기준 값이라 센서 → 앱 구간은 빠져 있음. 같은 결과를 다시 읽거나 time_stamp가 0이면 셈하지 않음
void  nc_meas_latency_record(uint64_t time_stamp_us);   // render에서 결과를 읽을 때 호출
float nc_meas_get_latency_ms(void);                     // 최근 1초 평균 (ms)
float nc_meas_get_latency_max_ms(void);                 // 최근 1초 최댓값 (ms)

const char *nc_meas_temp_tag(void);   // 온도 표시: " (FAKE)", " (STUCK)", ""
#endif