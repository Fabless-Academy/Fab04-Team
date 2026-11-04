# timestamp 전달 수정 정리

Nov 4, 2026 · 작성: 준이 · 패치 파일: `timestamp_final.patch`

## 각자 해야 할 일

### 공통: 패치 적용

`timestamp_final.patch`를 개발 PC의 홈 폴더에 넣고 실행한다. 시험 적용이 통과해야 실제로 적용된다.

```bash
cd ~/apache6-sr-linux_som_micro/applications
patch -p1 --dry-run < ~/timestamp_final.patch && patch -p1 -b < ~/timestamp_final.patch
cd wayland_npu_app
make clean
make > ~/timestamp_build.log 2>&1; echo "make 종료 코드: $?"
```

- 시험 적용이 실패하면 파일은 바뀌지 않는다. 위의 [변경한 코드](#변경한-코드)를 보고 직접 반영한다.
- `VIS0_MAX_CH`는 각자의 카메라 또는 입력 설정에 맞춘다.
- `-b` 옵션으로 생긴 `.orig` 백업 파일은 공유하지 않는다.

## 요약

- 원래 결과 버퍼의 timestamp는 항상 0이었다. 이제 **프레임을 꺼낸 시각**이 결과 버퍼까지 전달된다.
- 보드에서 TRICHIMERA, 카메라 1채널로 객체·차선 결과를 확인했다.
- 서영은 `nc_meas_get_latency_ms()`를, 준우와 준이는 이 값을 쓰는 기능을 이어서 구현하면 된다.

## 왜 필요한가

객체·차선 결과가 어느 프레임에서 나왔는지 알아야 다음 값을 계산할 수 있다.

- 시스템 지연 시간
- 칼만 필터와 차선 이동 속도에 쓰는 실제 시간 간격
- 끼어들기 판단에 쓸, 객체 결과와 가장 가까운 차선 결과

## 수정 내용

### 값이 전달되는 경로

```text
render()에서 프레임을 꺼낸 직후 nc_get_mono_us_time()으로 시각 기록
  → send_cnn_buf()
  → cnn_task()
  → nc_aiw_run_cnn()
  → run_cnn_program()에서 cnn_output.time_stamp_us에 저장
  → nc_cnn_send_inference_result()
  → 각 네트워크의 후처리 함수
  → pp_buf->time_stamp, 플립플롭 버퍼의 timestamp
  → render()에서 결과를 읽을 때 사용
```

### 변경한 코드

| 파일 | 변경 | 이유 |
| --- | --- | --- |
| `wayland_npu_app.c` | `time_stamp_us = 0` → `nc_get_mono_us_time()` | 입력 프레임의 시각 기록 |
| `wayland_npu_app.c` | `#include "nc_utils.h"` 추가 | 시간 함수 선언을 명시적으로 포함 |
| `wayland_npu_app.c` | `VIS0_MAX_CH`를 `(4)` → `(1)`로 변경 | 현재 보드의 카메라 1채널 설정에 맞춤 (timestamp와 별개) |
| `nc_cnn_aiware_runtime.h` | `cnn_output_info`에 `time_stamp_us` 추가 | 후처리 함수까지 값을 전달할 통로 |
| `nc_cnn_aiware_runtime.c` | `cnn_output.time_stamp_us = time_stamp;` 추가 | 모든 네트워크의 출력에 값 저장 |
| YOLOv8, UFLD, PeleeDet, Segmentation 후처리 | `time_stamp = 0` → `cnn_output->time_stamp_us` | 결과 버퍼에 실제 시각 기록 |

### 일부러 수정하지 않은 곳

- `wayland_npu_app.c`의 `render()`에 있는 `uint64_t time_stamp = 0;`: 읽기용 변수의 초기값이고, 다음 함수가 실제 값을 채운다.
- `nc_cnn_yolov5_postprocess.c`: 계획에 없는 모델이라 그대로 두었다. **값은 아직 0이다.** 사용하게 되면 같은 방식으로 고쳐야 한다.

## 보드 확인 결과

| 항목 | 결과 |
| --- | --- |
| `ts` | 0이 아니고 계속 증가 |
| 프레임 간격 | 평균 약 33.4ms (약 30fps) |
| 객체와 차선의 `ts` | 확인한 로그에서 모두 같음 |
| `age` | 약 34.8~35.8ms로 안정적 |

`age`는 프레임을 꺼낸 뒤 `render()`에서 결과를 읽을 때까지의 시간이다. **이 값을 시스템 전체 지연으로 쓰면 안 된다.**

## 사용 방법

| 항목 | 내용 |
| --- | --- |
| 읽는 곳 | `pp_buf->time_stamp` 또는 `nc_tsfs_ff_get_readable_buffer_and_timestamp()`의 출력값 |
| 단위 | 마이크로초(us) |
| 시계 | `nc_get_mono_us_time()` |
| 경과 시간 | `nc_get_mono_us_time() - time_stamp` |
| 프레임 간격 | `현재 결과의 time_stamp - 이전 결과의 time_stamp` |

주의할 점:

- **측정 시작점:** 센서 촬영 시각이 아니라 앱이 프레임을 꺼낸 직후다. 센서에서 앱까지의 지연은 빠져 있다.
- **같은 결과 반복:** 화면 갱신이 NPU 결과보다 빠르면 같은 `time_stamp`를 다시 읽을 수 있다. 시간 간격을 계산할 때 간격이 0인 경우를 처리해야 한다.
- **객체와 차선의 시각 차이:** 결과 버퍼가 따로라서, 읽는 시점에 따라 값이 다를 수 있다. 같다고 가정하지 말고 값을 비교해서 쓴다.

## 각 기능에 주는 영향

| 담당 | 기능 | 영향 |
| --- | --- | --- |
| 서영 | `nc_meas_get_latency_ms()` | 0이 아닌 값으로 지연을 계산할 수 있다. |
| 준우 | 차선 이탈 경고 | 경고 기준에 지연값을 넣고, 이동 속도를 실제 시간 간격으로 계산할 수 있다. |
| 준이 | 궤적 예측 | 칼만 필터 `dt`와 예측 거리 `k`를 측정값으로 계산할 수 있다. LIGHT 모드의 2프레임 간격도 반영할 수 있다. |
| 준이 | 끼어들기 판단 | 객체 결과와 시각이 가장 가까운 차선 결과를 고를 수 있다. |
| 준이 | 미니맵 | 직접 쓰지는 않지만, 표시할 예측 궤적이 이 값에 의존한다. |

## 각자 해야 할 일

~~### 공통: 패치 적용~~

~~`timestamp_final.patch`를 개발 PC의 홈 폴더에 넣고 실행한다. 시험 적용이 통과해야 실제로 적용된다.~~

~~```bash~~
~~cd ~/apache6-sr-linux_som_micro/applications
~~patch -p1 --dry-run < ~/timestamp_final.patch && patch -p1 -b < ~/timestamp_final.patch
~~cd wayland_npu_app
~~make clean
~~make > ~/timestamp_build.log 2>&1; echo "make 종료 코드: $?"
~~```~~

~~- 시험 적용이 실패하면 파일은 바뀌지 않는다. 위의 [변경한 코드](#변경한-코드)를 보고 직접 반영한다.~~
~~- `VIS0_MAX_CH`는 각자의 카메라 또는 입력 설정에 맞춘다.~~
~~- `-b` 옵션으로 생긴 `.orig` 백업 파일은 공유하지 않는다.~~

### 서영

- [ ] 패치 적용 후 빌드
- [ ] `nc_meas_get_latency_ms()` 구현
- [ ] `time_stamp == 0`일 때의 예외 처리 추가
- [ ] 측정 시작점이 "앱이 프레임을 꺼낸 직후"인 것을 함수 설명에 반영
- [ ] `MULTI` 빌드에서 PeleeDet의 timestamp 확인 (현재 빌드에 포함되지 않아 검증 전)
- [ ] SKIP 모드를 포함해 모드 전환 중에도 지연값이 정상인지 확인

### 준우

- [ ] 패치 적용 후 빌드
- [ ] `VIS0_MAX_CH` 변경이 도로 영상 입력 작업과 겹치는지 확인
- [ ] 경고 기준에 `nc_meas_get_latency_ms()` 반영
- [ ] 차선 이동 속도 계산에 실제 시간 간격 사용, 간격이 0인 경우 처리

### 준이

- [x] timestamp 전달 수정 및 보드 확인
- [ ] 패치와 이 문서 공유
- [ ] ByteTrack 활성화
- [ ] 칼만 필터 `dt`와 예측 거리 `k`에 측정값 반영
- [ ] 끼어들기 판단에 가장 가까운 차선 결과 사용
- [ ] `nc_traj_reset()` 구현

## 아직 확인하지 못한 것

# bytetrack

### wayland_npu_app.c 수정

경로: /home/devuser/apache6-sr-linux_som_micro/applications/wayland_npu_app/wayland_npu_app.c

951번 줄에 추가(951~953)

```jsx
#ifdef SHOW_TRI_CHIMERA
        net_id = NETWORK_TRI_CHIMERA;
#endif
```

비교(939~961)

```jsx
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
#ifdef SHOW_TRI_CHIMERA
        net_id = NETWORK_TRI_CHIMERA;
#endif
#ifndef SHOW_PELEE_SEG
        if (nc_init_bytetrackers(CAP_FPS, i, net_id) != 0) {
            perror("nc_init_bytetrackers() error");
            return -1;
        }
#endif
    }
#endif
```

### makefile 수정

경로: /home/devuser/apache6-sr-linux_som_micro/applications/wayland_npu_app/Makefile

54번 줄 주석 해제

```jsx
53 # USE_8MP_VI          = yes
54   USE_BYTETRACK       = yes
55 # USE_ADAS_LD         = yes
```

# nc_traj 스텁 함수

| 파일 | 변경 내용 |
| --- | --- |
| `common/nc_app_modules/cnn/core/nc_traj.h` | 새로 생성: `nc_traj_reset()` 선언 |
| `common/nc_app_modules/cnn/core/nc_traj.c` | 새로 생성: 빈 `nc_traj_reset()` 정의 |
| `wayland_npu_app/Makefile` | 331번 줄에 아래 줄 추가 |

```jsx
CSRCS               += $(NC_CNN_CORE_DIR)/nc_traj.c
```

서영에게 해당 파일 어떻게 받을건지 물어보기

| 서영 님의 답변 | 전달 방법 |
| --- | --- |
| 공용 저장소가 있음 | 그 저장소의 최신 코드에 이번 변경을 적용한 뒤 푸시 |
| 패치 파일로 받음 | 커밋으로 패치 파일을 만들어 전달 |
| 파일로 받음 | `nc_traj.c`, `nc_traj.h`를 보내고 Makefile에 추가할 한 줄을 안내 |

- PeleeDet, `MULTI` 빌드, 모드 전환 중 동작
- Segmentation 결과의 timestamp 값 (현재 빌드에 포함되어 있지만, 임시 로그는 객체와 차선만 확인)
- 장시간 실행 (이번 확인은 짧은 실행 로그 기준)
- 카메라 4채널, 도로 영상 입력(v4l2loopback) 환경
