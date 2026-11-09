# 준우 작업 파일 (A: 영상 입력 + 차선 이탈 경고, 끼어들기 연결·개선)

마지막 정리: 11/09

**처음 볼 문서: `3_공통/문서/알고리즘_정리_최종_1106.md`** (지금 코드의 동작·경고 기준·검증·한계·환경변수 전부)

```
준우/
├─ README.md                      ← 이 파일
├─ 변경사항_목록.md               ← 합칠 때 옮길 위치 (파일별·함수별)
├─ 1_차선이탈경고_영상입력/
│   ├─ 새파일/applications/...    nc_ldw.c/h, run_app_common.sh
│   ├─ 수정파일/applications/...  v4l2_interface.c, 실행 스크립트
│   └─ 문서/                      LDW 알고리즘(11/05), 내 차선 영역 함수
├─ 2_끼어들기예측/
│   ├─ 새파일/applications/...    nc_traj, nc_cutin, nc_class
│   ├─ 수정파일/applications/...  timestamp 전달 (SDK 후처리 파일)
│   ├─ diff/                      SDK 원본 대비, 준이 원본 대비
│   └─ 문서/                      연결·보드 확인(11/06), 시험 영상 실행 방법
└─ 3_공통/
    ├─ 수정파일/applications/wayland_npu_app/   wayland_npu_app.c, Makefile
    ├─ diff/                      끼어들기 추가 부분
    ├─ 시험도구/                  ldw_board_check.sh, pc_test/ (PC 가상 시험)
    └─ 문서/                      최종 알고리즘, 프로젝트 설명, 차선 의존 개선,
                                  상황별 시험 영상 기록(+sheets), 보드결과
```

- 모든 파일은 **SDK 안 경로 그대로** 들어 있어. `새파일/applications/...` 아래를 SDK의 `applications/...`에 넣으면 돼.
- **새파일**: SDK에 없던 파일 → 그대로 복사.
- **수정파일**: SDK에 원래 있던 파일 → 다른 사람도 고쳤으면 **덮어쓰지 말고** `diff/`나 `변경사항_목록.md`를 보고 바뀐 부분만 옮겨.

---

## 11/09 업데이트 (11/06 이후 바뀐 것)

| 파트 | 파일 | 내용 |
|---|---|---|
| 1. LDW | `nc_ldw.c/h` | 끼어들기용 **내 차선 출처**(실제 → held → lead → virtual), 앞차 기준 차선, **평소 위치 자동 보정**(카메라가 차 가운데에 없을 때), 카메라 각도 안내 로그, 시험용 차선 끊기 |
| 1. 영상입력 | `run_v4l2loopback.sh` | `FEED_VF`: 시험 영상 구도 맞추기 (예: 지평선이 너무 낮은 영상 아래쪽 확대) |
| 2. 끼어들기 | `nc_cutin.c/h` | 차선이 안 보여도 판단 계속, 가까워지며 끼어드는 차 놓침 수정, 빨리 멀어지는 차 오경보 제거, 같은 차 중복 녹화 방지 보강 |
| 3. 공통 | `ldw_board_check.sh`, `pc_test/` | 차선 출처 요약, PC 가상 시험 33개 |
| 3. 공통 | `문서/` | 최종 알고리즘, 상황별 영상 기록, 11/09 보드 결과 |

`wayland_npu_app.c`, `Makefile`, 준이 `nc_traj`는 **11/06 그대로**야.

### 보드 결과 요약 (11/09, 영상 13개)
- 끼어들기: 진짜 4건 모두 경고 (test3 3번, px 1번). 오경보 1건 (도심 영상 36.6초, 출발하는 옆 차 박스 흔들림)
- 차선을 일부러 끊어도: held / virtual / lead 차선으로 끼어들기 계속 잡음 (11/06 시험)
- LDW: 사용자가 확인한 실선 경고 유지. 남은 경고는 대부분 "선까지 0.5m" 경계값
- 지평선이 화면 70% 아래인 영상, 선 없는 길, 짙은 안개는 차선 인식 안 됨

자세한 건 `3_공통/문서/상황별_시험영상_기록.md`, `3_공통/문서/보드결과/`.

---

## 1. 차선 이탈 경고 + 영상 입력

| 구분 | 파일 | 내용 |
|---|---|---|
| 새파일 | `common/nc_app_modules/ldw/nc_ldw.c, nc_ldw.h` | 차선 이탈 경고, 내 차선 함수 `nc_ldw_get_ego_lane()`, `nc_ldw_get_ego_lane_src()`, `nc_ldw_feed_vehicle()` |
| 새파일 | `misc/run_app_common.sh` | 실행 스크립트 공용 함수 |
| 수정파일 | `common/nc_app_modules/v4l2/v4l2_interface.c` | 영상 공급 첫 프레임까지 최대 10초 기다림 |
| 수정파일 | `misc/run_v4l2loopback.sh` | 영상 공급(ffmpeg) 1개, 15fps, `--stop`, 버퍼 4, `FEED_VF` |
| 수정파일 | `misc/run_app_wayland_npu.sh` | 앱 실행·종료 정리, 영상 파일 지정 실행 |
| 수정파일 | `misc/run_app_wayland_cam.sh` | 카메라 앱 실행·종료 정리 |

## 2. 끼어들기 예측 (준이 코드 연결 + 개선)

| 구분 | 파일 | 출처 |
|---|---|---|
| 새파일 | `cnn/core/nc_traj.c, nc_traj.h` | 준이 원본 그대로 |
| 새파일 | `cnn/core/nc_cutin.c, nc_cutin.h` | 준이 원본 + 준우 수정 (`diff/nc_cutin_준이원본대비.diff`) |
| 새파일 | `cnn/core/nc_class.c, nc_class.h` | 준우 |
| 수정파일 | `cnn/core/nc_cnn_aiware_runtime.c/.h`, yolov8·ufld·pelee·segmentation 후처리 | timestamp 전달 (+ yolov8에서 `nc_traj_update()`) |

## 3. 공통

| 파일 | LDW / 영상 입력 쪽 | 끼어들기 쪽 |
|---|---|---|
| `wayland_npu_app/wayland_npu_app.c` | LDW 호출 3곳, 1채널, 종료 순서 | 프레임 시각, 예측 호출, 그리기, ByteTrack, `nc_rec_trigger(1)` |
| `wayland_npu_app/Makefile` | `USE_LDW` 4곳, `nc_c_stub.c` | `USE_BYTETRACK`, `nc_class/nc_traj/nc_cutin` |

---

## 빌드에 같이 필요한 것 (여기 없음)

| 것 | 어디 |
|---|---|
| 서영 임시 함수 `nc_c_stub.c`, `nc_meas.h`, `nc_model.h`, `nc_rec.h` | 서영 → `wayland_npu_app/` |
| 시험 영상 (`test3_1080p.mkv`, `px/px_*.mp4`, `sc/sc_*.mp4`) | Google Drive (용량이 커서 GitHub엔 안 올림). 목록·출처는 `3_공통/문서/Pexels_시험영상_목록.md`, `상황별_시험영상_기록.md` |

## 보드에서 실행

```sh
cd /mnt/user_data/applications
./run_app_wayland_npu.sh                                     # 기본 영상
./run_app_wayland_npu.sh misc/image_npu/test3_1080p.mkv      # 끼어들기 영상
./ldw_board_check.sh 30 misc/image_npu/test3_1080p.mkv       # 결과 파일까지
LDW_DROP_AFTER_S=1 ./ldw_board_check.sh 32 misc/image_npu/px/px_5915075.mp4   # 차선 끊고 시험
```
환경변수 목록은 `3_공통/문서/알고리즘_정리_최종_1106.md` 부록 A.

## 참고

- ByteTrack을 켜면 SDK Makefile 설정상 `-Werror`가 꺼져. 빌드 로그를 한 번씩 봐.
- 판정 통계 로그는 꺼져 있어. 값을 맞출 때는 `nc_cutin.h`의 `NC_CUTIN_STATS` 주석을 풀고 빌드해.
- `2_끼어들기예측/문서/끼어들기_연결_확인결과_1106.md`와 `1_.../문서/LDW_알고리즘_정리_1105.md`는 그때 기준이야. 지금 값은 최종 알고리즘 문서를 봐.
