# 준우 작업 파일 (A: 영상 입력 + 차선 이탈 경고, 끼어들기 연결·확인)

마지막 정리: 11/06

```
준우/
├─ README.md                      ← 이 파일
├─ 변경사항_목록.md               ← 합칠 때 옮길 위치 (파일별·함수별)
├─ 1_차선이탈경고_영상입력/
│   ├─ 새파일/applications/...
│   ├─ 수정파일/applications/...
│   └─ 문서/                      LDW 알고리즘 정리, 내 차선 영역 함수 설명
├─ 2_끼어들기예측/
│   ├─ 새파일/applications/...    nc_traj, nc_cutin, nc_class
│   ├─ 수정파일/applications/...  timestamp 전달 (SDK 후처리 파일)
│   ├─ diff/                      SDK 원본 대비, 준이 원본 대비
│   └─ 문서/                      연결·수정 내용, 보드 확인 결과
└─ 3_공통/
    ├─ 수정파일/applications/wayland_npu_app/   wayland_npu_app.c, Makefile (두 파트 모두 고침)
    ├─ diff/                      끼어들기 추가 부분
    └─ 시험도구/applications/misc/ldw_board_check.sh
```

- 모든 파일은 **SDK 안 경로 그대로** 들어 있어. `새파일/applications/...` 아래를 SDK의 `applications/...`에 넣으면 돼.
- **새파일**: SDK에 없던 파일 → 그대로 복사.
- **수정파일**: SDK에 원래 있던 파일 → 다른 사람도 고쳤으면 **덮어쓰지 말고** `diff/` 또는 `변경사항_목록.md`를 보고 바뀐 부분만 옮겨.

---

## 1. 차선 이탈 경고 + 영상 입력

| 구분 | 파일 | 내용 |
|---|---|---|
| 새파일 | `common/nc_app_modules/ldw/nc_ldw.c, nc_ldw.h` | 차선 이탈 경고 전체, 내 차선 영역 함수 `nc_ldw_get_ego_lane()` |
| 새파일 | `misc/run_app_common.sh` | 실행 스크립트 공용 함수 (weston 확인, 앱·영상 공급 정리) |
| 수정파일 | `common/nc_app_modules/v4l2/v4l2_interface.c` | 영상 공급 첫 프레임까지 최대 10초 기다림 |
| 수정파일 | `misc/run_v4l2loopback.sh` | 영상 공급(ffmpeg) 1개, 15fps, `--stop`, 버퍼 4 |
| 수정파일 | `misc/run_app_wayland_npu.sh` | 앱 실행·종료 정리, **영상 파일 지정 실행** |
| 수정파일 | `misc/run_app_wayland_cam.sh` | 카메라 앱 실행·종료 정리 |

문서: `문서/LDW_알고리즘_정리_1105.md` (동작 방식, 경고 기준, 알림 방식)

## 2. 끼어들기 예측 (준이 코드 연결 + 확인)

| 구분 | 파일 | 출처 |
|---|---|---|
| 새파일 | `common/nc_app_modules/cnn/core/nc_traj.c, nc_traj.h` | 준이 원본 그대로 |
| 새파일 | `common/nc_app_modules/cnn/core/nc_cutin.c, nc_cutin.h` | 준이 원본 + 준우 수정 |
| 새파일 | `common/nc_app_modules/cnn/core/nc_class.c, nc_class.h` | 준우 |
| 수정파일 | `cnn/core/nc_cnn_aiware_runtime.c, .h` | timestamp 전달 |
| 수정파일 | `cnn/networks/yolov8/nc_cnn_yolov8_postprocess.c` | timestamp + `nc_traj_update()` |
| 수정파일 | `cnn/networks/ufld/…`, `pelee/…`, `nc_cnn_segmentation_postprocess.c` | timestamp |

- `diff/수정파일_SDK원본대비.diff`: 위 수정파일의 바뀐 줄 (`patch -p1`로 적용 가능)
- `diff/nc_cutin_준이원본대비.diff`: 준이 원본에서 내가 바꾼 줄
- 문서: `문서/끼어들기_연결_확인결과_1106.md` (바꾼 값과 이유, 보드 결과, 합의할 것)

## 3. 공통 (두 파트가 같이 고친 파일)

| 파일 | LDW / 영상 입력 쪽 | 끼어들기 쪽 |
|---|---|---|
| `wayland_npu_app/wayland_npu_app.c` | LDW 호출 3곳, 1채널, 종료 순서 | 프레임 시각, 예측 호출, 그리기, ByteTrack, `nc_rec_trigger(1)` |
| `wayland_npu_app/Makefile` | `USE_LDW` 4곳, `nc_c_stub.c` | `USE_BYTETRACK`, `nc_class/nc_traj/nc_cutin` |

- `diff/끼어들기_추가부분.diff`: LDW까지 있던 상태 대비 끼어들기로 바뀐 줄
- LDW 쪽 위치는 `변경사항_목록.md`
- 시험도구 `ldw_board_check.sh`: 실행 2번 + 결과 요약 (LDW, 끼어들기 통계, 종료 정리, GPU 오류)

---

## 빌드에 같이 필요한 것 (여기 없음)

| 것 | 어디 |
|---|---|
| 서영 임시 함수 `nc_c_stub.c`, `nc_meas.h`, `nc_model.h`, `nc_rec.h` | 서영 → `wayland_npu_app/`에 넣기 (Makefile에 `CSRCS += nc_c_stub.c` 있음) |
| 시험 영상 (`test3_1080p.mkv`, Pexels `px_*.mp4`) | Google Drive (용량이 커서 GitHub엔 안 올림) |

## 보드에서 실행

```sh
cd /mnt/user_data/applications
./run_app_wayland_npu.sh                                   # 기본 영상
./run_app_wayland_npu.sh misc/image_npu/test3_1080p.mkv    # 끼어들기 영상
./ldw_board_check.sh 30 misc/image_npu/test3_1080p.mkv     # 결과 파일까지
```
LDW·끼어들기 기준은 환경변수로 바로 바꿀 수 있어 (`LDW_WARN_MARGIN=0.6`, `CUTIN_MIN_LANE_W=150` 등). 목록은 각 파트 문서에 있어.

## 참고

- ByteTrack을 켜면 SDK Makefile 설정상 `-Werror`가 꺼져. 새 경고가 빌드 오류로 안 잡히니 빌드 로그를 한 번씩 봐.
- 판정 통계 로그는 꺼져 있어. 끼어들기 값을 맞출 때는 `nc_cutin.h`의 `NC_CUTIN_STATS` 주석을 풀고 빌드해.
