# PC 가상 시험 (보드 없이)

| 파일 | 내용 |
|---|---|
| `test_lanesrc.c` | 끼어들기 39개 상황 (차선 보임 / 1초 끊김 / 계속 없음 / 도로 모양 변화 / 앞차 / 빨리 멀어짐 / 선에 붙은 차 / 먼 차). 가상 시계로 LDW 시간까지 흉내냄 |
| `test_center.c` | 카메라가 차 가운데에 없을 때 LDW 평소 위치 보정 |
| `stubs.c`, `model_stub.c`, `netid_stub.c`, `fake/` | 서영 함수·SDK 헤더 대신 쓰는 가짜 |

```bash
cd 준우/3_공통/시험도구/pc_test
R=~/apache6-sr-linux_som_micro/applications; M=$R/common/nc_app_modules
INC="-I fake -I $M/ldw -I $M/cnn/core -I $M/cnn -I $R/wayland_npu_app $(find $M/cnn/networks -type d | sed 's/^/-I /')"
gcc -O1 -DUSE_LDW $INC -Wl,--wrap=clock_gettime -o t_lanesrc test_lanesrc.c \
  $M/ldw/nc_ldw.c $M/cnn/core/nc_traj.c $M/cnn/core/nc_cutin.c $M/cnn/core/nc_class.c \
  stubs.c netid_stub.c model_stub.c -lm -lpthread
./t_lanesrc            # 14fps, RESULT: PASS
SIM_FPS=30 ./t_lanesrc ; NODROP=1 ./t_lanesrc
gcc -O1 $INC -o t_center test_center.c $M/ldw/nc_ldw.c stubs.c model_stub.c -lm
./t_center -0.6 1      # 카메라 왼쪽 0.6m, 보정 켬
```
SDK 경로에 이 브랜치의 새파일·수정파일이 들어가 있어야 빌드돼. `wayland_npu_app/nc_meas.h` 등 서영 헤더도 필요해.
