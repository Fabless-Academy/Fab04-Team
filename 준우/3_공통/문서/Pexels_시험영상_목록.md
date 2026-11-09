# Pexels 시험 영상 (11/06, 준우)

Pexels 무료 영상, 1920x1080 H.264. 보드 `misc/image_npu/px/`에도 같은 파일 있음.
실행: `./ldw_board_check.sh 32 misc/image_npu/px/px_5915075.mp4`

| 파일 | 내용 | 시험 용도 | 출처 |
|---|---|---|---|
| `px_5915075.mp4` | 독일 고속도로 블랙박스, 27.5초. **3~7초 오른쪽 은색 차가 끼어듦**, 9초쯤 옆 차선 버스 | ✅ 끼어들기 + 오경보 확인 | https://www.pexels.com/video/5915075/ |
| `px_4119548.mp4` | 4초, 왼쪽 차가 나를 추월 | 오경보 확인 | https://www.pexels.com/video/4119548/ |
| `px_5921059.mp4` | 시내 주행, 끼어들기 없음 | 오경보 확인 | https://www.pexels.com/video/5921059/ |
| `px_3010208.mp4` | 트럭 실내 촬영 | ❌ 구도 안 맞음 | https://www.pexels.com/video/3010208/ |
| `px_3602471.mp4` | 교차로 정지 상태 | ❌ | https://www.pexels.com/video/3602471/ |
| `px_4565728.mp4` | 초점 흐림, 핸들 너머 | ❌ | https://www.pexels.com/video/4565728/ |
| `px_4608282.mp4` | 타임랩스 (1초마다 장면 건너뜀) | ❌ 추적 불가 | https://www.pexels.com/video/4608282/ |
| `px_4309723.mp4` | 타임랩스 시내 | ❌ 추적 불가 | https://www.pexels.com/video/4309723/ |

- 사용 조건: Pexels 라이선스 (무료 사용, 출처 표기 선택). 발표 자료에 쓸 때는 위 출처를 남기기.
- 보드 결과(최종 앱): `px_5915075`에서 은색 차 3.7초 경고 1번, 버스 오경보 없음.
