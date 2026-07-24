# F1TENTH Bag Analyzer — 웹앱 (무터미널)

브라우저에 rosbag `.db3`를 드래그하면 **3D 리플레이 + 주행 그래프 + 자동 튜닝 가이드**까지
전부 클라이언트에서 처리한다. **ROS·터미널·설치 불필요.** 파일은 서버로 안 올라가고
브라우저 안에서만 파싱된다(sql.js WASM + 자체 CDR 파서).

## 쓰는 법 — 두 가지
1. **로컬 파일 (가장 확실):** `webapp.html`을 더블클릭 → 브라우저 열림 → `.db3` 드래그. 끝.
2. **URL 공유:** claude.ai 아티팩트로 게시된 링크를 팀원에게. (아티팩트 CSP가 WASM을 막으면
   1번 또는 자체 호스팅으로. GitHub Pages 등에 `webapp.html` 올리면 영구 팀 URL이 된다.)

## 파일
- `app.html` — 앱 본체(CSS/HTML/JS). sql.js는 토큰(`__WASM_B64__`, `/*__SQLJS_GLUE__*/`)으로 주입.
- `vendor/sql-wasm.js`, `vendor/sql-wasm.wasm` — 벤더링된 sql.js 1.10.3 (SQLite WASM).
- `build.py` — 조립: `app.html` + vendor → `webapp.html`(독립형) + `webapp.fragment.html`(아티팩트용).

## 빌드 (앱 수정 시에만)
```bash
python3 build.py     # → webapp.html, webapp.fragment.html
```

## 검증 (2026-07-24)
- CDR 파서: 순수 파이썬 미러를 rclpy 정답과 21/21 바이트 일치 확인 → JS로 포팅.
- 엔드투엔드: 헤드리스 Chrome에서 실제 `.db3` 분석 → 데스크톱 도구와 stats 완전 일치
  (peakV 4.11 · peakAl 22.2 · nSpin 67 · 권장 max_speed 2). 3D 뷰어·차트 렌더 확인.

## 지원 토픽
`/odom`(속도·pose), `/tf`(map 프레임 복원), `/sensors/imu/raw`·`/imu/data`, `/scan`,
`/estop_lock`, `/joy`, `/commands/motor/brake`. `/global_waypoints`·`/drive`가 있으면 추후
명령 vs 실측 비교도 확장 가능(현재 실측 기반).
