#!/usr/bin/env bash
# rosbag 녹화. 기본은 **랩탑에서 직접** 녹화한다(젯슨 CPU·디스크를 안 쓴다).
#
#   f1rec.sh [태그]          랩탑에서 녹화 → ~/rosbag_log/MMDD/run_MMDD_HHMMSS/
#   f1rec.sh --remote [태그] 젯슨에서 녹화 후 회수(무선 유실이 확인됐을 때의 대안)
#   f1rec.sh --check <bag>   기존 bag의 토픽 달성률 검사
#   옵션: --keep(--remote 시 젯슨 원본 유지)  --host <ssh호스트>
#         --with-map(정적 /map도 녹화)  --full(아무것도 안 빼고 옛 동작)
#
# 📉 기본으로 **젯슨 송신 부하를 줄인다** (2026-08-04 run_0804_163639 실측 737 KB/s 기준):
#   1) 시각화 전용/중복 토픽 제외 → 약 −45%. 분석에 쓰는 토픽은 하나도 안 뺀다.
#        /map 200.7 KB/s(27.3%) — 정적 맵을 MCL이 5Hz로 재발행. 13.79 MB가 같은 맵 337장
#        /local_waypoints/path 68.4(9.3%) — /local_waypoints의 RViz용 중복
#        /pf/viz/* 43.1(5.9%) — 순수 시각화,  */markers — 순수 시각화
#      ⚠️ /map은 MCL pose 오차 분석(스캔을 맵에 투영)에 쓰인다. 그 분석을 할 bag은
#         --with-map으로 뜰 것. 맵은 정적이라 세션당 하나만 있으면 되고, 젯슨의 맵 파일을
#         직접 읽어도 등가다.
#   2) QoS를 best_effort로 낮춘다(f1rec_qos.yaml 참고). RELIABLE이면 젯슨 DDS가 랩탑 ACK를
#      추적하고 유실분을 **재전송**한다 — 링크가 나쁠수록 젯슨 CPU를 더 쓰는 구조다.
#      🔴 latched 토픽은 오버라이드에서 제외돼 있다. 이유는 아래 ℹ️ 항목과 같다.
#
# ⚠️ 랩탑 녹화의 전제 (하나라도 빠지면 토픽이 안 잡힌다):
#   1) ROS_DOMAIN_ID가 젯슨과 같아야 한다(70).
#   2) wifi는 DDS 멀티캐스트를 막는다 → ROS_DISCOVERY_SERVER=10.1.1.1:11811 필요
#      (2026-08-09 젯슨 핫스팟, 주소 10.1.1.3→10.42.0.1→10.1.1.1로 계속 바뀌는 중 —
#      바뀌면 이 줄이 아니라 실제 IP를 `ip -4 addr show`로 다시 확인할 것).
#      유선(피트)에서는 불필요. 이 스크립트는 env가 이미 있으면 그대로 쓰고, 없으면 경고만 한다.
#   3) 메시지 타입이 랩탑에 있어야 한다. f110_msgs는 ~/2026_IFAC에 있고,
#      **vesc_msgs는 랩탑에 없어서 /sensors/core는 랩탑 녹화로는 안 잡힌다**(경고 후 스킵).
#      그 토픽이 필요하면 --remote 를 쓸 것.
#
# 녹화가 끝나면 토픽별 달성률을 자동 검사한다 — 무선 유실 여부를 매번 숫자로 남기기 위함.
#
# ℹ️ 시작 시점: control 을 다 띄운 **뒤에 시작해도 된다**. /global_waypoints·/tf_static·
#    /mppi_active 는 transient_local(latched)이고 rosbag2 가 발행자 QoS 에 맞춰 구독하므로
#    늦게 붙어도 과거 발행분을 받아온다(07-31 실측: tf_static 1개 / global_waypoints 5개).
#    실제 제약은 "조이스틱 A(자율) 를 누르기 전"이다 — 자율 진입 과도구간을 놓치면 안 된다.

set -uo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
HOST="${JETSON_HOST:-jetson}"
LOCAL_ROOT="${LOCAL_BAG_ROOT:-$HOME/rosbag_log}"
DOMAIN="${ROS_DOMAIN_ID:-70}"
MODE="local"; ARG=""; RM_REMOTE=1; WITH_MAP=0; FULL=0

while [ $# -gt 0 ]; do
  case "$1" in
    --remote) MODE="remote"; shift ;;
    --check)  MODE="check"; shift; ARG="${1:-}"; [ $# -gt 0 ] && shift ;;
    --keep)   RM_REMOTE=0; shift ;;
    --host)   HOST="$2"; shift 2 ;;
    --with-map) WITH_MAP=1; shift ;;
    --full)   FULL=1; shift ;;
    # 헤더 주석(2행~첫 빈 줄)을 그대로 도움말로 쓴다. 줄 번호를 박아두면 헤더를 늘릴 때마다
    # 조용히 잘려서, 끝을 빈 줄로 잡는다.
    -h|--help) sed -n '2,/^$/p' "$0" | sed '$d'; exit 0 ;;
    *) ARG="$1"; shift ;;
  esac
done

if [ "$MODE" = "check" ]; then
  [ -n "$ARG" ] || ARG=$(ls -td "$LOCAL_ROOT"/*/run_* 2>/dev/null | head -1)
  [ -n "$ARG" ] || { echo "❌ 검사할 bag이 없습니다"; exit 1; }
  exec python3 "$HERE/bag_rates.py" "$ARG"
fi

if [ "$MODE" = "remote" ]; then
  exec "$HERE/jetson_rec.sh" ${RM_REMOTE:+} $([ "$RM_REMOTE" = 0 ] && echo --keep) --host "$HOST" ${ARG:+"$ARG"}
fi

# ── 랩탑 녹화 ───────────────────────────────────────────────────────────────
# ⚠️ set -u 를 켠 채로 ROS setup.bash 를 소싱하면 안 된다 — 그 안의
#    AMENT_TRACE_SETUP_FILES 참조가 unbound variable 로 걸려 스크립트가 **그 자리에서
#    조용히 종료**한다(아랫줄 2>/dev/null 이 에러 메시지까지 삼켜서 화면엔 아무것도 안 남는다).
#    2026-07-31에 실제로 겪음. 소싱 구간만 -u 를 풀고 끝나면 다시 켠다.
set +u
. /opt/ros/jazzy/setup.bash 2>/dev/null || . /opt/ros/humble/setup.bash
[ -f "$HOME/2026_IFAC/install/setup.bash" ] && . "$HOME/2026_IFAC/install/setup.bash"
# /sensors/core(vesc_msgs)용 최소 워크스페이스. 정의는 젯슨 bag의 message_definitions에서
# 추출해 재구성했고 타입 해시가 젯슨과 일치함을 확인했다(RIHS01_9a9543e6…444c).
[ -f "$HOME/vesc_msgs_ws/install/setup.bash" ] && . "$HOME/vesc_msgs_ws/install/setup.bash"
set -u
export ROS_DOMAIN_ID="$DOMAIN"

if ! ros2 interface show vesc_msgs/msg/VescStateStamped >/dev/null 2>&1; then
  echo "⚠️  vesc_msgs를 못 찾음 → /sensors/core는 -a로 녹화해도 타입을 못 풀어 빠질 수 있습니다."
  echo "    cd ~/vesc_msgs_ws && colcon build --packages-select vesc_msgs"
fi

echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID  RMW=${RMW_IMPLEMENTATION:-기본}"
if [ -n "${ROS_DISCOVERY_SERVER:-}" ]; then
  echo "ROS_DISCOVERY_SERVER=$ROS_DISCOVERY_SERVER"
else
  echo "ℹ  ROS_DISCOVERY_SERVER 미설정 — 유선이면 정상, wifi면 토픽이 안 잡힐 수 있다:"
  echo "     export ROS_DISCOVERY_SERVER=\"10.1.1.1:11811\""
fi

DAY=$(date +%m%d)
NAME="run_$(date +%m%d_%H%M%S)${ARG:+_$ARG}"
DEST="$LOCAL_ROOT/$DAY/$NAME"
mkdir -p "$LOCAL_ROOT/$DAY"

# 녹화 전에 실제로 보이는 토픽을 세어 둔다 — 0이면 디스커버리가 안 붙은 것이다.
VIS=$(timeout 8 ros2 topic list 2>/dev/null | grep -cE '^/(scan|odom|drive)$' || true)
if [ "${VIS:-0}" = "0" ]; then
  echo "⚠️  /scan·/odom·/drive 가 하나도 안 보입니다. 젯슨 노드가 떠 있는지, 위 2)번(디스커버리)을 확인하세요."
  echo "    그래도 진행합니다 — 늦게 붙을 수도 있습니다. (Ctrl+C로 중단)"
fi

echo "🔴 랩탑에서 녹화: $DEST"
echo "   Ctrl+C 로 종료 → 달성률 자동 검사"
# 🔴 trap '' INT (무시)로 두면 Ctrl+C 가 레코더에 안 먹는다 — 화면에 ^C 만 찍히고 녹화가
#    안 멈춘다. 2026-07-31에 pty를 물려 A/B로 실측:
#      trap '' INT       → 20초 지나도 레코더 생존 (사용자가 겪은 증상)
#      trap 없음          → 0.2초 종료. 단 부모까지 같이 죽어 아래 달성률 검사가 안 돌아감
#      trap 'echo' INT   → 0.3초 종료 + 달성률 검사 정상 실행  ← 이것만 둘 다 만족
#    ⚠️ /proc/<pid>/status 의 SigIgn 으로는 이 차이가 안 보인다(세 경우 모두 0x4=SIGQUIT).
#       신호 disposition 이 아니라 **실제 종료 여부**로 판정할 것.
# ⚠️ --max-cache-size 기본값은 100 MiB이고 이중 버퍼라 최악 200 MiB가 RAM에 떠 있다가
#    **종료할 때 한꺼번에** 디스크로 쏟아진다 → Ctrl+C 후 한참 안 끝나는 원인.
#    10 MiB로 낮추면 주행 중에 조금씩 쓰고 종료가 즉시 끝난다. 우리 데이터율은
#    ~1 MB/s(26토픽, /scan 40Hz 포함)라 10 MiB면 쓰기 횟수도 부담이 아니다.
trap 'echo' INT
# -a(--all): 그 순간 그래프에 보이는 토픽 전부. 예전엔 26개 화이트리스트라 새 토픽(디버그용
# 임시 발행 등)을 넣으려면 이 스크립트를 매번 고쳐야 했다 — 이제 안 그래도 된다.
# ⚠️ 데이터율 가정(~1 MB/s, --max-cache-size 10MiB 근거)은 26토픽 기준이었다. -a로 바뀌면서
#    더 많이 잡히면 캐시가 자주 차서 쓰기가 잦아질 수 있다 — 녹화 중 디스크 I/O가 눈에 띄게
#    걸리면 --max-cache-size를 키우는 걸 고려할 것.
#
# 🔴 -a는 토픽 하나라도 typesupport를 못 찾으면 discovery 스레드 **전체**가 죽는다(개별
#    스킵이 아님) — 2026-08-03 실측: /sensors/imu(vesc_msgs/msg/VescImuStamped)가 랩탑
#    ~/vesc_msgs_ws에 없어서 "Failure in topics discovery"로 녹화가 그 이후 멈췄다.
#    ~/vesc_msgs_ws엔 VescState(Stamped)만 재구성돼 있고 VescImuStamped는 아직 없다.
#    근본 해결은 그것도 재구성해서 워크스페이스에 추가하는 것(VescStateStamped 했던 것과
#    같은 방식 — 젯슨 bag의 message_definitions에서 추출 + 타입 해시 대조). 그 전까지는
#    아래처럼 알려진 미빌드 타입의 토픽을 제외해야 -a가 안 죽는다. 새로 이런 게 또 나오면
#    이 목록에 추가할 것(로그의 "Failure in topics discovery" 직전 줄이 범인 토픽이다).
EXCLUDE_TYPES=(vesc_msgs/msg/VescImuStamped)

# 시각화 전용/중복 토픽 제외 — 헤더의 📉 1) 참고. -a는 그대로 두므로 "새로 생긴 토픽이
# 자동으로 잡힌다"는 성질(위 105행 주석)은 유지된다. 빼는 건 명시한 것뿐이다.
EXCLUDE_TOPICS=(/local_waypoints/path /pf/viz/particles /pf/viz/inferred_pose)
[ "$WITH_MAP" = 0 ] && EXCLUDE_TOPICS+=(/map)
# */markers 는 개수가 늘어날 수 있어 정규식으로 한 번에 막는다(새 마커 토픽도 자동 적용).
# ⚠️ /debug/l1_lookahead 는 마커 타입이지만 **분석용**이다(L1 목표점 50Hz). 이 정규식에
#    안 걸리게 이름이 /markers 로 안 끝나니 그대로 녹화된다 — 제외 목록에 넣지 말 것.
#    2026-08-07에 tools/jetson_rec.sh TOPICS 와 ~/.zshrc f1rec_here 에도 명시 추가했다.
EXCLUDE_RE='.*/markers$'

QOS_ARGS=()
[ -f "$HERE/f1rec_qos.yaml" ] && QOS_ARGS=(--qos-profile-overrides-path "$HERE/f1rec_qos.yaml")

if [ "$FULL" = 1 ]; then
  echo "ℹ  --full: 제외·QoS 오버라이드 없이 전부 녹화합니다(옛 동작)."
  ros2 bag record -s sqlite3 --max-cache-size 10485760 -o "$DEST" -a \
    --exclude-topic-types "${EXCLUDE_TYPES[@]}"
else
  echo "ℹ  제외: ${EXCLUDE_TOPICS[*]} + 정규식 '$EXCLUDE_RE'"
  [ "$WITH_MAP" = 1 ] && echo "ℹ  --with-map: /map 포함(MCL pose 오차 분석용)"
  [ ${#QOS_ARGS[@]} -gt 0 ] && echo "ℹ  QoS: best_effort 오버라이드 적용(f1rec_qos.yaml)"
  ros2 bag record -s sqlite3 --max-cache-size 10485760 -o "$DEST" -a \
    --exclude-topic-types "${EXCLUDE_TYPES[@]}" \
    --exclude-topics "${EXCLUDE_TOPICS[@]}" \
    --exclude-regex "$EXCLUDE_RE" \
    ${QOS_ARGS[@]+"${QOS_ARGS[@]}"}
fi
trap - INT

echo
[ -d "$DEST" ] && python3 "$HERE/bag_rates.py" "$DEST"
