#!/usr/bin/env bash
# rosbag 녹화. 기본은 **랩탑에서 직접** 녹화한다(젯슨 CPU·디스크를 안 쓴다).
#
#   f1rec.sh [태그]          랩탑에서 녹화 → ~/rosbag_log/MMDD/run_MMDD_HHMMSS/
#   f1rec.sh --remote [태그] 젯슨에서 녹화 후 회수(무선 유실이 확인됐을 때의 대안)
#   f1rec.sh --check <bag>   기존 bag의 토픽 달성률 검사
#   옵션: --keep(--remote 시 젯슨 원본 유지)  --host <ssh호스트>
#
# ⚠️ 랩탑 녹화의 전제 (하나라도 빠지면 토픽이 안 잡힌다):
#   1) ROS_DOMAIN_ID가 젯슨과 같아야 한다(67).
#   2) wifi는 DDS 멀티캐스트를 막는다 → ROS_DISCOVERY_SERVER=10.1.1.3:11811 필요.
#      유선(피트)에서는 불필요. 이 스크립트는 env가 이미 있으면 그대로 쓰고, 없으면 경고만 한다.
#   3) 메시지 타입이 랩탑에 있어야 한다. f110_msgs는 ~/2026_IFAC에 있고,
#      **vesc_msgs는 랩탑에 없어서 /sensors/core는 랩탑 녹화로는 안 잡힌다**(경고 후 스킵).
#      그 토픽이 필요하면 --remote 를 쓸 것.
#
# 녹화가 끝나면 토픽별 달성률을 자동 검사한다 — 무선 유실 여부를 매번 숫자로 남기기 위함.

set -uo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
HOST="${JETSON_HOST:-jetson}"
LOCAL_ROOT="${LOCAL_BAG_ROOT:-$HOME/rosbag_log}"
DOMAIN="${ROS_DOMAIN_ID:-67}"
MODE="local"; ARG=""; RM_REMOTE=1

while [ $# -gt 0 ]; do
  case "$1" in
    --remote) MODE="remote"; shift ;;
    --check)  MODE="check"; shift; ARG="${1:-}"; [ $# -gt 0 ] && shift ;;
    --keep)   RM_REMOTE=0; shift ;;
    --host)   HOST="$2"; shift 2 ;;
    -h|--help) sed -n '2,22p' "$0"; exit 0 ;;
    *) ARG="$1"; shift ;;
  esac
done

TOPICS=(/drive_autonomous /drive_mppi /drive /joy /drive_mode /mppi_active /estop_lock
        /pf/pose/odom /odom /tf /tf_static /scan /sensors/imu/raw /imu/data
        /global_waypoints /local_waypoints /state /avoid_waypoints /overtake_waypoints
        /car_state/frenet/odom /commands/motor/speed /commands/motor/brake
        /commands/servo/position /sensors/core
        # control_map_node가 매 사이클 내는 L1 목표점·룩어헤드 벡터(MarkerArray, 표시 전용).
        # 조향 명령의 "왜"를 사후에 재구성할 수 있는 유일한 토픽이라 넣어둔다.
        /debug/l1_lookahead
        # 장애물 종방향 감속(obstacle_brake)의 입력. 감속 캡이 왜 걸렸는지 추적용.
        /perception/detection/raw_obstacles)

if [ "$MODE" = "check" ]; then
  [ -n "$ARG" ] || ARG=$(ls -td "$LOCAL_ROOT"/*/run_* 2>/dev/null | head -1)
  [ -n "$ARG" ] || { echo "❌ 검사할 bag이 없습니다"; exit 1; }
  exec python3 "$HERE/bag_rates.py" "$ARG"
fi

if [ "$MODE" = "remote" ]; then
  exec "$HERE/jetson_rec.sh" ${RM_REMOTE:+} $([ "$RM_REMOTE" = 0 ] && echo --keep) --host "$HOST" ${ARG:+"$ARG"}
fi

# ── 랩탑 녹화 ───────────────────────────────────────────────────────────────
. /opt/ros/jazzy/setup.bash 2>/dev/null || . /opt/ros/humble/setup.bash
[ -f "$HOME/2026_IFAC/install/setup.bash" ] && . "$HOME/2026_IFAC/install/setup.bash"
# /sensors/core(vesc_msgs)용 최소 워크스페이스. 정의는 젯슨 bag의 message_definitions에서
# 추출해 재구성했고 타입 해시가 젯슨과 일치함을 확인했다(RIHS01_9a9543e6…444c).
[ -f "$HOME/vesc_msgs_ws/install/setup.bash" ] && . "$HOME/vesc_msgs_ws/install/setup.bash"
export ROS_DOMAIN_ID="$DOMAIN"

if ! ros2 interface show vesc_msgs/msg/VescStateStamped >/dev/null 2>&1; then
  echo "⚠️  vesc_msgs를 못 찾음 → /sensors/core는 녹화에서 빠집니다."
  echo "    cd ~/vesc_msgs_ws && colcon build --packages-select vesc_msgs"
  TOPICS=("${TOPICS[@]/\/sensors\/core}")
fi

echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID  RMW=${RMW_IMPLEMENTATION:-기본}"
if [ -n "${ROS_DISCOVERY_SERVER:-}" ]; then
  echo "ROS_DISCOVERY_SERVER=$ROS_DISCOVERY_SERVER"
else
  echo "ℹ  ROS_DISCOVERY_SERVER 미설정 — 유선이면 정상, wifi면 토픽이 안 잡힐 수 있다:"
  echo "     export ROS_DISCOVERY_SERVER=\"10.1.1.3:11811\""
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
trap '' INT
ros2 bag record -s sqlite3 -o "$DEST" "${TOPICS[@]}"
trap - INT

echo
[ -d "$DEST" ] && python3 "$HERE/bag_rates.py" "$DEST"
