#!/usr/bin/env bash
# 젯슨에서 rosbag을 녹화하고, Ctrl+C로 끝내면 랩탑 ~/rosbag_log/MMDD/ 로 자동 회수한다.
#
# 왜 이런 구조인가:
#   - 녹화 자체는 반드시 젯슨에서 해야 한다(랩탑 wifi로 빼면 /scan이 드롭된다).
#   - ssh -t로 원격 tty를 잡으면 Ctrl+C가 원격 프로세스로 전달돼 bag이 정상 종료(=db3 flush)된다.
#     로컬 셸은 raw 모드라 SIGINT를 안 받으므로, ssh가 리턴한 뒤 회수 단계가 이어서 실행된다.
#   - 젯슨 저장공간을 쓰지 않는 것이 목적이라 회수에 성공하면 젯슨 원본을 지운다(기본).
#     단 크기 대조까지 통과했을 때만 지운다 — 전송이 조금이라도 어긋나면 원본을 남기고 실패시킨다.
#     남기고 싶으면 --keep.
#
#   - 녹화는 젯슨의 tmux 세션 안에서 돈다. 주행 중 wifi가 끊겨도 녹화는 계속되고,
#     --attach 로 다시 붙어 정상 종료(=회수)할 수 있다. tmux 없으면 자동으로 평범한 ssh -t.
#
# 사용법:
#   jetson_rec.sh [태그]              녹화 → 자동 회수 → 젯슨 원본 삭제
#   jetson_rec.sh --attach            끊긴 녹화에 재접속 → Ctrl+C → 회수
#   jetson_rec.sh --pull [이름]       회수만 (이름 생략 시 젯슨의 최신 bag)
#   jetson_rec.sh --list              젯슨에 있는 bag 목록
#   옵션: --keep(젯슨 원본 유지)  --host <ssh호스트>

set -uo pipefail

HOST="${JETSON_HOST:-jetson}"
SESSION="f1bag"                                  # 젯슨 tmux 세션 이름
# wifi 순간 끊김에 ssh가 바로 죽지 않게(그래도 최종 방어선은 tmux다)
SSH_OPTS=(-o ServerAliveInterval=10 -o ServerAliveCountMax=6)
REMOTE_DIR="${JETSON_BAG_DIR:-~/rosbags}"
LOCAL_ROOT="${LOCAL_BAG_ROOT:-$HOME/rosbag_log}"
DOMAIN="${ROS_DOMAIN_ID:-67}"
RM_REMOTE=1          # 기본 삭제 — 젯슨 디스크를 안 쓰는 게 이 스크립트의 목적
MODE="record"
ARG=""

while [ $# -gt 0 ]; do
  case "$1" in
    --pull)   MODE="pull"; shift; [ $# -gt 0 ] && [[ "$1" != --* ]] && { ARG="$1"; shift; } ;;
    --list)   MODE="list"; shift ;;
    --attach) MODE="attach"; shift ;;
    --rm)     RM_REMOTE=1; shift ;;
    --keep)   RM_REMOTE=0; shift ;;
    --host)   HOST="$2"; shift 2 ;;
    -h|--help) sed -n '2,20p' "$0"; exit 0 ;;
    *) ARG="$1"; shift ;;
  esac
done

# 녹화 토픽. /state·/avoid_waypoints·/overtake_waypoints는 2026-07-30 추가 —
# 07-29 저속 주행 원인이 state machine의 AVOID 고착이었는데 /state가 없어 역추론해야 했다.
TOPICS="/drive_autonomous /drive_mppi /drive /joy /drive_mode /mppi_active /estop_lock \
/pf/pose/odom /odom /tf /tf_static /scan /sensors/imu/raw /imu/data \
/global_waypoints /local_waypoints /state /avoid_waypoints /overtake_waypoints \
/car_state/frenet/odom /commands/motor/speed /commands/motor/brake \
/commands/servo/position /sensors/core"

# 원격에서 ROS 환경을 명시적으로 세운다(.zshrc 의존 제거 — 비대화형 ssh는 .zshrc를 안 읽는다)
REMOTE_ENV='
for s in /opt/ros/jazzy/setup.bash /opt/ros/humble/setup.bash; do [ -f "$s" ] && . "$s" && break; done
[ -f "$HOME/f1tenth_ws/install/setup.bash" ] && . "$HOME/f1tenth_ws/install/setup.bash"
[ -f "$HOME/2026_IFAC/install/setup.bash" ] && . "$HOME/2026_IFAC/install/setup.bash"
export ROS_DOMAIN_ID='"$DOMAIN"'
'

die() { echo "❌ $*" >&2; exit 1; }

ssh "${SSH_OPTS[@]}" -o ConnectTimeout=8 -o BatchMode=yes "$HOST" true 2>/dev/null \
  || die "젯슨($HOST)에 SSH가 안 됩니다. 전원/wifi/~/.ssh/config 확인."

if [ "$MODE" = "list" ]; then
  ssh "${SSH_OPTS[@]}" "$HOST" "ls -1dt $REMOTE_DIR/*/ 2>/dev/null | head -20"
  exit 0
fi

# ── 회수 ────────────────────────────────────────────────────────────────────
pull_bag() {
  local name="$1"
  local day="${name#run_}"; day="${day%%_*}"           # run_0730_2115 → 0730
  [ -n "$day" ] || day="$(date +%m%d)"
  local dest="$LOCAL_ROOT/$day/$name"

  ssh "${SSH_OPTS[@]}" "$HOST" "test -f $REMOTE_DIR/$name/metadata.yaml" \
    || die "젯슨에 $name/metadata.yaml이 없습니다(녹화가 정상 종료되지 않았을 수 있음). 원본은 그대로 둡니다."

  mkdir -p "$dest"
  echo "⬇  $HOST:$REMOTE_DIR/$name → $dest"
  if ssh "${SSH_OPTS[@]}" "$HOST" 'command -v rsync' >/dev/null 2>&1; then
    rsync -ah --info=progress2 -e "ssh ${SSH_OPTS[*]}" "$HOST:$REMOTE_DIR/$name/" "$dest/" || die "rsync 실패. 젯슨 원본은 남아 있습니다."
  else
    scp -r "$HOST:$REMOTE_DIR/$name/." "$dest/" || die "scp 실패. 젯슨 원본은 남아 있습니다."
  fi

  # 크기 대조로 전송 검증 (원격 == 로컬)
  local rsz lsz
  rsz=$(ssh "${SSH_OPTS[@]}" "$HOST" "du -sb $REMOTE_DIR/$name | cut -f1")
  lsz=$(du -sb "$dest" | cut -f1)
  if [ "$rsz" != "$lsz" ]; then
    die "크기 불일치 (원격 $rsz != 로컬 $lsz). 젯슨 원본은 남아 있습니다."
  fi
  echo "✅ 회수 완료 ($(du -sh "$dest" | cut -f1))  $dest"

  if [ "$RM_REMOTE" = "1" ]; then
    ssh "${SSH_OPTS[@]}" "$HOST" "rm -rf $REMOTE_DIR/$name" && echo "🗑  젯슨 원본 삭제됨 (남기려면 --keep)"
  else
    echo "ℹ  젯슨 원본 유지: $REMOTE_DIR/$name"
  fi

  # 로컬에 ROS가 있으면 요약까지
  if command -v ros2 >/dev/null 2>&1; then
    ros2 bag info "$dest" 2>/dev/null | head -20
  fi
}

if [ "$MODE" = "pull" ]; then
  NAME="$ARG"
  if [ -z "$NAME" ]; then
    NAME=$(ssh "${SSH_OPTS[@]}" "$HOST" "ls -1dt $REMOTE_DIR/*/ 2>/dev/null | head -1 | xargs -r basename")
    [ -n "$NAME" ] || die "젯슨 $REMOTE_DIR 에 bag이 없습니다."
    echo "ℹ  최신 bag: $NAME"
  fi
  pull_bag "$NAME"
  exit 0
fi

# ── 재접속 (--attach): wifi가 끊겨 터미널만 떨어졌을 때 ──────────────────────
# tmux 세션은 젯슨에서 계속 녹화 중이다. 다시 붙어서 Ctrl+C로 끝내고 회수한다.
if [ "$MODE" = "attach" ]; then
  has_tmux=$(ssh "${SSH_OPTS[@]}" "$HOST" "tmux has-session -t $SESSION 2>/dev/null && echo yes" || true)
  [ "$has_tmux" = "yes" ] || die "젯슨에 '$SESSION' 세션이 없습니다. 이미 끝났다면 jpull 로 회수하세요."
  echo "🔗 젯슨 tmux '$SESSION' 재접속 — Ctrl+C 로 녹화를 끝내면 회수됩니다"
  trap '' INT
  ssh "${SSH_OPTS[@]}" -t "$HOST" "tmux attach -t $SESSION"
  trap - INT
  echo
  NAME=$(ssh "${SSH_OPTS[@]}" "$HOST" "ls -1dt $REMOTE_DIR/*/ 2>/dev/null | head -1 | xargs -r basename")
  [ -n "$NAME" ] || die "젯슨에 bag이 없습니다."
  pull_bag "$NAME"
  exit 0
fi

# ── 녹화 ────────────────────────────────────────────────────────────────────
NAME="run_$(date +%m%d_%H%M%S)${ARG:+_$ARG}"

# 젯슨에서 실행할 녹화 명령을 파일로 넘긴다(중첩 따옴표 회피).
REMOTE_RUNNER="\$HOME/.jetson_rec_cmd.sh"
printf '%s\nmkdir -p %s\nexec ros2 bag record -s sqlite3 -o %s/%s %s\n' \
  "$REMOTE_ENV" "$REMOTE_DIR" "$REMOTE_DIR" "$NAME" "$TOPICS" \
  | ssh "${SSH_OPTS[@]}" "$HOST" "cat > $REMOTE_RUNNER && chmod +x $REMOTE_RUNNER" \
  || die "원격 실행 스크립트 전송 실패."

# tmux가 있으면 그 안에서 돌린다 — wifi가 끊겨도 녹화는 젯슨에서 계속되고,
# jrec --attach 로 다시 붙어 정상 종료할 수 있다. tmux 없으면 그냥 ssh -t.
if ssh "${SSH_OPTS[@]}" "$HOST" 'command -v tmux' >/dev/null 2>&1; then
  ssh "${SSH_OPTS[@]}" "$HOST" "tmux kill-session -t $SESSION 2>/dev/null" || true
  REMOTE_CMD="tmux new-session -A -s $SESSION 'bash $REMOTE_RUNNER'"
  GUARD="wifi 끊겨도 젯슨에서 녹화 계속 — 다시 붙으려면 jrec --attach"
else
  REMOTE_CMD="bash $REMOTE_RUNNER"
  GUARD="⚠️ 젯슨에 tmux가 없어 wifi가 끊기면 녹화도 끊깁니다"
fi

echo "🔴 젯슨에서 녹화: $REMOTE_DIR/$NAME"
echo "   Ctrl+C 로 끝내면 $LOCAL_ROOT/$(date +%m%d)/ 로 회수 후 젯슨 원본 삭제"
echo "   $GUARD"

# Ctrl+C는 ssh -t의 원격 tty로 전달된다. 로컬 스크립트는 INT를 무시해야
# ssh 리턴 후 회수 단계가 실행된다.
trap '' INT
ssh "${SSH_OPTS[@]}" -t "$HOST" "$REMOTE_CMD"
trap - INT

echo
pull_bag "$NAME"
