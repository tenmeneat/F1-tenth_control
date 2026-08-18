#!/usr/bin/env bash
# 주행 중 젯슨 연산 부하를 측정하고 랩탑으로 회수한다.
#
# 왜 필요한가 (2026-08-18):
#   `run_0818_134408`에서 MCL이 1125 ms, 라이다가 850 ms 통째로 멈췄는데 같은 시각
#   VESC 드라이버·컨트롤러는 50 Hz를 한 사이클도 안 빠뜨렸다. 정지 직전까지 MCL 간격은
#   완벽한 25.0 ms였다 — 서서히 밀린 게 아니라 한 번에 구멍이 났다. bag만으로는
#   CPU·메모리·I/O 중 무엇인지 못 가른다. 이 스크립트가 그걸 가른다.
#
# 구조 (jetson_rec.sh와 같은 관례):
#   - 샘플러를 젯슨에 **파이프로 흘려보낸다** → 젯슨 쪽 사본이 필요 없다
#     (`jetsonup`이 tools/ 를 젯슨에 안 보내므로 이게 유일하게 맞는 방식이다).
#   - tmux 안에서 돌린다 → 주행 중 wifi가 끊겨도 측정은 계속되고 --attach로 회수한다.
#   - 시계: 젯슨에서 epoch로 찍는다. bag도 젯슨에서 같은 시계로 녹화되므로
#     (jetson_rec.sh) 후처리에서 그대로 붙는다.
#
# 사용법:
#   jetson_load.sh [태그]      측정 시작 → Ctrl+C → 회수
#   jetson_load.sh --attach    끊긴 측정에 재접속 → Ctrl+C → 회수
#   jetson_load.sh --pull      회수만
#   옵션: --host <ssh호스트>  --keep(젯슨 원본 유지)
#
# 🔑 bag 녹화(jrec)와 **같이** 돌릴 것. 둘을 붙여야 "MCL이 멈춘 그 순간의 부하"가 나온다.
#    분석: tools/bag_analyzer/analyze_jetson_load.py <bag> <부하폴더>

set -uo pipefail

HOST="${JETSON_HOST:-jetson}"
SESSION="f1load"
SSH_OPTS=(-o ServerAliveInterval=10 -o ServerAliveCountMax=6)
REMOTE_DIR="${JETSON_LOAD_DIR:-~/jetson_load}"
LOCAL_ROOT="${LOCAL_LOAD_ROOT:-$HOME/jetson_load}"
SAMPLER="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/jetson_load_sampler.py"
RM_REMOTE=1
MODE="record"
ARG=""

die() { echo "❌ $*" >&2; exit 1; }

while [ $# -gt 0 ]; do
  case "$1" in
    --pull)   MODE="pull";   shift; [ $# -gt 0 ] && [[ "$1" != --* ]] && { ARG="$1"; shift; } ;;
    --attach) MODE="attach"; shift ;;
    --keep)   RM_REMOTE=0;   shift ;;
    --host)   HOST="$2";     shift 2 ;;
    -h|--help) sed -n '2,24p' "$0"; exit 0 ;;
    *) ARG="$1"; shift ;;
  esac
done

[ -f "$SAMPLER" ] || die "샘플러가 없습니다: $SAMPLER"

pull_dir() {
  local name="$1"
  local dest="$LOCAL_ROOT/$(date +%m%d)"
  mkdir -p "$dest"
  echo "⬇️  회수: $REMOTE_DIR/$name → $dest/$name"
  rsync -a --info=progress2 "$HOST:$REMOTE_DIR/$name/" "$dest/$name/" \
    || die "회수 실패 — 젯슨 원본은 그대로 둡니다."
  local nl
  nl=$(wc -l < "$dest/$name/sys.csv" 2>/dev/null || echo 0)
  [ "$nl" -gt 1 ] || die "sys.csv가 비었습니다 — 젯슨 원본을 남깁니다."
  echo "✅ $dest/$name  (sys.csv $nl 행)"
  if [ "$RM_REMOTE" = "1" ]; then
    ssh "${SSH_OPTS[@]}" "$HOST" "rm -rf $REMOTE_DIR/$name" && echo "🧹 젯슨 원본 삭제"
  fi
  echo
  echo "다음: python3 ~/F1tenth_control/tools/bag_analyzer/analyze_jetson_load.py <bag> $dest/$name"
}

if [ "$MODE" = "pull" ]; then
  NAME="$ARG"
  [ -n "$NAME" ] || NAME=$(ssh "${SSH_OPTS[@]}" "$HOST" "ls -1dt $REMOTE_DIR/*/ 2>/dev/null | head -1 | xargs -r basename")
  [ -n "$NAME" ] || die "젯슨에 측정 결과가 없습니다."
  pull_dir "$NAME"; exit 0
fi

if [ "$MODE" = "attach" ]; then
  has=$(ssh "${SSH_OPTS[@]}" "$HOST" "tmux has-session -t $SESSION 2>/dev/null && echo yes" || true)
  [ "$has" = "yes" ] || die "젯슨에 '$SESSION' 세션이 없습니다. 끝났다면 --pull 로 회수하세요."
  echo "🔗 재접속 — Ctrl+C 로 끝내면 회수됩니다"
  trap '' INT
  ssh "${SSH_OPTS[@]}" -t "$HOST" "tmux attach -t $SESSION"
  trap - INT
  NAME=$(ssh "${SSH_OPTS[@]}" "$HOST" "ls -1dt $REMOTE_DIR/*/ 2>/dev/null | head -1 | xargs -r basename")
  pull_dir "$NAME"; exit 0
fi

# ── 측정 ────────────────────────────────────────────────────────────────────
NAME="load_$(date +%m%d_%H%M%S)${ARG:+_$ARG}"
REMOTE_PY="\$HOME/.jetson_load_sampler.py"

ssh "${SSH_OPTS[@]}" "$HOST" "cat > $REMOTE_PY" < "$SAMPLER" \
  || die "샘플러 전송 실패 — 젯슨이 켜져 있고 ssh가 되는지 확인하세요(ssh $HOST)."

if ssh "${SSH_OPTS[@]}" "$HOST" 'command -v tmux' >/dev/null 2>&1; then
  ssh "${SSH_OPTS[@]}" "$HOST" "tmux kill-session -t $SESSION 2>/dev/null" || true
  CMD="tmux new-session -A -s $SESSION 'python3 $REMOTE_PY $REMOTE_DIR/$NAME'"
  GUARD="wifi 끊겨도 젯슨에서 측정 계속 — 다시 붙으려면 $(basename "$0") --attach"
else
  CMD="python3 $REMOTE_PY $REMOTE_DIR/$NAME"
  GUARD="⚠️ 젯슨에 tmux가 없어 wifi가 끊기면 측정도 끊깁니다"
fi

echo "📊 젯슨 부하 측정: $REMOTE_DIR/$NAME"
echo "   Ctrl+C 로 끝내면 $LOCAL_ROOT/$(date +%m%d)/ 로 회수"
echo "   $GUARD"
echo "   🔑 bag 녹화(jrec)도 같이 돌리세요 — 붙여야 원인이 나옵니다"

trap '' INT
ssh "${SSH_OPTS[@]}" -t "$HOST" "$CMD"
trap - INT
echo
pull_dir "$NAME"
