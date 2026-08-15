#!/usr/bin/env bash
# f1time.sh — 랩탑 시계를 젯슨에 밀어넣는다 (2026-08-14 신설)
# ============================================================================
# 왜 필요한가: 젯슨 RTC에 백업 배터리가 없어 **전원을 끊을 때마다 시계가 1970으로
# 돌아간다.** 0814 실측 — 20시 재부팅 이후 bag 3개가 전부 "epoch 0 + uptime"으로
# 찍혔다(run_0814_200000 = 1970-01-01 00:19:10).
#
# 시각이 틀리면 조용히 깨지는 것들:
#   · rsync/f1learn 이 mtime 비교로 파일을 건너뛴다 (1970 파일 = "오래된" 파일)
#   · colcon/make 가 소스 > 산출물 판정을 뒤집어 **재빌드를 건너뛴다**
#   · rosbag 폴더가 run_0101_* 로 나와 엉뚱한 날짜 폴더에 들어간다
#
# 🔴 **ROS 스택이 떠 있는 동안 실행하지 말 것.** control_map_node 는 this->now()
#    (시스템 시계)로 신선도를 재므로 56년 점프에 local_fresh_timeout /
#    drive_mode_timeout / sector_scale_timeout / IMU 0.2 s 가 전부 오작동한다.
#    → 이 스크립트는 젯슨에서 ROS 프로세스를 발견하면 **거부한다**(--force 로 무시).
#
# 사용법:
#   bash tools/f1time.sh              # 확인 + 필요하면 동기화
#   bash tools/f1time.sh --check      # 오차만 보고 아무것도 안 바꾼다
#   bash tools/f1time.sh --force      # ROS 실행 중에도 강행 (권장하지 않음)
#   bash tools/f1time.sh --auto       # ssh LocalCommand 용 (조용함·실패해도 0)
#
# 근본 대책은 캐리어보드 RTC 코인셀(CR2032) 장착이다 — 이건 그때까지의 땜빵이다.
set -u

HOST="${F1_JETSON_HOST:-jetson}"
SSH_OPTS=(-o ConnectTimeout=5 -o BatchMode=yes)

# ── 젯슨에서 "주행 스택이 떠 있나"를 보는 명령 ────────────────────────────────
# 🔴 브래킷 트릭(`[c]ontrol...`)이 **필수**다. 이 패턴 문자열은 원격 명령줄에 그대로
#    실려 가므로, 그냥 "control_map_node" 라고 쓰면 그 명령을 실행하는 셸 자신의
#    cmdline 이 매치돼 **항상 "ROS 실행 중"으로 오판한다**(2026-08-14 실제로 발생 —
#    `6170 zsh -c date +%s.%N; pgrep -fa "ros2|..."` 가 자기 자신을 잡았다).
#    정규식 `[c]ontrol_map_node` 는 "control_map_node" 에 매치되지만, 명령줄에 남는
#    문자열 "[c]ontrol_map_node" 에는 매치되지 않는다.
# ⚠️ `ros2` 단독으로 매치하지 않는다 — `_ros2_daemon` 은 주행과 무관한데 잡힌다.
ROS_PGREP='pgrep -fa "[c]ontrol_map_node|[r]os2 launch|[v]esc_driver|[p]article_filter"'
CHECK_ONLY=0; FORCE=0; AUTO=0
for a in "$@"; do
  case "$a" in
    --check) CHECK_ONLY=1 ;;
    --force) FORCE=1 ;;
    --auto)  AUTO=1 ;;
    -h|--help) sed -n '2,32p' "$0"; exit 0 ;;
    *) echo "알 수 없는 인자: $a"; exit 2 ;;
  esac
done

# ══ --auto: ssh 로 젯슨에 붙을 때마다 자동 실행되는 경로 ═════════════════════
# ~/.ssh/config 의  Host jetson  에  PermitLocalCommand yes + LocalCommand 로 건다.
# 여기서 지켜야 할 것 세 가지 — 하나라도 어기면 **젯슨으로 가는 모든 ssh가 망가진다**:
#   ① 재귀 금지  : 이 스크립트가 여는 ssh 도 같은 LocalCommand 를 다시 부른다
#                  → F1TIME_INFLIGHT 를 export 해서 안쪽 호출은 즉시 빠진다
#   ② 속도       : rsync(jetsonup/f1learn)·git 은 연결을 여러 번 연다. 스탬프 파일로
#                  F1TIME_MIN_INTERVAL 초에 한 번만 실제로 재고 나머지는 즉시 반환
#   ③ 절대 실패 금지 : 어떤 경로로도 exit 0. 조용하고, 뭔가 했을 때만 한 줄 찍는다
if [ "$AUTO" -eq 1 ]; then
  [ -n "${F1TIME_INFLIGHT:-}" ] && exit 0            # ① 재귀 차단
  stamp="/tmp/.f1time-${HOST}-$(id -u).stamp"
  interval="${F1TIME_MIN_INTERVAL:-60}"
  if [ -f "$stamp" ]; then
    last=$(stat -c %Y "$stamp" 2>/dev/null || echo 0)
    # ⚠️ 랩탑 시계가 뒤로 갔거나 스탬프가 미래면 (now-last)가 음수 → 그냥 다시 잰다
    age=$(( $(date +%s) - last ))
    [ "$age" -ge 0 ] && [ "$age" -lt "$interval" ] && exit 0   # ② 레이트 리밋
  fi
  touch "$stamp" 2>/dev/null
  # 실제 작업은 서브셸에서. 여기서 죽어도 바깥은 항상 0 으로 끝난다(③).
  (
    export F1TIME_INFLIGHT=1
    out=$(timeout 6 ssh -o ConnectTimeout=3 -o BatchMode=yes "$HOST" \
            "date +%s; $ROS_PGREP | head -1" 2>/dev/null) || exit 0
    jt=$(printf '%s\n' "$out" | head -1)
    ros=$(printf '%s\n' "$out" | tail -n +2)
    case "$jt" in ''|*[!0-9]*) exit 0 ;; esac
    now=$(date +%s); d=$(( jt - now )); [ "$d" -lt 0 ] && d=$(( -d ))
    [ "$d" -le 2 ] && exit 0                          # 이미 맞음 — 조용히 끝
    if [ -n "$ros" ]; then
      echo "⚠️ 젯슨 시계가 ${d}s 어긋났지만 ROS 가 돌고 있어 건드리지 않았다 (스택 내린 뒤 f1time.sh)"
      exit 0
    fi
    # ⚠️ systemd 는 NTP 동기화가 활성이면 `date -s` 를 거부한다("Automatic time
    #    synchronization is enabled") — 그래서 먼저 끈다. 인터넷 되는 곳에 가져가면
    #    `sudo timedatectl set-ntp true` 로 되돌릴 것.
    if timeout 6 ssh -o ConnectTimeout=3 -o BatchMode=yes "$HOST" \
         "sudo -n timedatectl set-ntp false >/dev/null 2>&1; \
          sudo -n date -s @$now >/dev/null 2>&1 && sudo -n hwclock -w >/dev/null 2>&1; true" 2>/dev/null; then
      chk=$(timeout 6 ssh -o ConnectTimeout=3 -o BatchMode=yes "$HOST" 'date +%s' 2>/dev/null)
      case "$chk" in ''|*[!0-9]*) exit 0 ;; esac
      nd=$(( chk - $(date +%s) )); [ "$nd" -lt 0 ] && nd=$(( -nd ))
      if [ "$nd" -le 2 ]; then
        echo "🕐 젯슨 시계 동기화 (${d}s 어긋나 있었음 → $(date '+%H:%M:%S'))"
      else
        echo "⚠️ 젯슨 시계 ${d}s 어긋남 — 자동 설정 실패(sudo 비밀번호?). 수동: bash tools/f1time.sh"
      fi
    fi
  ) 2>/dev/null
  exit 0
fi

echo "── 젯슨 시계 점검 ($HOST) ──────────────────────────────────────────"

# ── 1. 도달성 + 젯슨 현재 시각 ─────────────────────────────────────────────
# ⚠️ ssh 왕복 지연이 오차에 섞이므로 왕복 시간을 재서 절반을 보정한다.
t0=$(date +%s.%N)
remote=$(ssh "${SSH_OPTS[@]}" "$HOST" "date +%s.%N; $ROS_PGREP | head -3" 2>/dev/null)
rc=$?
t1=$(date +%s.%N)
if [ $rc -ne 0 ] || [ -z "$remote" ]; then
  echo "🔴 젯슨에 못 붙었다 ($HOST). AP 연결·ssh config 를 먼저 볼 것."
  echo "   네트워크 자체가 의심되면: bash tools/f1net_client.sh"
  exit 1
fi

jt=$(printf '%s\n' "$remote" | head -1)
ros_procs=$(printf '%s\n' "$remote" | tail -n +2)
rtt=$(echo "$t1 - $t0" | bc -l)
mid=$(echo "($t0 + $t1) / 2" | bc -l)
skew=$(echo "$jt - $mid" | bc -l)

printf "   랩탑 : %s\n" "$(date '+%Y-%m-%d %H:%M:%S %Z')"
printf "   젯슨 : %s\n" "$(date -d "@$jt" '+%Y-%m-%d %H:%M:%S %Z' 2>/dev/null || echo "$jt")"
printf "   오차 : %+.3f s   (ssh 왕복 %.3f s — 이 절반이 측정 불확실성)\n" "$skew" "$rtt"

# ── 2. 랩탑 자신이 맞는지부터 ─────────────────────────────────────────────
# 기준자가 틀렸으면 동기화는 오차를 퍼뜨리는 짓이다.
if ! timedatectl show -p NTPSynchronized --value 2>/dev/null | grep -q yes; then
  echo "   ⚠️ 랩탑이 NTP 동기화 상태가 아니다 — 이걸 기준자로 쓰면 안 된다."
  echo "      인터넷 되는 곳에서 'timedatectl set-ntp true' 후 다시 할 것."
  [ "$FORCE" -eq 1 ] || exit 1
fi

# 1초 미만이면 손대지 않는다. 클럭을 스텝시키는 건 공짜가 아니다.
if awk "BEGIN{exit !($skew < 1 && $skew > -1)}"; then
  echo "✅ 1초 이내 — 동기화 불필요."
  exit 0
fi

if [ "$CHECK_ONLY" -eq 1 ]; then
  echo "   (--check: 아무것도 바꾸지 않았다)"
  exit 0
fi

# ── 3. ROS 가 돌고 있으면 거부 ────────────────────────────────────────────
if [ -n "$ros_procs" ]; then
  echo ""
  echo "🔴 젯슨에서 ROS 프로세스가 돌고 있다 — 시계를 옮기면 신선도 타임아웃이 전부 깨진다:"
  printf '      %s\n' "$ros_procs"
  echo "   스택을 내리고 다시 실행할 것. (그래도 강행하려면 --force)"
  [ "$FORCE" -eq 1 ] || exit 1
  echo "   ⚠️ --force: 강행한다."
fi

# ── 4. 설정 ───────────────────────────────────────────────────────────────
# BatchMode 를 풀고 -t 를 준다 — 젯슨 sudo 가 비밀번호를 물을 수 있다.
# hwclock -w 는 RTC 배터리가 있을 때만 의미가 있지만, 있으면 다음 부팅이 살아난다.
now=$(date +%s)
echo ""
echo "   젯슨 시각을 설정한다 (sudo 비밀번호를 물을 수 있음)..."
ssh -t -o ConnectTimeout=5 "$HOST" \
    "sudo timedatectl set-ntp false 2>/dev/null; sudo date -s @$now >/dev/null && \
     { sudo hwclock -w 2>/dev/null && echo '   (RTC 에도 기록했다 — 배터리가 있으면 다음 부팅에 살아남는다)' \
       || echo '   ⚠️ hwclock 기록 실패 — RTC 배터리가 없다는 뜻일 수 있다'; } ; date"
echo ""
echo "── 다시 확인 ──"
exec "$0" --check
