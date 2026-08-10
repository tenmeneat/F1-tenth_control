#!/usr/bin/env bash
# f1net_client.sh — "젯슨 토픽이 내 랩탑에 안 보인다" 자가진단 (젯슨 ssh 불필요)
#
# 전제: ① 젯슨 스택이 떠 있을 것  ② 내 랩탑이 HY_MIRU(10.1.1.x)에 붙어 있을 것
# 사용:  bash f1net_client.sh
#
# 원리: 젯슨의 DDS는 자기 존재를 239.255.0.1 멀티캐스트로 계속 방송한다(SPDP).
#       그 포트가 7400 + 250*도메인번호라서, 받아보는 것만으로
#       네트워크·방화벽·도메인 일치를 한 번에 검증할 수 있다.

JET=10.1.1.1
FAIL=0
say() { printf '%s\n' "$*"; }

say "════════ f1net 자가진단  $(date '+%m-%d %H:%M:%S') ════════"

# ── 1. 네트워크 ────────────────────────────────────────────
IF=$(ip -4 -o route get $JET 2>/dev/null | awk '{for(i=1;i<=NF;i++) if($i=="dev") print $(i+1)}')
MYIP=$(ip -4 -o route get $JET 2>/dev/null | awk '{for(i=1;i<=NF;i++) if($i=="src") print $(i+1)}')
say ""
say "[1] 네트워크"
if [ -z "$MYIP" ]; then
  say "  🔴 젯슨($JET)로 가는 경로가 없다 → HY_MIRU에 안 붙어 있음"
  say "     고치기: nmcli con up HY_MIRU"
  exit 1
fi
say "  인터페이스 = $IF / 내 IP = $MYIP"
case "$MYIP" in
  10.1.1.*) ;;
  *) say "  🔴 내 IP가 10.1.1.x가 아니다 → HY_MIRU가 아닌 다른 WiFi에 붙어 있다"
     say "     고치기: nmcli con up HY_MIRU"; exit 1 ;;
esac
if ping -c1 -W2 $JET >/dev/null 2>&1; then say "  🟢 젯슨 ping OK"
else say "  🔴 젯슨 ping 실패 → WiFi 재접속(nmcli con up HY_MIRU)"; exit 1; fi

NIC=$(ip -br addr show up 2>/dev/null | grep -vE '^(lo|'"$IF"')\s' | awk '{print $1}' | tr '\n' ' ')
[ -n "$NIC" ] && say "  ⚠️  다른 인터페이스도 살아있음: $NIC  (VPN/docker면 DDS가 헷갈릴 수 있다)"

# ── 2. ROS 환경 ────────────────────────────────────────────
D=${ROS_DOMAIN_ID:-0}
say ""
say "[2] ROS 환경"
say "  ROS_DOMAIN_ID       = ${ROS_DOMAIN_ID:-(미설정 → 0)}"
say "  ROS_LOCALHOST_ONLY  = ${ROS_LOCALHOST_ONLY:-(미설정)}"
say "  RMW_IMPLEMENTATION  = ${RMW_IMPLEMENTATION:-(미설정 → rmw_fastrtps_cpp)}"
[ "$D" != "70" ] && { say "  🔴 도메인이 70이 아니다 → ~/.zshrc의 ROS_DOMAIN_ID를 70으로"; FAIL=1; }
[ "${ROS_LOCALHOST_ONLY:-0}" = "1" ] && { say "  🔴 LOCALHOST_ONLY=1 → 네트워크를 아예 안 쓴다. unset 할 것"; FAIL=1; }
case "${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}" in
  rmw_fastrtps_cpp) ;;
  *) say "  🔴 젯슨은 rmw_fastrtps_cpp다. 맞출 것"; FAIL=1 ;;
esac

# ── 3. 방화벽 ──────────────────────────────────────────────
say ""
say "[3] 방화벽"
FWBAD=0
if command -v ufw >/dev/null && [ "$(systemctl is-active ufw 2>/dev/null)" = "active" ]; then
  say "  ufw       : active   ← 인바운드 UDP를 막는다"; FWBAD=1
else
  say "  ufw       : $(systemctl is-active ufw 2>/dev/null || echo 없음)"
fi
if [ "$(systemctl is-active firewalld 2>/dev/null)" = "active" ]; then
  say "  firewalld : active   ← 인바운드 UDP를 막는다"; FWBAD=2
fi
[ $FWBAD -eq 0 ] && say "  🟢 활성 방화벽 없음"

# ── 4. 젯슨 DDS 방송 수신 (핵심) ───────────────────────────
PORT=$((7400 + 250 * D))
say ""
say "[4] 젯슨 DDS 방송 수신 시험 — 239.255.0.1:$PORT (도메인 $D), 12초 대기"
python3 - "$MYIP" "$PORT" <<'PY'
import socket, struct, sys, time
myip, port = sys.argv[1], int(sys.argv[2])
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
try:
    s.bind(('', port))
except OSError as e:
    print(f"  ⚠️  포트 {port} bind 실패({e}) — ros2 데몬이 쥐고 있을 수 있다. 'ros2 daemon stop' 후 재시도")
    sys.exit(2)
s.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP,
             struct.pack('4s4s', socket.inet_aton('239.255.0.1'), socket.inet_aton(myip)))
s.settimeout(12)
peers, t0 = set(), time.time()
while time.time() - t0 < 12:
    try:
        d, a = s.recvfrom(2048)
    except socket.timeout:
        break
    if d[:4] == b'RTPS':
        peers.add(a[0])
if peers:
    print(f"  🟢 RTPS 방송 수신됨: {sorted(peers)}")
    print("     → 네트워크·방화벽·도메인 전부 정상. 문제는 ros2 데몬 캐시일 가능성이 크다")
    sys.exit(0)
print("  🔴 12초간 RTPS 패킷 0개")
sys.exit(1)
PY
RX=$?

# ── 5. 판정 ────────────────────────────────────────────────
say ""
say "════════ 판정 ════════"
if [ $RX -eq 0 ]; then
  say "  네트워크 계층은 정상이다. 아래를 실행하고 다시 볼 것:"
  say "      ros2 daemon stop && ros2 topic list"
  say "  그래도 0개면 젯슨 스택이 안 떠 있는 것 (젯슨에서 ros2 node list 확인)"
elif [ $FWBAD -eq 1 ]; then
  say "  🔴 ufw가 원인이다. 고치기:"
  say "      sudo ufw allow from 10.1.1.0/24 comment 'jetson AP (ROS2 DDS)'"
  say "      sudo ufw reload && ros2 daemon stop && ros2 topic list"
elif [ $FWBAD -eq 2 ]; then
  say "  🔴 firewalld가 원인이다. 고치기:"
  say "      sudo firewall-cmd --permanent --zone=trusted --add-source=10.1.1.0/24"
  say "      sudo firewall-cmd --reload && ros2 daemon stop && ros2 topic list"
elif [ $FAIL -eq 1 ]; then
  say "  🔴 [2] ROS 환경 설정이 틀렸다. 위 🔴 항목을 고치고 새 터미널에서 재실행"
else
  say "  🔴 방화벽도 환경도 정상인데 젯슨 방송이 안 온다. 다음 중 하나다:"
  say "     · 젯슨 스택이 안 떠 있다 (제일 흔함)"
  say "     · 젯슨이 도메인 $D 가 아니다"
  say "     · docker/VPN 인터페이스가 멀티캐스트를 가로챈다 → 끄고 재시도"
  say "     · WiFi 절전이 멀티캐스트를 버린다 → sudo iw dev $IF set power_save off"
fi
