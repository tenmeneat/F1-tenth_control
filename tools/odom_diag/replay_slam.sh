#!/bin/bash
# slam_toolbox 오프라인 재현 — 차 없이 파라미터 스윕.
#
# 사용:  bash replay_slam.sh <라벨> [param:=value ...]
#   예:  bash replay_slam.sh base
#        bash replay_slam.sh avp002 angle_variance_penalty:=0.02
#
# 결과: /tmp/slamrun/<라벨>/ 에 맵(pgm/yaml) + map->odom 궤적(tfbag) + slam.log
# 평가: metric.py (map->odom 표류) + map_quality.py (실제 맵)
#
# ⚠️ 반드시 bash 로 실행할 것. zsh 는 따옴표 없는 변수를 단어 분할하지 않아
#    `bash replay_slam.sh $LABEL $PARAMS` 형태로 넘기면 다중 파라미터가 한 인자로
#    뭉치고, ros2 가 그걸 문자열로 받아 InvalidParameterTypeException 으로 죽는다.
#    (2026-07-28 실제로 겪음 — 노드가 죽어도 tfbag 은 생기므로 조용히 오독하게 된다)

# ⚠️ set -u 금지 — ROS setup.bash 가 미정의 변수를 참조해 즉시 죽는다
set +u
# 2026-07-29: jazzy 우선, humble 폴백 (젯슨·랩탑 둘 다 24.04+Jazzy 로 올라감)
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
else
    source /opt/ros/humble/setup.bash
fi
# f1tenth_ws 는 젯슨에만 있다. 랩탑에서 오프라인 재현할 때는 없어도 된다
# (slam_toolbox 는 apt 로 /opt/ros 에 깔리고, 파라미터는 아래 저장소 사본을 쓴다).
[ -f ~/f1tenth_ws/install/setup.bash ] && source ~/f1tenth_ws/install/setup.bash
export ROS_DOMAIN_ID=91          # 실차 도메인(70)과 격리 — 재생이 차에 안 샌다
export ROS_LOCALHOST_ONLY=1                      # Jazzy 에선 폐기 예정이지만 아직 동작
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST   # Jazzy 의 정식 대체 변수

LABEL="${1:-base}"; shift || true
BAG="${BAG:-/tmp/bag_nomap}"     # filter_bag.py 로 map->odom 을 제거한 bag
# 파라미터 파일: 저장소 사본(랩탑) 우선, 없으면 젯슨 f1tenth_ws 설치본.
# ⚠️ f1tenth_stack 의 f1tenth_online_async.yaml 은 업스트림 원본이라 쓰면 안 된다 —
#    base_frame 이 laser 이고 angle_variance_penalty 가 1.0 이다.
YAML="$(dirname "$0")/slam_toolbox_params.yaml"
[ -f "$YAML" ] || YAML=~/f1tenth_ws/install/f1tenth_stack/share/f1tenth_stack/config/slam_toolbox_params.yaml
OUT=/tmp/slamrun/$LABEL
rm -rf "$OUT"; mkdir -p "$OUT"

# ⚠️ 시작 전 잔재 제거. `kill $!` 는 ros2 run 래퍼만 죽이고 실제 노드는 남는다.
# 남으면 다음 실행의 tf 기록에 여러 인스턴스의 map->odom 이 섞여 측정이 통째로 오염된다
# (2026-07-28: 실행할수록 샘플이 2661->5229->7843 으로 누적되는 걸로 발각).
pkill -9 -f "[a]sync_slam_toolbox_node" 2>/dev/null
pkill -9 -f "[r]os2 bag record" 2>/dev/null
sleep 2

EXTRA=()
for kv in "$@"; do EXTRA+=(-p "$kv"); done

echo "=== [$LABEL] slam_toolbox 기동 ==="
# map_update_interval 기본 5초면 /map 발행 사이에 저장 호출이 걸려 map_saver 가
# "Failed to spin map subscription"(result=255) 으로 죽는다. 발행 주기일 뿐
# 맵 품질과 무관하므로 재현 중에는 1초로 낮춘다.
# ⚠️ 단 이 값이 타이밍을 바꿔 결과에 영향을 준다 — 비교는 같은 하네스 설정끼리만.
ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
    --params-file "$YAML" -p use_sim_time:=true -p map_update_interval:=1.0 \
    "${EXTRA[@]}" > "$OUT/slam.log" 2>&1 &
SLAM=$!
sleep "${T_BOOT:-6}"

ros2 bag record -o "$OUT/tfbag" /tf /tf_static > "$OUT/rec.log" 2>&1 &
REC=$!
sleep 2

echo "=== bag 재생 ==="
# ⚠️ RATE 로 배속하면 slam 이 스캔 처리를 놓쳐 결과가 달라진다
#    (2026-07-28: 4배속에서 -2.00° -> -3.41°). 배속은 쓰지 말 것.
ros2 bag play "$BAG" --clock 100 -r "${RATE:-1.0}" > "$OUT/play.log" 2>&1
sleep 5

echo "=== 맵 저장 ==="
for try in 1 2 3; do
    ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
        "{name: {data: '$OUT/map'}}" > "$OUT/save.log" 2>&1
    sleep 3
    [ -f "$OUT/map.pgm" ] && { echo "  저장 성공 (시도 $try)"; break; }
    echo "  저장 실패 — 재시도 $try"; sleep 3
done
[ -f "$OUT/map.pgm" ] || echo "  ⚠️ 맵 저장 최종 실패"

kill -INT $REC 2>/dev/null; sleep 3          # bag record 는 INT 로 끝내야 db 가 닫힌다
kill $SLAM 2>/dev/null; sleep 2
pkill -9 -f "[a]sync_slam_toolbox_node" 2>/dev/null
pkill -9 -f "[r]os2 bag record" 2>/dev/null
sleep 1

# 파라미터 타입 에러 등으로 조용히 죽었으면 결과가 통째로 거짓이 된다
if grep -q "Exception\|Aborted" "$OUT/slam.log" 2>/dev/null; then
    echo "  ❌ slam_toolbox 가 죽었다 — 결과 신뢰 불가:"
    grep -m2 "Exception\|Aborted" "$OUT/slam.log"
fi
LEFT=$(pgrep -cf "[a]sync_slam_toolbox_node")
[ "$LEFT" != "0" ] && echo "  ⚠️ slam_toolbox 잔재 $LEFT 개 — 다음 측정이 오염된다"

echo "=== 결과 ==="; ls -la "$OUT" | grep -E "map|tfbag"; echo "완료: $OUT"
