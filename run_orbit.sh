#!/usr/bin/env bash
# 원운동 짐벌링 실험 — a = -0.06 / -0.10 두 지점
#
# 시나리오
#    5.0 s  이륙        -> 9 s 호버 진입
#    9.5 s  DoB ON
#   10.0 s  arm 40 60 0 0   (특이점 회피 자세)
#   18.0 s  hold            (TCP 를 world 좌표에 고정)
#   20.0 s  orbit 0.05 10 yz  (반지름 5 cm, 주기 10 s, y-z 수직 평면)
#   30.0 s  orbit off       (10초 = 1바퀴)
#   34.0 s  hold off
#   38.0 s  착륙
#
# 사용법:  bash run_orbit.sh
set -e

DRONE=src/plant/xml/Palletrone.xml
ARM=~/cone_harvester_sim_ws/src/cone_harvester_sim/models/Forest_Cone_Harvesting_Robot.xml
OUTDIR=orbit_runs
FIGDIR=orbit_figs

mkdir -p "$OUTDIR" "$FIGDIR"

A_LIST_CM="-6 -10"

for CM in $A_LIST_CM; do
  A=$(awk "BEGIN{printf \"%.4f\", $CM/100}")
  N=$(echo "$CM" | sed 's/^-/m/; s/^\([0-9]\)/p\1/')

  echo "=================== a = $CM cm  (tag $N) ==================="

  # 1) 모델 생성 (팔 관절 살아있는 버전 — --rigid 없음)
  python3 build_arm_drone.py \
      --drone "$DRONE" --arm "$ARM" \
      --j1 90 --j2 5 --a "$A" --kp 400 \
      --base-mass 3.3 --arm-len 0.23 \
      --cw-mass 0.610 --cw-pos -0.251 --cw-z 0.165 \
      --top-height 0.404 \
      --out "src/plant/xml/orbit_$N.xml"

  # 2) 시나리오 실행 + CSV 기록
  python3 fly_sim.py --model "src/plant/xml/orbit_$N.xml" --no-viewer \
      --script "1:log $OUTDIR/orbit_$N.csv;5:takeoff;9.5:dob on;10:arm 40 60 0 0;18:hold;20:orbit 0.05 15 yz;35:orbit off;45:land"

  # 3) 그래프
  python3 plot_result.py "$OUTDIR/orbit_$N.csv" \
      --title "a = $CM cm   |   DoB ON + TCP hold   |   orbit R=5cm T=10s (yz), 20-30 s" \
      --hover 14.41 --att-rad \
      --ylim-att -0.2 0.2 --ylim-pos -0.3 1.4 --ylim-f 0 30 \
      --out "$FIGDIR/orbit_a$N.png"
done

echo
echo "완료:  CSV -> $OUTDIR/    그림 -> $FIGDIR/"
echo
echo "영상 촬영용 (뷰어 실행):"
for CM in $A_LIST_CM; do
  N=$(echo "$CM" | sed 's/^-/m/; s/^\([0-9]\)/p\1/')
  echo "  python3 fly_sim.py --model src/plant/xml/orbit_$N.xml \\"
  echo "    --script \"5:takeoff;9.5:dob on;10:arm 40 60 0 0;18:hold;20:orbit 0.05 15 yz;35:orbit off;45:land\""
done
