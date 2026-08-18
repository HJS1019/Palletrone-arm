#!/usr/bin/env bash
# Palletrone + 로봇팔 : 장착 위치(a) 스윕 + DoB 유무 비교
#
# 시나리오
#    5 s  이륙 -> 9 s 호버 진입
#    9.5s DoB ON  (DoB 버전만)
#   15 s  외란 +x 5 N 인가
#   35 s  외란 해제        (20초간 지속)
#   50 s  착륙
#
# 사용법:  bash run_sweep.sh
set -e

DRONE=src/plant/xml/Palletrone.xml
ARM=~/cone_harvester_sim_ws/src/cone_harvester_sim/models/Forest_Cone_Harvesting_Robot.xml
OUTDIR=runs2
FIGDIR=figs_a

mkdir -p "$OUTDIR" "$FIGDIR"

# 장착점 [cm] : 경계 안(-16~0) + 경계 밖(+4~+12)
A_LIST_CM="-16 -12 -8 -4 0 4 8 12"

for CM in $A_LIST_CM; do
  A=$(awk "BEGIN{printf \"%.4f\", $CM/100}")
  N=$(echo "$CM" | sed 's/^-/m/; s/^\([0-9]\)/p\1/')

  echo "=================== a = $CM cm  (tag $N) ==================="

  # 1) 모델 생성 : 팔 수평(j1=90) 강체, 변압기 윗면, 다리 포함
  python3 build_arm_drone.py \
      --drone "$DRONE" --arm "$ARM" \
      --rigid --j1 90 --a "$A" \
      --base-mass 3.3 --arm-len 0.23 \
      --cw-mass 0.610 --cw-pos -0.251 --cw-z 0.165 \
      --top-height 0.404 \
      --out "src/plant/xml/_a$N.xml"

  # 2) DoB 없이
  python3 fly_sim.py --model "src/plant/xml/_a$N.xml" --no-viewer \
      --script "1:log $OUTDIR/nodob_$N.csv;5:takeoff;15:dist 5 0 0;35:dist off;50:land"

  # 3) DoB 켜고 (호버 진입 직후)
  python3 fly_sim.py --model "src/plant/xml/_a$N.xml" --no-viewer \
      --script "1:log $OUTDIR/dob_$N.csv;5:takeoff;9.5:dob on;15:dist 5 0 0;35:dist off;50:land"
done

# 4) 그래프 16장
for CM in $A_LIST_CM; do
  N=$(echo "$CM" | sed 's/^-/m/; s/^\([0-9]\)/p\1/')
  for TAG in nodob dob; do
    if [ "$TAG" = "dob" ]; then LBL="DoB ON"; else LBL="DoB off"; fi
    python3 plot_result.py "$OUTDIR/${TAG}_$N.csv" \
        --title "a = $CM cm   |   $LBL   |   +5 N at 15-35 s" \
        --hover 14.41 --att-rad \
        --ylim-att -0.2 0.2 --ylim-pos -0.6 1.4 --ylim-f 0 30 \
        --out "$FIGDIR/${TAG}_a$N.png"
  done
done

echo
echo "완료:  CSV -> $OUTDIR/    그림 -> $FIGDIR/"
