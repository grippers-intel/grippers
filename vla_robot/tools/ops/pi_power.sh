#!/usr/bin/env bash
# Pi 5 전력·CPU 재기(2026-10-12, 차체 배터리) — **Pi 호스트에서** 돌린다(컨테이너 안이 아니다. vcgencmd 가 호스트에 있다).
#
#   bash /home/pi/docker/shared/grippers/vla_deploy/vla_robot/tools/ops/pi_power.sh [초=30] [이름]
#
# 차체 배터리(2S)가 Pi 5 와 주변(그리퍼캠 · Hailo · 컨트롤러 보드)을 같이 먹인다. 10-12 궤적: 판 시간의 60~80 % 가
# 정지인데도 전압이 내려갔다 → 늘 켜진 바탕 부하를 숫자로 본다. 같은 대기 상태에서 조건만 바꿔 두 번 재 비교한다
# (예: 그리퍼캠 idle_period_s 0 / 0.5, Hailo 드라이버 있음 / 없음).
#
# 보는 것: PMIC 레일 전력 합(W, Pi 5 보드 안쪽 — USB 장치 몫은 빠진다) · 온도 · 스로틀 · CPU 많이 쓰는 프로세스.
set -u
SECS=${1:-30}
NAME=${2:-}
OUT=/tmp/pi_power_$(date +%H%M%S)${NAME:+_$NAME}.txt

if ! command -v vcgencmd >/dev/null; then
    echo "vcgencmd 가 없다 — Pi 호스트에서 돌릴 것(컨테이너 안이 아니다)"; exit 1
fi

pmic_watts() {
    # "XXX_A current(n)=0.123A" 와 "XXX_V volt(n)=0.789V" 를 이름으로 짝지어 곱한 합
    vcgencmd pmic_read_adc | awk '
        /current\(/ { k = $1; sub(/_A$/, "", k); gsub(/.*=/, "", $2); gsub(/A/, "", $2); cur[k] = $2 }
        /volt\(/    { k = $1; sub(/_V$/, "", k); gsub(/.*=/, "", $2); gsub(/V/, "", $2); vol[k] = $2 }
        END { w = 0; for (k in cur) if (k in vol) w += cur[k] * vol[k]; printf "%.3f\n", w }'
}

{
    echo "== $(date '+%F %T') ${NAME} — ${SECS}s"
    echo "Hailo: $(lsmod | awk '/^hailo/ {print $1" 사용 "$3}')"
    echo "컨테이너: $(docker ps --format '{{.Names}}' 2>/dev/null | tr '\n' ' ')"
    n=0; sum=0; mn=999; mx=0
    for _ in $(seq "$SECS"); do
        w=$(pmic_watts)
        n=$((n + 1)); sum=$(awk -v a="$sum" -v b="$w" 'BEGIN{print a+b}')
        mn=$(awk -v a="$mn" -v b="$w" 'BEGIN{print (b<a)?b:a}'); mx=$(awk -v a="$mx" -v b="$w" 'BEGIN{print (b>a)?b:a}')
        sleep 1
    done
    echo "PMIC 전력 평균 $(awk -v s="$sum" -v n="$n" 'BEGIN{printf "%.2f", s/n}') W (최소 $mn · 최대 $mx, ${n}번)"
    echo "온도 $(vcgencmd measure_temp | cut -d= -f2) · 스로틀 $(vcgencmd get_throttled | cut -d= -f2)"
    echo "-- CPU 많이 쓰는 프로세스(5 s 평균, 100 % = 코어 하나)"
    top -b -d 5 -n 2 -o %CPU | awk '/^ *PID/ {blk++} blk == 2' | head -12 | cut -c1-120
} | tee "$OUT"
echo "(저장: $OUT)"
