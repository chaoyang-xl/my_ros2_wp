#!/usr/bin/env bash
# RK3588 实时监控面板 -- CPU/NPU/GPU/温度/内存
# 用法: sudo bash ~/monitor.sh   (Ctrl+C 退出)

while true; do
  clear
  echo "=========================================="
  echo "  RK3588 系统监控  $(date '+%H:%M:%S')"
  echo "=========================================="

  # --- CPU 各核频率与占用 ---
  echo "[CPU]"
  for i in 0 1 2 3 4 5 6 7; do
    freq=$(cat /sys/devices/system/cpu/cpufreq/policy${i}/scaling_cur_freq 2>/dev/null || echo 0)
    freq_mhz=$((freq / 1000))
    usage=$(grep "^cpu${i} " /proc/stat | awk '{t=$2+$3+$4+$5+$6+$7+$8; printf "%.0f",100*(t-$5)/t}')
    [ $i -lt 4 ] && ct="A55" || ct="A76"
    printf "  cpu%d(%s) %4d MHz  %3s%%\n" "$i" "$ct" "$freq_mhz" "$usage"
  done

  # --- NPU ---
  echo "------------------------------------------"
  echo "[NPU]"
  if [ -f /sys/kernel/debug/rknpu/load ]; then
    cat /sys/kernel/debug/rknpu/load
  else
    echo "  不可用 (需 sudo 或驱动未加载)"
  fi

  # --- GPU ---
  echo "------------------------------------------"
  echo "[GPU]"
  gpu_freq=$(cat /sys/class/devfreq/fb000000.gpu/cur_freq 2>/dev/null)
  [ -n "$gpu_freq" ] && echo "  freq: $((gpu_freq / 1000000)) MHz" || echo "  不可用"

  # --- 温度 ---
  echo "------------------------------------------"
  echo "[温度]"
  for tz in /sys/class/thermal/thermal_zone*/; do
    type=$(cat "${tz}type" 2>/dev/null || continue)
    temp=$(cat "${tz}temp" 2>/dev/null || echo 0)
    printf "  %-22s %s C\n" "$type" "$(echo "scale=1;$temp/1000" | bc)"
  done

  # --- 内存 ---
  echo "------------------------------------------"
  echo "[内存]"
  free -h | awk '/^Mem:/{printf "  总计:%s  已用:%s  可用:%s\n",$2,$3,$7}'
  free -h | awk '/^Swap:/{printf "  Swap 总计:%s  已用:%s\n",$2,$3}'

  echo "=========================================="
  sleep 2
done