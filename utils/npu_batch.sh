# 1초 간격, % 표시
prev=$(cat /sys/devices/pci0000:00/0000:00:0b.0/npu_busy_time_us); \
while true; do \
  sleep 1; \
  curr=$(cat /sys/devices/pci0000:00/0000:00:0b.0/npu_busy_time_us); \
  echo "NPU: $(( (curr - prev) / 10000 ))%"; \
  prev=$curr; \
done
