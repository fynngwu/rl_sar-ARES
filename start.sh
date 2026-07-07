#!/bin/bash
for can in can0 can1 can2 can3; do
    sudo ip link set "$can" down 2>/dev/null || true
    sudo ip link set "$can" up type can bitrate 1000000 restart-ms 10 2>/dev/null || true
    sudo ip link set "$can" txqueuelen 1000
done
