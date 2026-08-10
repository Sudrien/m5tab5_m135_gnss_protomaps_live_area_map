#!/usr/bin/env bash
# Compile, upload and monitor the Arduino build with the FQBN this board needs.
#
# The FQBN has to be repeated for compile and upload, and retyping it is how
# "Invalid FQBN: not an FQBN: ..." happens. Keep it in one place.
set -euo pipefail

FQBN="esp32:esp32:m5stack_tab5:FlashSize=16M,PSRAM=enabled,PartitionScheme=app3M_fat9M_16MB,CDCOnBoot=cdc,USBMode=hwcdc"
PORT="${PORT:-/dev/ttyACM0}"

cd "$(dirname "$0")"
arduino-cli compile --fqbn "$FQBN" .
arduino-cli upload  --fqbn "$FQBN" --port "$PORT" .
sleep 2
exec arduino-cli monitor --port "$PORT" --config baudrate=115200
