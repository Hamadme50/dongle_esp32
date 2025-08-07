# Freeze a tiny blink app into the ESP32-C3 firmware.
# Build with (Linux/WSL):
#   make BOARD=ESP32_GENERIC_C3 FROZEN_MANIFEST=/ABS/PATH/blink_c3/manifest.py -j
#
# After build, flash the single image:
#   esptool.py --chip esp32c3 --baud 460800 write_flash 0 ports/esp32/build-ESP32_GENERIC_C3/firmware.bin

include("$(BOARD_DIR)/manifest.py")

# Freeze top-level main.py from this directory
module("main.py", base_path=".", opt=2)
