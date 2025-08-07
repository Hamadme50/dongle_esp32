# boot.py — ESP32-C3 (MicroPython)
# Give REPL 10s for file uploads, then start main.py

import time, machine

WAIT_S   = 10          # seconds to keep REPL free
SAFE_PIN = 9           # BOOT button on many ESP32-C3 boards (active-low)

def boot_safe_pin():
    try:
        pin = machine.Pin(SAFE_PIN, machine.Pin.IN, machine.Pin.PULL_UP)
        return pin.value() == 0   # held low -> stay in REPL
    except Exception:
        return False

def boot_safe_file():
    try:
        import os
        return "safe_mode" in os.listdir()
    except Exception:
        return False

def countdown(seconds):
    print("REPL is free for {}s. Press Ctrl-C to cancel autostart...".format(seconds))
    try:
        for s in range(seconds, 0, -1):
            print(" Autostart in {}s".format(s))
            time.sleep(1)
    except KeyboardInterrupt:
        print("Autostart cancelled. REPL is now free.")

# Optional: silence verbose ROM logs (uncomment if present on your build)
# try:
#     import esp
#     esp.osdebug(None)
# except Exception:
#     pass

# Optional: start WebREPL if you've already run webrepl_setup()
# try:
#     import webrepl
#     webrepl.start()
# except Exception as e:
#     print("WebREPL start failed:", e)

# Safe modes
if boot_safe_pin():
    print("SAFE MODE: BOOT held -> staying in REPL.")
    raise SystemExit
if boot_safe_file():
    print("SAFE MODE: 'safe_mode' file present -> staying in REPL.")
    raise SystemExit

# Give REPL time for uploads; Ctrl-C here drops to REPL
countdown(WAIT_S)

# Start your app
try:
    import main
    if hasattr(main, "start"):
        main.start()
except KeyboardInterrupt:
    print("Startup interrupted; staying in REPL.")
    raise SystemExit
except Exception as e:
    print("Boot error:", e)
    raise
