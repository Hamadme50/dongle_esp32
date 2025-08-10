# boot.py — ESP32-C3 (MicroPython)
# Show files, keep REPL free, and only start main.py if not in safe mode.
# Press Ctrl-C during the countdown to create 'safe_mode' and stay in REPL (persistently).

import time, machine, os

WAIT_S     = 10            # seconds to keep REPL free
SAFE_PIN   = 9             # optional: BOOT button (active-low)
SAFE_FILE  = "safe_mode"   # delete this file to resume autostart

# ---- helpers ----
def _is_dir(p):
    try:
        return (os.stat(p)[0] & 0x4000) != 0  # DIR flag on MicroPython
    except:
        return False

def _fmt_size(sz):
    try:
        return "{} B".format(sz)
    except:
        return "-"

def print_tree(root="/", max_depth=2, _level=0):
    try:
        names = sorted(os.listdir(root))
    except Exception as e:
        print(" <cannot list {}: {}>".format(root, e))
        return
    pref = "  " * _level
    for name in names:
        p = (root.rstrip("/") + "/" + name) if root != "/" else "/" + name
        if _is_dir(p):
            print("{}{}/".format(pref, name))
            if _level + 1 < max_depth:
                print_tree(p, max_depth, _level + 1)
        else:
            try:
                sz = os.stat(p)[6]
            except:
                sz = "-"
            print("{}{}  ({})".format(pref, name, _fmt_size(sz)))

def boot_safe_pin():
    try:
        pin = machine.Pin(SAFE_PIN, machine.Pin.IN, machine.Pin.PULL_UP)
        return pin.value() == 0   # held low -> stay in REPL
    except Exception:
        return False

def boot_safe_file():
    try:
        return SAFE_FILE in os.listdir()
    except Exception:
        return False

def create_safe_file():
    try:
        with open(SAFE_FILE, "w") as f:
            f.write("latched at {}\n".format(time.time()))
    except Exception as e:
        print("WARN: couldn't create {}: {}".format(SAFE_FILE, e))

def countdown(seconds):
    print("REPL is free for {}s. Press Ctrl-C to cancel autostart...".format(seconds))
    try:
        for s in range(seconds, 0, -1):
            print(" Autostart in {}s".format(s))
            time.sleep(1)
        return False  # not cancelled
    except KeyboardInterrupt:
        return True   # cancelled

# --- optional noise reducers ---
# try:
#     import esp; esp.osdebug(None)
# except Exception: pass
# try:
#     import webrepl; webrepl.start()
# except Exception as e: print("WebREPL start failed:", e)

# ---- safe modes ----
safe = False
if boot_safe_pin():
    print("SAFE MODE: BOOT held -> staying in REPL.")
    safe = True
elif boot_safe_file():
    print("SAFE MODE: 'safe_mode' present -> staying in REPL (delete to resume).")
    safe = True

if not safe:
    # normal path
    print("Device files (root):")
    print_tree("/", max_depth=2)
    cancelled = countdown(WAIT_S)
    if cancelled:
        create_safe_file()
        print("Autostart cancelled; 'safe_mode' created. Delete it to resume autostart.")
    else:
        try:
            import main
            if hasattr(main, "start"):
                main.start()
        except KeyboardInterrupt:
            print("Startup interrupted; staying in REPL.")
        except Exception as e:
            print("Boot error:", e)
# IMPORTANT: no raise/exit here; just let boot.py finish and show the >>> prompt
