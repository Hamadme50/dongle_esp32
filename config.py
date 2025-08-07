# config.py — user settings for main.py

# Wi‑Fi connect timeout (ms)
WIFI_CONNECT_TIMEOUT_MS = 15000

# SoftAP radio channel (1..11)
AP_CHANNEL = 1

# Shown in telemetry
SW_VERSION = "8.9"

# GPIOs (change if your board differs)
LED_PIN = 8        # on many ESP32‑C3 devkits this is the BLUE LED
BOOT_PIN = 9       # BOOT button

# Optional ADC to estimate VCC (set to None to disable)
ADC_VCC_PIN = None  # e.g. 0 if wired with a divider

# ===== MQTT =====
MQTT_HOST = "test.mosquitto.org"  # <-- CHANGE to your broker
MQTT_PORT = 1883
MQTT_USER = None                  # or "user"
MQTT_PASS = None                  # or "pass"
MQTT_TOPIC = b"/Realtime"
MQTT_KEEPALIVE = 60
