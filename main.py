"""
ESP32-C3 MicroPython — Non-Blocking Wi‑Fi Provisioning + MQTT
- AP (hotspot) comes up immediately on Wi‑Fi loss and HTTP works right away
- STA does short background retries while AP stays visible
- On connect, AP stops immediately
- DHCP hostname (STA): "inverterzone"

Routes:
- POST /wifisave  (x-www-form-urlencoded or multipart/form-data) fields: s, p
- GET  /livejson  -> returns current JSON (independent of MQTT publish)
- GET  /          -> returns 200 OK ("OK")

MQTT (no leading slash):
- Realtime/Alive  (retained true)
- Realtime/IP     (retained true)
- Realtime/Data   (retained false)

LED (GPIO 8 active-low):
- AP active & STA not connected -> SOLID ON
- Wi‑Fi OK, MQTT X              -> FAST BLINK (200 ms)
- Wi‑Fi OK, MQTT OK             -> SLOW BLINK (2000 ms)
"""
import gc, os, network, machine, ubinascii, utime, ujson, usocket, micropython
from machine import Pin, Timer
from serial import InverterSerial
from protocol_config import apply_protocols

# ---------------- CONFIG ----------------
MQTT_HOST   = "node.inverterzone.com"
MQTT_PORT   = 1883
MQTT_USER   = "usb"
MQTT_PASS   = "335500usb"
TOPIC_BASE  = b"Realtime"      # NO leading slash
CONTROL_TOPIC = None  # set in start(): Realtime/DeviceControl/<MAC>/Set_Command

SW_VERSION  = "8.9"
DEV_TYPE    = "L"
ANSWER_VAL  = "0"

HOSTNAME    = "inverterzone"   # DHCP hostname for STA

ADC_VCC_PIN = None
LED_PIN     = 8
LED_ACTIVE_HIGH = False        # active-low (0=ON, 1=OFF)
BOOT_PIN    = 9
WIFI_FILE   = "wifi.json"

T0_MS       = 200
STATUS_MS   = 5000
PUBLISH_MS  = 15000
SERIAL_MS   = 1000
LED_FAST_MS = 200
LED_SLOW_MS = 2000

HTTP_DEBUG  = True
MAX_REQ     = 8192

AP_CHANNEL  = 6
AP_MAXCLI   = 4
# ---------------------------------------
inv = None

# ---- Custom command (MQTT-triggered) state ----
_custom_pending = False
_custom_deadline = 0
_custom_key = "ANSWER"
micropython.alloc_emergency_exception_buf(256)
def log(*a): print("[LOG]", *a)
def now(): return utime.ticks_ms()

# --------------- Helpers ----------------
def mac_hex():
    sta = network.WLAN(network.STA_IF); sta.active(True)
    return ubinascii.hexlify(sta.config("mac")).decode().upper()
def ap_ssid(): return "Solar_" + mac_hex()

def url_unquote(s):
    s = s.replace("+"," ")
    out = bytearray()
    b = s.encode() if isinstance(s,str) else s
    i=0
    while i < len(b):
        if b[i]==37 and i+2<len(b):
            try:
                out.append(int(b[i+1:i+3].decode(),16)); i+=3; continue
            except: pass
        out.append(b[i]); i+=1
    return out.decode()

def parse_urlencoded(txt):
    kv={}
    for part in txt.split("&"):
        if "=" in part:
            k,v = part.split("=",1)
            kv[k.lower()] = url_unquote(v)
    return kv

def parse_boundary_from_ctype(ctype):
    i = ctype.find("boundary=")
    if i < 0: return None
    b = ctype[i+9:]
    if ";" in b: b = b.split(";",1)[0]
    b = b.strip().strip('"')
    return b or None

def parse_multipart(body_bytes, ctype):
    b = parse_boundary_from_ctype(ctype)
    if not b: return None
    bnd = ("--" + b).encode()
    kv = {}
    parts = body_bytes.split(bnd)
    for sec in parts:
        if not sec: continue
        if sec.startswith(b"--"): break
        if sec.startswith(b"\r\n"): sec = sec[2:]
        elif sec.startswith(b"\n"): sec = sec[1:]
        idx = sec.find(b"\r\n\r\n"); sep = 4
        if idx < 0:
            idx = sec.find(b"\n\n"); sep = 2
        if idx < 0: continue
        headers = sec[:idx].decode('utf-8','ignore').lower()
        value   = sec[idx+sep:]
        if value.endswith(b"\r\n"): value = value[:-2]
        elif value.endswith(b"\n"): value = value[:-1]
        name = None
        mark = 'name="'
        j = headers.find(mark)
        if j >= 0:
            j2 = headers.find('"', j + len(mark))
            if j2 > j: name = headers[j+len(mark):j2]
        if name:
            kv[name] = value.decode('utf-8','ignore')
    if ("s" in kv) or ("p" in kv):
        return kv
    return None

def heap_used_pct():
    gc.collect(); a=gc.mem_alloc(); f=gc.mem_free(); t=a+f
    return int(a*100/t) if t else 0

def read_vcc():
    if ADC_VCC_PIN is None: return None
    try:
        adc = machine.ADC(machine.Pin(ADC_VCC_PIN))
        raw = adc.read_u16()
        return round((raw/65535.0)*3.3*2.0,3)
    except: return None

# --------------- Wi-Fi creds --------------
def wifi_load():
    try:
        with open(WIFI_FILE) as f: d=ujson.loads(f.read())
        return d.get("s"), d.get("p") or ""
    except: return None, None

def wifi_save(ssid,pwd):
    try:
        with open(WIFI_FILE,"w") as f: f.write(ujson.dumps({"s":ssid,"p":pwd or ""}))
        log("Saved Wi-Fi:", ssid); return True
    except Exception as e:
        log("Save Wi-Fi error:", e); return False

def wifi_clear():
    try:
        if WIFI_FILE in os.listdir():
            os.remove(WIFI_FILE); log("Wi-Fi creds cleared"); return True
    except Exception as e:
        log("Clear Wi-Fi error:", e)
    return False

# --------------- Hardware -----------------
led = Pin(LED_PIN, Pin.OUT, value=(1 if not LED_ACTIVE_HIGH else 0))
def _led_write(on):
    if LED_ACTIVE_HIGH: led.value(1 if on else 0)
    else:               led.value(0 if on else 1)

_boot_flag=False
def _boot_irq(_):
    global _boot_flag; _boot_flag=True
Pin(BOOT_PIN, Pin.IN, Pin.PULL_UP).irq(trigger=Pin.IRQ_FALLING, handler=_boot_irq)

_led_t = 0
_led_mode = "off"
_led_period = LED_SLOW_MS
_led_on = False

def _led_set(mode):
    global _led_mode, _led_period, _led_t, _led_on
    if mode == _led_mode: return
    _led_mode = mode; _led_t = now()
    if mode == "ap":
        _led_on = True;  _led_write(True)
    elif mode == "wifi_fast":
        _led_period = LED_FAST_MS; _led_on=False; _led_write(False)
    elif mode == "mqtt_slow":
        _led_period = LED_SLOW_MS; _led_on=False; _led_write(False)
    else:
        _led_on = False; _led_write(False)

# --------------- Net / MQTT state ----------
sta = network.WLAN(network.STA_IF)
ap  = network.WLAN(network.AP_IF)

wifi_state = {
    "mode": "idle",        # "idle"|"connecting"|"connecting_ap"|"connected"|"ap"|"failed"
    "deadline": 0,
    "rssi": None,
    "retry_at": 0,
}
mqtt_state = {"cli":None,"ok":False,"need":False,"retry_at":0,"cid":mac_hex().encode(),"sub":False}

_last_ssid = None
_last_pwd  = None

# --------------- Hostname helper -----------
def set_sta_hostname(name=HOSTNAME):
    ok=False
    try:
        sta.config(dhcp_hostname=name); ok=True
    except Exception:
        pass
    try:
        network.hostname(name); ok=True
    except Exception:
        pass
    try:
        sta.config(hostname=name); ok=True
    except Exception:
        pass
    log("STA hostname set" + (": "+name if ok else " (best-effort failed)"))
    return ok

# --------------- HTTP server ---------------
_http_srv=None; _http_cli=None; _rx=b""; _on=False; _closed=False
_sent100=False; _head_parsed=False; _method=""; _path=""; _ctype=""; _clen=-1; _chunked=False; _body=b""

def http_start():
    global _http_srv,_on
    if _on: return
    try:
        s=usocket.socket(usocket.AF_INET, usocket.SOCK_STREAM)
        s.setsockopt(usocket.SOL_SOCKET, usocket.SO_REUSEADDR,1)
        s.bind(("0.0.0.0",80)); s.listen(4); s.setblocking(False)   # backlog 4
        _http_srv=s; _on=True; log("HTTP :80 -> GET / (OK), GET /livejson (JSON), POST /wifisave (form)")
    except Exception as e:
        log("HTTP start error:", e); http_stop()

def http_stop():
    global _http_srv,_http_cli,_rx,_on,_closed,_sent100,_head_parsed,_method,_path,_ctype,_clen,_chunked,_body
    try:
        if _http_cli: _http_cli.close()
    except: pass
    try:
        if _http_srv: _http_srv.close()
    except: pass
    _http_srv=None; _http_cli=None; _rx=b""; _closed=False; _sent100=False
    _head_parsed=False; _method=""; _path=""; _ctype=""; _clen=-1; _chunked=False; _body=b""
    _on=False
    log("HTTP stopped")

def http_restart():
    # Fully restart server to re-bind after interface/IP changes
    http_stop(); http_start()

def _send(conn, status="200 OK", body="OK", ctype="text/plain"):
    try: conn.send("HTTP/1.1 %s\r\nConnection: close\r\nContent-Type: %s\r\n\r\n%s"%(status,ctype,body))
    except: pass

def _send_json(conn, status="200 OK", body="{}"):
    try: conn.send("HTTP/1.1 %s\r\nConnection: close\r\nContent-Type: application/json\r\n\r\n%s"%(status, body))
    except: pass

def _find_header_end(data):
    i = data.find(b"\r\n\r\n")
    if i>=0: return i,4
    j = data.find(b"\n\n")
    if j>=0: return j,2
    return -1,0

def _normalize_path(path):
    if path.startswith("http://") or path.startswith("https://"):
        k = path.find("://"); s = path.find("/", k+3); path = path[s:] if s>=0 else "/"
    if "?" in path: path = path.split("?",1)[0]
    return path

def _parse_head(head_txt):
    global _method,_path,_ctype,_clen,_chunked
    lines = head_txt.split("\n")
    line0 = lines[0].strip("\r")
    parts = line0.split(" ")
    _method = (parts[0].upper() if len(parts)>0 else "")
    _path   = _normalize_path(parts[1] if len(parts)>1 else "/")
    _ctype=""; _clen=-1; _chunked=False
    for raw in lines[1:]:
        h = raw.strip("\r"); hl = h.lower()
        if hl.startswith("content-type:"): _ctype = h.split(":",1)[1].strip().lower()
        elif hl.startswith("content-length:"):
            try: _clen=int(h.split(":",1)[1].strip())
            except: _clen=-1
        elif hl.startswith("transfer-encoding:"):
            if "chunked" in hl: _chunked=True

def _maybe_send_100():
    global _sent100
    if _sent100 or not _http_cli: return
    try: _http_cli.send(b"HTTP/1.1 100 Continue\r\n\r\n"); _sent100=True
    except: pass

def _decode_chunked(buf):
    out = bytearray(); pos=0; L=len(buf)
    while True:
        nl = buf.find(b"\r\n", pos); sep=2
        if nl < 0:
            nl = buf.find(b"\n", pos); sep=1
            if nl < 0: return bytes(out), pos, False
        size_line = buf[pos:nl].strip()
        semi = size_line.find(b";")
        if semi >= 0: size_line = size_line[:semi]
        try: sz = int(size_line,16)
        except: return bytes(out), pos, False
        pos = nl + sep
        if sz == 0:
            if L - pos >= 2 and buf[pos:pos+2] in (b"\r\n", b"\n\r"): pos += 2
            return bytes(out), pos, True
        if L - pos < sz + 2: return bytes(out), pos - (len(size_line)+sep), False
        out += buf[pos:pos+sz]; pos += sz
        if buf[pos:pos+2] == b"\r\n": pos += 2
        elif buf[pos:pos+1] == b"\n": pos += 1
        else: return bytes(out), pos, False

def http_step():
    global _http_cli,_rx,_closed,_head_parsed,_sent100,_body
    if not _on or not _http_srv: return

    if not _http_cli:
        try:
            conn, addr = _http_srv.accept()
            conn.setsockopt(usocket.IPPROTO_TCP, usocket.TCP_NODELAY, 1)
            conn.setblocking(False)
            _http_cli = conn
            _rx=b""; _closed=False; _head_parsed=False; _sent100=False; _body=b""
        except:
            return

    try:
        chunk = _http_cli.recv(1024)
        if chunk:
            _rx += chunk
            if len(_rx) > MAX_REQ:
                _send(_http_cli, "413 Payload Too Large", "Too large")
                try: _http_cli.close()
                except: pass
                _http_cli=None; _rx=b""; _closed=False; return
        else:
            _closed=True
    except:
        pass

    if not _head_parsed:
        i, sep = _find_header_end(_rx)
        if i < 0: return
        head_txt = _rx[:i].decode('utf-8','ignore')
        _parse_head(head_txt)
        _rx = _rx[i+sep:]
        _head_parsed = True
        if "100-continue" in head_txt.lower(): _maybe_send_100()

        if HTTP_DEBUG:
            log("HTTP", _method, _path, "ctype=", (_ctype or "-"), "len=", _clen, "chunked=", _chunked)

        if _path in ("/","/index.html") and _method in ("GET","HEAD"):
            if HTTP_DEBUG: log("HTTP -> 200 OK /")
            _send(_http_cli, "200 OK", "OK")
            try: _http_cli.close()
            except: pass
            _http_cli=None; _rx=b""
            return

        if _path == "/livejson" and _method in ("GET","HEAD"):
            js = build_json()
            if HTTP_DEBUG: log("HTTP -> 200 OK /livejson")
            _send_json(_http_cli, "200 OK", js if _method=="GET" else "")
            try: _http_cli.close()
            except: pass
            _http_cli=None; _rx=b""
            return

        if _path not in ("/wifisave","/wifisave/"):
            _send(_http_cli, "404 Not Found", "Not Found"); _http_cli.close(); _http_cli=None; _rx=b""; return
        if _method != "POST":
            _send(_http_cli, "405 Method Not Allowed", "POST only"); _http_cli.close(); _http_cli=None; _rx=b""; return
        ct = _ctype.split(";",1)[0].strip()
        if ct not in ("application/x-www-form-urlencoded","multipart/form-data"):
            _send(_http_cli, "415 Unsupported Media Type", "Use x-www-form-urlencoded or multipart/form-data"); _http_cli.close(); _http_cli=None; _rx=b""; return

    if _head_parsed:
        if _chunked:
            dec, consumed, complete = _decode_chunked(_rx)
            if complete:
                _body = dec; _rx = _rx[consumed:]
            else:
                return
        else:
            if _clen >= 0:
                if len(_rx) < _clen: return
                _body = _rx[:_clen]; _rx = _rx[_clen:]
            else:
                if not _closed: return
                _body = _rx; _rx = b""

        kv = None
        ct_full = _ctype
        ct_base = ct_full.split(";",1)[0].strip() if ct_full else ""
        if ct_base == "application/x-www-form-urlencoded":
            try: kv = parse_urlencoded(_body.decode('utf-8','ignore'))
            except: kv = None
        elif ct_base == "multipart/form-data":
            kv = parse_multipart(_body, ct_full)

        s = kv.get("s") if kv else None
        p = kv.get("p","") if kv else ""
        if s is not None:
            if HTTP_DEBUG: log("HTTP -> 200 OK /wifisave (s,p)")
            _send(_http_cli, "200 OK", "OK")
            try: _http_cli.close()
            except: pass
            _http_cli=None
            if wifi_save(s,p): ensure_sta_after_save(s,p)
        else:
            _send(_http_cli, "400 Bad Request", "Missing s/p")
            try: _http_cli.close()
            except: pass
            _http_cli=None

# --------------- AP / STA -------------------
def start_ap(soft=False):
    """Start AP. If soft=True, keep FSM mode (used during initial connect)."""
    try:
        ap.active(True)
        try:
            ap.config(essid=ap_ssid(), authmode=network.AUTH_OPEN, channel=AP_CHANNEL, hidden=False)
            try: ap.config(max_clients=AP_MAXCLI)
            except: pass
        except:
            ap.config(essid=ap_ssid(), authmode=network.AUTH_OPEN)
        try: ap.ifconfig(("192.168.4.1","255.255.255.0","192.168.4.1","8.8.8.8"))
        except: pass
        if not soft: wifi_state["mode"]="ap"
        # IMPORTANT: re-bind HTTP so routes work immediately on AP
        http_restart()
        _led_set("ap")
        log("AP:", ap.config("essid"), "IP:", ap.ifconfig()[0])
    except Exception as e:
        log("AP start error:", e)
        if not soft: wifi_state["mode"]="failed"

def ap_watchdog():
    # Keep AP up whenever we're not connected
    if not wifi_ok() and not ap.active():
        start_ap(soft=(wifi_state["mode"]=="connecting"))

def _prep_sta(allow_reset):
    if allow_reset and not ap.active():
        try:
            sta.active(False)
            utime.sleep_ms(50)
        except: pass
    sta.active(True)
    set_sta_hostname(HOSTNAME)
    try:
        if sta.isconnected(): sta.disconnect()
    except: pass

def start_sta(ssid,pwd):
    try: ap.active(False)
    except: pass
    try:
        if not ssid: 
            log("STA start skipped (no SSID)"); return
        _prep_sta(True)
        sta.connect(ssid, pwd or "")
        wifi_state["mode"]="connecting"
        wifi_state["deadline"]=utime.ticks_add(now(),20000)
        log("STA connecting ->", ssid)
    except Exception as e:
        log("STA start error:", e); wifi_state["mode"]="failed"

def start_sta_bg(ssid,pwd):
    if wifi_state["mode"] in ("connecting","connecting_ap"):
        return
    try:
        if not ssid: return
        _prep_sta(False)                   # no reset during AP
        try: sta.disconnect()
        except: pass
        sta.connect(ssid, pwd or "")
        wifi_state["mode"]="connecting_ap"
        wifi_state["deadline"]=utime.ticks_add(now(),5000)  # short window
        log("STA (bg) connecting ->", ssid)
    except Exception as e:
        log("STA(bg) start error:", e)
        wifi_state["retry_at"] = utime.ticks_add(now(), 8000)
        wifi_state["mode"]="ap"

def ensure_sta_after_save(ssid, pwd):
    global _last_ssid, _last_pwd
    _last_ssid, _last_pwd = ssid, (pwd or "")
    start_sta_bg(_last_ssid, _last_pwd)

def wifi_ok():
    try: return sta.isconnected()
    except: return False

def wifi_ip():
    try:
        if sta.isconnected(): return sta.ifconfig()[0]
        if ap.active(): return ap.ifconfig()[0]
    except: pass
    return "--"

# --------------- MQTT -----------------------
def mqtt_make():
    from umqtt.simple import MQTTClient
    return MQTTClient(mqtt_state["cid"], server=MQTT_HOST, port=MQTT_PORT, user=MQTT_USER, password=MQTT_PASS, keepalive=30)

def mqtt_step():
    if not wifi_ok():
        if mqtt_state["ok"]:
            try: mqtt_state["cli"].disconnect()
            except: pass
        mqtt_state["cli"]=None; mqtt_state["ok"]=False; mqtt_state["sub"]=False; return
    if not mqtt_state["ok"] and (mqtt_state["need"] or now()>=mqtt_state["retry_at"]):
        try:
            if mqtt_state["cli"] is None: mqtt_state["cli"]=mqtt_make()
            try:
                ip = usocket.getaddrinfo(MQTT_HOST, MQTT_PORT)[0][-1][0]
                log("MQTT resolving:", MQTT_HOST, "->", ip)
            except:
                pass
            mqtt_state["cli"].connect()
            mqtt_state["ok"]=True; mqtt_state["need"]=False
            # attach callback & subscribe control topic
            try:
                mqtt_state["cli"].set_callback(mqtt_on_msg)
                if CONTROL_TOPIC and not mqtt_state.get("sub"):
                    mqtt_state["cli"].subscribe(CONTROL_TOPIC)
                    mqtt_state["sub"]=True
                    log("MQTT subscribed:", CONTROL_TOPIC.decode() if isinstance(CONTROL_TOPIC,(bytes,bytearray)) else str(CONTROL_TOPIC))
            except Exception as e:
                log("MQTT subscribe error:", e)
            log("MQTT connected:", MQTT_HOST)
        except:
            mqtt_state["retry_at"]=utime.ticks_add(now(),5000); mqtt_state["ok"]=False; return
    if mqtt_state["ok"]:
        try:
            mqtt_state["cli"].check_msg()
        except:
            mqtt_state["ok"]=False; mqtt_state["sub"]=False; mqtt_state["retry_at"]=utime.ticks_add(now(),3000)

def mqtt_pub(topic, payload, retain=False, qos=0):
    if not mqtt_state["ok"]: return False
    try:
        mqtt_state["cli"].publish(topic, payload, retain, qos)
        if HTTP_DEBUG:
            try:
                if isinstance(topic, bytes): t = topic.decode()
                else: t = str(topic)
            except:
                t = "<bin>"
            log("MQTT PUB ->", t)
        return True
    except:
        mqtt_state["ok"]=False; return False

# --------------- MQTT command callback ---------------
def mqtt_on_msg(topic, msg):
    # Control topic: Realtime/DeviceControl/<MAC>/Set_Command
    try:
        t = topic.decode() if isinstance(topic, (bytes, bytearray)) else str(topic)
        m = msg.decode('utf-8','ignore') if isinstance(msg, (bytes, bytearray)) else str(msg)
        m = m.strip()
    except Exception:
        return

    try:
        ct = CONTROL_TOPIC.decode() if isinstance(CONTROL_TOPIC, (bytes, bytearray)) else str(CONTROL_TOPIC or '')
    except Exception:
        ct = ''
    if not ct or t != ct or not m:
        return

    global _custom_pending, _custom_deadline
    if _custom_pending:
        log("Custom busy; dropped:", m)
        return
    try:
        inv.send_custom(m, name=_custom_key)
        _custom_pending = True
        _custom_deadline = utime.ticks_add(now(), 5000)  # 5s timeout
        log("Custom queued:", m)
    except Exception as e:
        log("send_custom error:", e)


# --------------- JSON payload ---------------
def build_json():
    esp = {
        "Device_name": mac_hex(),
        "Wifi_RSSI": wifi_state["rssi"] if isinstance(wifi_state["rssi"], int) else -999,
        "sw_version": SW_VERSION,
        "Free_Heap": gc.mem_free(),
        "json_memory_usage": 0,
        "HEAP_Fragmentation": heap_used_pct(),
        "type": DEV_TYPE,
        "answer": ANSWER_VAL,
    }
    # Populate from inverter snapshots if available; keep existing fallbacks
    try:
        dev = inv.devicedata if (inv and isinstance(inv.devicedata, dict)) else {"QPIRI": ""}
    except Exception:
        dev = {"QPIRI": ""}
    try:
        live = inv.livedata if (inv and isinstance(inv.livedata, dict)) else {"QPIGS": ""}
    except Exception:
        live = {"QPIGS": ""}

    js_esp  = ujson.dumps(esp)
    js_dev  = ujson.dumps(dev)
    js_live = ujson.dumps(live)
    top     = '{"EspData":' + js_esp + ',"DeviceData":' + js_dev + ',"LiveData":' + js_live + '}'

    esp["json_memory_usage"] = len(top)
    js_esp  = ujson.dumps(esp)
    top     = '{"EspData":' + js_esp + ',"DeviceData":' + js_dev + ',"LiveData":' + js_live + '}'
    return top


# --------------- Scheduler ------------------
_stat_t=0
_pub_due=0
_busy=0

_inv_last=0
def _pub_cb(_):
    global _pub_due; _pub_due=1

def _scheduled(_):
    global _busy,_stat_t,_pub_due,_boot_flag,_led_t,_led_on,_last_ssid,_last_pwd,_inv_last

    # BOOT short press -> clear creds + AP
    if _boot_flag:
        _boot_flag=False
        if wifi_clear():
            _last_ssid, _last_pwd = None, ""
            try:
                if sta.isconnected(): sta.disconnect()
            except: pass
            start_ap(soft=False)

    # HTTP + Wi-Fi FSM
    http_step()
    ap_watchdog()

    try: wifi_state["rssi"]=sta.status("rssi") if sta.isconnected() else None
    except: wifi_state["rssi"]=None

    if wifi_state["mode"]=="connecting":
        if sta.isconnected():
            wifi_state["mode"]="connected"
            log("Wi-Fi connected. IP:", wifi_ip())
            mqtt_state["need"]=True
            if ap.active():
                try: ap.active(False); log("AP stopped (STA connected)")
                except: pass
        elif utime.ticks_diff(wifi_state["deadline"], now())<=0:
            log("Wi-Fi connect timeout")
            wifi_state["mode"]="ap"
            start_ap(soft=False)
            wifi_state["retry_at"] = utime.ticks_add(now(), 8000)

    elif wifi_state["mode"]=="connecting_ap":
        if sta.isconnected():
            wifi_state["mode"]="connected"
            log("Wi-Fi connected. IP:", wifi_ip())
            mqtt_state["need"]=True
            if ap.active():
                try: ap.active(False); log("AP stopped (STA connected)")
                except: pass
        elif utime.ticks_diff(wifi_state["deadline"], now())<=0:
            try: sta.disconnect()
            except: pass
            wifi_state["mode"]="ap"
            wifi_state["retry_at"] = utime.ticks_add(now(), 9000)

    elif wifi_state["mode"]=="connected" and (not sta.isconnected()):
        log("Wi-Fi dropped -> AP on, retry bg")
        start_ap(soft=False)
        wifi_state["mode"]="ap"
        wifi_state["retry_at"] = utime.ticks_add(now(), 2000)

    # Background retry while AP is ON (but NOT while connecting)
    if (wifi_state["mode"] in ("ap","failed")) and (not sta.isconnected()) and _last_ssid:
        if utime.ticks_diff(now(), wifi_state.get("retry_at", 0)) >= 0:
            start_sta_bg(_last_ssid, _last_pwd)
            wifi_state["retry_at"] = utime.ticks_add(now(), 10000)

    # LED mode
    if ap.active() and not wifi_ok():
        _led_set("ap")
    elif wifi_ok() and mqtt_state["ok"]:
        _led_set("mqtt_slow")
    elif wifi_ok():
        _led_set("wifi_fast")
    else:
        _led_set("off")

    # Drive LED
    if _led_mode in ("wifi_fast","mqtt_slow"):
        if utime.ticks_diff(now(), _led_t) >= _led_period:
            _led_on = not _led_on
            _led_write(_led_on); _led_t = now()
    elif _led_mode == "ap":
        if not _led_on:
            _led_on = True; _led_write(True)
    else:
        if _led_on:
            _led_on = False; _led_write(False)


    # Inverter step every SERIAL_MS (6s)
    try:
        if inv and utime.ticks_diff(now(), _inv_last) >= SERIAL_MS:
            inv.step()
            _inv_last = now()
    except Exception as _e:
        pass
    # If a custom command was queued, publish its answer once
    global ANSWER_VAL, _custom_pending, _custom_deadline
    
    if _custom_pending:
        ans = None
        try:
            if inv and isinstance(inv.livedata, dict):
                ans = getattr(inv, '_custom_answer_cache', None)
                if not ans:
                    ans = inv.livedata.get(_custom_key)
        except:
            ans = None

        if ans:
            ANSWER_VAL = str(ans)
            if mqtt_state["ok"]:
                # defensive purge: ensure custom/alias keys are gone before publish
                try:
                    if inv and isinstance(inv.livedata, dict):
                        inv.livedata.pop(_custom_key, None)
                        for _k in list(inv.livedata.keys()):
                            if (len(_k)==5 and _k.startswith("POP") and _k[3:].isdigit()):
                                inv.livedata.pop(_k, None)
                except:
                    pass
                mqtt_pub(TOPIC_BASE + b"/Data", build_json().encode(), False, 0)
            ANSWER_VAL = "0"
            try:
                inv._custom_answer_cache = None
            except:
                pass
            _custom_pending = False
        elif utime.ticks_diff(now(), _custom_deadline) >= 0:
            _custom_pending = False

# MQTT
    mqtt_step()
    
    # Status
    if utime.ticks_diff(now(), _stat_t) >= STATUS_MS:
        print("WIFI:{:4}  MQTT:{:4}  RSSI:{}  IP:{}".format(
            "OK" if wifi_ok() else "FAIL",
            "OK" if mqtt_state["ok"] else "FAIL",
            wifi_state["rssi"] if wifi_state["rssi"] is not None else "--",
            wifi_ip()))
        _stat_t = now()

    # Publish
    if _pub_due and mqtt_state["ok"]:
        _pub_due=0
        mqtt_pub(TOPIC_BASE + b"/Alive", b"true", True, 0)
        mqtt_pub(TOPIC_BASE + b"/IP", wifi_ip().encode(), True, 0)
        if mqtt_pub(TOPIC_BASE + b"/Data", build_json().encode(), False, 0):
            log("Published Realtime/*")

    _busy=0

def _tick_cb(_):
    global _busy
    if _busy: return
    _busy=1
    try: micropython.schedule(_scheduled, 0)
    except: _busy=0

# --------------- Start ----------------------
def start():
    global _last_ssid, _last_pwd, inv
    global CONTROL_TOPIC
    log("Boot... MAC:", mac_hex())

    s,p = wifi_load()
    _last_ssid, _last_pwd = s, (p or "")


    if s: start_sta(s,p)
    else: start_ap(soft=False)

    http_start()         # HTTP in both modes

    Timer(0).init(period=T0_MS, mode=Timer.PERIODIC, callback=_tick_cb)
    Timer(1).init(period=PUBLISH_MS, mode=Timer.PERIODIC, callback=_pub_cb)
    log("Timers: T0={}ms (work), T1={}ms (publish)".format(T0_MS, PUBLISH_MS))
    CONTROL_TOPIC = TOPIC_BASE + b"/DeviceControl/" + mac_hex().encode() + b"/Set_Command"
    log("Control topic:", CONTROL_TOPIC.decode())
    log("AP SSID (if AP):", ap_ssid())
    
    
    inv = InverterSerial(uart_id=1, rx_pin=20, tx_pin=21, timeout_ms=1500)
    inv.debug = False 
    inv.start()
    apply_protocols(inv, keep_command_keys=True)
    # BOOT WARMUP: fetch static+dynamic once (up to ~3s)
    try:
        t_dead = utime.ticks_add(now(), 3000)
        seen_static = False
        seen_dynamic = False
        # ensure we start with static
        inv.request_static = True
        inv.request_counter = 0
        while utime.ticks_diff(t_dead, now()) > 0:
            if inv.step():
                # heuristic checks
                try:
                    seen_static = seen_static or bool(inv.devicedata and len(inv.devicedata))
                    seen_dynamic = seen_dynamic or bool(inv.livedata and len(inv.livedata))
                except:
                    pass
                if seen_static and seen_dynamic:
                    break
            utime.sleep_ms(10)
    except Exception as _e:
        log("Warmup skipped:", _e)

    
    #print("QPI ->", inv.protocol)
    #print("QPI ->", inv.request_data("QPI"))
    #print("QPIGS ->", inv.request_data("QPIGS"))
    
    # for step
    # Use a separate timer ID to avoid clobbering Timer(1) used for publish
    # inv.step() scheduled from _scheduled() every SERIAL_MS (no extra Timer needed)
log("Ready. REPL free.")

if __name__ == "__main__":
    try: start()
    except Exception as e: log("Fatal:", e)
