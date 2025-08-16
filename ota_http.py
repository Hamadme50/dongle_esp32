# ota_http.py  — simple HTTP OTA for MicroPython (ESP32/ESP32-C3)
from esp32 import Partition
import socket, machine

def _parse_url(url):
    if not url.startswith('http://'):
        raise ValueError('Only http:// URLs supported')
    rest = url[7:]
    slash = rest.find('/')
    if slash == -1:
        hostport, path = rest, '/'
    else:
        hostport, path = rest[:slash], rest[slash:]
    if ':' in hostport:
        host, port = hostport.split(':', 1); port = int(port)
    else:
        host, port = hostport, 80
    return host, port, path

def _read_headers(sock):
    data = b''
    while b'\r\n\r\n' not in data:
        chunk = sock.recv(512)
        if not chunk:
            break
        data += chunk
        if len(data) > 8192:
            raise RuntimeError('HTTP header too large')
    head, body = data.split(b'\r\n\r\n', 1)
    lines = head.split(b'\r\n')
    status = lines[0].split()
    if len(status) < 2 or status[1] != b'200':
        raise RuntimeError('HTTP status not 200: %r' % lines[0])
    clen, chunked = None, False
    for line in lines[1:]:
        ll = line.lower()
        if ll.startswith(b'content-length:'):
            clen = int(line.split(b':',1)[1].strip())
        elif ll.startswith(b'transfer-encoding:') and b'chunked' in ll:
            chunked = True
    if chunked:
        raise RuntimeError('Chunked encoding not supported')
    if clen is None:
        raise RuntimeError('Content-Length missing')
    return clen, body

def update(url, log=None, timeout=15, progress_step_bytes=131072):
    """
    HTTP OTA with detailed logging.
    - url: http://host[:port]/path/to/your.bin  (any filename OK)
    - log: optional callable like log(*args); if None, prints with [OTA]
    - timeout: socket timeout in seconds
    - progress_step_bytes: how often to log progress (default ~128 KB)
    """
    # --- logger shim ---
    def _log(*args):
        try:
            if log:
                log(*args)
            else:
                print("[OTA]", *args)
        except Exception:
            # never let logging crash the OTA
            pass

    _log("Start:", url)

    # --- parse URL ---
    try:
        host, port, path = _parse_url(url)
        _log("URL parsed -> host:", host, "port:", port, "path:", path)
    except Exception as e:
        _log("URL parse error:", e)
        raise

    # --- preflight: partitions (fail early if no OTA slot) ---
    try:
        from esp32 import Partition
        cur = Partition(Partition.RUNNING)
        curi = cur.info()
        nxt = cur.get_next_update()      # OSError(2) if no OTA slot
        nxti = nxt.info()
        blk = nxt.ioctl(5, 0)            # typical 4096
        max_blocks = nxt.ioctl(4, 0)     # total blocks in OTA slot
        _log("Partition running:", curi[4], "@0x%X" % curi[2], "size:", curi[3])
        _log("Partition next   :", nxti[4], "@0x%X" % nxti[2], "size:", nxti[3],
             "blk:", blk, "blocks:", max_blocks)
    except OSError as e:
        # Common case: ENOENT (2) -> no OTA slot / wrong partition table
        code = e.args[0] if e.args else e
        if code == 2:
            _log("Error: No OTA slot available (get_next_update failed: ENOENT). "
                 "Flash an OTA partition table (factory + ota_0) first.")
        else:
            _log("Partition OSError:", code)
        raise
    except Exception as e:
        _log("Partition preflight error:", e)
        raise

    # --- DNS + connect ---
    import socket as usocket
    try:
        ai = usocket.getaddrinfo(host, port, 0, usocket.SOCK_STREAM)
        if not ai:
            raise OSError("DNS lookup returned no results")
        addr = ai[0][-1]
        _log("DNS:", addr)
    except Exception as e:
        _log("DNS error:", e)
        raise

    s = None
    try:
        s = usocket.socket()
        try:
            s.settimeout(timeout)
        except Exception:
            pass
        _log("Connecting...")
        s.connect(addr)
        _log("Connected.")

        # --- request ---
        req = 'GET {} HTTP/1.1\r\nHost: {}\r\nConnection: close\r\n\r\n'.format(path, host)
        sent = s.send(req.encode())
        _log("HTTP GET sent:", sent, "bytes")

        # --- headers ---
        try:
            content_len, first_body = _read_headers(s)
            _log("HTTP 200 OK, Content-Length:", content_len, "first-body-bytes:", len(first_body))
        except Exception as e:
            _log("HTTP header parse error:", e)
            raise

        # --- capacity check ---
        need_blocks = (content_len + blk - 1) // blk
        if need_blocks > max_blocks:
            _log("Error: Image too large for OTA slot.",
                 "need_blocks:", need_blocks, "slot_blocks:", max_blocks,
                 "bytes:", content_len)
            raise RuntimeError('Image too large: {} bytes'.format(content_len))

        # --- stream + flash ---
        block_idx = 0
        written = 0
        buf = first_body
        next_progress = progress_step_bytes

        _log("Flashing start: total", content_len, "bytes;",
             "block", blk, "bytes;", "blocks needed:", need_blocks)

        while True:
            # write full blocks from buffer
            while len(buf) >= blk:
                nxt.writeblocks(block_idx, buf[:blk])
                buf = buf[blk:]
                block_idx += 1
                written += blk
                if written >= next_progress:
                    pct = (written * 100) // content_len
                    _log("Progress:", written, "/", content_len, "bytes (", pct, "% )")
                    next_progress += progress_step_bytes

            if written >= content_len:
                break

            # receive more
            chunk = s.recv(4096)
            if not chunk:
                # socket ended early
                break
            buf += chunk

        # close socket before finalization
        try:
            s.close()
        except Exception:
            pass
        s = None

        # final partial block
        remaining = content_len - written
        if remaining > 0:
            if len(buf) < remaining:
                _log("Error: Truncated download. Have", len(buf), "need", remaining)
                raise RuntimeError('Truncated download')
            last = buf[:remaining]
            if remaining < blk:
                last += b'\xff' * (blk - remaining)
            nxt.writeblocks(block_idx, last)
            written += remaining

        if written != content_len:
            _log("Warning: wrote", written, "bytes but expected", content_len)

        _log("Flashing done. Total written:", written, "bytes; blocks:", need_blocks)

        # --- switch + reboot ---
        nxt.set_boot()
        _log("set_boot OK for partition:", nxti[4], "@0x%X" % nxti[2])
        _log("Rebooting...")
        import machine
        machine.reset()

    except Exception as e:
        _log("OTA exception:", e)
        # ensure socket is closed on errors
        try:
            if s:
                s.close()
        except Exception:
            pass
        raise

