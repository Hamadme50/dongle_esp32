# serial_named_proto_skip_nobuffer.py
# ESP32-C3 MicroPython RS232 (PI protocol) helper
# Features:
#  • Named sequences: define (name, command) pairs for JSON keys
#  • Protocol-aware sequences: PI30 / PI18 / DEFAULT
#  • Skip errors: if a command returns NAK/ERCRC, don't store it; continue with others
#  • NO double-buffering: writes go straight into devicedata/livedata as replies arrive
#
import utime, machine

def now():
    return utime.ticks_ms()

def _norm_proto_key(k):
    if not k:
        return 'DEFAULT'
    k = str(k).upper()
    if k in ('DEFAULT','OTHER','ELSE','*'):
        return 'DEFAULT'
    return k

class InverterSerial:
    # ---- public config/state ----
    protocol = 'NoD'
    delimiter = ' '
    start_char = '('
    connection = False
    request_static = True
    request_counter = 0
    delay_ms = 150
    _last_at = 0
    debug = False

    # Live snapshots (direct write, no staging)
    devicedata = None
    livedata   = None
    last_live_at = 0   # ticks_ms when any dynamic field was last updated

    # Active sequences (used by step). item: 'CMD' or ('name','CMD')
    static_seq  = ('QPIRI','QMN','QVFW','QFLAG','QPI','QBEQI')
    dynamic_seq = ('QPIGS','QPIGS2','QMOD','Q1','QEX','QPIWS','PVPOWER')

    # Per-protocol mapping
    sequences_by_protocol = {
        'PI30': {
            'static':  ('QPIRI','QMN','QVFW','QFLAG','QPI','QBEQI'),
            'dynamic': ('QPIGS','QPIGS2','QMOD','Q1','QEX','QPIWS','PVPOWER'),
        },
        'PI18': {
            'static':  ('QPIRI','QMN','QVFW','QFLAG','QPI','QBEQI'),   # adjust if your PI18 uses ^P... set
            'dynamic': ('QPIGS','QPIGS2','QMOD','Q1','QEX','QPIWS','PVPOWER'),
        },
        'DEFAULT': {
            'static':  ('QPIRI','QMN','QVFW','QFLAG','QPI','QBEQI'),
            'dynamic': ('QPIGS','QPIGS2','QMOD','Q1','QEX','QPIWS','PVPOWER'),
        }
    }

    # Also store under the raw command key when alias name != command
    keep_command_keys = True

    # ---- internal ----
    _uart = None
    _uart_id = None
    _rx_pin = None
    _tx_pin = None
    _baud = 2400
    _timeout = 1200

    _custom_cmd = None
    _custom_name = None
    _conn_ok = 0
    _conn_fail = 0

    def __init__(self, uart_id, rx_pin, tx_pin, baud=2400, timeout_ms=1200):
        if rx_pin is None or tx_pin is None:
            raise ValueError("rx_pin and tx_pin are required")
        self._uart_id = int(uart_id)
        self._rx_pin = int(rx_pin)
        self._tx_pin = int(tx_pin)
        self._baud = int(baud)
        self._timeout = max(int(timeout_ms), 600)
        self.devicedata = {}
        self.livedata = {}
        self.reset_state()

    # ---------- lifecycle ----------
    def start(self):
        try:
            machine.Pin(self._rx_pin, machine.Pin.IN, machine.Pin.PULL_UP)
        except Exception:
            pass
        self._uart = machine.UART(self._uart_id,
                                  baudrate=self._baud, bits=8, parity=None, stop=1,
                                  tx=self._tx_pin, rx=self._rx_pin,
                                  timeout=self._timeout, timeout_char=2)
        utime.sleep_ms(30)
        try:
            if self._uart.any():
                self._uart.read()
        except Exception:
            pass
        self.auto_detect()
        self._apply_protocol_sequences()
        self._last_at = now()

    def stop(self):
        if self._uart:
            try:
                self._uart.deinit()
            except Exception:
                pass
        self._uart = None

    def reset_state(self):
        self.connection = False
        self.request_static = True
        self.request_counter = 0
        self._last_at = now()

    # ---------- CRC helpers ----------
    @staticmethod
    def _crc16_ccitt_raw(data_bytes):
        crc = 0x0000
        table = (
            0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
            0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF
        )
        for b in data_bytes:
            da = ((crc >> 8) & 0xFF) >> 4
            crc = ((crc << 4) & 0xFFFF) ^ table[da ^ (b >> 4)]
            da = ((crc >> 8) & 0xFF) >> 4
            crc = ((crc << 4) & 0xFFFF) ^ table[da ^ (b & 0x0F)]
        return crc & 0xFFFF

    @staticmethod
    def _calibrate_crc_byte(x):
        if x in (0x28, 0x0D, 0x0A):
            return (x + 1) & 0xFF
        return x & 0xFF

    def _compute_tx_crc_bytes(self, payload_bytes):
        raw = self._crc16_ccitt_raw(payload_bytes)
        hi = self._calibrate_crc_byte((raw >> 8) & 0xFF)
        lo = self._calibrate_crc_byte(raw & 0xFF)
        return hi, lo

    def _verify_rx_crc(self, payload_bytes, recv_hi, recv_lo):
        raw = self._crc16_ccitt_raw(payload_bytes)
        hi = self._calibrate_crc_byte((raw >> 8) & 0xFF)
        lo = self._calibrate_crc_byte(raw & 0xFF)
        if (hi == 0 and lo == 0) or (recv_hi == 0 and recv_lo == 0):
            return False
        return (hi == recv_hi) and (lo == recv_lo)

    @staticmethod
    def _chk_sum(data_bytes):
        s = 0
        for b in data_bytes:
            s = (s + (b & 0xFF)) & 0xFF
        return s

    # ---------- IO helpers ----------
    def _write_cmd(self, cmd_str):
        if not self._uart:
            return False
        cbytes = cmd_str.encode('ascii')
        hi, lo = self._compute_tx_crc_bytes(cbytes)
        pkt = cbytes + bytes([hi, lo, 0x0D])
        if self.debug:
            print('[TX]', cmd_str, '->', ' '.join('{:02X}'.format(b) for b in pkt))
        self._uart.write(pkt)
        utime.sleep_ms(30)
        return True

    def _read_until_cr(self, timeout_ms=None):
        if timeout_ms is None:
            timeout_ms = self._timeout
        buf = bytearray()
        t0 = now()
        while utime.ticks_diff(now(), t0) < timeout_ms:
            if self._uart.any():
                ch = self._uart.read(1)
                if not ch:
                    continue
                if ch == b'\r':
                    break
                buf.extend(ch)
            else:
                utime.sleep_ms(2)
        if self.debug and buf:
            print('[RX]', ' '.join('{:02X}'.format(b) for b in buf))
        return bytes(buf)

    def _strip_start(self, payload):
        if payload.startswith(b'('):
            return payload[1:]
        if payload.startswith(b'^D') and len(payload) >= 5:
            return payload[5:]
        return payload

    @staticmethod
    def _bytes_to_clean_str(b):
        return b.decode('ascii', 'ignore').replace('\x00', '')

    # ---------- request path ----------
    def request_data(self, cmd_str):
        if not self._write_cmd(cmd_str):
            return 'NAK'
        raw = self._read_until_cr(self._timeout)
        if not raw or raw == b' ':
            if self.debug:
                print('[RX-empty]')
            return 'NAK'
        if b'NAK' in raw:
            if self.debug:
                print('[RX-NAK]', raw)
            return 'NAK'
        if len(raw) >= 3:
            body = raw[:-2]
            recv_hi = raw[-2]
            recv_lo = raw[-1]
            if self._verify_rx_crc(body, recv_hi, recv_lo):
                payload = self._strip_start(body)
                return self._bytes_to_clean_str(payload)
            else:
                # Accept frames with 1-byte CHK+1
                if len(raw) >= 2 and raw[-1] == ((self._chk_sum(raw[:-1]) + 1) & 0xFF):
                    payload = self._strip_start(raw[:-1])
                    return self._bytes_to_clean_str(payload)
                if self.debug:
                    print('[ERCRC] CRC mismatch')
                return 'ERCRC'
        if self.debug:
            print('[RX-short]', len(raw))
        return 'NAK'

    # ---------- sequences / protocol ----------
    def set_sequences(self, static_seq=None, dynamic_seq=None):
        if static_seq is not None:
            self.static_seq = tuple(static_seq)
        if dynamic_seq is not None:
            self.dynamic_seq = tuple(dynamic_seq)
        self.sequences_by_protocol['DEFAULT'] = {
            'static': self.static_seq, 'dynamic': self.dynamic_seq
        }
        self._restart_groups()

    def set_sequences_for(self, protocol, static_seq=None, dynamic_seq=None):
        key = _norm_proto_key(protocol)
        entry = self.sequences_by_protocol.get(key, {}).copy()
        if static_seq is not None:
            entry['static'] = tuple(static_seq)
        if dynamic_seq is not None:
            entry['dynamic'] = tuple(dynamic_seq)
        self.sequences_by_protocol[key] = entry
        if key == _norm_proto_key(self.protocol):
            self._apply_protocol_sequences()

    def set_sequences_by_protocol(self, mapping):
        newmap = {}
        for k, v in mapping.items():
            key = _norm_proto_key(k)
            sv = tuple(v.get('static', ()))
            dv = tuple(v.get('dynamic', ()))
            newmap[key] = {'static': sv, 'dynamic': dv}
        if 'DEFAULT' not in newmap:
            newmap['DEFAULT'] = {
                'static': self.static_seq, 'dynamic': self.dynamic_seq
            }
        self.sequences_by_protocol.update(newmap)
        self._apply_protocol_sequences()

    def _apply_protocol_sequences(self):
        key = _norm_proto_key(self.protocol)
        entry = self.sequences_by_protocol.get(key) or self.sequences_by_protocol.get('DEFAULT')
        if entry:
            self.static_seq = entry.get('static', self.static_seq)
            self.dynamic_seq = entry.get('dynamic', self.dynamic_seq)
            self._restart_groups()

    def _restart_groups(self):
        self.request_static = True
        self.request_counter = 0

    @staticmethod
    def _normalize_item(item):
        if isinstance(item, tuple) and len(item) == 2:
            return item[0], item[1]
        return item, item

    # ---------- high-level ----------
    def auto_detect(self):
        attempts = 3
        for _ in range(attempts):
            self.start_char = '('
            self.delimiter = ' '
            self.protocol = 'NoD'
            qpi = self.request_data('QPI')
            if qpi and qpi != 'NAK':
                qpi_chk = qpi.lstrip('(')
                if qpi_chk[:2] == 'PI':
                    self.protocol = 'PI30'
                    self.delimiter = ' '
                    self._apply_protocol_sequences()
                    return True
            self.start_char = '^Dxxx'
            self.delimiter = ','
            p = self.request_data('^P005PI')
            if p and p != 'NAK':
                p_chk = p.lstrip('^').lstrip('D')
                if p_chk.startswith('18'):
                    self.protocol = 'PI18'
                    self.delimiter = ','
                    self._apply_protocol_sequences()
                    return True
        self._apply_protocol_sequences()
        return False

    def send_custom(self, cmd_or_tuple, name=None):
        if isinstance(cmd_or_tuple, tuple) and len(cmd_or_tuple) == 2:
            self._custom_name, self._custom_cmd = cmd_or_tuple[0], cmd_or_tuple[1]
            return True
        if name is not None:
            self._custom_name, self._custom_cmd = name, cmd_or_tuple
            return True
        self._custom_cmd = cmd_or_tuple
        self._custom_name = None
        return True

    def step(self):
        t = now()
        if utime.ticks_diff(t, self._last_at) < self.delay_ms:
            return False

        # Custom first (store in livedata by convention)
        if self._custom_cmd:
            ans = self.request_data(self._custom_cmd)
            alias = self._custom_name or self._custom_cmd
            if ans not in ('ERCRC','NAK'):
                self._store_answer(alias, self._custom_cmd, ans, group='dynamic')
                # prevent raw custom command key from persisting in livedata
                try:
                    if self.keep_command_keys and alias != self._custom_cmd:
                        self.livedata.pop(self._custom_cmd, None)
                except:
                    pass
                self._mark_ok()
                self.last_live_at = t
            else:
                self._mark_fail()
            self._custom_cmd = None
            self._custom_name = None
            # After custom, restart static
            self.request_static = True
            self.request_counter = 0
            self._last_at = t
            return True

        if self.request_static:
            seq = self.static_seq
            if self.request_counter < len(seq):
                name, cmd = self._normalize_item(seq[self.request_counter])
                ans = self.request_data(cmd)
                if ans not in ('ERCRC','NAK'):
                    self._store_answer(name, cmd, ans, group='static')
                    self._mark_ok()
                else:
                    self._mark_fail()
                # Always advance (skip errors)
                self.request_counter += 1
            else:
                # done static -> move to dynamic
                self.request_static = False
                self.request_counter = 0
        else:
            seq = self.dynamic_seq
            if self.request_counter < len(seq):
                name, cmd = self._normalize_item(seq[self.request_counter])
                ans = self.request_data(cmd)
                if ans not in ('ERCRC','NAK'):
                    self._store_answer(name, cmd, ans, group='dynamic')
                    self._mark_ok()
                    self.last_live_at = t
                else:
                    self._mark_fail()
                self.request_counter += 1
            else:
                # done dynamic -> loop to static again
                self.request_static = True
                self.request_counter = 0

        self._last_at = t
        self.connection = (self._conn_fail < 10)
        return True

    # ---------- helpers ----------
    def _mark_ok(self):
        self._conn_ok += 1
        if self._conn_fail > 0:
            self._conn_fail -= 1

    def _mark_fail(self):
        self._conn_fail += 1

    def _store_answer(self, name, cmd, ans, group='dynamic'):
        tgt = self.devicedata if group == 'static' else self.livedata
        tgt[name] = ans
        if self.keep_command_keys and name != cmd:
            tgt[cmd] = ans
        if cmd == 'QPIGS' and ans not in ('ERCRC','NAK'):
            tgt['QPIGS'] = ans
