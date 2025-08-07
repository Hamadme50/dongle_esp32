# protocol_config.py — per‑protocol sequences for InverterSerial
# Usage in main.py:
#   from serial import InverterSerial
#   from protocol_config import apply_protocols
#   inv = InverterSerial(uart_id=0, rx_pin=20, tx_pin=21, timeout_ms=1200)
#   inv.start()
#   apply_protocols(inv, keep_command_keys=True)  # or False to keep only your names
#
# Notes:
#  • Sequences use (name, command) pairs; JSON will use 'name' keys.
#  • With keep_command_keys=True, replies are also stored under raw command
#    keys (e.g., 'QPIRI') so existing code keeps working.
#  • PI18 mapping below defaults to the same commands as PI30; change to your
#    real '^P...' commands if your device uses pure-PI18 syntax.
#
def apply_protocols(inv, keep_command_keys=True):
    inv.keep_command_keys = bool(keep_command_keys)
    pi30_static = (
        ('QPIRI',      'QPIRI'),
        ('QMN',        'QMN'),
        ('QVFW',       'QVFW'),
        ('QFLAG',      'QFLAG'),
        ('QPI',        'QPI'),
        ('QBEQI',      'QBEQI'),
    )
    pi30_dynamic = (
        ('QPIGS',      'QPIGS'),
        ('QPIGS2',     'QPIGS2'),
        ('QMOD',       'QMOD'),
        ('Q1',         'Q1'),
        ('QPIWS',      'QPIWS'),
        ('PVPOWER',    'PVPOWER'),
    )
    
    
    pi18_static = (
        ('QPIRI',      '^P007PIRI'),
        ('QVFW',       '^P006VFW'),
        ('QFLAG',      '^P007FLAG'),
        ('QPI',        '^P005PI'),
    )
    pi18_dynamic = (
        ('QPIGS',      '^P005GS'),
        ('QMOD',       '^P006MOD'),
        ('QPIWS',      '^P005FWS'),
    )
    
    

    default_static  = pi30_static
    default_dynamic = pi30_dynamic

    inv.set_sequences_by_protocol({
        'PI30':   {'static': pi30_static,   'dynamic': pi30_dynamic},
        'PI18':   {'static': pi18_static,   'dynamic': pi18_dynamic},
        'DEFAULT':{'static': default_static,'dynamic': default_dynamic},
    })
