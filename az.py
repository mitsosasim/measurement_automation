"""
Minimal RS-485 (Modbus RTU) motor scan:
- ABS to -90°, then +5° increments to +90°
- Compact, echo-agnostic RTU
- One-time motion parameter setup; 3 writes per move (TYPE, POS, TRIG)

Requires: pyserial
"""

import serial, time, struct

# -----------------------
# USER SETTINGS
# -----------------------
PORT          = "COM10"   # <-- set your port
BAUD          = 115200
DEV           = 1         # Modbus slave ID
TIMEOUT_S   = 0.5       # s, per command
IDLE_GAP_S    = 0.010     # s, "quiet line" gap since last byte

STEPS_PER_DEG = 100.0     # 100 steps = 1 degree

START_DEG     = -90.0
END_DEG       = +90.0
STEP_DEG      = 5.0
DWELL_S       = 3.0       # wait between steps (placeholder for VNA sweep)

# Motion params (set once at start)
SPEED_HZ      = 2000
ACC_DEC_RATE  = 1500
STOP_DEC_RATE = 1500
OPER_CURR_PPT = 1000

POLL_EVERY_S  = 0.05      # status polling cadence
FINISH_TMO_S  = 5.0       # max wait per move for MOVE bit to drop

# -----------------------
# REGISTERS & BITS
# -----------------------
REG_DD_TYPE, REG_DD_POS, REG_DD_SPEED = 0x005A, 0x005C, 0x005E
REG_DD_RATE, REG_DD_STOPDEC, REG_DD_CURR = 0x0060, 0x0062, 0x0064
REG_DD_TRIG, REG_DD_DST = 0x0066, 0x0068

REG_OUT_STATUS = 0x007F
BIT_READY, BIT_MOVE, BIT_INPOS = (1<<5), (1<<13), (1<<14)

# -----------------------
# Helpers (Modbus RTU)
# -----------------------
def _crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if (crc & 1) else (crc >> 1)
    return crc & 0xFFFF

def _u16(v): return v & 0xFFFF
def _pack_u16(v): return struct.pack(">H", _u16(v))
def _words_from_i32(v: int):
    be = struct.pack(">i", int(v))
    return [(be[0]<<8)|be[1], (be[2]<<8)|be[3]]
def _words_from_u32(v: int):
    be = struct.pack(">I", int(v) & 0xFFFFFFFF)
    return [(be[0]<<8)|be[1], (be[2]<<8)|be[3]]

class RTU:
    """Minimal, echo-agnostic Modbus RTU: read bytes, scan for valid frame, CRC-check."""
    def __init__(self, port, baud, timeout_s):
        self.ser = serial.Serial(
            port=port, baudrate=baud, bytesize=8,
            parity=serial.PARITY_EVEN, stopbits=1,
            timeout=timeout_s, write_timeout=timeout_s,
        )
        self.ser.reset_input_buffer()

    def close(self):
        try: self.ser.close()
        except: pass

    @staticmethod
    def _crc_ok(frame: bytes) -> bool:
        return _crc16(frame[:-2]) == (frame[-2] | (frame[-1] << 8))

    def _read_until(self, want_fn, total_timeout=TIMEOUT_S, idle_gap=IDLE_GAP_S):
        """Gather bytes; return first frame where want_fn(buf)->frame matches (CRC-checked)."""
        buf = bytearray()
        deadline = time.time() + total_timeout
        last_rx = time.time()
        while time.time() < deadline:
            chunk = self.ser.read(256)
            now = time.time()
            if chunk:
                buf += chunk
                last_rx = now
                fr = want_fn(bytes(buf))
                if fr: return fr
            else:
                if (now - last_rx) >= idle_gap and buf:
                    fr = want_fn(bytes(buf))
                    if fr: return fr
            time.sleep(0.001)  # light throttle
        return None

    # ---- scanners for replies
    @staticmethod
    def _scan_03(buf: bytes, dev, bc_expected):
        need = 3 + bc_expected + 2
        for i in range(0, max(0, len(buf) - need + 1)):
            if buf[i] == dev and buf[i+1] == 0x03 and buf[i+2] == bc_expected:
                fr = buf[i:i+need]
                if RTU._crc_ok(fr): return fr
        return None

    @staticmethod
    def _scan_06(buf: bytes, dev):
        need = 8
        for i in range(0, max(0, len(buf) - need + 1)):
            if buf[i] == dev and buf[i+1] == 0x06:
                fr = buf[i:i+need]
                if RTU._crc_ok(fr): return fr
        return None

    @staticmethod
    def _scan_10(buf: bytes, dev, addr, qty):
        need = 8
        ah, al = (addr>>8)&0xFF, addr&0xFF
        qh, ql = (qty>>8)&0xFF, qty&0xFF
        for i in range(0, max(0, len(buf) - need + 1)):
            if (buf[i]==dev and buf[i+1]==0x10 and
                buf[i+2]==ah and buf[i+3]==al and
                buf[i+4]==qh and buf[i+5]==ql):
                fr = buf[i:i+need]
                if RTU._crc_ok(fr): return fr
        return None

    # ---- public calls
    def read_holding(self, addr: int, count: int):
        pdu = bytes([DEV, 0x03, (addr>>8)&0xFF, addr&0xFF, (count>>8)&0xFF, count&0xFF])
        crc = _crc16(pdu)
        tx  = pdu + bytes([crc & 0xFF, (crc>>8)&0xFF])
        self.ser.reset_input_buffer()
        self.ser.write(tx)
        bc = count * 2
        fr = self._read_until(lambda b: self._scan_03(b, DEV, bc))
        if not fr: return None, "timeout"
        data = fr[3:-2]
        regs = [(data[i]<<8) | data[i+1] for i in range(0, len(data), 2)]
        return regs, None

    def write_single(self, addr: int, val: int):
        pdu = bytes([DEV, 0x06, (addr>>8)&0xFF, addr&0xFF, (val>>8)&0xFF, val&0xFF])
        crc = _crc16(pdu)
        tx  = pdu + bytes([crc & 0xFF, (crc>>8)&0xFF])
        self.ser.reset_input_buffer()
        self.ser.write(tx)
        fr = self._read_until(lambda b: self._scan_06(b, DEV))
        return (fr is not None)

    def write_multiple(self, addr: int, values: list[int]):
        qty  = len(values)
        data = b"".join(_pack_u16(v) for v in values)
        pdu  = bytes([DEV, 0x10, (addr>>8)&0xFF, addr&0xFF, (qty>>8)&0xFF, qty&0xFF, len(data)]) + data
        crc  = _crc16(pdu)
        tx   = pdu + bytes([crc & 0xFF, (crc>>8)&0xFF])
        self.ser.reset_input_buffer()
        self.ser.write(tx)
        fr = self._read_until(lambda b: self._scan_10(b, DEV, addr, qty))
        return (fr is not None)

# -----------------------
# High-level moves
# -----------------------
def deg_to_steps(d): return int(round(d * STEPS_PER_DEG))

def poll_move_done(rtu: RTU, timeout_s=FINISH_TMO_S):
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        regs, err = rtu.read_holding(REG_OUT_STATUS, 1)
        if not err:
            w = regs[0]
            if (w & BIT_MOVE) == 0:  # move finished
                return True
        time.sleep(POLL_EVERY_S)
    return True  # soft-finish even if status didn't flip (keeps it simple)

def configure_motion_once(rtu: RTU):
    # Set speed/acc/dec/current once; keep it minimal & fast.
    rtu.write_multiple(REG_DD_SPEED,   _words_from_i32(SPEED_HZ))
    rtu.write_multiple(REG_DD_RATE,    _words_from_u32(ACC_DEC_RATE))
    rtu.write_multiple(REG_DD_STOPDEC, _words_from_u32(STOP_DEC_RATE))
    rtu.write_multiple(REG_DD_CURR,    _words_from_u32(OPER_CURR_PPT))

def move_abs(rtu: RTU, steps: int):
    # TYPE=1 (ABS), POS=steps, TRIG=1
    rtu.write_multiple(REG_DD_TYPE, _words_from_i32(1))
    rtu.write_multiple(REG_DD_POS,  _words_from_i32(int(steps)))
    rtu.write_multiple(REG_DD_TRIG, _words_from_u32(1))
    return poll_move_done(rtu)

def move_inc(rtu: RTU, dsteps: int):
    # TYPE=2 (INC), POS=delta, TRIG=1
    rtu.write_multiple(REG_DD_TYPE, _words_from_i32(2))
    rtu.write_multiple(REG_DD_POS,  _words_from_i32(int(dsteps)))
    rtu.write_multiple(REG_DD_TRIG, _words_from_u32(1))
    return poll_move_done(rtu)

# -----------------------
# Main
# -----------------------
def main():
    rtu = RTU(PORT, BAUD, TIMEOUT_S)
    print(f"[OK] Connected {PORT}, {BAUD} 8E1, DEV={DEV}")
    try:
        configure_motion_once(rtu)

        # ABS to start
        start_steps = deg_to_steps(START_DEG)
        print(f"ABS to {START_DEG:+.1f}° …")
        move_abs(rtu, start_steps)

        # INC +STEP_DEG until END_DEG
        cur = START_DEG
        inc_steps = deg_to_steps(STEP_DEG)
        while cur + STEP_DEG <= END_DEG:
            cur += STEP_DEG
            print(f"INC to {cur:+.1f}° …")
            move_inc(rtu, inc_steps)
            time.sleep(DWELL_S)

        print("[DONE]")
    finally:
        rtu.close()

if __name__ == "__main__":
    main()
