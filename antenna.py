"""
Full antenna scan system (single-file version).

-Controls the stepper motor over RS-485 (Modbus RTU).
-Controls a Keysight N8957A PSU over GPIB (via PyVISA).
-Controls an HP 8720C VNA over GPIB (via PyVISA).
-Provides a Tkinter GUI to set all parameters, including absolute V/I tolerances and to start/stop the automated scan.
"""

import csv
import threading
import time
from pathlib import Path
from typing import Tuple, Callable
import serial
import struct
import numpy as np
import pyvisa
import tkinter as tk
from tkinter import ttk, filedialog, messagebox


#####RS485 MOTOR

#Motor and Serial/Modbus settings
PORT        = "COM3"   
BAUD        = 115200
DEV         = 1         #Modbus slave ID
TIMEOUT_S   = 0.5       #s, per command
IDLE_GAP_S  = 0.010     #s, "quiet line" gap since last byte
STEPS_PER_DEG = 100.0   #100 steps = 1 degree

#Motion params 
SPEED_HZ      = 2000
ACC_DEC_RATE  = 1500
STOP_DEC_RATE = 1500
OPER_CURR_PPT = 1000
POLL_EVERY_S  = 0.05    # status polling cadence
FINISH_TMO_S  = 5.0     # max wait per move for MOVE bit to drop

#registers according to manual
REG_DD_TYPE, REG_DD_POS, REG_DD_SPEED = 0x005A, 0x005C, 0x005E
REG_DD_RATE, REG_DD_STOPDEC, REG_DD_CURR = 0x0060, 0x0062, 0x0064
REG_DD_TRIG, REG_DD_DST = 0x0066, 0x0068
REG_OUT_STATUS = 0x007F
BIT_READY, BIT_MOVE, BIT_INPOS = (1 << 5), (1 << 13), (1 << 14)


#helper functions for Modbus RTU
def _crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF

def _u16(v: int) -> int:
    return v & 0xFFFF

def _pack_u16(v: int) -> bytes:
    return struct.pack(">H", _u16(v))

def _words_from_i32(v: int):
    be = struct.pack(">i", int(v))
    return [(be[0] << 8) | be[1], (be[2] << 8) | be[3]]

def _words_from_u32(v: int):
    be = struct.pack(">I", int(v) & 0xFFFFFFFF)
    return [(be[0] << 8) | be[1], (be[2] << 8) | be[3]]



#echo-agnostic Modbus RTU: read bytes, scan for valid frame, CRC-check.
class RTU:
    def __init__(self, port: str, baud: int, timeout_s: float):
        self.ser = serial.Serial(
            port=port,
            baudrate=baud,
            bytesize=8,
            parity=serial.PARITY_EVEN,
            stopbits=1,
            timeout=timeout_s,
            write_timeout=timeout_s,
        )
        self.ser.reset_input_buffer()

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass

    @staticmethod
    def _crc_ok(frame: bytes) -> bool:
        return _crc16(frame[:-2]) == (frame[-2] | (frame[-1] << 8))

    def _read_until(self, want_fn, total_timeout: float = TIMEOUT_S, idle_gap: float = IDLE_GAP_S):
        """
        Gather bytes; return first frame where want_fn(buf)->frame matches (CRC-checked).
        Returns None on timeout.
        """
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
                if fr:
                    return fr
            else:
                if (now - last_rx) >= idle_gap and buf:
                    fr = want_fn(bytes(buf))
                    if fr:
                        return fr
            time.sleep(0.001)  #light throttle
        return None

    #scanners for replies
    @staticmethod
    def _scan_03(buf: bytes, dev: int, bc_expected: int):
        need = 3 + bc_expected + 2
        for i in range(0, max(0, len(buf) - need + 1)):
            if buf[i] == dev and buf[i + 1] == 0x03 and buf[i + 2] == bc_expected:
                fr = buf[i : i + need]
                if RTU._crc_ok(fr):
                    return fr
        return None

    @staticmethod
    def _scan_06(buf: bytes, dev: int):
        need = 8
        for i in range(0, max(0, len(buf) - need + 1)):
            if buf[i] == dev and buf[i + 1] == 0x06:
                fr = buf[i : i + need]
                if RTU._crc_ok(fr):
                    return fr
        return None

    @staticmethod
    def _scan_10(buf: bytes, dev: int, addr: int, qty: int):
        need = 8
        ah, al = (addr >> 8) & 0xFF, addr & 0xFF
        qh, ql = (qty >> 8) & 0xFF, qty & 0xFF
        for i in range(0, max(0, len(buf) - need + 1)):
            if (
                buf[i] == dev
                and buf[i + 1] == 0x10
                and buf[i + 2] == ah
                and buf[i + 3] == al
                and buf[i + 4] == qh
                and buf[i + 5] == ql
            ):
                fr = buf[i : i + need]
                if RTU._crc_ok(fr):
                    return fr
        return None


    #public Modbus calls
    def read_holding(self, addr: int, count: int):
        pdu = bytes(
            [
                DEV,
                0x03,
                (addr >> 8) & 0xFF,
                addr & 0xFF,
                (count >> 8) & 0xFF,
                count & 0xFF,
            ]
        )
        crc = _crc16(pdu)
        tx = pdu + bytes([crc & 0xFF, (crc >> 8) & 0xFF])
        self.ser.reset_input_buffer()
        self.ser.write(tx)
        bc = count * 2
        fr = self._read_until(lambda b: self._scan_03(b, DEV, bc))
        if not fr:
            return None, "timeout"
        data = fr[3:-2]
        regs = [(data[i] << 8) | data[i + 1] for i in range(0, len(data), 2)]
        return regs, None

    def write_single(self, addr: int, val: int) -> bool:
        pdu = bytes(
            [
                DEV,
                0x06,
                (addr >> 8) & 0xFF,
                addr & 0xFF,
                (val >> 8) & 0xFF,
                val & 0xFF,
            ]
        )
        crc = _crc16(pdu)
        tx = pdu + bytes([crc & 0xFF, (crc >> 8) & 0xFF])
        self.ser.reset_input_buffer()
        self.ser.write(tx)
        fr = self._read_until(lambda b: self._scan_06(b, DEV))
        return fr is not None

    def write_multiple(self, addr: int, values: list[int]) -> bool:
        qty = len(values)
        data = b"".join(_pack_u16(v) for v in values)
        pdu = bytes(
            [
                DEV,
                0x10,
                (addr >> 8) & 0xFF,
                addr & 0xFF,
                (qty >> 8) & 0xFF,
                qty & 0xFF,
                len(data),
            ]
        ) + data
        crc = _crc16(pdu)
        tx = pdu + bytes([crc & 0xFF, (crc >> 8) & 0xFF])
        self.ser.reset_input_buffer()
        self.ser.write(tx)
        fr = self._read_until(lambda b: self._scan_10(b, DEV, addr, qty))
        return fr is not None


def deg_to_steps(d: float) -> int:
    return int(round(d * STEPS_PER_DEG))


def poll_move_done(rtu: RTU, timeout_s: float = FINISH_TMO_S) -> bool:
    """Poll MOVE bit until it clears or timeout is reached."""
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        regs, err = rtu.read_holding(REG_OUT_STATUS, 1)
        if not err and regs:
            w = regs[0]
            if (w & BIT_MOVE) == 0:
                return True
        time.sleep(POLL_EVERY_S)
    # Soft-finish even if status didn't flip (keeps it simple).
    return True


#set speed/acc/dec/current once at startup
def configure_motion_once(rtu: RTU) -> None:

    rtu.write_multiple(REG_DD_SPEED, _words_from_i32(SPEED_HZ))
    rtu.write_multiple(REG_DD_RATE, _words_from_u32(ACC_DEC_RATE))
    rtu.write_multiple(REG_DD_STOPDEC, _words_from_u32(STOP_DEC_RATE))
    rtu.write_multiple(REG_DD_CURR, _words_from_u32(OPER_CURR_PPT))


#absolute move: TYPE=1, POS=steps, TRIG=1 per manual
def move_abs(rtu: RTU, steps: int) -> bool:
    rtu.write_multiple(REG_DD_TYPE, _words_from_i32(1))
    rtu.write_multiple(REG_DD_POS, _words_from_i32(int(steps)))
    rtu.write_multiple(REG_DD_TRIG, _words_from_u32(1))
    return poll_move_done(rtu)



##### PSU (Keysight N8957A)

PSU_ADDR = "GPIB0::5::INSTR"

""" helper for:
      - set/get programmed voltage & current
      - measure actual voltage & current
      - output on/off and state query
    """
class N8957A:
    def __init__(self, addr: str = PSU_ADDR, timeout_ms: int = 10000):
        rm = pyvisa.ResourceManager()
        self.inst = rm.open_resource(addr)
        self.inst.timeout = timeout_ms
        self.clear()

    def clear(self):
        try:
            self.inst.clear()
        except Exception:
            pass

    def idn(self) -> str:
        return self.inst.query("*IDN?").strip()

    #output control
    def output_on(self):
        self.inst.write("OUTP ON")

    def output_off(self):
        self.inst.write("OUTP OFF")

    def output_state(self) -> bool:
        return bool(int(self.inst.query("OUTP?").strip()))
    

    #program setpoints
    def set_voltage(self, volts: float):
        self.inst.write(f"VOLT {volts}")

    def set_current(self, amps: float):
        self.inst.write(f"CURR {amps}")

    def get_voltage_set(self) -> float:
        return float(self.inst.query("VOLT?"))

    def get_current_set(self) -> float:
        return float(self.inst.query("CURR?"))

    def voltage_limits(self) -> Tuple[float, float]:
        vmin = float(self.inst.query("VOLT? MIN"))
        vmax = float(self.inst.query("VOLT? MAX"))
        return vmin, vmax

    def current_limits(self) -> Tuple[float, float]:
        imin = float(self.inst.query("CURR? MIN"))
        imax = float(self.inst.query("CURR? MAX"))
        return imin, imax

    #measurements
    def measure_voltage(self) -> float:
        return float(self.inst.query("MEAS:VOLT?"))

    def measure_current(self) -> float:
        return float(self.inst.query("MEAS:CURR?"))

    def close(self):
        try:
            self.inst.close()
        except Exception:
            pass


####VNA helper: HP 8720C (GPIB)

VNA_ADDR = "GPIB0::16::INSTR"
VNA_TIMEOUT_MS = 12000

#configure sweep and grab complex trace
class HP8720C:
    def __init__(self, addr: str = VNA_ADDR, timeout_ms: int = VNA_TIMEOUT_MS):
        rm = pyvisa.ResourceManager()
        self.inst = rm.open_resource(addr)
        self.inst.timeout = timeout_ms
        self.inst.clear()
        self._f_start = None
        self._f_stop = None

    #instrument id
    def idn(self) -> str:
        self.inst.write("IDN?")
        return self.inst.read().strip()

    #Set center/span and ASCII data format (FORM4 = real,imag ASCII)
    def configure_sweep(self, center_hz: float, span_hz: float) -> None:
        self._f_start = center_hz - span_hz / 2.0
        self._f_stop = center_hz + span_hz / 2.0
        self.inst.write(f"CENT {center_hz}")
        self.inst.write(f"SPAN {span_hz}")
        self.inst.write("FORM4;")  # ASCII corrected data


    def grab_current_trace(self, query_delay_s: float) -> Tuple[np.ndarray, np.ndarray]:
        """
        Request corrected complex trace data with OUTPDATA.
        Returns:
            freq: 1D np.ndarray of frequencies in Hz
            z:    1D np.ndarray of complex S-parameter samples
        """
        if self._f_start is None or self._f_stop is None:
            raise RuntimeError("VNA sweep not configured. Call configure_sweep() first.")

        self.inst.write("OUTPDATA")
        time.sleep(query_delay_s)
        raw = self.inst.read_raw().decode(errors="ignore")
        lines = raw.strip().split("\n")
        z_list = []

        for line in lines:
            line = line.strip()
            if not line:
                continue
            try:
                re_s, im_s = line.split(",")
                re_v = float(re_s)
                im_v = float(im_s)
            except ValueError:
                continue
            z_list.append(re_v + 1j * im_v)

        if not z_list:
            raise RuntimeError("No VNA data parsed from OUTPDATA response.")

        z = np.array(z_list, dtype=np.complex128)
        freq = np.linspace(self._f_start, self._f_stop, z.size)
        return freq, z
    

    #grab whichever S-parameter is currently active on the VNA front panel
    def grab_trace(self, query_delay_s: float) -> Tuple[np.ndarray, np.ndarray]:
        return self.grab_current_trace(query_delay_s)
    

    #select a specific S-parameter (e.g. 'S11' or 'S21') then grab the trace.
    def grab_trace_sparam(self, s_param: str, query_delay_s: float) -> Tuple[np.ndarray, np.ndarray]:
        s = s_param.upper().strip()
        if s not in {"S11", "S21", "S12", "S22"}:
            raise ValueError(f"Unsupported S-parameter '{s}'. Use 'S11', 'S21', 'S12', or 'S22'.")

        #self.inst.write("SWE:POIN 51")
        self.inst.write(f"{s};")
        #small delay so the instrument has time to update the trace
        #time.sleep(1)

        return self.grab_current_trace(query_delay_s)

    def close(self) -> None:
        try:
            self.inst.close()
        except Exception:
            pass



#PSU helper: sanity checks
def check_psu(
    psu: N8957A,
    set_v: float,
    set_i: float,
    v_tol_abs: float,
    i_tol_abs: float,
    label: str,
    log: Callable[[str], None],
) -> Tuple[float, float]:
    

    #read programmed and measured V/I from PSU, print them, and check tolerance
    # v_set = psu.get_voltage_set()
    # i_set = psu.get_current_set()
    v_meas = psu.measure_voltage()
    i_meas = psu.measure_current()

    log(
        f"[PSU {label}] "
        f"set V={set_v:.3f} V, meas V={v_meas:.3f} V | "
        f"set I={set_i:.3f} A, meas I={i_meas:.3f} A"
    )

    if v_tol_abs < 0 or i_tol_abs < 0:
        raise RuntimeError("Voltage/current tolerances must be non-negative.")

    if v_tol_abs > 0 and abs(v_meas - set_v) > v_tol_abs:
        raise RuntimeError(
            f"Measured PSU voltage {v_meas:.3f} V deviates from requested {set_v:.3f} V "
            f"by more than ±{v_tol_abs:.3f} V."
        )

    if i_tol_abs > 0 and abs(i_meas - set_i) > i_tol_abs:
        raise RuntimeError(
            f"Measured PSU current {i_meas:.3f} A deviates from requested {set_i:.3f} A "
            f"by more than ±{i_tol_abs:.3f} A."
        )

    return v_meas, i_meas



#One full measurement cycle at a single angle

def acquire_at_angle(
    rtu: RTU,
    psu: N8957A,
    vna: HP8720C,
    angle_deg: float,
    set_v: float,
    set_i: float,
    psu_settle_s: float,
    vna_query_delay_s: float,
    v_tol_abs: float,
    i_tol_abs: float,
    out_dir: Path,
    log: Callable[[str], None],
    stop_flag: Callable[[], bool],
    measure_s11: bool,
    measure_s21: bool,
) -> None:
    """
    Pipeline for each angle
      - move motor
      - PSU on + check
      - VNA sweep
      - PSU check again
      - PSU off
      - save CSV
    """

    if stop_flag():
        log("[SCAN] Stop requested before angle step, skipping angle.")
        return

    log(f"\n===== ANGLE {angle_deg:+.1f}° =====")

    psu_on_t0 = None

    #1) Move motor to the requested absolute angle
    steps = deg_to_steps(angle_deg)
    log(f"[MOTOR] Moving to {angle_deg:+.1f}° ({steps} steps)")
    ok = move_abs(rtu, steps)
    if not ok:
        log("[MOTOR] WARNING: move_abs reported timeout, continuing anyway.")

    if stop_flag():
        log("[SCAN] Stop requested after motor move, aborting angle.")
        return

    # 2) Enable PSU and apply setpoints
    log(f"[PSU] Enabling output: {set_v:.3f} V, {set_i:.3f} A")
    psu.set_voltage(set_v)
    psu.set_current(set_i)
    psu.output_on()
    psu_on_t0 = time.time()
    # time.sleep(psu_settle_s)

    # 3) First PSU sanity check
    v_meas1, i_meas1 = check_psu(
         psu, set_v, set_i, v_tol_abs, i_tol_abs, label="check #1", log=log
     )

    if stop_flag():
        log("[SCAN] Stop requested after PSU check #1, aborting angle.")
        if psu_on_t0 is not None:
            elapsed_time = time.time() - psu_on_t0
            log(f"ELAPSED ON-TIME: {elapsed_time}")
        psu.output_off()
        return
    
    freq = None
    s11_mag_db = None
    s11_phase_deg = None
    s21_mag_db = None
    s21_phase_deg = None

    # 4) VNA sweeps: S11 and S21
    if measure_s11:
        log("[VNA] Sweep S11 / acquiring trace...")
        freq_s11, z_s11 = vna.grab_trace_sparam("S11", vna_query_delay_s)
        log(f"[VNA] S11 sweep finished, got {len(freq_s11)} points.")
        s11_mag_db = 20.0 * np.log10(np.abs(z_s11))
        s11_phase_deg = np.degrees(np.angle(z_s11))
        freq = freq_s11

        if stop_flag():
            log("[SCAN] Stop requested after S11 sweep, aborting angle.")
            if psu_on_t0 is not None:
                elapsed_time = time.time() - psu_on_t0
                log(f"ELAPSED ON-TIME: {elapsed_time}")
            psu.output_off()
            return

    if measure_s21:
        log("[VNA] Sweep S21 / acquiring trace...")
        freq_s21, z_s21 = vna.grab_trace_sparam("S21", vna_query_delay_s)
        log(f"[VNA] S21 sweep finished, got {len(freq_s21)} points.")
        s21_mag_db = 20.0 * np.log10(np.abs(z_s21))
        s21_phase_deg = np.degrees(np.angle(z_s21))

        if freq is None:
            freq = freq_s21
        else:
            #both traces must share the same frequency axis
            if freq.size != freq_s21.size or not np.allclose(freq, freq_s21):
                raise RuntimeError("S11 and S21 traces do not share the same frequency axis. Check VNA setup.")

    # 5) Second PSU sanity check
    # v_meas2, i_meas2 = check_psu(
    #      psu, set_v, set_i, v_tol_abs, i_tol_abs, label="check #2", log=log
    #  )

    # 6) Turn PSU off
    if psu_on_t0 is not None:
        elapsed_time = time.time() - psu_on_t0
        log(f"ELAPSED ON-TIME: {elapsed_time}")
    log("[PSU] Disabling output")
    psu.output_off()

    if freq is None:
        log("[VNA] No S-parameter trace captured for this angle; skipping CSV write.")
        return

    #compute magnitude [dB] and phase [deg] for both S11 and S21
    # s11_mag_db = 20.0 * np.log10(np.abs(z_s11))
    # s11_phase_deg = np.degrees(np.angle(z_s11))

    # s21_mag_db = 20.0 * np.log10(np.abs(z_s21))
    # s21_phase_deg = np.degrees(np.angle(z_s21))

    # 7) Save CSV with naming: set_current_set_voltage_angle.csv
    out_dir.mkdir(parents=True, exist_ok=True)
    angle_int = int(round(angle_deg))
    sign = "+" if angle_int >= 0 else "-"
    angle_tag = f"{sign}{abs(angle_int):03d}deg"  
    fname = f"{set_i:.1f}A_{set_v:.1f}V_{angle_tag}.csv"
    csv_path = out_dir / fname
    log(f"[FILE] Writing {csv_path}")

    #write csv
    with csv_path.open("w", newline="") as f:
        w = csv.writer(f)
        #one row per frequency point, plus metadata per row
        header = ["freq_Hz"]
        if measure_s11:
            header += ["S11_mag_dB", "S11_phase_deg"]
        if measure_s21:
            header += ["S21_mag_dB", "S21_phase_deg"]
        header += [
            "V_set",
            "I_set",
            "V_meas_1",
            "I_meas_1",
            # "V_meas_2",
            # "I_meas_2",
            "angle_deg",
        ]
        w.writerow(header)

        n_pts = len(freq)
        for idx in range(n_pts):
            row = [freq[idx]]
            if measure_s11 and s11_mag_db is not None and s11_phase_deg is not None:
                row += [s11_mag_db[idx], s11_phase_deg[idx]]
            if measure_s21 and s21_mag_db is not None and s21_phase_deg is not None:
                row += [s21_mag_db[idx], s21_phase_deg[idx]]
            row += [
                set_v,
                set_i,
                v_meas1,
                i_meas1,
                # v_meas2,
                # i_meas2,
                angle_deg,
            ]
            w.writerow(row)



#Overall scan orchestration
def run_scan(
    start_angle_deg: float,
    end_angle_deg: float,
    angle_step_deg: float,
    set_voltage_v: float,
    set_current_a: float,
    cooldown_s: float,
    psu_settle_s: float,
    vna_query_delay_s: float,
    vna_center_ghz: float,
    vna_span_ghz: float,
    v_tol_abs: float,
    i_tol_abs: float,
    output_dir: Path,
    log: Callable[[str], None],
    stop_flag: Callable[[], bool],
    measure_s11: bool,
    measure_s21: bool,
) -> None:

    center_hz = vna_center_ghz * 1e9
    span_hz = vna_span_ghz * 1e9
    output_dir.mkdir(parents=True, exist_ok=True)

    log("===== Initializing hardware =====")

    # Motor
    rtu = RTU(PORT, BAUD, TIMEOUT_S)
    log(f"[MOTOR] Connected {PORT}, {BAUD} 8E1, DEV={DEV}")
    configure_motion_once(rtu)

    # PSU
    psu = N8957A()
    log(f"[PSU] ID: {psu.idn()}")

    # VNA
    vna = HP8720C()
    log(f"[VNA] ID: {vna.idn()}")
    vna.configure_sweep(center_hz, span_hz)

    # Move to start angle once
    start_steps = deg_to_steps(start_angle_deg)
    log(f"[MOTOR] Moving to start angle {start_angle_deg:+.1f}°")
    move_abs(rtu, start_steps)

    # Number of positions (inclusive of both ends)
    n_pos = int(round((end_angle_deg - start_angle_deg) / angle_step_deg)) + 1
    log(
        f"[SCAN] {n_pos} positions from {start_angle_deg:+.1f}° "
        f"to {end_angle_deg:+.1f}° in {angle_step_deg:.1f}° steps"
    )

    try:
        for idx in range(n_pos):
            if stop_flag():
                log("[SCAN] Stop requested, ending scan.")
                break

            angle = start_angle_deg + idx * angle_step_deg
            acquire_at_angle(
                rtu=rtu,
                psu=psu,
                vna=vna,
                angle_deg=angle,
                set_v=set_voltage_v,
                set_i=set_current_a,
                psu_settle_s=psu_settle_s,
                vna_query_delay_s=vna_query_delay_s,
                v_tol_abs=v_tol_abs,
                i_tol_abs=i_tol_abs,
                out_dir=output_dir,
                log=log,
                stop_flag=stop_flag,
                measure_s11=measure_s11,
                measure_s21=measure_s21,
            )

            if stop_flag():
                log("[SCAN] Stop requested after angle, ending scan.")
                break

            log(f"[SCAN] Cooldown for {cooldown_s:.1f} s")
            #cooldown in small chunks so Stop works quickly
            t0 = time.time()
            while time.time() - t0 < cooldown_s:
                if stop_flag():
                    log("[SCAN] Stop requested during cooldown.")
                    break
                #time.sleep(0.05)
            if stop_flag():
                break

        log("[SCAN] Done.")

    finally:
        #this block always runs, even if there is an exception
        log("[SHUTDOWN] Cleaning up (PSU OFF, closing devices)...")
        try:
            psu.output_off()
        except Exception:
            pass
        try:
            psu.close()
        except Exception:
            pass
        try:
            vna.close()
        except Exception:
            pass
        try:
            rtu.close()
        except Exception:
            pass
        log("[SHUTDOWN] Done.")


#Tkinter GUI 
class ScanGUI:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("METEORITE Antenna Scan Controller")
        self.stop_requested = False

        # main frame 
        main = ttk.Frame(root, padding=10)
        main.grid(row=0, column=0, sticky="nsew")
        root.rowconfigure(0, weight=1)
        root.columnconfigure(0, weight=1)

        # Sub-frames for parameters and buttons/log
        params_frame = ttk.LabelFrame(main, text="Scan parameters", padding=10)
        params_frame.grid(row=0, column=0, sticky="nsew")
        main.rowconfigure(0, weight=0)
        main.columnconfigure(0, weight=1)
        log_frame = ttk.LabelFrame(main, text="Log", padding=5)
        log_frame.grid(row=1, column=0, sticky="nsew", pady=(8, 0))
        main.rowconfigure(1, weight=1)

        # parameters
        row = 0

        def add_entry(label_text, default, units):
            nonlocal row
            ttk.Label(params_frame, text=label_text).grid(row=row, column=0, sticky="w", pady=2)
            var = tk.StringVar(value=str(default))
            entry = ttk.Entry(params_frame, textvariable=var, width=10)
            entry.grid(row=row, column=1, sticky="w", pady=2)
            ttk.Label(params_frame, text=units).grid(row=row, column=2, sticky="w")
            row += 1
            return var

        ttk.Label(params_frame, text="Motor").grid(row=row, column=0, sticky="w", pady=(0, 2))
        row += 1
        self.start_angle_var = add_entry("Start angle", 0.0, "deg")
        self.end_angle_var = add_entry("End angle", 360.0, "deg")
        self.step_angle_var = add_entry("Angle step", 5.0, "deg")

        ttk.Separator(params_frame).grid(row=row, column=0, columnspan=3, sticky="ew", pady=4)
        row += 1

        ttk.Label(params_frame, text="PSU").grid(row=row, column=0, sticky="w", pady=(0, 2))
        row += 1
        self.voltage_var = add_entry("Voltage", 750.0, "V")
        self.current_var = add_entry("Current", 0.25, "A")
        self.v_tol_var = add_entry("V tol ±", 750.0, "V") #manual states programming accuracy of 0.1% of full-scale voltage so 1500V * 0.1% = 1.5V
        self.i_tol_var = add_entry("I tol ±", 0.25, "A")

        ttk.Separator(params_frame).grid(row=row, column=0, columnspan=3, sticky="ew", pady=4)
        row += 1

        ttk.Label(params_frame, text="Timing").grid(row=row, column=0, sticky="w", pady=(0, 2))
        row += 1
        self.cooldown_var = add_entry("Cooldown", 5.0, "s")
        self.psu_settle_var = add_entry("PSU settle", 0.05, "s")
        self.vna_delay_var = add_entry("VNA query delay", 0.01, "s")

        ttk.Separator(params_frame).grid(row=row, column=0, columnspan=3, sticky="ew", pady=4)
        row += 1

        ttk.Label(params_frame, text="VNA").grid(row=row, column=0, sticky="w", pady=(0, 2))
        row += 1
        self.center_var = add_entry("Center freq", 10.0, "GHz")
        self.span_var = add_entry("Span", 4.0, "GHz")

        # S-parameter selection checkboxes
        self.s11_var = tk.BooleanVar(value=False)
        self.s21_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(params_frame, text="Measure S11", variable=self.s11_var).grid(
            row=row, column=0, sticky="w", pady=2
        )
        ttk.Checkbutton(params_frame, text="Measure S21", variable=self.s21_var).grid(
            row=row, column=1, sticky="w", pady=2
        )
        row += 1

        ttk.Separator(params_frame).grid(row=row, column=0, columnspan=3, sticky="ew", pady=4)
        row += 1

        ttk.Label(params_frame, text="Output").grid(row=row, column=0, sticky="w", pady=(0, 2))
        row += 1
        ttk.Label(params_frame, text="Directory").grid(row=row, column=0, sticky="w", pady=2)
        self.output_dir_var = tk.StringVar(value="data")
        output_entry = ttk.Entry(params_frame, textvariable=self.output_dir_var, width=18)
        output_entry.grid(row=row, column=1, sticky="w", pady=2)
        ttk.Button(params_frame, text="Browse…", command=self.browse_output_dir).grid(
            row=row, column=2, sticky="w", padx=4
        )
        row += 1

        #buttons (start/stop)
        button_frame = ttk.Frame(params_frame)
        button_frame.grid(row=row, column=0, columnspan=3, pady=(8, 0), sticky="w")
        self.start_button = ttk.Button(button_frame, text="Start scan", command=self.on_start)
        self.start_button.grid(row=0, column=0, padx=(0, 6))
        self.stop_button = ttk.Button(button_frame, text="Stop", command=self.on_stop, state="disabled")
        self.stop_button.grid(row=0, column=1)

        #log area 
        self.log_text = tk.Text(log_frame, height=18, wrap="none", state="disabled")
        self.log_text.grid(row=0, column=0, sticky="nsew")
        log_frame.rowconfigure(0, weight=1)
        log_frame.columnconfigure(0, weight=1)
        scroll_y = ttk.Scrollbar(log_frame, orient="vertical", command=self.log_text.yview)
        scroll_y.grid(row=0, column=1, sticky="ns")
        self.log_text.configure(yscrollcommand=scroll_y.set)

        # Window close handler
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    #GUI helpers
    def browse_output_dir(self):
        d = filedialog.askdirectory()
        if d:
            self.output_dir_var.set(d)

    def gui_log(self, msg: str):
        def append():
            self.log_text.configure(state="normal")
            self.log_text.insert("end", msg + "\n")
            self.log_text.see("end")
            self.log_text.configure(state="disabled")

        self.root.after(0, append)

    def on_start(self):
        try:
            start_angle = float(self.start_angle_var.get())
            end_angle = float(self.end_angle_var.get())
            step_angle = float(self.step_angle_var.get())

            voltage = float(self.voltage_var.get())
            current = float(self.current_var.get())
            v_tol_abs = float(self.v_tol_var.get())
            i_tol_abs = float(self.i_tol_var.get())

            cooldown = float(self.cooldown_var.get())
            psu_settle = float(self.psu_settle_var.get())
            vna_delay = float(self.vna_delay_var.get())

            center_ghz = float(self.center_var.get())
            span_ghz = float(self.span_var.get())

            output_dir = Path(self.output_dir_var.get())
            s11_enabled = self.s11_var.get()
            s21_enabled = self.s21_var.get()
        except ValueError as e:
            messagebox.showerror("Invalid input", f"Please check your inputs.\n\n{e}")
            return

        if step_angle == 0:
            messagebox.showerror("Invalid step", "Angle step must be non-zero.")
            return

        if not (s11_enabled or s21_enabled):
            messagebox.showerror("Invalid selection", "Select at least one S-parameter (S11 and/or S21).")
            return

        self.stop_requested = False
        self.start_button.config(state="disabled")
        self.stop_button.config(state="normal")

        self.gui_log("Starting scan…")

        def stop_flag():
            return self.stop_requested

        def worker():
            try:
                run_scan(
                    start_angle_deg=start_angle,
                    end_angle_deg=end_angle,
                    angle_step_deg=step_angle,
                    set_voltage_v=voltage,
                    set_current_a=current,
                    cooldown_s=cooldown,
                    psu_settle_s=psu_settle,
                    vna_query_delay_s=vna_delay,
                    vna_center_ghz=center_ghz,
                    vna_span_ghz=span_ghz,
                    v_tol_abs=v_tol_abs,
                    i_tol_abs=i_tol_abs,
                    output_dir=output_dir,
                    log=self.gui_log,
                    stop_flag=stop_flag,
                    measure_s11=s11_enabled,
                    measure_s21=s21_enabled,
                )
            except Exception as e:
                self.gui_log(f"ERROR: {e}")
                messagebox.showerror("Scan error", str(e))
            finally:
                def reenable():
                    self.start_button.config(state="normal")
                    self.stop_button.config(state="disabled")

                self.root.after(0, reenable)

        t = threading.Thread(target=worker, daemon=True)
        t.start()

    def on_stop(self):
        """Stop button: ask scan to stop after current angle."""
        self.stop_requested = True
        self.gui_log("[USER] Stop requested – will abort after current angle / cooldown.")

    def on_close(self):
        if messagebox.askokcancel("Quit", "Quit the GUI?\n(Current scan will be stopped safely.)"):
            self.stop_requested = True
            self.root.destroy()


def main():
    root = tk.Tk()
    style = ttk.Style(root)
    try:
        style.theme_use("clam")
    except Exception:
        pass
    ScanGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
