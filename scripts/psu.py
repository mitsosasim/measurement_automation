import socket
import pyvisa
from typing import Optional, Protocol, Tuple, Literal

PsuKind = Literal["keysight_n8957a", "gwinstek_psw720h88_lan"]


class PsuDevice(Protocol):
    def idn(self) -> str: ...
    def output_on(self) -> None: ...
    def output_off(self) -> None: ...
    def output_state(self) -> bool: ...
    def set_voltage(self, volts: float) -> None: ...
    def set_current(self, amps: float) -> None: ...
    def get_voltage_set(self) -> float: ...
    def get_current_set(self) -> float: ...
    def voltage_limits(self) -> Tuple[float, float]: ...
    def current_limits(self) -> Tuple[float, float]: ...
    def measure_voltage(self) -> float: ...
    def measure_current(self) -> float: ...
    def close(self) -> None: ...


KEYSIGHT_N8957A_ADDR = "GPIB0::5::INSTR"
GWINSTEK_PSW720H88_DEFAULT_IP = "192.168.1.123"
GWINSTEK_PSW720H88_DEFAULT_PORT = 2268
GWINSTEK_PSW720H88_DEFAULT_CHANNEL = 1

class N8957A:
    """
    Minimal Keysight N8957A helper for:
      - set/get programmed voltage & current
      - measure actual voltage & current
      - output on/off and state query
    SCPI per Keysight N8900 Series OSG.
    """
    def __init__(self, addr: str = KEYSIGHT_N8957A_ADDR, timeout_ms: int = 10000):
        rm = pyvisa.ResourceManager()
        self.inst = rm.open_resource(addr)
        self.inst.timeout = timeout_ms
        # Terminations typically fine as PyVISA defaults; uncomment if needed:
        # self.inst.write_termination = '\n'
        # self.inst.read_termination  = '\n'
        self.clear()

    # ---------- basic I/O ----------
    def clear(self):
        try: self.inst.clear()
        except: pass

    def idn(self) -> str:
        return self.inst.query("*IDN?").strip()

    # ---------- output control ----------
    def output_on(self):
        self.inst.write("OUTP ON")

    def output_off(self):
        self.inst.write("OUTP OFF")

    def output_state(self) -> bool:
        return bool(int(self.inst.query("OUTP?").strip()))

    # ---------- program setpoints (SOURce) ----------
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

    # ---------- measured readback (MEASure) ----------
    def measure_voltage(self) -> float:
        return float(self.inst.query("MEAS:VOLT?"))

    def measure_current(self) -> float:
        return float(self.inst.query("MEAS:CURR?"))

    # ---------- tidy ----------
    def close(self):
        try: self.inst.close()
        except: pass


class PSW720H88Lan:
    def __init__(self, ip: str = GWINSTEK_PSW720H88_DEFAULT_IP, port: int = GWINSTEK_PSW720H88_DEFAULT_PORT, timeout_s: float = 5.0, channel: int = GWINSTEK_PSW720H88_DEFAULT_CHANNEL):
        self.ip = ip
        self.port = int(port)
        self.timeout_s = float(timeout_s)
        self.channel = int(channel)
        self.sock = socket.create_connection((self.ip, self.port), timeout=self.timeout_s)
        self.sock.settimeout(self.timeout_s)

    def _write(self, cmd: str) -> None:
        self.sock.sendall((cmd + "\n").encode("ascii"))

    def _query(self, cmd: str) -> str:
        self._write(cmd)
        chunks = []
        while True:
            data = self.sock.recv(4096)
            if not data:
                break
            chunks.append(data)
            if b"\n" in data:
                break
        return b"".join(chunks).decode(errors="ignore").strip()

    def _chan_cmd(self, base_cmd: str) -> str:
        return f"{base_cmd}{self.channel}" if self.channel > 0 else base_cmd

    def clear(self):
        pass

    def idn(self) -> str:
        return self._query("*IDN?")

    def output_on(self):
        self._write(self._chan_cmd("OUTP") + " ON")

    def output_off(self):
        self._write(self._chan_cmd("OUTP") + " OFF")

    def output_state(self) -> bool:
        resp = self._query(self._chan_cmd("OUTP") + "?")
        return bool(int(float(resp)))

    def set_voltage(self, volts: float):
        self._write(self._chan_cmd("VOLT") + f" {volts}")

    def set_current(self, amps: float):
        self._write(self._chan_cmd("CURR") + f" {amps}")

    def get_voltage_set(self) -> float:
        return float(self._query(self._chan_cmd("VOLT") + "?"))

    def get_current_set(self) -> float:
        return float(self._query(self._chan_cmd("CURR") + "?"))

    def voltage_limits(self) -> Tuple[float, float]:
        vmin = float(self._query(self._chan_cmd("VOLT") + "? MIN"))
        vmax = float(self._query(self._chan_cmd("VOLT") + "? MAX"))
        return vmin, vmax

    def current_limits(self) -> Tuple[float, float]:
        imin = float(self._query(self._chan_cmd("CURR") + "? MIN"))
        imax = float(self._query(self._chan_cmd("CURR") + "? MAX"))
        return imin, imax

    def measure_voltage(self) -> float:
        return float(self._query(self._chan_cmd("MEAS:VOLT") + "?"))

    def measure_current(self) -> float:
        return float(self._query(self._chan_cmd("MEAS:CURR") + "?"))

    def close(self):
        try:
            self.sock.close()
        except Exception:
            pass


def create_psu(kind: PsuKind = "keysight_n8957a", *, addr: Optional[str] = None, ip: Optional[str] = None, port: Optional[int] = None, channel: Optional[int] = None) -> PsuDevice:
    if kind == "keysight_n8957a":
        return N8957A(addr=addr or KEYSIGHT_N8957A_ADDR)
    if kind == "gwinstek_psw720h88_lan":
        return PSW720H88Lan(
            ip=ip or GWINSTEK_PSW720H88_DEFAULT_IP,
            port=port or GWINSTEK_PSW720H88_DEFAULT_PORT,
            channel=channel or GWINSTEK_PSW720H88_DEFAULT_CHANNEL,
        )
    raise ValueError(f"Unsupported PSU kind: {kind}")

if __name__ == "__main__":
    psu = N8957A()
    print("ID:", psu.idn())
    # Example use:
    vmin, vmax = psu.voltage_limits()
    imin, imax = psu.current_limits()
    print("PSU limits:", (vmin, vmax), "V ;", (imin, imax), "A")

    # Safe sequence: program limits first, then enable
    psu.set_voltage(10.0)
    psu.set_current(2.0)
    psu.output_on()

    print("Set V:", psu.get_voltage_set(), "V   Set I:", psu.get_current_set(), "A")
    print("Meas V:", psu.measure_voltage(), "V   Meas I:", psu.measure_current(), "A")

    psu.output_off()
    psu.close()
