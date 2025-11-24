import pyvisa
from typing import Tuple

PSU_ADDR = "GPIB0::5::INSTR"

class N8957A:
    """
    Minimal Keysight N8957A helper for:
      - set/get programmed voltage & current
      - measure actual voltage & current
      - output on/off and state query
    SCPI per Keysight N8900 Series OSG.
    """
    def __init__(self, addr: str = PSU_ADDR, timeout_ms: int = 10000):
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
