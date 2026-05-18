import pyvisa
import time
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
    """
    GW Instek PSW-720H88 LAN backend.

    Controls one selected channel only.
    PSW-720H88 channel rating:
      CH1: 0-800 V, 0-1.44 A
      CH2: 0-800 V, 0-1.44 A
    """

    V_MIN = 0.0
    V_MAX = 800.0
    I_MIN = 0.0
    I_MAX = 1.44

    def __init__(
        self,
        ip_address: str = GWINSTEK_PSW720H88_DEFAULT_IP,
        port: int = GWINSTEK_PSW720H88_DEFAULT_PORT,
        channel: int = GWINSTEK_PSW720H88_DEFAULT_CHANNEL,
        timeout_ms: int = 10000,
        strict_idn: bool = True,
    ):
        if channel not in (1, 2):
            raise ValueError("PSW-720H88 channel must be 1 or 2.")

        ip_address = str(ip_address).strip()
        if not ip_address:
            raise ValueError("PSW-720H88 LAN IP address must not be empty.")

        if int(port) != 2268:
            raise ValueError("PSW-720H88 socket server port must be 2268.")

        self.ip_address = ip_address
        self.port = int(port)
        self.channel = int(channel)
        self.resource_name = f"TCPIP0::{self.ip_address}::{self.port}::SOCKET"

        rm = pyvisa.ResourceManager()
        self.inst = rm.open_resource(self.resource_name)
        self.inst.timeout = timeout_ms
        self.inst.write_termination = "\n"
        self.inst.read_termination = "\n"

        try:
            self.inst.set_visa_attribute(pyvisa.constants.VI_ATTR_TERMCHAR, ord("\n"))
            self.inst.set_visa_attribute(pyvisa.constants.VI_ATTR_TERMCHAR_EN, True)
        except Exception:
            pass

        self.clear()

        if strict_idn:
            idn = self.idn()
            idn_upper = idn.upper()
            if "GW" not in idn_upper and "INSTEK" not in idn_upper:
                raise RuntimeError(f"Expected GW Instek PSW supply, got IDN: {idn!r}")
            if "PSW" not in idn_upper:
                raise RuntimeError(f"Expected PSW model, got IDN: {idn!r}")

    def _chan_set(self) -> str:
        return f",(@{self.channel})"

    def _chan_query(self) -> str:
        return f" (@{self.channel})"

    def clear(self) -> None:
        try:
            self.inst.clear()
        except Exception:
            pass
        try:
            self.inst.write("*CLS")
        except Exception:
            pass

    def idn(self) -> str:
        return self.inst.query("*IDN?").strip()

    def _query_float(self, cmd: str) -> float:
        reply = self.inst.query(cmd).strip()
        try:
            return float(reply)
        except ValueError as exc:
            raise RuntimeError(f"Invalid numeric reply for {cmd!r}: {reply!r}") from exc

    def _query_bool(self, cmd: str) -> bool:
        reply = self.inst.query(cmd).strip()
        try:
            return bool(int(float(reply)))
        except ValueError as exc:
            raise RuntimeError(f"Invalid boolean reply for {cmd!r}: {reply!r}") from exc

    def _check_error_queue(self) -> None:
        try:
            reply = self.inst.query("SYST:ERR?").strip()
        except Exception:
            return

        first = reply.split(",", 1)[0].strip()
        try:
            code = int(float(first))
        except ValueError:
            return

        if code != 0:
            raise RuntimeError(f"GW Instek PSW SCPI error after command: {reply}")

    def _validate_voltage(self, volts: float) -> None:
        if not (self.V_MIN <= float(volts) <= self.V_MAX):
            raise ValueError(
                f"Requested voltage {volts:.6g} V is outside PSW-720H88 "
                f"channel {self.channel} software range {self.V_MIN}..{self.V_MAX} V."
            )

    def _validate_current(self, amps: float) -> None:
        if not (self.I_MIN <= float(amps) <= self.I_MAX):
            raise ValueError(
                f"Requested current {amps:.6g} A is outside PSW-720H88 "
                f"channel {self.channel} software range {self.I_MIN}..{self.I_MAX} A."
            )

    def set_voltage(self, volts: float) -> None:
        self._validate_voltage(volts)
        self.inst.write(f"SOUR:VOLT:LEV:IMM:AMPL {float(volts):.12g}{self._chan_set()}")
        self._check_error_queue()

    def set_current(self, amps: float) -> None:
        self._validate_current(amps)
        self.inst.write(f"SOUR:CURR:LEV:IMM:AMPL {float(amps):.12g}{self._chan_set()}")
        self._check_error_queue()

    def get_voltage_set(self) -> float:
        return self._query_float(f"SOUR:VOLT:LEV:IMM:AMPL?{self._chan_query()}")

    def get_current_set(self) -> float:
        return self._query_float(f"SOUR:CURR:LEV:IMM:AMPL?{self._chan_query()}")

    def voltage_limits(self) -> Tuple[float, float]:
        return self.V_MIN, self.V_MAX

    def current_limits(self) -> Tuple[float, float]:
        return self.I_MIN, self.I_MAX

    def output_on(self) -> None:
        self.inst.write(f"OUTP ON{self._chan_set()}")
        self._check_error_queue()
        for _ in range(10):
            if self.output_state():
                return
            time.sleep(0.1)
        raise RuntimeError(f"PSW-720H88 channel {self.channel} did not report output ON.")

    def output_off(self) -> None:
        self.inst.write(f"OUTP OFF{self._chan_set()}")
        self._check_error_queue()

    def output_state(self) -> bool:
        return self._query_bool(f"OUTP?{self._chan_query()}")

    def measure_voltage(self) -> float:
        return self._query_float(f"MEAS:SCAL:VOLT:DC?{self._chan_query()}")

    def measure_current(self) -> float:
        return self._query_float(f"MEAS:SCAL:CURR:DC?{self._chan_query()}")

    def close(self) -> None:
        try:
            self.inst.close()
        except Exception:
            pass


class DualPSW720H88Lan:
    def __init__(
        self,
        ip_address: str = GWINSTEK_PSW720H88_DEFAULT_IP,
        port: int = GWINSTEK_PSW720H88_DEFAULT_PORT,
        timeout_ms: int = 10000,
    ):
        self.ch1 = PSW720H88Lan(ip_address=ip_address, port=port, channel=1, timeout_ms=timeout_ms)
        self.ch2 = PSW720H88Lan(ip_address=ip_address, port=port, channel=2, timeout_ms=timeout_ms)

    def idn(self) -> str:
        return f"CH1={self.ch1.idn()} | CH2={self.ch2.idn()}"

    def output_on(self) -> None:
        self.ch1.output_on()
        try:
            self.ch2.output_on()
        except Exception:
            try:
                self.ch1.output_off()
            except Exception:
                pass
            raise

    def output_off(self) -> None:
        errors = []
        for channel in (self.ch1, self.ch2):
            try:
                channel.output_off()
            except Exception as exc:
                errors.append(str(exc))
        if errors:
            raise RuntimeError(" ; ".join(errors))

    def output_state(self) -> bool:
        return self.ch1.output_state() and self.ch2.output_state()

    def set_voltage(self, volts: float) -> None:
        self.ch1.set_voltage(volts)
        self.ch2.set_voltage(volts)

    def set_current(self, amps: float) -> None:
        self.ch1.set_current(amps)
        self.ch2.set_current(amps)

    def get_voltage_set(self) -> float:
        return self.ch1.get_voltage_set()

    def get_current_set(self) -> float:
        return self.ch1.get_current_set()

    def voltage_limits(self) -> Tuple[float, float]:
        return self.ch1.voltage_limits()

    def current_limits(self) -> Tuple[float, float]:
        return self.ch1.current_limits()

    def measure_voltage(self) -> float:
        return self.ch1.measure_voltage()

    def measure_current(self) -> float:
        return self.ch1.measure_current()

    def measure_voltage_both(self) -> Tuple[float, float]:
        return self.ch1.measure_voltage(), self.ch2.measure_voltage()

    def measure_current_both(self) -> Tuple[float, float]:
        return self.ch1.measure_current(), self.ch2.measure_current()

    def close(self) -> None:
        errors = []
        for channel in (self.ch1, self.ch2):
            try:
                channel.close()
            except Exception as exc:
                errors.append(str(exc))
        if errors:
            raise RuntimeError(" ; ".join(errors))


def create_psu(
    psu_kind: PsuKind,
    *,
    keysight_addr: str = KEYSIGHT_N8957A_ADDR,
    gwinstek_ip: str = GWINSTEK_PSW720H88_DEFAULT_IP,
    gwinstek_port: int = GWINSTEK_PSW720H88_DEFAULT_PORT,
    gwinstek_channel: int = GWINSTEK_PSW720H88_DEFAULT_CHANNEL,
    gwinstek_channel_selection: Optional[str] = None,
    timeout_ms: int = 10000,
) -> PsuDevice:
    if psu_kind == "keysight_n8957a":
        return N8957A(addr=keysight_addr, timeout_ms=timeout_ms)

    if psu_kind == "gwinstek_psw720h88_lan":
        if gwinstek_channel_selection is not None:
            if gwinstek_channel_selection == "both":
                return DualPSW720H88Lan(
                    ip_address=gwinstek_ip,
                    port=gwinstek_port,
                    timeout_ms=timeout_ms,
                )
            if gwinstek_channel_selection in ("1", "2"):
                gwinstek_channel = int(gwinstek_channel_selection)
            else:
                raise ValueError(f"Unsupported PSW channel selection: {gwinstek_channel_selection!r}")
        return PSW720H88Lan(
            ip_address=gwinstek_ip,
            port=gwinstek_port,
            channel=gwinstek_channel,
            timeout_ms=timeout_ms,
        )

    raise ValueError(f"Unsupported PSU type: {psu_kind!r}")

if __name__ == "__main__":
    psu = create_psu("keysight_n8957a")
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
