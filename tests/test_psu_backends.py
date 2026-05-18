import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parents[1]))

import pytest

from scripts.psu import N8957A, PSW720H88Lan, create_psu


class FakeInstrument:
    def __init__(self):
        self.timeout = None
        self.write_termination = None
        self.read_termination = None
        self.writes = []
        self.queries = []
        self.commands = []
        self.closed = False
        self.replies = {
            "*IDN?": "GW-INSTEK,PSW-720H88,TW000000,01.00\n",
            "SYST:ERR?": "0,\"No error\"\n",
            "OUTP? (@1)": "1\n",
            "OUTP? (@2)": "0\n",
            "SOUR:VOLT:LEV:IMM:AMPL? (@1)": "100\n",
            "SOUR:CURR:LEV:IMM:AMPL? (@1)": "1\n",
            "MEAS:SCAL:VOLT:DC? (@1)": "99.9\n",
            "MEAS:SCAL:CURR:DC? (@1)": "0.99\n",
            "MEAS:VOLT?": "10\n",
            "MEAS:CURR?": "2\n",
        }

    def clear(self):
        return None

    def close(self):
        self.closed = True

    def set_visa_attribute(self, *_args, **_kwargs):
        return None

    def write(self, cmd: str):
        self.writes.append(cmd)
        self.commands.append(cmd)

    def query(self, cmd: str) -> str:
        self.queries.append(cmd)
        self.commands.append(cmd)
        return self.replies.get(cmd, "0\n")


class FakeResourceManager:
    def __init__(self):
        self.last_resource_name = None
        self.last_instrument = None

    def open_resource(self, name: str):
        self.last_resource_name = name
        self.last_instrument = FakeInstrument()
        return self.last_instrument


def _install_fake_pyvisa(monkeypatch):
    fake_rm = FakeResourceManager()
    monkeypatch.setattr("scripts.psu.pyvisa.ResourceManager", lambda: fake_rm)
    return fake_rm


def test_keysight_legacy_commands_unchanged(monkeypatch):
    fake_rm = _install_fake_pyvisa(monkeypatch)
    psu = N8957A()

    psu.set_voltage(10.0)
    psu.set_current(2.0)
    psu.output_on()
    psu.measure_voltage()
    psu.measure_current()
    psu.output_off()

    assert fake_rm.last_instrument.commands == [
        "VOLT 10.0",
        "CURR 2.0",
        "OUTP ON",
        "MEAS:VOLT?",
        "MEAS:CURR?",
        "OUTP OFF",
    ]


def test_psw_channel_1_command_strings(monkeypatch):
    fake_rm = _install_fake_pyvisa(monkeypatch)
    psu = PSW720H88Lan(ip_address="192.168.1.123", channel=1)

    psu.set_voltage(100.0)
    psu.set_current(1.0)
    psu.output_on()
    psu.measure_voltage()
    psu.measure_current()
    psu.output_off()

    for cmd in [
        "SOUR:VOLT:LEV:IMM:AMPL 100,(@1)",
        "SOUR:CURR:LEV:IMM:AMPL 1,(@1)",
        "OUTP ON,(@1)",
        "MEAS:SCAL:VOLT:DC? (@1)",
        "MEAS:SCAL:CURR:DC? (@1)",
        "OUTP OFF,(@1)",
    ]:
        assert cmd in fake_rm.last_instrument.commands


def test_psw_rejects_invalid_current_before_output_on(monkeypatch):
    fake_rm = _install_fake_pyvisa(monkeypatch)
    psu = PSW720H88Lan(ip_address="192.168.1.123", channel=1)

    with pytest.raises(ValueError):
        psu.set_current(1.45)

    assert "OUTP ON,(@1)" not in fake_rm.last_instrument.commands


def test_psw_rejects_invalid_voltage_before_output_on(monkeypatch):
    fake_rm = _install_fake_pyvisa(monkeypatch)
    psu = PSW720H88Lan(ip_address="192.168.1.123", channel=1)

    with pytest.raises(ValueError):
        psu.set_voltage(801.0)

    assert "OUTP ON,(@1)" not in fake_rm.last_instrument.commands


def test_psw_rejects_invalid_channel(monkeypatch):
    _install_fake_pyvisa(monkeypatch)

    with pytest.raises(ValueError):
        PSW720H88Lan(ip_address="192.168.1.123", channel=3)




def test_psw_requires_non_empty_ip(monkeypatch):
    _install_fake_pyvisa(monkeypatch)

    with pytest.raises(ValueError, match="must not be empty"):
        PSW720H88Lan(ip_address="   ", channel=1)


def test_psw_requires_port_2268(monkeypatch):
    _install_fake_pyvisa(monkeypatch)

    with pytest.raises(ValueError, match="must be 2268"):
        PSW720H88Lan(ip_address="192.168.1.123", port=5025, channel=1)


def test_psw_uses_expected_socket_resource_and_lf_terminators(monkeypatch):
    fake_rm = _install_fake_pyvisa(monkeypatch)

    psu = PSW720H88Lan(ip_address="10.1.2.3", port=2268, channel=2)

    assert fake_rm.last_resource_name == "TCPIP0::10.1.2.3::2268::SOCKET"
    assert fake_rm.last_instrument.write_termination == "\n"
    assert fake_rm.last_instrument.read_termination == "\n"
    psu.close()


def test_psw_channel_2_only_uses_selected_channel(monkeypatch):
    fake_rm = _install_fake_pyvisa(monkeypatch)
    psu = PSW720H88Lan(ip_address="192.168.1.123", channel=2)

    psu.set_voltage(50.0)
    psu.set_current(0.5)
    psu.get_voltage_set()
    psu.get_current_set()
    psu.measure_voltage()
    psu.measure_current()

    joined = "\n".join(fake_rm.last_instrument.commands)
    assert "(@2)" in joined
    assert "(@1)" not in joined

def test_factory_returns_expected_backends(monkeypatch):
    _install_fake_pyvisa(monkeypatch)

    keysight = create_psu("keysight_n8957a")
    psw = create_psu("gwinstek_psw720h88_lan")

    for obj in (keysight, psw):
        for method_name in (
            "idn",
            "output_on",
            "output_off",
            "output_state",
            "set_voltage",
            "set_current",
            "get_voltage_set",
            "get_current_set",
            "voltage_limits",
            "current_limits",
            "measure_voltage",
            "measure_current",
            "close",
        ):
            assert hasattr(obj, method_name)
            assert callable(getattr(obj, method_name))
