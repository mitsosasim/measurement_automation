
"""
Minimal HP 8720C VNA grabber (HPIB/GPIB) — converted from the working VNA.ipynb

What it does
------------
- Connects to the VNA over GPIB (HPIB) using PyVISA.
- Sets CENTER and SPAN.
- Chooses ASCII transfer (FORM4) and requests corrected complex trace data (OUTPDATA).
- Parses real,imag pairs into a complex array `z`.
- Builds a frequency axis from `CENTER_HZ ± SPAN_HZ/2`.
- Saves magnitude [dB] and phase [deg] to CSV.
- Saves a magnitude-vs-frequency plot to PDF/PNG.

Requirements
------------
- NI-VISA + NI-488.2 (or a VISA implementation that supports your GPIB adapter)
- Python packages: pyvisa, numpy, matplotlib
  pip install pyvisa numpy matplotlib

How to run
----------
- Edit the CONFIG section below (GPIB address, center/span, output file names).
- Run: python vna_8720c.py
  (You can also import and call main() from another script.)

Notes
-----
- The active measurement (e.g., S21 or S11) is whatever is currently selected on the VNA.
  This script does not change the measurement selection; it just reads the current trace.
- The instrument returns corrected data as real,imag pairs with OUTPDATA.
- FORM4 sets ASCII transfer on this family of VNAs.
"""

import time
import csv
from pathlib import Path

import numpy as np
import pyvisa
import matplotlib.pyplot as plt

# =========================
# ======== CONFIG =========
# =========================
ADDR = "GPIB0::16::INSTR"   # Do not change unless your VNA address differs
CSV  = "prova.csv"
GRAFICO_DB = "prova.pdf"    # Magnitude (dB) plot output
CENTER_HZ = 7.0e9           # Center frequency (Hz)
SPAN_HZ   = 2.0e9           # Span (Hz) — start = CENTER - SPAN/2, stop = CENTER + SPAN/2
TIMEOUT_MS = 12000          # VISA timeout (ms)
QUERY_DELAY_S = 5.0         # Small wait between write/read to let the VNA prepare data

def main():
    f_start = CENTER_HZ - SPAN_HZ / 2.0
    f_stop  = CENTER_HZ + SPAN_HZ / 2.0

    rm = pyvisa.ResourceManager()
    print("VISA resources:", rm.list_resources())

    # Open instrument
    inst = rm.open_resource(ADDR)
    inst.timeout = TIMEOUT_MS  # ms
    # Terminations (the working notebook had them commented; keep them default)
    # inst.write_termination = "\n"
    # inst.read_termination  = "\n"

    try:
        # Good hygiene: clear device buffers
        inst.clear()

        # Identify instrument (the notebook used write + read)
        inst.write("IDN?")
        model = inst.read()
        print("ID:", model.strip())

        # Set center/span
        inst.write(f"CENT {CENTER_HZ}")
        inst.write(f"SPAN {SPAN_HZ}")

        # Data format = ASCII on 8720C family (FORM4)
        inst.write("FORM4;")

        # Request corrected complex data: real,imag pairs
        inst.write("OUTPDATA")
        time.sleep(QUERY_DELAY_S)

        # Read raw bytes and decode to text
        raw = inst.read_raw().decode(errors="ignore")

        # Parse "real,imag" per line
        lines = raw.strip().split("\n")
        re_list = []
        im_list = []
        z_list = []

        for line in lines:
            # Each line is "real,imag"
            if not line.strip():
                continue
            re_s, im_s = line.split(",")
            re_v = float(re_s)
            im_v = float(im_s)
            re_list.append(re_v)
            im_list.append(im_v)
            z_list.append(re_v + 1j * im_v)

        z = np.array(z_list, dtype=np.complex128)
        print(f"Captured {z.size} complex points.")

        # Frequency axis — match number of points returned
        freq = np.linspace(f_start, f_stop, z.size)

        # Save CSV: freq_Hz, |S|_dB, angle_deg
        mag_dB = 20.0 * np.log10(np.abs(z))
        phase_deg = np.degrees(np.angle(z))

        csv_path = Path(CSV)
        with csv_path.open("w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["freq_Hz", "S_mag_dB", "S_phase_deg"])
            for fr, m, ph in zip(freq, mag_dB, phase_deg):
                w.writerow([fr, m, ph])
        print(f"Saved CSV: {csv_path.resolve()} ({z.size} rows)")

        # Plot #1: Real part vs frequency (as in the notebook)
        plt.figure(figsize=(8, 5))
        plt.plot(freq, np.real(z), linewidth=1.2)
        plt.xlabel("Frequency [Hz]")
        plt.ylabel("Real(S)")
        plt.title("Real part of S-parameter (current trace)")
        plt.grid(True)
        plt.tight_layout()
        plt.show(block=False)

        # Plot #2: Magnitude (dB) vs frequency and save
        plt.figure(figsize=(8, 5))
        plt.plot(freq, mag_dB, linewidth=1.2)
        plt.xlabel("Frequency [Hz]")
        plt.ylabel("|S| [dB]")
        plt.title("Magnitude of S-parameter (current trace)")
        plt.grid(True)
        # Optional y-limits used in the notebook for S11; comment out if not needed
        # plt.ylim(-32, 0)
        plt.tight_layout()
        out_path = Path(GRAFICO_DB)
        plt.savefig(out_path)
        print(f"Saved plot: {out_path.resolve()}")
        plt.show()

    finally:
        try:
            inst.close()
        except Exception:
            pass

if __name__ == "__main__":
    main()
