# measurement_automation
Automated antenna characterization rig combining a stepping motor, a vector analyzer and a PSU

## Antenna Measurement Rig

Concise, single-file control program for automated antenna S-parameter measurements
using a stepper motor (Modbus RTU), a Keysight N8957A power supply (GPIB), and an
HP 8720C-family VNA (GPIB). The script provides a small Tkinter GUI to configure
scans, run sweeps, enforce simple safety checks, and save results as CSV.


## Features

- Motor control via Modbus RTU over RS-485 (positioning in steps/degree).
- PSU control and readback (program/measure voltage and current).
- VNA sweep capture (S11/S21) with corrected ASCII output parsing.
- Per-angle safety checks, logging, and CSV output with metadata.
- Minimal dependencies and clear configuration constants at the top of `antenna.py`.

## Requirements

- Python 3.12 (recommended)
- pip packages: requirements.txt 
	- Install with: `pip install -r requirements.txt`
- For GPIB: NI‑VISA and NI‑488.2 drivers (or equivalent) installed on the host.
- A USB→RS485 adapter for Modbus RTU and USB→GPIB adapter for the instruments.


## Configuration

Key variables:

- `PORT` — serial port for RS-485 (e.g. `COM10` on Windows).
- `BAUD` — serial baud rate for the motor controller.
- `DEV` — Modbus slave ID of the motor controller.
- `STEPS_PER_DEG` — microsteps per degree (calibrates position ↔ angle).
- `VNA_ADDR`, `PSU_ADDR` — VISA resource strings for GPIB instruments.
- Timeout and timing constants (`TIMEOUT_S`, `IDLE_GAP_S`, `POLL_EVERY_S`) can be tuned if you observe CRC/frame timeouts or slow responses.


## How it works

- The script constructs Modbus RTU PDUs, appends a CRC (computed by `_crc16`), and
	sends them to the motor controller. Responses are scanned and CRC-checked.
- 32-bit parameters (speed, acceleration, position) are split into two 16-bit
	registers using helpers (`_words_from_i32` / `_words_from_u32`) before writing.
- The VNA is configured via SCPI (VISA). The script requests ASCII-corrected data
	and parses each line as a (real,imag) pair to rebuild complex traces.
- Per-angle the program moves the motor, powers the DUT, validates PSU readbacks,
	acquires VNA traces (S11 and S21), re-checks the PSU, turns the PSU off, and
	writes a CSV with trace and metadata.
- The GUI launches and allows configuring start/end angles, step size, PSU setpoints,
    VNA sweep parameters, and the output directory. Output CSV files are written to the
    chosen directory (default `data/`) with names like `2.0A_10.0V_+090deg.csv`.


## Troubleshooting

- Serial/Modbus CRC failures: verify wiring, correct `PORT`, `BAUD`, device `DEV`,
	and that no other node is echoing bytes. If needed increase `TIMEOUT_S` or
	`IDLE_GAP_S` in `full.py`.
- GPIB/VISA errors: ensure NI‑VISA and NI‑488.2 (or your adapter vendor drivers)
	are installed and that the VISA resource strings match `VISA` manager output.
- Word/byte order: the code uses big-endian packing (high-word first). If the
	motor controller expects swapped 32-bit word order adjust the helpers.

<img width="690" height="966" alt="Screenshot 2025-11-24 103321" src="https://github.com/user-attachments/assets/f6a91a58-02b5-4486-8df0-34c62a2c1871" />
