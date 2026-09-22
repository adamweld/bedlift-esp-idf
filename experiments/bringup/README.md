# bringup — carrier-board bring-up console

Serial REPL for exercising each subsystem of the bedlift carrier PCB in
isolation. Console runs over the Feather's native USB-C (same cable as
flashing). MOTOR_EN and LOCK_EN are forced **low** at boot.

```
idf.py set-target esp32s3
idf.py build flash monitor
```

Bench rule: the carrier back-feeds the Feather through its USB pin from the
12V→5V regulator. When the console (USB-C) and board power are needed at the
same time, leave the J13 regulator module unsocketed so the two 5V sources
never meet.

## Commands

| Command | What it does |
|---|---|
| `en motor 1` / `en lock 0` / `en status` | Toggle/read the SSR enables (IO5/IO6) |
| `i2cscan` | Probe 0x03–0x77 on SDA=IO3 SCL=IO4 |
| `adxl id [front\|rear]` | Check ADXL345 DEVID (0xE5) at 0x1D/0x53 |
| `adxl read [front\|rear] [n]` | Stream n acceleration samples |
| `adxl sdo <0\|1>` | Drive IO8 address strap (1 → J7 device at 0x1D) |
| `i2cfreq [khz]` | Get/set bus speed (default 100 kHz) |
| `i2creg <addr> <reg> [n]` | Generic register read via the hardware controller |
| `i2cbb <addr> <reg> [n]` | Bit-banged register read (bypasses the controller; prints per-phase ACKs) |
| `i2creset` | Drop/re-create the I2C bus handle |
| `hall [secs]` | Read hall inputs IO9/IO11; optionally watch for edges |
| `can up [kbps] [--swap]` | Start TWAI (default 1000 kbps, TX=38 RX=39; `--swap` reverses) |
| `can status` | Error counters + alerts (the DMM-less way to spot a bad bus) |
| `can send <id-hex> [bytes…]` / `can dump [ms]` | Raw frame TX / RX sniff |
| `cg ping <id>` / `cg stop <id>` | CyberGear liveness check / reset-stop |
| `gpio <pin> [0\|1]` | Raw pin read/write escape hatch |
| `btn` | Read on-module buttons D0/D1/D2 |

CAN wiring note (bench-verified 2026-09-22): **TX=IO39 / RX=IO38** — the PCB
net names were correct, and the firmware default now matches. All four
CyberGears ping and init cleanly with this config through the M5Stack CAN
unit (CA-IS3050G) on a straight Grove cable. If a future harness misbehaves
(`can status` shows bus_err/tx_err climbing to BUS_OFF), `can down` /
`can up --swap` tests the opposite pinning in seconds.
