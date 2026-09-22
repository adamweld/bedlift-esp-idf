# bedlift-esp-idf

Firmware for the bedlift van bed-lift: four CyberGear CAN winches + lock
solenoids, driven by an Adafruit ESP32-S3 Reverse TFT Feather on the custom
carrier PCB (see the `bedlift-pcb` repo).

## What's here

| Path | What it is |
|---|---|
| `bedlift/` | **Production app** (control system + UI). On-target currently boots to splash + driver selftest; the control/UI components run in the host sim while target task integration lands. |
| `bedlift/hostsim/` | **Host simulator** — the production UI, button logic, motion FSM, safety rules and CyberGear driver running against a virtual plant in an SDL window on macOS. No hardware needed. |
| `experiments/bringup/` | **Bring-up console** — serial REPL for exercising the board: SSR enables, I2C/IMU, halls, CAN/CyberGear (`cg ping/init/vel/stop`). The bench workhorse. |
| `experiments/display/` | Legacy UI prototype (reference only; superseded by `bedlift/`). |
| `examples/` | Vendor/upstream samples (`position_test` is the upstream cybergear example). |

## Prerequisites

- **ESP-IDF v5.5.1** (the version everything is built against):
  ```sh
  git clone --recursive -b v5.5.1 https://github.com/espressif/esp-idf ~/esp/esp-idf
  ~/esp/esp-idf/install.sh esp32s3
  ```
- **Submodule** (LovyanGFX, needed by `bedlift/` and `hostsim/`):
  ```sh
  git submodule update --init bedlift/components/LovyanGFX
  ```
- Host sim only: `brew install sdl2 cmake ninja`

### Dev-laptop quirks (macOS)

- `/usr/bin/python3` is a broken Xcode CLT shim on this machine; a working
  Python 3.13 is shimmed at `~/.local/idf-shim`. Activate IDF with:
  ```sh
  export PATH="$HOME/.local/idf-shim:/opt/homebrew/bin:$PATH"
  source ~/esp/esp-idf/export.sh
  ```
- The newest CLT SDK (MacOSX27.0) is incompatible with its own linker;
  `hostsim/CMakeLists.txt` pins `MacOSX26.5.sdk` automatically. If a host
  build fails at link on another setup, override with `SDKROOT`.

## Production app (`bedlift/`)

```sh
cd bedlift
idf.py set-target esp32s3        # first time only
idf.py -p /dev/cu.usbmodem1101 flash monitor
```

Boots safety-first (SSR/lock gates forced low), shows the splash, and runs
the CyberGear driver's Phase-0 selftest (~25 byte-exact frame checks) with
the verdict on screen and on the console. Monitor exits with `ctrl-]`.

## Bring-up console (`experiments/bringup/`)

```sh
cd experiments/bringup
idf.py -p /dev/cu.usbmodem1101 flash monitor    # REPL on the same USB-C
```

Type `help` at the `bedlift>` prompt. Highlights:

- `en motor|lock 0|1` — SSR enables (forced low at boot)
- `i2cscan`, `adxl id|read front|rear`, `adxl sdo 0|1` — IMUs (front 0x1D, rear 0x53)
- `hall [secs]` — endstop inputs with edge watch
- `can up [kbps] [--swap]`, `can status`, `can send`, `can dump` — TWAI
  (verified pins TX=IO39 RX=IO38 @ 1 Mbps)
- `cg ping|init|vel|stop <id>` — CyberGear (ids: M1=1 M2=4 M3=3 M4=2).
  **`cg vel` spins motors — never with the lift loaded/attached.**
- `buswire`, `i2cbb`, `i2creg` — electrical diagnostics

See `experiments/bringup/README.md` for the full table and bench rules
(notably: pull the J13 5V regulator module when USB and 12V/24V are
connected at the same time).

## Host simulator (`bedlift/hostsim/`)

```sh
cd bedlift/hostsim
cmake -B build -G Ninja .
cmake --build build
./build/hostsim
```

An SDL window (3x-scaled 240x135) runs the production UI against the
virtual plant — the real driver, button SM, motion FSM, safety rules and
panels; only the transport and physics are simulated.

**Keys** — buttons: `Up`/`Down` arrows, `M` = center (hold both arrows
200 ms for the mode chord; triple-click M toggles the debug table).
`TAB` flips to the bench view. Bench controls: `P` SSR, `K` lock rail,
`E` init motors, `SPACE` stop, `7/8/9/0` select M1..M4 then
`u/o/d/c/s/w` inject undervolt/overcurrent/driver/comm-loss/snag/runaway,
`x` clears. Avoid `L`/`R` and `1-6` (SDL window rotate/scale).
Console prints 2 Hz telemetry while anything moves.

**Headless tests / tools:**

```sh
./build/hostsim --test-level      # disturb pitch/roll/twist, then self-level:
                                  # must auto-complete + lock within 1 deg
./build/hostsim --test-travel     # tilt trim must cancel drift during a raise
./build/hostsim --snap <dir>      # render every UI screen to BMP for review
```

## Ground truth (bench-verified)

- CAN: TWAI **TX=IO39, RX=IO38**, 1 Mbps (PCB net names were right; early
  firmware had it swapped). Transceiver: M5Stack CAN unit, straight Grove
  cable on J3.
- Halls: physical hall-1 = IO11, hall-2 = IO9 (PCB net/silk crossed).
- I2C: SDA=IO3 SCL=IO4, and **IO7 must be driven high** or the rail is dead.
- Motor CAN ids are non-sequential: M1=0x01, M2=0x04, M3=0x03, M4=0x02.
- The carrier back-feeds the Feather via its USB pin from the 12V->5V
  regulator: USB-C and board power together only with the J13 module pulled.
