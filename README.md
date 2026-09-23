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

- **ESP-IDF v5.5.1** (the version everything is built against), installed
  with Espressif's installer, [EIM](https://docs.espressif.com/projects/idf-im-ui/en/latest/):
  ```sh
  brew install python@3.12          # a Python IDF 5.5 supports
  brew tap espressif/eim
  brew install eim

  # Put brew's 3.12 first on PATH for this one command so EIM builds
  # the IDF venv with it. eim_config.toml (repo root) pins v5.5.1 + tools.
  PATH="$(brew --prefix python@3.12)/libexec/bin:$PATH" \
    eim install -c eim_config.toml -p ~/.espressif
  ```
  EIM prints an activation script path at the end (under
  `~/.espressif/tools/`). Source it in each shell that builds, via the
  repo wrapper (`activate_idf.ps1` is the Windows equivalent):
  ```sh
  source ./activate_idf.sh    # set IDF_ACTIVATE to override the script path
  ```
  The script points straight at the venv EIM built, so it keeps working when
  Homebrew's default `python3` changes. Use `eim list` / `eim select` to
  manage installed versions and `brew upgrade eim` to update the installer.
- **Submodule** (LovyanGFX, needed by `bedlift/` and `hostsim/`):
  ```sh
  git submodule update --init bedlift/components/LovyanGFX
  ```
- Host sim only: `brew install sdl2 cmake ninja`

### Dev-laptop notes (macOS)

- Homebrew's default `python3` (3.14) is too new for IDF 5.5, so the IDF
  venv is built on `python@3.12` at install time (above). After that, the
  Python on `PATH` doesn't matter. If a shell still has a *stale* IDF env
  from an earlier session, open a fresh one.
- `xcrun` defaults to the newest installed SDK (e.g. MacOSX27.0), which can be
  newer than the CLT linker supports. `hostsim/CMakeLists.txt` pins the
  `MacOSX.sdk` symlink (the CLT's canonical SDK, which always matches its
  linker) automatically — no manual step. Override with `SDKROOT` if needed.

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
