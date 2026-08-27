# 16×12 Sokoban Solver

[![Build and smoke test](https://github.com/anyexueli-gif/sokoban-solver/actions/workflows/build-smoke.yml/badge.svg)](https://github.com/anyexueli-gif/sokoban-solver/actions/workflows/build-smoke.yml)

<p align="right"><a href="README.md">中文</a> · English</p>

This project is the core Sokoban solver developed for the Intelligent Vision Group of the 21st National Undergraduate Smart Car Competition, and it can be ported directly to the RT1064. As a beginner, I relied heavily on AI tools for design, implementation, and debugging, so the project may still contain messy legacy code, undiscovered defects, and areas that need further refinement. It will continue to evolve; feedback, issue reports, and pull requests are welcome, and your understanding is appreciated.

### Highlights

- 16×12 bitboard maps with capacity for up to 10 boxes, 10 targets, and 5 bombs.
- A* navigation and box routing for multi-box, multi-target puzzles.
- Pushable bombs, blast handling, and dynamic removal of destructible walls.
- Regular target matching plus strict box/target ID matching.
- Scan-path generation, identification-point ID submission, and residual-state recovery APIs.
- `solver_solve_robust()` validates the replay of a generated path before reporting success.

The solver focuses on executable solutions and does not promise a globally shortest path. Map dimensions and capacities are defined in [`Driver/sokoban_types.h`](Driver/sokoban_types.h).

> **Capacity and runtime boundary:** The current core and PC demo configuration support up to 10 boxes, 10 targets, and 5 bombs. Whether a particular map finishes within a given time or memory budget still depends on its layout. The PC demo is a host-side runner and is not proof of RT1064 board performance.

### Repository layout

| Path | Description |
| --- | --- |
| [`Driver/`](Driver/) | Portable core: A*, state transitions, bomb rules, scanning, recovery, and cache interfaces |
| [`Driver/rt1064_porting_example.c`](Driver/rt1064_porting_example.c) | RT1064 porting example showing the integration call sequence; excluded from the default PC build |
| [`User/`](User/) | Windows build script, protocol entry point, Pygame demo, and PC Flash adapter |
| [`map/`](map/) | Example and custom maps |
| [`map_eligibility.py`](map_eligibility.py) | Map-capacity policy used by the demo |

### Quick start (Windows)

Requirements: PowerShell, GCC available as `gcc` on `PATH`, and Python 3.

```powershell
# Install the tested demo dependencies: Pygame plus optional serial and host-information support
python -m pip install -r .\requirements-demo.txt

# Build
powershell -NoProfile -ExecutionPolicy Bypass -File .\User\build_fast.ps1 --profile mcu-fast

# Run the Pygame demo
python .\User\demo.py
```

`requirements-demo.txt` pins the GUI dependencies tested with this repository and lists the optional serial and host-information dependencies.

`User/start_demo.ps1` is an alternative launcher. To run only the protocol process:

```powershell
.\User\main.exe
```

Build outputs (`main.exe`, `sokoban_solver.dll`, and intermediate files) are local runtime artifacts and are ignored by Git.

On first launch, `User/demo.py` automatically invokes `User/build_fast.ps1` for an incremental build. The Flash scan cache is disabled by default (`g_sokoban_flash_cache_enabled = 0`); the PC adapter creates the ignored `FLASH/scan_cache.bin` only when an integration layer explicitly enables it. Map editing and ordinary saves write to `map/`; the “export as original 12×16 format” action creates the ignored `othermap/` directory on demand. A parseable file whose name does not start with `map` may also be assigned a numbered map filename, so keep a backup of custom maps.

`Driver/rt1064_porting_example.c` is a standalone porting reference, not a directly runnable PC program, and it is excluded from the default build. Its `map1`/`map2` symbols and camera/motor hooks must be supplied by the target RT1064 firmware; the file documents how to connect scan, solve, and residual-recovery phases.

### Local validation and CI

After building, run `python -m py_compile User/demo.py map_eligibility.py` for a Python syntax check, then send `WARMUP` and `EXIT` to `User/main.exe` to smoke-test the protocol entry point. GitHub Actions runs equivalent Python checks, a GCC build, and the protocol smoke test for pushes and pull requests; see [`.github/workflows/build-smoke.yml`](.github/workflows/build-smoke.yml).

### Map format

`.txt` files under `map/` may contain a JSON string array or plain text rows. A normalized map has 16 rows of 12 ASCII characters; spaces are valid floor cells, so trailing spaces must be preserved.

Some historical source maps (for example, `map_2_b8_a1.txt`) use 12 rows of 16 columns. In that source format, `-` means floor and `*` means bomb; the demo rotates the map counter-clockwise into the 16×12 layout when loading it. If a source file has no `@`, the demo inserts the player at its preferred empty cell or the first available empty cell.

| Character | Meaning |
| --- | --- |
| `#` | Wall |
| Space | Floor |
| `@` | Player start |
| `$` | Unassigned box |
| `.` | Unassigned target |
| `*` | Box on target |
| `B` | Bomb |
| `0`–`9` | Box ID (strict mode) |
| `a`–`j` | Target ID 0–9 (strict mode) |

The C API receives rows joined with `|`: `row1|row2|...|row16`.

### Minimal C API example

```c
#include "sokoban_solver.h"

SokobanSolver *solver = solver_create();
const char *map = "row1|row2|...|row16";

if (solver && solver_load_map_from_string(solver, map) &&
    solver_solve_robust(solver)) {
    uint16_t length = 0;
    Direction *path = solver_get_solution(solver, &length);
    /* path[0..length) contains the direction steps. */
}

solver_destroy(solver);
```

For on-site identification, use `solver_generate_scan_path()`, `solver_assign_next_scan_id()`, and `solver_set_identified_solve_mode()`. For continuing from an observed residual state, use the `sokoban_recovery_*()` APIs.

### PC protocol entry point

`User/pc_main.c` reads one command per line from stdin and writes `RESP:<type>:<payload>`.

| Command | Purpose |
| --- | --- |
| `LOAD_MAP:<map-string>` | Load a map |
| `WARMUP` | Initialize solver lookup tables and runtime caches |
| `FLASH_CLEAR` | Clear the Flash scan cache (requires an enabled cache and adapter) |
| `START_SOLVE` | Solve directly |
| `START_SCAN` | Generate a scan path |
| `SCAN_ID:<0-9\|no\|?>` | Submit the current scan identification |
| `SET_ID_AT:x,y,id` | Set an entity ID at a coordinate |
| `FINALIZE_SCAN_IDS` | Finish ID assignment and enable strict mode |
| `RESET` / `EXIT` | Reset or exit |

### Optional serial demo

The demo uses 115200 baud by default. Select a port through an environment variable:

```powershell
$env:SOKOBAN_SERIAL_PORT = "COM3"
python .\User\demo.py
```

### Porting notes

- `Driver/` uses shared static workspaces and is intended to be called as a single serial instance; add a lock for multi-task callers.
- An MCU project must map the custom BSS sections used by `Driver` and provide a target-specific Flash adapter.
- C/H files under `Driver` retain their original CP936 encoding; verify that editors and formatters do not silently transcode them.
- Direction steps are `U`, `D`, `L`, and `R`; scan paths may also contain pause markers for identification points.
- Use `User/build_fast.ps1 --force-close` only after confirming that a project process is locking a build output; it force-terminates matching local processes and may discard unsaved work.

### License

This project is released under the [MIT License](LICENSE). Keep the included copyright and license notice when using, copying, modifying, or distributing it.
