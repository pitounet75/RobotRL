# Building this firmware

The firmware is built with **[tup](http://gittup.org/tup/)** and **GNU Arm Embedded** (`arm-none-eabi-gcc`). A small `Makefile` in this folder forwards `make` to tup and adds optional flash / DFU helpers.

Upstream reference: [ODrive developer guide](https://docs.odriverobotics.com/developer-guide).

## Prerequisites

Install the following software before building. On **Windows**, Makerbase groups several of these downloads in one place (see below); the ARM compiler is still taken from Arm’s site unless you mirror it yourself.

### Makerbase “VS Code build components” bundle (Windows)

Makerbase maintains a folder of zipped tools tuned for their ODrive workflow:

[04_VSCODE_Build components in development environment](https://github.com/makerbase-mks/ODrive-MKS/tree/main/03_Makerbase%20ODrive%20related%20components/04_VSCODE_Build%20components%20in%20development%20environment)

From that directory, download and install or extract as needed:

| Archive | Purpose |
|--------|---------|
| `tup-latest.zip` | **tup** build system — put `tup.exe` on your **PATH**. |
| `gnu-mcu-eclipse-windows-build-tools-2.12-20190422-1053-win64.zip` | **Make** and other common build utilities on Windows (the `Makefile` in this folder expects `make`). |
| `OpenOCD.zip` | **OpenOCD** — used by `make flash` (SWD + ST-Link style workflow). |
| `ODrive-fw-v0.5.1.zip` | Makerbase’s packaged **ODrive v0.5.1** firmware tree — useful as a reference or if you align with their layout; if you already use **this** git checkout as `Firmware`, you do not need a second copy of the sources for compiling here. |
| `04_VSCODE_Build components in development environment.txt` | Step-by-step notes for **VS Code** and PATH setup — follow this for IDE integration and any extra paths Makerbase documents. |

Other zips in the same folder (for example ST-Link drivers) may be required for debugging or flashing, depending on your hardware.

### Required tools (install first)

1. **GNU Arm Embedded toolchain — Windows 32-bit archive** (`gcc-arm-none-eabi-7-2018-q2-update-win32`)
   - Download: [gcc-arm-none-eabi-7-2018-q2-update-win32.zip](https://developer.arm.com/-/media/files/downloads/gnu-rm/7-2018q2/gcc-arm-none-eabi-7-2018-q2-update-win32.zip?rev=9b83e69ce2794d169401990c836197b3&revision=9b83e69c-e279-4d16-9401-990c836197b3?product=Downloads,ZIP,,Windows,7-2018-q2-update)
   - Extract the archive and add the toolchain **`bin`** directory (the folder that contains `arm-none-eabi-gcc.exe`) to your **PATH** so `arm-none-eabi-gcc` is available in the shell you use for `make` / tup.

2. **Python 3** on your `PATH` (tup invokes it for code generation from `odrive-interface.yaml`).
   - On Windows, use a real Python install (e.g. from python.org). The Microsoft Store shim can confuse tup’s Python detection.

3. **Python packages** for the Fibre interface generator (used during the build):
   ```bash
   pip install PyYAML Jinja2 jsonschema
   ```

4. **tup** on your **PATH** — use `tup-latest.zip` from the [Makerbase folder above](https://github.com/makerbase-mks/ODrive-MKS/tree/main/03_Makerbase%20ODrive%20related%20components/04_VSCODE_Build%20components%20in%20development%20environment), or install from [gittup.org/tup](http://gittup.org/tup/).

5. **Make** (Windows) — use the GNU MCU Eclipse build tools archive from the same Makerbase folder so `make` works in the environment where you run the firmware build.

6. **Lua 5.5** on your **PATH** — tup evaluates **`Tupfile.lua`** with Lua; on Windows, install **Lua 5.5.x** and ensure **`lua.exe`** is available in the same environment as **`tup`** (add its directory to **PATH**). Do **not** commit Lua binaries into this repository.
   - Releases and download area: [Lua version history](https://www.lua.org/versions.html) (follow the links to the **download area** for your platform; use **Lua 5.5.x**).
   - Extract or install so `lua.exe` is on **PATH**.

Optional: install the repo’s Python tools from `../tools` if you use odrivetool / DFU targets from the `Makefile` (see `../tools/requirements.txt`).

## One-time setup

From **this directory** (`Firmware`):

1. Ensure **`tup.config`** exists and matches your hardware. Edit the file in this folder if needed; key line is the board profile:
   ```text
   CONFIG_BOARD_VERSION=v3.6-56V
   ```
   Valid values are the ones handled in `Tupfile.lua` (e.g. `v3.1` … `v3.6-56V`, `v3.4-24V`, `v3.5-48V`, …). **Must match your hardware.**
2. Initialize tup here if you have not already:
   ```bash
   tup init
   ```

Other `CONFIG_*` entries in `tup.config` (USB/UART protocol, debug flags, etc.) follow the same pattern as stock ODrive firmware.

## Build

From **this directory**:

```bash
make
```

This runs `tup --quiet --no-environ-check`. The first build may take a while; tup regenerates C++ headers under `autogen/` from the YAML definitions.

### Outputs

- **`build/ODriveFirmware.elf`**
- **`build/ODriveFirmware.hex`**

(Exact paths are under the `build/` tree tup produces.)

## Flashing (optional)

With SWD (OpenOCD + ST-Link), from this directory after a successful build:

```bash
make flash
```

DFU (after putting the device in DFU mode), from this directory:

```bash
make dfu
```

See the `Makefile` for `gdb`, `erase`, `erase_config`, and other targets.

## Troubleshooting

- **`board version not specified`**: Add or fix `CONFIG_BOARD_VERSION` in `tup.config`.
- **`Python 3 not found`**: Install Python 3 and ensure `python` or `python3` runs **Python 3** from a normal terminal (not only from an IDE).
- **Import errors during codegen**: Install `PyYAML`, `Jinja2`, and `jsonschema` in the environment tup uses.
- **Windows path or tup issues**: If `make` is awkward, you can run `tup --quiet --no-environ-check` directly from this folder after `tup init` and a valid `tup.config`.
- **Lua / `Tupfile.lua` errors**: Ensure **Lua 5.5** is installed and on `PATH` (see prerequisites); tup needs `lua` to load `Tupfile.lua`.
