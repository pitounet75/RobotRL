# Reminder – Compiler le firmware ODrive (v0.5.1)

Aide-mémoire perso. Détails complets : `GettingStarted.md` dans ce dossier.

## TL;DR

```cmd
cd H:\Projects\RobotRL\ODrive\ODrive_S-fw-v0.5.1\Firmware
SetBuildEnv.cmd noexit   REM prépare le PATH (ARM GCC, make, Lua, tup)
tup init                 REM une seule fois
make                     REM compile (= tup --quiet --no-environ-check)
```

Sorties dans `build/` :
- `build/ODriveFirmware.elf`
- `build/ODriveFirmware.hex`
- `build/ODriveFirmware.bin`

## Où on travaille

Toutes les commandes se lancent depuis le dossier **`Firmware/`** :

```
H:\Projects\RobotRL\ODrive\ODrive_S-fw-v0.5.1\Firmware
```

`make` n'est qu'un stub : il appelle `tup`, qui lit `Tupfile.lua` (d'où la
dépendance à Lua). Le premier build est long : tup régénère les headers C++
sous `autogen/` à partir de `odrive-interface.yaml`.

## Prérequis (à installer une fois)

Tous doivent être **sur le PATH** dans le terminal utilisé pour `make` :

| Outil | Version / archive | Rôle |
|-------|-------------------|------|
| GNU Arm Embedded | `gcc-arm-none-eabi-7-2018-q2-update-win32` | compilateur `arm-none-eabi-gcc` |
| tup | `tup-latest.zip` (Makerbase) ou gittup.org | système de build |
| make | GNU MCU Eclipse Windows Build Tools 2.12 | lance tup + cibles flash/dfu |
| Lua | **5.5.x** (`lua.exe`) | tup évalue `Tupfile.lua` |
| Python 3 | python.org (pas le shim Microsoft Store) | codegen Fibre |
| Paquets Python | `pip install PyYAML Jinja2 jsonschema` | générateur d'interface |

### SetBuildEnv.cmd

Ce script (dans `Firmware/`) prépend au PATH les outils rangés dans le dossier
parent `ODrive/` (deux niveaux au-dessus de `Firmware/`) :

```
ODrive\
  gcc-arm-none-eabi-7-2018-q2-update-win32\bin\
  gnu-mcu-eclipse-windows-build-tools-2.12-20190422-1053-win64\...\bin\
  lua-5.5.0_Win32\bin\
  tup\tup.exe   (ou ODrive\tup.exe)
```

Usage :
- `SetBuildEnv.cmd` → ouvre un nouveau `cmd` avec le PATH prêt
- `SetBuildEnv.cmd noexit` → applique le PATH dans le prompt courant (à préférer
  pour enchaîner avec `make`)

Si un outil manque, le script affiche un `WARNING: ... not found`.

## Config board – tup.config

Fichier `Firmware/tup.config`. La ligne critique doit correspondre au matériel :

```text
CONFIG_BOARD_VERSION=v3.6-56V
CONFIG_USB_PROTOCOL=native
CONFIG_UART_PROTOCOL=ascii
CONFIG_DEBUG=false
CONFIG_DOCTEST=false
#CONFIG_STRICT=true   # décommenter pour transformer les warnings en erreurs
```

Valeurs de board valides : celles gérées dans `Tupfile.lua`
(`v3.1` … `v3.6-56V`, `v3.4-24V`, `v3.5-48V`, …).

## Build

```cmd
make
```

Si `make` fait des siennes sous Windows, lancer tup directement (après
`tup init` + `tup.config` valide) :

```cmd
tup --quiet --no-environ-check
```

Nettoyage :

```cmd
make clean   REM supprime .dep et build/
```

## Flash (optionnel)

Depuis `Firmware/`, après un build OK :

```cmd
make flash   REM SWD : OpenOCD + ST-Link v2  (cible stm32f4x)
make dfu     REM DFU : device en mode DFU, via ../tools/odrivetool
make gdb     REM debug : arm-none-eabi-gdb + openocd.gdbinit
```

Autres cibles utiles dans le `Makefile` : `erase`, `erase_config`, `unlock`,
`flashbmp` / `bmp` (Black Magic Probe), `write_otp` (irréversible – ne pas
toucher sur une v3.5+).

## Dépannage

| Symptôme | Cause / solution |
|----------|------------------|
| `board version not specified` | `CONFIG_BOARD_VERSION` absent/faux dans `tup.config` |
| `Python 3 not found` | Python 3 pas sur le PATH depuis un terminal normal (pas juste l'IDE) |
| Import errors pendant la codegen | `pip install PyYAML Jinja2 jsonschema` dans l'env utilisé par tup |
| Erreurs `Tupfile.lua` | Lua 5.5 absent du PATH |
| `make` capricieux | lancer `tup --quiet --no-environ-check` directement |
