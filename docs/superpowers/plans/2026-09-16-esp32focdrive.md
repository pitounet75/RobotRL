# ESP32FOCDrive Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Construire `ESP32FOCDrive`, firmware FOC voltage à deux axes pour la carte MKS ESP32 FOC V2.0, en repartant de SimpleFOC 2.3.3 natif et en ne réintroduisant que les mécanismes validés par `ESP32FOCHardwareCheck`.

**Architecture:** La tâche FOC tourne seule sur le core 0, cadencée par un timer matériel, et possède les encodeurs et les moteurs. Le core 1 porte la ligne de commande et l'OTA, et parle au core 0 uniquement par l'API `drive_api.h`, avec failsafe et instantané d'état. Toute la logique pure est en en-têtes autonomes, testables par Unity sur la cible.

**Tech Stack:** PlatformIO `espressif32 @ 7.0.1`, framework Arduino (arduino-esp32 2.0.17, IDF 4.4), `askuric/Simple FOC @ 2.3.3`, PCNT et timer group legacy IDF, NVS via `Preferences`, `ArduinoOTA`.

**Spec:** `docs/superpowers/specs/2026-09-16-esp32focdrive-design.md`

## Global Constraints

- **Matériel :** MKS ESP32 FOC V2.0, deux MKS FS2804 (12N14P, `FOC_POLE_PAIRS = 7`), deux MT6835 en ABZ, bus 3S (`FOC_VBUS = 12.6`).
- **Broches, axe gauche (moteur 0) :** PWM 32/33/25, ABZ A18 / B19 / Z22, PCNT unit 0.
- **Broches, axe droit (moteur 1) :** PWM 26/27/14, ABZ A5 / B23 / Z21, PCNT unit 1.
- **`M_EN` = GPIO12, commun aux deux axes.** Broche de strap : tenue basse avant tout le reste. Les drivers sont construits **sans** enable pin.
- **Aucun sense de courant, aucun anticogging.** Couple voltage uniquement, `tq` en volts.
- **`ENC_PPR = 16384`**, PCNT en quadrature complète, donc CPR = 65536.
- **`ENC_VEL_MIN_DT = 0.001f`** (1 ms).
- **Cadence FOC :** `FOC_LOOP_HZ = 4000`, bornes 4000 à 16000.
- **Langue :** commentaires et identifiants **en anglais** dans le code C++ (`.h`, `.cpp`), comme le reste du dépôt. Les fichiers de configuration (`platformio.ini`) et la documentation (`README.md`) sont **en français**, comme leurs équivalents dans `ESP32FOCHardwareCheck`.
- **Tests :** toute logique testée vit dans un en-tête autonome de `include/`, sans `#include <Arduino.h>`. Les vérifications sont centralisées dans `include/self_test.h` (tâche 12) et lancées de deux façons : la commande **`selftest`** de l'application, qui arrive par OTA et est le chemin normal, et la suite Unity sur cible (`pio test -e dual -f test_logic`), secondaire. Il n'y a pas de compilateur hôte sur cette machine, donc pas d'env `native`. **Aucune tâche postérieure à la 12 n'écrit de test Unity séparé** : elle ajoute ses vérifications à `self_test.h`.
- **Flash :** cette carte a un défaut, l'auto-reset par DTR/RTS ne fonctionne pas et tout flash USB impose un geste physique de l'humain. L'OTA est donc le chemin normal, et un firmware dont l'OTA est cassé coûte cher.
- **Sans carte branchée**, le garde-fou minimal est la compilation : `pio run -e left`, `pio run -e right`, `pio run -e dual`.
- **Chaque commit** se termine par les lignes d'attribution de la session en cours (`Co-Authored-By` et `Claude-Session`).
- **`platformio.local.ini` n'est jamais commité** ; il est déjà couvert par `.gitignore` via `*.local.ini`.

## Ordre d'exécution

**1, 2, 8, 3, 4, 12, puis 5, 6, 7, 9, 10, 11.** La numérotation des tâches ne change pas ; seul l'ordre change.

La tâche 12 est ajoutée en cours de route, après la tâche 4. Raison : la carte a un défaut matériel qui impose de maintenir BOOT et d'appuyer sur RESET pour **tout** flash USB, et le binaire Unity n'embarque pas de serveur OTA. Exécuter les tests coûtait donc deux gestes physiques : un pour téléverser les tests, un pour revenir à l'application. La tâche 12 expose les mêmes vérifications derrière une commande `selftest` de l'application, validable en OTA.

La tâche 8, réseau et OTA, est exécutée **en troisième**, juste après la ligne de commande. Sans elle, chaque itération imposerait un flash USB, pénible sur cette carte à cause du CH340 qui pulse DTR. Une fois la tâche 8 en place, tous les uploads suivants passent par OTA.

Trois conséquences, déjà intégrées dans le texte des tâches concernées :
- **Tâche 8 :** à cette position, ni `driveStop` ni `focPauseTimer` n'existent. `onStart` arrête les moteurs par la couture `mainIdle(0b11)` de la tâche 2 et ne met aucun timer en pause.
- **Tâche 4 :** ajoute la mise en pause et la reprise du timer dans `net.cpp`, une fois le timer créé.
- **Tâche 7 :** remplace `mainIdle(0b11)` par la boucle `driveStop`, dans le même mouvement que la suppression des autres coutures.

## Structure des fichiers

| Fichier | Responsabilité |
|---|---|
| `platformio.ini` | envs `left` / `right` / `dual` et variantes `_ota`, section `[wifi]` |
| `include/config.h` | masque d'axes, table de broches, gains, limites, modulation |
| `include/enc_math.h` | **en-tête pur** : dépliage 16 bits, compteur vers angle |
| `include/cmd_parse.h` | **en-tête pur** : découpage d'une ligne de commande, préfixe d'axe |
| `include/failsafe.h` | **en-tête pur** : expiration d'une consigne, avec débordement de `millis()` |
| `include/cal_record.h` | **en-tête pur** : `CalRecord` et sa validation |
| `include/pcnt_encoder.h`, `src/pcnt_encoder.cpp` | PCNT, ISR Z par instance, diagnostics |
| `include/board.h`, `src/board.cpp` | `M_EN` par compteur, broches au repos, entrée en mode download |
| `include/axis.h`, `src/axis.cpp` | `Axis`, modes, limites, calibration, NVS |
| `include/foc_task.h`, `src/foc_task.cpp` | timer, tâche core 0, métriques, failsafe, `vcap`, état publié |
| `include/drive_api.h`, `src/drive_api.cpp` | consignes et état, frontière entre les cores |
| `include/cli.h`, `src/cli.cpp` | lecture série, exécution des commandes, `status` |
| `include/net.h`, `src/net.cpp` | WiFi, OTA |
| `src/main.cpp` | `setup()` et `loop()` |
| `include/self_test.h` | **en-tête pur** : toutes les vérifications, un seul jeu d'assertions |
| `test/test_logic/test_main.cpp` | exécuteur Unity, délègue à `self_test.h` |
| `scripts/` | `wrap_uploader.py`, `esptool_nodtr.py` |
| `README.md` | câblage, séquence de mise au point, commandes |

Les quatre en-têtes purs sont la seule logique testable hors matériel. Tout le reste est validé au banc, avec des critères explicites.

---

### Task 1 : squelette de projet, `M_EN`, open-loop SimpleFOC natif

**Files:**
- Create: `ESP32FOCDrive/platformio.ini`
- Create: `ESP32FOCDrive/include/config.h`
- Create: `ESP32FOCDrive/include/board.h`, `ESP32FOCDrive/src/board.cpp`
- Create: `ESP32FOCDrive/src/main.cpp`
- Create: `ESP32FOCDrive/scripts/wrap_uploader.py`, `ESP32FOCDrive/scripts/esptool_nodtr.py`

**Interfaces:**
- Consomme : rien.
- Produit : `boardInit()`, `boardMotorPowerRef(int delta)`, `boardMotorPowered()`, `boardEnterDownload()`, et les macros de `config.h` : `FOC_AXIS_MASK`, `AXIS_PRESENT(i)`, `FOC_VBUS`, `FOC_VOLTAGE_LIMIT`, `FOC_VOLTAGE_ALIGN`, `FOC_PWM_HZ`, `FOC_MODULATION`, `FOC_POLE_PAIRS`, `FOC_VEL_LIMIT`, `FOC_PIN_MEN`, et les tables `kAxisPwm[2][3]`, `kAxisEnc[2][3]`.

- [ ] **Step 1 : créer l'arborescence et copier l'outillage de flash**

```bash
cd H:/Projects/RobotRL
mkdir -p ESP32FOCDrive/include ESP32FOCDrive/src ESP32FOCDrive/scripts ESP32FOCDrive/test/test_logic
cp ESP32FOCHardwareCheck/scripts/wrap_uploader.py ESP32FOCDrive/scripts/
cp ESP32FOCHardwareCheck/scripts/esptool_nodtr.py ESP32FOCDrive/scripts/
```

Ces deux scripts sont repris sans modification : le CH340 de cette carte pulse DTR à l'ouverture du port, ce qui casse le hold RTC sur GPIO0 dont dépend la commande `download`.

- [ ] **Step 2 : écrire `platformio.ini`**

```ini
; ESP32FOCDrive — deux FS2804 en FOC voltage, deux MT6835 en ABZ.
; Les identifiants WiFi ne sont PAS ici. Mettre dans un platformio.local.ini
; non commité, en surchargeant UNIQUEMENT la section [wifi] : PlatformIO
; REMPLACE une clé build_flags redéfinie dans un fichier inclus, il ne la
; fusionne pas, ce qui avait silencieusement supprimé FOC_VBUS sur le projet
; précédent.
;   [wifi]
;   build_flags =
;       -DWIFI_SSID=\"yourssid\"
;       -DWIFI_PASS=\"yourpass\"
[platformio]
extra_configs = *.local.ini

[wifi]
build_flags =

[env]
platform = espressif32 @ 7.0.1
board = esp32dev
framework = arduino
monitor_speed = 115200
monitor_dtr = 0
monitor_rts = 0
extra_scripts = post:scripts/wrap_uploader.py
lib_deps =
    askuric/Simple FOC @ 2.3.3
lib_archive = false
build_flags =
    -DCORE_DEBUG_LEVEL=0
    -DFOC_POLE_PAIRS=7
    -DENC_PPR=16384
    -DFOC_VBUS=12.6
    ${wifi.build_flags}

[env:left]
build_flags =
    ${env.build_flags}
    -DFOC_AXIS_MASK=0b01

[env:right]
build_flags =
    ${env.build_flags}
    -DFOC_AXIS_MASK=0b10

[env:dual]
build_flags =
    ${env.build_flags}
    -DFOC_AXIS_MASK=0b11

; Flash normal = OTA. upload_port = l'IP STA affichée au boot ou par `ota`.
[env:left_ota]
extends = env:left
upload_protocol = espota
upload_port = 192.168.1.1

[env:right_ota]
extends = env:right
upload_protocol = espota
upload_port = 192.168.1.1

[env:dual_ota]
extends = env:dual
upload_protocol = espota
upload_port = 192.168.1.1
```

- [ ] **Step 3 : écrire `include/config.h`**

```cpp
#pragma once

#include <stdint.h>

/**
 * MKS ESP32 FOC V2.0, two FS2804 (12N14P, 7 pole pairs) + MT6835 in ABZ.
 * Voltage FOC only: no current sense anywhere in this firmware.
 */

/* Bit 0 = left (motor 0), bit 1 = right (motor 1). */
#ifndef FOC_AXIS_MASK
#define FOC_AXIS_MASK 0b01
#endif
#define AXIS_PRESENT(i) (((FOC_AXIS_MASK) >> (i)) & 1u)
#define AXIS_COUNT 2

/** Shared BL9342 gate-drive enable. GPIO12 is a strap pin: hold it low first. */
#ifndef FOC_PIN_MEN
#define FOC_PIN_MEN 12
#endif

/* {UH, VH, WH} per axis. */
static const int kAxisPwm[AXIS_COUNT][3] = {{32, 33, 25}, {26, 27, 14}};
/* {A, B, Z} per axis. Right A is GPIO5, a strap pin: verified on the bench. */
static const int kAxisEnc[AXIS_COUNT][3] = {{18, 19, 22}, {5, 23, 21}};
static const char kAxisName[AXIS_COUNT] = {'L', 'R'};
/* Robot frame: a positive command drives the robot forward. */
static const int8_t kAxisCmdSign[AXIS_COUNT] = {-1, +1};
/* PCNT unit index equals the axis index. */

/** MT6835 Z is idle-low, pulse-high. */
#ifndef ENC_Z_EDGE
#define ENC_Z_EDGE RISING
#endif
/** Must match the MT6835 ABZ_PPR register. PCNT full-quad -> CPR = 4 * PPR. */
#ifndef ENC_PPR
#define ENC_PPR 16384u
#endif
/** Glitch filter in APB cycles (max 1023). 250 ~ 3 us @ 80 MHz. */
#ifndef ENC_PCNT_FILTER
#define ENC_PCNT_FILTER 250
#endif
/**
 * Velocity window. 1 ms, not 2: a 400 Hz balance loop reading a 500 Hz
 * estimate gets a 0-2 ms variable age, which costs phase margin. At 65536 CPR
 * a 1 ms window still resolves 2*PI/65536/1ms = 0.096 rad/s.
 */
#ifndef ENC_VEL_MIN_DT
#define ENC_VEL_MIN_DT 0.001f
#endif

#ifndef FOC_POLE_PAIRS
#define FOC_POLE_PAIRS 7
#endif
#ifndef FOC_VBUS
#define FOC_VBUS 12.6f
#endif
/** 3.0 V over the 5.2 ohm phase is 0.58 A at stall. Raise at runtime. */
#ifndef FOC_VOLTAGE_LIMIT
#define FOC_VOLTAGE_LIMIT 3.0f
#endif
#ifndef FOC_VOLTAGE_ALIGN
#define FOC_VOLTAGE_ALIGN 2.0f
#endif
#ifndef FOC_PWM_HZ
#define FOC_PWM_HZ 20000u
#endif
/**
 * SinePWM is centered, so usable Uq tops out near Vbus/2 (~6.3 V here).
 * SpaceVectorPWM reaches Vbus/sqrt(3) if the robot ever needs more.
 */
#ifndef FOC_MODULATION
#define FOC_MODULATION FOCModulationType::SinePWM
#endif

/* Velocity PID output is in VOLTS (no current loop). */
#ifndef FOC_VEL_P
#define FOC_VEL_P 0.2f
#endif
#ifndef FOC_VEL_I
#define FOC_VEL_I 2.0f
#endif
#ifndef FOC_VEL_D
#define FOC_VEL_D 0.0f
#endif
#ifndef FOC_VEL_LPF
#define FOC_VEL_LPF 0.005f
#endif
#ifndef FOC_VEL_RAMP
#define FOC_VEL_RAMP 50.0f
#endif
#ifndef FOC_VEL_LIMIT
#define FOC_VEL_LIMIT 80.0f
#endif

#ifndef FOC_LOOP_HZ
#define FOC_LOOP_HZ 4000u
#endif
#ifndef FOC_LOOP_HZ_MIN
#define FOC_LOOP_HZ_MIN 4000u
#endif
#ifndef FOC_LOOP_HZ_MAX
#define FOC_LOOP_HZ_MAX 16000u
#endif

/** Open-loop Z search. */
#ifndef Z_SEARCH_RPS
#define Z_SEARCH_RPS 1.0f
#endif
#ifndef Z_SEARCH_TURNS
#define Z_SEARCH_TURNS 2.0f
#endif

/** Display only: I_est = (Uq - BEMF) / R. Never fed back into any loop. */
#ifndef FOC_PHASE_R
#define FOC_PHASE_R 5.2f
#endif
#ifndef FOC_KV
#define FOC_KV 220.0f
#endif

#ifndef WIFI_SSID
#define WIFI_SSID ""
#endif
#ifndef WIFI_PASS
#define WIFI_PASS ""
#endif
#ifndef OTA_HOSTNAME
#define OTA_HOSTNAME "esp32focdrive"
#endif
#ifndef OTA_AP_SSID
#define OTA_AP_SSID "ESP32FOCDrive"
#endif
```

- [ ] **Step 4 : écrire `include/board.h`**

```cpp
#pragma once

/**
 * Board-level pins shared by both axes. M_EN (GPIO12) feeds BOTH gate
 * drivers, so it cannot be a per-driver enable pin: disabling one axis would
 * cut the other. Drivers are built without an enable pin and this refcount
 * owns M_EN instead.
 */
void boardInit();
/** delta = +1 when an axis arms, -1 when it disarms. Clamped at zero. */
void boardMotorPowerRef(int delta);
bool boardMotorPowered();
/** Hold GPIO0 low across a restart so the ROM enters download mode. */
void boardEnterDownload();
```

- [ ] **Step 5 : écrire `src/board.cpp`**

```cpp
#include "board.h"

#include <Arduino.h>
#include <driver/rtc_io.h>
#include <esp_system.h>

#include "config.h"

namespace {
int power_refs = 0;

void holdPinLow(int pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
}
}  // namespace

void boardInit() {
  holdPinLow(FOC_PIN_MEN);
  power_refs = 0;
  for (int i = 0; i < AXIS_COUNT; ++i) {
    if (AXIS_PRESENT(i)) {
      continue;
    }
    for (int p = 0; p < 3; ++p) {
      holdPinLow(kAxisPwm[i][p]);
    }
  }
}

void boardMotorPowerRef(int delta) {
  power_refs += delta;
  if (power_refs < 0) {
    power_refs = 0;
  }
  digitalWrite(FOC_PIN_MEN, power_refs > 0 ? HIGH : LOW);
}

bool boardMotorPowered() { return power_refs > 0; }

void boardEnterDownload() {
  Serial.println("download: IO0 held low + restart -> ROM");
  Serial.flush();
  delay(50);
  /* ESP32 (not S2/S3) has no FORCE_DOWNLOAD_BOOT register: keep GPIO0 low
   * across esp_restart() with the RTC pad hold so the ROM samples download. */
  rtc_gpio_init(GPIO_NUM_0);
  rtc_gpio_set_direction(GPIO_NUM_0, RTC_GPIO_MODE_OUTPUT_ONLY);
  rtc_gpio_set_level(GPIO_NUM_0, 0);
  rtc_gpio_hold_en(GPIO_NUM_0);
  esp_restart();
}
```

- [ ] **Step 6 : écrire `src/main.cpp`, version open-loop SimpleFOC natif**

C'est le point de départ « SimpleFOC natif » : un seul axe, boucle dans `loop()`, aucune des couches à venir. Il sera démonté par les tâches suivantes.

```cpp
/** ESP32FOCDrive — step 1: bare SimpleFOC open-loop, one axis, loop() driven. */

#include <Arduino.h>
#include <SimpleFOC.h>

#include "board.h"
#include "config.h"

namespace {
constexpr int kAxis = (FOC_AXIS_MASK & 0b01) ? 0 : 1;

BLDCMotor motor(FOC_POLE_PAIRS);
BLDCDriver3PWM driver(kAxisPwm[kAxis][0], kAxisPwm[kAxis][1], kAxisPwm[kAxis][2]);
}  // namespace

void setup() {
  boardInit();
  Serial.begin(115200);
  delay(200);

  driver.voltage_power_supply = FOC_VBUS;
  driver.voltage_limit = FOC_VOLTAGE_LIMIT;
  driver.pwm_frequency = FOC_PWM_HZ;
  driver.init();

  motor.linkDriver(&driver);
  motor.voltage_limit = FOC_VOLTAGE_LIMIT;
  motor.foc_modulation = FOC_MODULATION;
  motor.controller = MotionControlType::velocity_openloop;
  motor.target = 3.0f;
  motor.init();
  motor.enable();
  boardMotorPowerRef(+1);

  Serial.printf("ESP32FOCDrive step1 axis=%c Vbus=%.1f Ulim=%.1f MEN=%d\n",
                kAxisName[kAxis], (double)FOC_VBUS, (double)FOC_VOLTAGE_LIMIT,
                (int)boardMotorPowered());
}

void loop() { motor.move(); }
```

- [ ] **Step 7 : vérifier la compilation des trois envs**

```bash
cd H:/Projects/RobotRL/ESP32FOCDrive
pio run -e left && pio run -e right && pio run -e dual
```
Attendu : trois `SUCCESS`.

- [ ] **Step 8 : validation au banc, roue levée**

```bash
pio run -e left -t upload
pio device monitor -e left
```
Attendu :
- la ligne de boot affiche `axis=L Vbus=12.6 Ulim=3.0 MEN=1` ;
- **le shaft tourne** lentement et régulièrement ;
- au multimètre ou à l'oscilloscope, `M_EN` (GPIO12) est haut.

Si le shaft ne tourne pas : vérifier `M_EN`, puis le câblage PWM. Ne pas monter `FOC_VOLTAGE_LIMIT` pour compenser.

- [ ] **Step 9 : commit**

```bash
cd H:/Projects/RobotRL
git add ESP32FOCDrive
git commit -m "feat(esp32focdrive): project skeleton, shared M_EN refcount, native SimpleFOC open-loop"
```

---

### Task 2 : découpage des commandes et ligne de commande minimale

**Files:**
- Create: `ESP32FOCDrive/include/cmd_parse.h`
- Create: `ESP32FOCDrive/test/test_logic/test_main.cpp`
- Create: `ESP32FOCDrive/include/cli.h`, `ESP32FOCDrive/src/cli.cpp`
- Modify: `ESP32FOCDrive/src/main.cpp`

**Interfaces:**
- Consomme : `config.h` de la tâche 1.
- Produit : `struct ParsedCmd { char cmd[12]; uint8_t axis_mask; bool has_value; float value; }`, `bool parseCmd(const char *line, ParsedCmd *out)`, `void cliInit()`, `void cliPoll()`.

`axis_mask` vaut 0b01 pour `L`, 0b10 pour `R`, et `FOC_AXIS_MASK` quand aucun préfixe n'est donné.

- [ ] **Step 1 : écrire le test qui échoue**

Créer `ESP32FOCDrive/test/test_logic/test_main.cpp` :

```cpp
#include <Arduino.h>
#include <unity.h>

#include "cmd_parse.h"

void setUp() {}
void tearDown() {}

void test_parse_bare_command() {
  ParsedCmd p{};
  TEST_ASSERT_TRUE(parseCmd("status", &p));
  TEST_ASSERT_EQUAL_STRING("status", p.cmd);
  TEST_ASSERT_EQUAL_UINT8(0b11, p.axis_mask);
  TEST_ASSERT_FALSE(p.has_value);
}

void test_parse_value_without_axis() {
  ParsedCmd p{};
  TEST_ASSERT_TRUE(parseCmd("  vel  3.5 ", &p));
  TEST_ASSERT_EQUAL_STRING("vel", p.cmd);
  TEST_ASSERT_EQUAL_UINT8(0b11, p.axis_mask);
  TEST_ASSERT_TRUE(p.has_value);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 3.5f, p.value);
}

void test_parse_axis_prefix() {
  ParsedCmd p{};
  TEST_ASSERT_TRUE(parseCmd("vel R -2", &p));
  TEST_ASSERT_EQUAL_STRING("vel", p.cmd);
  TEST_ASSERT_EQUAL_UINT8(0b10, p.axis_mask);
  TEST_ASSERT_TRUE(p.has_value);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, -2.0f, p.value);
}

void test_parse_axis_only() {
  ParsedCmd p{};
  TEST_ASSERT_TRUE(parseCmd("cal l", &p));
  TEST_ASSERT_EQUAL_STRING("cal", p.cmd);
  TEST_ASSERT_EQUAL_UINT8(0b01, p.axis_mask);
  TEST_ASSERT_FALSE(p.has_value);
}

void test_parse_rejects_empty() {
  ParsedCmd p{};
  TEST_ASSERT_FALSE(parseCmd("   ", &p));
}

void test_parse_long_command_is_truncated_not_overflowed() {
  ParsedCmd p{};
  TEST_ASSERT_TRUE(parseCmd("abcdefghijklmnopqrstuvwxyz 1", &p));
  TEST_ASSERT_EQUAL_UINT32(11, strlen(p.cmd));
  TEST_ASSERT_TRUE(p.has_value);
}

void setup() {
  delay(2000);
  UNITY_BEGIN();
  RUN_TEST(test_parse_bare_command);
  RUN_TEST(test_parse_value_without_axis);
  RUN_TEST(test_parse_axis_prefix);
  RUN_TEST(test_parse_axis_only);
  RUN_TEST(test_parse_rejects_empty);
  RUN_TEST(test_parse_long_command_is_truncated_not_overflowed);
  UNITY_END();
}

void loop() {}
```

Note : `test_parse_bare_command` attend `0b11`, donc les tests se lancent avec l'env `dual`.

- [ ] **Step 2 : lancer le test et vérifier qu'il échoue**

```bash
cd H:/Projects/RobotRL/ESP32FOCDrive
pio test -e dual -f test_logic
```
Attendu : échec de compilation, `cmd_parse.h: No such file or directory`.

- [ ] **Step 3 : écrire `include/cmd_parse.h`**

```cpp
#pragma once

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "config.h"

/**
 * One command line: verb, optional axis prefix, optional numeric value.
 * Header-only and Arduino-free so the Unity suite compiles without src/.
 */
struct ParsedCmd {
  char cmd[12];
  uint8_t axis_mask;
  bool has_value;
  float value;
};

inline const char *cmdSkipSpaces(const char *s) {
  while (*s == ' ' || *s == '\t') {
    ++s;
  }
  return s;
}

inline bool parseCmd(const char *line, ParsedCmd *out) {
  memset(out, 0, sizeof(*out));
  out->axis_mask = (uint8_t)FOC_AXIS_MASK;

  const char *s = cmdSkipSpaces(line);
  if (*s == '\0') {
    return false;
  }
  size_t n = 0;
  while (*s != '\0' && *s != ' ' && *s != '\t') {
    if (n + 1u < sizeof(out->cmd)) {
      out->cmd[n++] = *s;
    }
    ++s;
  }
  out->cmd[n] = '\0';

  s = cmdSkipSpaces(s);
  if (*s == 'L' || *s == 'l') {
    out->axis_mask = 0b01;
    ++s;
  } else if (*s == 'R' || *s == 'r') {
    out->axis_mask = 0b10;
    ++s;
  }

  s = cmdSkipSpaces(s);
  if (*s != '\0') {
    char *end = nullptr;
    const float v = strtof(s, &end);
    if (end != s) {
      out->has_value = true;
      out->value = v;
    }
  }
  return true;
}
```

- [ ] **Step 4 : lancer le test et vérifier qu'il passe**

```bash
pio test -e dual -f test_logic
```
Attendu : `6 Tests 0 Failures 0 Ignored` puis `PASSED`.

- [ ] **Step 5 : écrire `include/cli.h`**

```cpp
#pragma once

/** Serial command line, core 1 only. Never called from the FOC task. */
void cliInit();
void cliPoll();
void cliPrintHelp();
void cliPrintStatus();
```

- [ ] **Step 6 : écrire `src/cli.cpp`, commandes `help`, `status`, `ol`, `idle`, `limit`, `download`**

Pas de `alignv` ici : elle règle `voltage_sensor_align`, qui ne sert qu'à la calibration. Elle arrive avec elle, au step 6 de la tâche 5.

Le lecteur de ligne est repris de `ESP32FOCHardwareCheck/src/main.cpp:1117` : lecture caractère par caractère, écho, gestion du backspace, tampon de 96 octets. À cette étape, `ol` et `idle` agissent encore directement sur le `motor` global de `main.cpp`, exposé par deux fonctions temporaires `mainSetOpenloop(uint8_t axis_mask, float rad_s)` et `mainIdle(uint8_t axis_mask)` déclarées dans `cli.cpp`. La tâche 7 remplacera ces deux appels par `drive_api.h`.

```cpp
#include "cli.h"

#include <Arduino.h>

#include "board.h"
#include "cmd_parse.h"
#include "config.h"

/* Temporary seam, replaced by drive_api.h in task 7. */
void mainSetOpenloop(uint8_t axis_mask, float rad_s);
void mainIdle(uint8_t axis_mask);
float mainVoltageLimit();
void mainSetVoltageLimit(float v);

namespace {
char line[96];
uint8_t line_len = 0;

void handleLine(const char *raw) {
  ParsedCmd p{};
  if (!parseCmd(raw, &p)) {
    return;
  }
  if (strcmp(p.cmd, "help") == 0 || strcmp(p.cmd, "?") == 0) {
    cliPrintHelp();
  } else if (strcmp(p.cmd, "status") == 0) {
    cliPrintStatus();
  } else if (strcmp(p.cmd, "ol") == 0) {
    if (!p.has_value) {
      Serial.println("usage: ol [L|R] <rad/s>");
      return;
    }
    float v = p.value;
    if (v > FOC_VEL_LIMIT) {
      v = FOC_VEL_LIMIT;
    } else if (v < -FOC_VEL_LIMIT) {
      v = -FOC_VEL_LIMIT;
    }
    mainSetOpenloop(p.axis_mask, v);
    Serial.printf("ol: mask=%u tgt=%.2f rad/s\n", (unsigned)p.axis_mask, (double)v);
  } else if (strcmp(p.cmd, "idle") == 0 || strcmp(p.cmd, "stop") == 0) {
    mainIdle(p.axis_mask);
    Serial.println("idle");
  } else if (strcmp(p.cmd, "limit") == 0) {
    if (!p.has_value) {
      Serial.printf("limit: %.2f V\n", (double)mainVoltageLimit());
      return;
    }
    float v = p.value;
    if (v < 0.2f) {
      v = 0.2f;
    }
    if (v > FOC_VBUS) {
      v = FOC_VBUS;
    }
    mainSetVoltageLimit(v);
    Serial.printf("limit: %.2f V\n", (double)v);
  } else if (strcmp(p.cmd, "download") == 0 || strcmp(p.cmd, "dl") == 0) {
    mainIdle(0b11);
    boardEnterDownload();
  } else {
    Serial.println("unknown - help");
  }
}
}  // namespace

void cliInit() { line_len = 0; }

void cliPrintHelp() {
  Serial.println("ESP32FOCDrive  FS2804 x2  voltage FOC  MT6835 ABZ");
  Serial.println("  help status");
  Serial.println("  ol [L|R] <rad/s>   idle [L|R]");
  Serial.println("  limit [L|R] <V>    download");
}

void cliPoll() {
  while (Serial.available() > 0) {
    const char c = (char)Serial.read();
    if (c == '\r' || c == '\n') {
      if (line_len == 0) {
        continue;
      }
      line[line_len] = '\0';
      line_len = 0;
      Serial.write('\n');
      handleLine(line);
      continue;
    }
    if (c == 0x08 || c == 0x7f) {
      if (line_len > 0) {
        line_len -= 1;
        Serial.print("\b \b");
      }
      continue;
    }
    if (line_len + 1u < sizeof(line)) {
      line[line_len++] = c;
      Serial.write(c);
    }
  }
}
```

- [ ] **Step 7 : brancher la CLI dans `main.cpp`**

Dans `src/main.cpp` : ne plus démarrer le moteur au boot, implémenter les quatre fonctions de couture, appeler `cliInit()` dans `setup()` et `cliPoll()` dans `loop()`. `cliPrintStatus()` affiche pour l'instant l'axe, la cible, `Uq`, la limite et l'état de `M_EN`. `mainIdle` fait `motor.target = 0`, `motor.disable()` puis `boardMotorPowerRef(-1)` si l'axe était armé ; `mainSetOpenloop` fait l'inverse, sans réarmer un moteur déjà armé (`if (!motor.enabled)`), parce que `enable()` de SimpleFOC fait toujours `setPwm(0,0,0)` et arrêterait le shaft à chaque commande.

- [ ] **Step 8 : valider au banc**

```bash
pio run -e left -t upload && pio device monitor -e left
```
Attendu : au boot le shaft **ne tourne pas** et `M_EN` est bas ; `ol 3` le fait tourner ; `ol 6` accélère **sans** à-coup d'arrêt entre les deux commandes ; `idle` l'arrête et `M_EN` retombe ; `status` reflète ces états.

- [ ] **Step 9 : commit**

```bash
git add ESP32FOCDrive
git commit -m "feat(esp32focdrive): command parser with axis prefix, minimal CLI, Unity suite on target"
```

---

### Task 3 : encodeur PCNT

**Files:**
- Create: `ESP32FOCDrive/include/enc_math.h`
- Create: `ESP32FOCDrive/include/pcnt_encoder.h`, `ESP32FOCDrive/src/pcnt_encoder.cpp`
- Modify: `ESP32FOCDrive/test/test_logic/test_main.cpp`
- Modify: `ESP32FOCDrive/src/main.cpp`, `ESP32FOCDrive/src/cli.cpp`

**Interfaces:**
- Consomme : `config.h`, `cmd_parse.h`.
- Produit : `encFoldDelta(int16_t prev, int16_t now, int32_t lim)`, `encAngleFromCount(int64_t count, int64_t cpr, int32_t *rot, float *shaft)`, et la classe `PcntEncoder` avec `init()`, `update()`, `count()`, `countAngle()`, `cpr()`, `ok()`, `indexFound()`, `clearIndex()`, `zEdges()`, `jumps()`, `overflowEvents()`, `jumpLogCap()`, `jumpLogAt(uint8_t)`, plus les surcharges `Sensor`.

Constructeur : `PcntEncoder(int pin_a, int pin_b, int pin_z, uint32_t ppr, pcnt_unit_t unit, float max_plausible_vel_rad_s, float vel_min_dt_s)`.

- [ ] **Step 1 : écrire les tests qui échouent**

Ajouter dans `test/test_logic/test_main.cpp`, avec `#include "enc_math.h"` :

```cpp
void test_fold_small_forward_delta() {
  TEST_ASSERT_EQUAL_INT32(5, encFoldDelta(100, 105, 16384));
}

void test_fold_small_backward_delta() {
  TEST_ASSERT_EQUAL_INT32(-5, encFoldDelta(105, 100, 16384));
}

/* PCNT resets to 0 at h_lim instead of wrapping two's-complement: going
 * forward past +16384 reappears near 0, which is a -16384 raw jump. */
/* Les deux arguments sont des LECTURES SUCCESSIVES du compteur, pas un delta :
 * +10 counts depuis 16380 atterrit à 6, -10 depuis -16380 atterrit à -6.
 * Passer le delta brut (-16374) à la place de la lecture est l'erreur que ces
 * deux vecteurs contenaient, et que `selftest` a révélée sur le matériel. */
void test_fold_hlim_reset_reads_as_forward() {
  TEST_ASSERT_EQUAL_INT32(10, encFoldDelta(16380, 6, 16384));
}

void test_fold_llim_reset_reads_as_backward() {
  TEST_ASSERT_EQUAL_INT32(-10, encFoldDelta(-16380, -6, 16384));
}

void test_angle_from_count_positive() {
  int32_t rot = 0;
  float shaft = 0.0f;
  encAngleFromCount(65536 + 16384, 65536, &rot, &shaft);
  TEST_ASSERT_EQUAL_INT32(1, rot);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 1.5708f, shaft);
}

/* Negative counts must still yield shaft in [0, 2PI): SimpleFOC 2.3.3 treats a
 * negative getSensorAngle() as an error and skips the update entirely. */
void test_angle_from_count_negative_stays_in_range() {
  int32_t rot = 0;
  float shaft = 0.0f;
  encAngleFromCount(-16384, 65536, &rot, &shaft);
  TEST_ASSERT_EQUAL_INT32(-1, rot);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 4.7124f, shaft);
}
```

Les ajouter aux `RUN_TEST` de `setup()`.

- [ ] **Step 2 : lancer et vérifier l'échec**

```bash
pio test -e dual -f test_logic
```
Attendu : `enc_math.h: No such file or directory`.

- [ ] **Step 3 : écrire `include/enc_math.h`**

```cpp
#pragma once

#include <stdint.h>

#ifndef ENC_MATH_2PI
#define ENC_MATH_2PI 6.28318530718f
#endif

/**
 * ESP32 PCNT is 16-bit and RESETS TO 0 at h_lim/l_lim instead of wrapping in
 * two's complement. With symmetric +/-lim, every reset is a known step of
 * `lim` counts, so the raw difference can be folded in software on each read:
 * no limit ISR, no pcnt_counter_clear, no lost counts.
 *
 * Caller must read often enough that a true motion never exceeds lim/2
 * (8192 counts here, about 10 ms at 80 rad/s with 65536 CPR).
 */
inline int32_t encFoldDelta(int16_t prev, int16_t now, int32_t lim) {
  int32_t d = (int32_t)now - (int32_t)prev;
  if (d < -(lim / 2)) {
    d += lim;
  } else if (d > (lim / 2)) {
    d -= lim;
  }
  return d;
}

/** Split a 64-bit count into full rotations and a shaft angle in [0, 2PI). */
inline void encAngleFromCount(int64_t count, int64_t cpr, int32_t *rotations,
                              float *shaft) {
  int64_t rot = count / cpr;
  int64_t rem = count % cpr;
  if (rem < 0) {
    rem += cpr;
    rot -= 1;
  }
  *rotations = (int32_t)rot;
  *shaft = (float)rem * (ENC_MATH_2PI / (float)cpr);
}
```

- [ ] **Step 4 : lancer et vérifier que ça passe**

```bash
pio test -e dual -f test_logic
```
Attendu : `12 Tests 0 Failures 0 Ignored`.

- [ ] **Step 5 : porter `PcntEncoder` depuis HardwareCheck**

```bash
cp ESP32FOCHardwareCheck/include/pcnt_encoder.h ESP32FOCDrive/include/
cp ESP32FOCHardwareCheck/src/pcnt_encoder.cpp ESP32FOCDrive/src/
```

Puis appliquer exactement ces changements :

1. **ISR Z par instance.** Supprimer `s_z_owner` et le `portMUX_TYPE s_mux` du namespace anonyme. Ajouter `portMUX_TYPE mux_;` en membre, initialisé à `portMUX_INITIALIZER_UNLOCKED`, et remplacer la signature par `static void IRAM_ATTR zIsr(void *arg)`, dont le corps commence par `PcntEncoder *self = static_cast<PcntEncoder *>(arg);`. Dans `init()`, remplacer l'attachement par :

```cpp
attachInterruptArg(digitalPinToInterrupt(pin_z_), zIsr, this, ENC_Z_EDGE);
```

2. **Paramètres au constructeur.** Ajouter `float max_plausible_vel_rad_s` et `float vel_min_dt_s`, stockés en membres. `update()` utilise `max_plausible_vel_rad_s_` là où le code lisait `6.0f * FOC_VEL_LIMIT`, et `init()` fait `min_elapsed_time = vel_min_dt_s_;`. L'encodeur ne dépend donc plus de `config.h` pour ces deux valeurs.

3. **Utiliser les fonctions pures.** Dans `foldLocked()`, remplacer le calcul inline par `const int32_t d = encFoldDelta(hw_prev_, hw, kPcntLim);` en gardant l'incrément de `wrap_events_` quand `d` ne vaut pas `hw - hw_prev_`. Dans `angleFromCount()`, déléguer à `encAngleFromCount(count, cprInt(), rotations, shaft)`.

Tout le reste — dépliage 64 bits, origine au premier front Z seulement, `getAngle()` et `getVelocity()` calculés depuis le compteur, journal de sauts — est conservé **tel quel** : ce sont les corrections qui ont rendu HardwareCheck utilisable.

- [ ] **Step 6 : instancier l'encodeur et ajouter `enc` à la CLI**

Dans `main.cpp`, créer l'encodeur de l'axe actif avec `pcnt_unit_t(kAxis)`, `ENC_PPR`, `6.0f * FOC_VEL_LIMIT` et `ENC_VEL_MIN_DT`, appeler `init()` dans `setup()` **avant** toute autre initialisation moteur, le lier par `motor.linkSensor(&encoder)`, et appeler `encoder.update()` dans `loop()`.

Dans `cli.cpp`, ajouter la commande `enc [0|1]` : sans valeur elle affiche une fois la ligne encodeur, `enc 1` l'affiche à 1 Hz, `enc 0` arrête. Format :

```
enc L cnt=12345 idx=1 zn=3 A=1 B=0 Z=0 ang=1.1837
```

- [ ] **Step 7 : valider au banc**

Attendu, moteur non alimenté :
- `cnt` bouge quand on tourne la roue à la main, et change de signe selon le sens ;
- un tour complet fait varier `cnt` de 65536, à quelques counts près ;
- `idx` passe de 0 à 1 au premier passage de l'index, et `zn` s'incrémente à chaque tour ;
- `ang` reste continu et ne se fige jamais, y compris quand `cnt` devient négatif.

- [ ] **Step 8 : commit**

```bash
git add ESP32FOCDrive
git commit -m "feat(esp32focdrive): PCNT ABZ encoder with software 16-bit unfolding, per-instance Z ISR"
```

---

### Task 4 : tâche FOC sur le core 0

**Files:**
- Create: `ESP32FOCDrive/include/axis.h`, `ESP32FOCDrive/src/axis.cpp`
- Create: `ESP32FOCDrive/include/foc_task.h`, `ESP32FOCDrive/src/foc_task.cpp`
- Modify: `ESP32FOCDrive/src/main.cpp`, `ESP32FOCDrive/src/cli.cpp`

**Interfaces:**
- Consomme : `PcntEncoder`, `board.h`, `config.h`.
- Produit :
  - `enum class Mode : uint8_t { Off, Openloop, Velocity, Torque }`, `enum class Owner : uint8_t { Task, Cli }` ;
  - `struct Axis` avec `name`, `idx`, `cmd_sign`, `present`, `encoder`, `driver`, `motor`, `mode`, `owner`, `last_cmd_ms`, `cmd_timeout_ms`, `calibrated`, `voltage_limit`, `align_voltage`, `armed` ;
  - `Axis axes[AXIS_COUNT]`, `void axisInitAll()`, `void axisApplyLimits(Axis &)`, `void axisArm(Axis &)`, `void axisDisarm(Axis &)`, `void axisSetMode(Axis &, Mode)` ;
  - `void focTaskStart()`, `void focSetHz(uint32_t)`, `uint32_t focHz()`, `[[nodiscard]] bool focSyncWithTask()`, `void focPauseTimer()`, `void focResumeTimer()`, `struct FocMetrics { uint32_t dt_us, dt_max_us, late, hz; uint64_t loops; }`, `FocMetrics focGetMetrics()`, `void focResetMetrics()`.

**Signatures amendées pendant la ronde de correction de cette tâche, et qui font foi :** `focSyncWithTask()` renvoie un `[[nodiscard]] bool` — elle abandonne après 50 ms, et un appelant qui dépend du transfert doit pouvoir le savoir ; les appelants qui l'ignorent délibérément le font par un `(void)` commenté. La même ronde a ajouté la paire `[[nodiscard]] bool axisTakeOwnership(Axis &)` / `void axisReleaseOwnership(Axis &)`, qui encapsulent l'écriture de `owner` **et** la synchronisation, avec le contrat de relecture de l'encodeur documenté au-dessus. Le croquis de code ci-dessous précède cet amendement et montre encore la version sans valeur de retour.

`axisArm` fait `axisApplyLimits`, arme le moteur seulement s'il ne l'est pas déjà, et appelle `boardMotorPowerRef(+1)` une seule fois par axe. `axisDisarm` fait l'inverse. `axisSetMode` écrit le mode **puis** appelle `focSyncWithTask()`.

- [ ] **Step 1 : écrire `include/axis.h` et `src/axis.cpp`**

`axisInitAll()` construit, pour chaque axe présent : l'encodeur (broches de `kAxisEnc`, unité PCNT `idx`), le driver `BLDCDriver3PWM(UH, VH, WH)` **sans enable pin**, le moteur `BLDCMotor(FOC_POLE_PAIRS)`. Il applique ensuite :

```cpp
driver.voltage_power_supply = FOC_VBUS;
driver.pwm_frequency = FOC_PWM_HZ;
driver.init();
motor.linkDriver(&driver);
motor.linkSensor(&encoder);
motor.foc_modulation = FOC_MODULATION;
motor.torque_controller = TorqueControlType::voltage;
motor.controller = MotionControlType::torque;
motor.target = 0.0f;
motor.velocity_limit = FOC_VEL_LIMIT;
motor.PID_velocity.P = FOC_VEL_P;
motor.PID_velocity.I = FOC_VEL_I;
motor.PID_velocity.D = FOC_VEL_D;
motor.PID_velocity.output_ramp = FOC_VEL_RAMP;
motor.LPF_velocity.Tf = FOC_VEL_LPF;
motor.voltage_sensor_align = FOC_VOLTAGE_ALIGN;
axisApplyLimits(ax);
motor.init();
motor.disable();
```

`axisApplyLimits` :

```cpp
void axisApplyLimits(Axis &ax) {
  ax.driver.voltage_limit = ax.voltage_limit;
  ax.motor.voltage_limit = ax.voltage_limit;
  ax.motor.PID_velocity.limit = ax.voltage_limit;  /* output is VOLTS */
  ax.motor.velocity_limit = FOC_VEL_LIMIT;
}
```

Il n'y a **pas** de `P_angle` : ce firmware n'a pas de mode position.

- [ ] **Step 2 : écrire `include/foc_task.h` et `src/foc_task.cpp`**

Le corps de la tâche, repris de `ESP32FOCHardwareCheck/src/main.cpp:1185` et étendu aux deux axes :

```cpp
void focTask(void *) {
  disableCore0WDT();
  startFocTimer();
  for (;;) {
    const uint32_t n = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (n > 1u) {
      s_late += n - 1u;
    }
    const int64_t t0 = esp_timer_get_time();
    for (int i = 0; i < AXIS_COUNT; ++i) {
      Axis &ax = axes[i];
      if (!ax.present || ax.owner != Owner::Task) {
        continue;
      }
      if (ax.mode == Mode::Off) {
        ax.encoder.update();
        continue;
      }
      ax.motor.loopFOC();
      ax.motor.move();
    }
    const uint32_t dt = (uint32_t)(esp_timer_get_time() - t0);
    s_dt_us = dt;
    if (dt > s_dt_max_us) {
      s_dt_max_us = dt;
    }
    s_loops += 1;
    s_seq += 1;
    /* Overrun: notifications pile up, Take returns immediately, IDLE0 never
     * runs and the task WDT aborts. Drop the backlog, wait for a fresh tick. */
    (void)ulTaskNotifyTake(pdTRUE, 0);
  }
}
```

La boucle ci-dessus n'a **pas** de failsafe : il arrive à la tâche 7, en même temps que l'en-tête `failsafe.h` qui le teste. Le code livré à cette étape est complet et cohérent sans lui, puisque seule la CLI commande le firmware et qu'elle n'utilise pas de timeout.

Le timer, repris tel quel : groupe `TIMER_GROUP_1`, timer `TIMER_0`, diviseur 80, auto-reload, alarme à `1000000 / hz`, `timer_isr_callback_add(..., ESP_INTR_FLAG_IRAM)`, callback qui fait uniquement `vTaskNotifyGiveFromISR`. Il est **démarré depuis la tâche**, pour que l'ISR soit allouée sur le core 0.

```cpp
void focTaskStart() {
  xTaskCreatePinnedToCore(focTask, "foc", 4096, nullptr, 20, &s_task, 0);
}
```

`focSyncWithTask()` :

```cpp
void focSyncWithTask() {
  if (s_task == nullptr || s_timer_paused) {
    return;
  }
  const uint32_t start = s_seq;
  const uint32_t t0 = millis();
  while ((s_seq - start) < 2u) {
    if ((millis() - t0) > 50u) {
      return;   /* timer stopped or starved: do not block the CLI */
    }
    vTaskDelay(1);
  }
}
```

Attendre deux incréments, et pas un seul, garantit qu'une itération **complète** a commencé après l'écriture de l'appelant.

- [ ] **Step 3 : sortir le FOC de `loop()`**

Dans `main.cpp` : `setup()` appelle `boardInit()`, `Serial.begin`, `axisInitAll()`, `cliInit()`, `focTaskStart()`. `loop()` ne contient plus que `cliPoll()`, l'affichage périodique et `vTaskDelay(1)`. Aucun appel à `motor.move()` ou `encoder.update()` depuis le core 1 : la tâche les fait dans tous les modes, y compris `Off`, ce qui garantit aussi la relecture du PCNT avant 8192 counts.

Les fonctions de couture de la tâche 2 deviennent : `mainSetOpenloop` écrit `motor.controller = MotionControlType::velocity_openloop`, la cible, puis `axisArm` ; `mainIdle` fait `axisSetMode(ax, Mode::Off)` puis `axisDisarm`. L'ordre compte : le mode passe à `Off` et la synchronisation a lieu **avant** le désarmement, sinon la tâche peut réécrire une PWM après coup.

- [ ] **Step 4 : ajouter `hz` et `dt` à la CLI**

`hz` sans valeur affiche la cadence et les métriques ; avec une valeur, elle appelle `focSetHz` qui borne entre `FOC_LOOP_HZ_MIN` et `FOC_LOOP_HZ_MAX`, remet `dt_max` à zéro et redémarre le timer. `dt` remet `dt_max` et `late` à zéro. `status` affiche désormais `hz`, `dt`, `dtmax`, `late` et `loops`.

- [ ] **Step 5 : mettre le timer FOC en pause pendant un OTA**

`net.cpp` existe déjà — la tâche 8 a été exécutée avant celle-ci, voir « Ordre d'exécution » — et son `onStart` arrête les moteurs sans toucher au timer, qui n'existait pas. Maintenant qu'il existe, ajouter `focPauseTimer()` dans `onStart` après l'arrêt des moteurs, `focResumeTimer()` dans `onError`, et compléter le message en `"ota: start - motors off, FOC timer paused"`.

Sans cette pause, le timer continue de réveiller la tâche FOC à 4 kHz pendant l'écriture de la flash.

- [ ] **Step 6 : valider au banc**

```bash
pio run -e left_ota -t upload && pio device monitor -e left
```
Attendu :
- `ol 20` fait tourner le shaft **régulièrement**, et un `status` toutes les secondes ne provoque plus d'à-coup ;
- `dt` est de l'ordre de 30 à 60 µs pour un axe, `dtmax` reste très en dessous de 250 µs ;
- `late` reste à 0 ou augmente très lentement ;
- `hz 8000` fonctionne et `dtmax` reste sous 0.7 × 125 µs ; revenir ensuite à `hz 4000`.

Si `late` s'emballe ou si la CLI meurt : c'est un problème de priorité ou de core, pas une raison de baisser la cadence.

- [ ] **Step 7 : commit**

```bash
git add ESP32FOCDrive
git commit -m "feat(esp32focdrive): isochronous FOC task on core 0 with ownership handshake and timing metrics"
```

---

### Task 5 : calibration et NVS

**Files:**
- Create: `ESP32FOCDrive/include/cal_record.h`
- Modify: `ESP32FOCDrive/include/self_test.h`
- Modify: `ESP32FOCDrive/include/axis.h`, `ESP32FOCDrive/src/axis.cpp`, `ESP32FOCDrive/src/cli.cpp`, `ESP32FOCDrive/src/main.cpp`

**Interfaces:**
- Consomme : `Axis`, `focSyncWithTask`.
- Produit : `struct CalRecord`, `bool calRecordValid(const CalRecord &rec, char axis, uint16_t ppr)`, et sur `axis.h` : `bool axisCal(Axis &)`, `bool axisZSearch(Axis &, bool park)`, `bool axisSaveCal(Axis &)`, `void axisForgetCal(Axis &)`, `bool axisLoadCal(Axis &)`, `bool axisRequireCal(Axis &)`.

- [ ] **Step 1 : ajouter les vérifications de `cal_record.h` à `self_test.h`**

La tâche 12 a centralisé toutes les vérifications de logique pure dans `include/self_test.h`, appelé à la fois par la suite Unity et par la commande `selftest` de l'application. **Ne crée donc aucun test Unity séparé** : `test/test_logic/test_main.cpp` ne doit pas être modifié par cette tâche.

Dans `self_test.h` : ajouter `#include "cal_record.h"` en tête, le petit constructeur d'enregistrement en `inline`, et les cinq vérifications avant le `return res;` final.

```cpp
inline CalRecord selfTestMakeCalRecord() {
  CalRecord r{};
  r.magic = kCalMagic;
  r.zero_electric_angle = 1.234f;
  r.sensor_direction = 1;
  r.pole_pairs = 7;
  r.enc_ppr = 16384;
  r.axis = 'L';
  return r;
}
```

```cpp
  SELF_TEST_CHECK("cal_match", calRecordValid(selfTestMakeCalRecord(), 'L', 16384));
  SELF_TEST_CHECK("cal_other_axis", !calRecordValid(selfTestMakeCalRecord(), 'R', 16384));
  /* Changing ENC_PPR silently invalidates every stored electrical zero. */
  SELF_TEST_CHECK("cal_ppr_change", !calRecordValid(selfTestMakeCalRecord(), 'L', 4096));
  {
    CalRecord bad = selfTestMakeCalRecord();
    bad.magic = 0xdeadbeefu;
    SELF_TEST_CHECK("cal_bad_magic", !calRecordValid(bad, 'L', 16384));
  }
  {
    CalRecord bad = selfTestMakeCalRecord();
    bad.sensor_direction = 0;
    SELF_TEST_CHECK("cal_no_dir", !calRecordValid(bad, 'L', 16384));
  }
```

- [ ] **Step 2 : vérifier que la compilation échoue**

```bash
cd H:/Projects/RobotRL/ESP32FOCDrive
pio run -e dual
```
Attendu : `cal_record.h: No such file or directory`. C'est la compilation qui sert de test rouge ici : les vérifications s'exécutent sur la carte, et l'implémenteur ne téléverse pas.

- [ ] **Step 3 : écrire `include/cal_record.h`**

```cpp
#pragma once

#include <math.h>
#include <stdint.h>

/** 'DRV1'. Deliberately different from HardwareCheck's 'HCK2': that firmware
 * stored a zero taken with a current-sense alignment path that no longer
 * exists, so its records must not be reused. */
constexpr uint32_t kCalMagic = 0x44525631u;

struct CalRecord {
  uint32_t magic;
  float zero_electric_angle;
  int8_t sensor_direction;  /* +1 CW, -1 CCW, 0 unknown */
  uint8_t pole_pairs;
  uint16_t enc_ppr;
  char axis;
};

inline bool calRecordValid(const CalRecord &rec, char axis, uint16_t ppr) {
  if (rec.magic != kCalMagic || rec.axis != axis) {
    return false;
  }
  if (rec.pole_pairs == 0 || rec.sensor_direction == 0) {
    return false;
  }
  if (rec.enc_ppr != ppr) {
    return false;
  }
  return isfinite(rec.zero_electric_angle) != 0;
}
```

- [ ] **Step 4 : vérifier que la compilation passe**

```bash
pio run -e left && pio run -e right && pio run -e dual
```
Attendu : trois `SUCCESS`. L'exécution des vérifications se fait ensuite par OTA, avec la commande `selftest` : `17 run, 0 failed`.

- [ ] **Step 5 : porter la séquence de calibration**

Depuis `ESP32FOCHardwareCheck/src/main.cpp`, porter dans `axis.cpp`, par axe et sans aucune partie liée au courant : `runZSearch` (`:383`), `slewElectric` (`:419`), `rampElectricDown` (`:433`), `rampElectricUp` (`:442`), `runOpenloopDirectionAndZero` (`:450`), `runInitFoc` (`:503`). Supprimer tous les appels à `current_sense`, `driverAlign` et `skip_align`.

`axisCal(Axis &ax)` enchaîne :

```cpp
ax.owner = Owner::Cli;
focSyncWithTask();
/* 1. index */        ax.encoder.clearIndex(); zSearch(ax);
/* 2. direction */    const float moved = openloopSpin(ax, 3.0f, 500);
/* 3. sanity */       reportPpRatio(ax, moved);
/* 4. zero */         rampUp / slewTo(_3PI_2) / settle 400 ms / read zero / rampDown
/* 5. */              ax.motor.initFOC();
ax.owner = Owner::Task;
```

Le contrôle de cohérence, absent de HardwareCheck et repris de l'`alignSensor` natif :

```cpp
/* An open-loop spin at 3 rad/s for 500 ms should move the shaft ~1.5 rad.
 * A large mismatch means ENC_PPR differs from the MT6835 ABZ register or
 * pole_pairs is wrong. Warn only: slipping in open loop is possible. */
const float expected = 3.0f * 0.5f;
const float ratio = fabsf(moved) / expected;
Serial.printf("cal %c: moved=%.3f rad expected=%.3f ratio=%.2f%s\n", ax.name,
              (double)moved, (double)expected, (double)ratio,
              (ratio < 0.7f || ratio > 1.3f) ? "  WARNING check ENC_PPR / pole_pairs" : "");
```

NVS : namespace `drive`, clés `cal0` et `cal1`, `Preferences::putBytes` et `getBytes` sur `CalRecord`, validation par `calRecordValid(rec, ax.name, ENC_PPR)`.

- [ ] **Step 6 : câbler `cal`, `zsearch`, `save`, `forget`, `alignv` dans la CLI**

Avec préfixe d'axe : sans préfixe, les axes sont traités **l'un après l'autre**, jamais en parallèle. `vel` et `tq` n'existent pas encore ; `axisRequireCal` est écrite ici et servira à la tâche 6.

**`status` doit aussi gagner un champ `cal=`**, par axe. `cliPrintStatus()` vit dans `main.cpp` — d'où sa présence dans la liste des fichiers ci-dessus. Sans ce champ, l'étape 7 n'est pas vérifiable : c'est en lisant `cal=0` après un redémarrage puis `cal=1` après `zsearch` qu'on prouve que la calibration a bien été rechargée depuis la mémoire non volatile.

- [ ] **Step 7 : valider au banc**

Attendu :
- `cal L` fait tourner le shaft, affiche `zsearch: ok`, un `ratio` proche de 1.00 sans avertissement, puis `dir` et `zero` ;
- `save L` confirme l'écriture ;
- après un reboot, `status` montre `cal=0`, et `zsearch L` seul suffit pour repasser à `cal=1` avec le même `zero` qu'avant le reboot, à quelques millièmes près ;
- `forget L` puis reboot : `zsearch` répond qu'il n'y a pas de zéro en NVS.

- [ ] **Step 8 : commit**

```bash
git add ESP32FOCDrive
git commit -m "feat(esp32focdrive): per-axis calibration with gentle zero alignment, PPR-guarded NVS records"
```

---

### Task 6 : boucles fermées `vel` et `tq`

**Files:**
- Modify: `ESP32FOCDrive/include/axis.h`, `ESP32FOCDrive/src/axis.cpp`, `ESP32FOCDrive/src/cli.cpp`

**Interfaces:**
- Consomme : `axisRequireCal`, `axisArm`, `axisSetMode`, `axisApplyLimits`.
- Produit : `bool axisSetVelocity(Axis &, float rad_s)`, `bool axisSetTorque(Axis &, float volts)`, `float axisCurrentEstimate(const Axis &)`.

- [ ] **Step 1 : implémenter `axisSetVelocity` et `axisSetTorque`**

```cpp
bool axisSetVelocity(Axis &ax, float rad_s) {
  if (!axisRequireCal(ax)) {
    return false;
  }
  rad_s = _constrain(rad_s, -FOC_VEL_LIMIT, FOC_VEL_LIMIT);  /* SimpleFOC macro */
  axisApplyLimits(ax);
  if (ax.armed && ax.mode == Mode::Velocity) {
    ax.motor.target = rad_s;   /* live setpoint: do NOT re-arm */
    return true;
  }
  ax.motor.controller = MotionControlType::velocity;
  ax.motor.torque_controller = TorqueControlType::voltage;
  ax.motor.target = rad_s;
  /* No encoder priming here. The encoder belongs to the FOC task: calling
   * update()/getVelocity() from core 1 races on fields the instance lock does
   * not cover, and move() calls getVelocity() on every tick once armed. A
   * stale sample is harmless anyway — getVelocity() divides by the elapsed
   * time, so it reads near zero for one cycle, never a spike. */
  axisSetMode(ax, Mode::Velocity);
  axisArm(ax);
  return true;
}
```

`axisSetTorque` est identique, avec `MotionControlType::torque`, une cible bornée à `±ax.voltage_limit`, et `Mode::Torque`.

Réarmer à chaque commande arrêterait le shaft : `enable()` de SimpleFOC fait toujours `setPwm(0,0,0)` et remet les PID à zéro. C'est `enableIfNeeded` de HardwareCheck, intégré ici à `axisArm`.

- [ ] **Step 2 : ajouter le courant estimé, en affichage seulement**

```cpp
/** Display only. Renseigner phase_resistance dans SimpleFOC changerait aussi
 * le sens des gains du PID de vitesse, donc on calcule à côté. */
float axisCurrentEstimate(const Axis &ax) {
  const float bemf = ax.motor.shaft_velocity / (FOC_KV * _SQRT3) / _RPM_TO_RADS;
  return (ax.motor.voltage.q - bemf) / FOC_PHASE_R;
}
```

- [ ] **Step 3 : câbler `vel`, `tq` et `mon` dans la CLI, et compléter `status`**

`mon 0|1` arme ou coupe l'affichage périodique : quand il est actif, `loop()` appelle `cliPrintStatus()` toutes les secondes. Il est indépendant de `enc 0|1`, qui affiche la ligne encodeur. Les deux sont éteints par défaut, pour qu'une session de mesure ne soit pas polluée. Ajouter les deux au texte de `help`.

Une ligne par axe :

```
L mode=VEL armed=1 cal=1 idx=1 cnt=123456 ang=2.1837 vel=3.002 tgt=3.000 Uq=0.412 Iest=0.079 zero=1.2345 dir=CW lim=3.00
```
Puis une ligne globale :
```
hz=4000 dt=62 dtmax=118 late=0 loops=1234567 MEN=1 wifi=STA 192.168.1.42
```

- [ ] **Step 4 : valider au banc, roue levée**

Attendu :
- `vel 3` : la roue atteint 3 rad/s, `vel` colle à `tgt` à quelques centièmes ;
- `vel 6` puis `vel 3` : transitions **sans** arrêt intermédiaire ;
- `tq 1` : couple sensible au doigt ; `tq 0` : roue presque libre ;
- `vel` et `tq` sont refusés tant que `cal=0` ;
- si la roue oscille, baisser `FOC_VEL_P` avant toute autre chose, et noter les valeurs retenues dans le README à la tâche 11.

- [ ] **Step 5 : commit**

```bash
git add ESP32FOCDrive
git commit -m "feat(esp32focdrive): closed-loop velocity and voltage torque, estimated current display"
```

---

### Task 7 : API entre les cores, failsafe et instantané d'état

**Files:**
- Create: `ESP32FOCDrive/include/failsafe.h`
- Create: `ESP32FOCDrive/include/drive_api.h`, `ESP32FOCDrive/src/drive_api.cpp`
- Modify: `ESP32FOCDrive/include/self_test.h`, `ESP32FOCDrive/src/foc_task.cpp`, `ESP32FOCDrive/src/cli.cpp`, `ESP32FOCDrive/src/main.cpp`

**Interfaces:**
- Consomme : `axisSetVelocity`, `axisSetTorque`, `axisDisarm`, `Axis`.
- Produit : `bool failsafeExpired(uint32_t now_ms, uint32_t last_ms, uint32_t timeout_ms)`, `bool driveSetVelocity(uint8_t axis, float rad_s, uint32_t timeout_ms)`, `bool driveSetTorque(uint8_t axis, float volts, uint32_t timeout_ms)`, `bool driveSetOpenloop(uint8_t axis, float rad_s, uint32_t timeout_ms)`, `void driveStop(uint8_t axis)`, `struct AxisState { float angle, velocity, uq; bool armed, calibrated; }`, `bool driveGetState(uint8_t axis, AxisState *out)`.

`axis` est ici un **index** (0 ou 1), pas un masque : c'est l'API qu'utilisera la boucle d'équilibrage. La CLI convertit son masque en appels par axe.

- [ ] **Step 1 : ajouter les vérifications de `failsafe.h` à `self_test.h`**

La tâche 12 a centralisé les vérifications de logique pure dans `include/self_test.h`, appelé à la fois par la suite Unity et par la commande `selftest`. **Ne crée aucun test Unity séparé** : `test/test_logic/test_main.cpp` ne doit pas être modifié par cette tâche.

Dans `self_test.h` : ajouter `#include "failsafe.h"` en tête, et les cinq vérifications avant le `return res;` final.

```cpp
  SELF_TEST_CHECK("fs_zero_never", !failsafeExpired(1000000u, 0u, 0u));
  SELF_TEST_CHECK("fs_within", !failsafeExpired(1005u, 1000u, 10u));
  SELF_TEST_CHECK("fs_after", failsafeExpired(1011u, 1000u, 10u));
  /* millis() wraps every ~49.7 days; unsigned subtraction must carry it. */
  SELF_TEST_CHECK("fs_wrap_ok", !failsafeExpired(0x00000005u, 0xFFFFFFF0u, 50u));
  SELF_TEST_CHECK("fs_wrap_expired", failsafeExpired(0x00000040u, 0xFFFFFFF0u, 50u));
```

- [ ] **Step 2 : vérifier que la compilation échoue**

```bash
cd H:/Projects/RobotRL/ESP32FOCDrive
pio run -e dual
```
Attendu : `failsafe.h: No such file or directory`. C'est la compilation qui sert de test rouge : les vérifications s'exécutent sur la carte, et l'implémenteur ne téléverse pas.

- [ ] **Step 3 : écrire `include/failsafe.h`**

```cpp
#pragma once

#include <stdint.h>

/**
 * A command is stale once timeout_ms has elapsed since last_ms.
 * timeout_ms == 0 disables the check: that is what the CLI passes, so a
 * bench session is never cut off. A control loop passes ~10 ms (4 periods at
 * 400 Hz), so a hung core 1 stops the wheels.
 * Unsigned arithmetic carries the millis() wrap; never compare timestamps.
 */
inline bool failsafeExpired(uint32_t now_ms, uint32_t last_ms, uint32_t timeout_ms) {
  if (timeout_ms == 0u) {
    return false;
  }
  return (uint32_t)(now_ms - last_ms) > timeout_ms;
}
```

- [ ] **Step 4 : vérifier que la compilation passe**

```bash
pio run -e left && pio run -e right && pio run -e dual
```
Attendu : trois `SUCCESS`. L'exécution se fait ensuite par OTA, avec la commande `selftest` : toutes les vérifications passent, `0 failed`.

- [ ] **Step 5 : écrire `drive_api.h` et `drive_api.cpp`**

```cpp
#pragma once

#include <stdint.h>

/**
 * Boundary between core 1 (CLI today, balance loop later) and the core 0 FOC
 * task. Commands are in the ROBOT frame: cmd_sign is applied here, once.
 *
 * In steady state a setpoint call is a bound plus two aligned 32-bit stores,
 * with no lock and no handshake: at 400 Hz on two axes that is under 0.1% of
 * a core. focSyncWithTask() only runs on arm and disarm.
 */
/* Return false when the command was refused (axis absent, not calibrated):
 * the CLI must not confirm, and a control loop must be able to tell
 * "commanded" from "refused" without polling the state snapshot. */
bool driveSetVelocity(uint8_t axis, float rad_s, uint32_t timeout_ms);
bool driveSetTorque(uint8_t axis, float volts, uint32_t timeout_ms);
bool driveSetOpenloop(uint8_t axis, float rad_s, uint32_t timeout_ms);
void driveStop(uint8_t axis);

struct AxisState {
  float angle;
  float velocity;
  float uq;
  bool armed;
  bool calibrated;
};
/** Consistent snapshot published once per FOC iteration. */
bool driveGetState(uint8_t axis, AxisState *out);
```

Dans `drive_api.cpp`, chaque consigne applique `kAxisCmdSign[axis]`, borne, puis délègue à `axisSetVelocity` / `axisSetTorque`.

**L'estampillage — `last_cmd_ms` puis `cmd_timeout_ms` — est le point délicat de cette tâche, et il n'a pas la même règle selon le chemin :**

- **Chemin rapide** (axe déjà armé dans le bon mode) : l'estampille doit être **dans la même section critique** que le test de `armed` et l'écriture de la consigne. Sinon le failsafe du cœur 0 peut désarmer entre l'écriture et l'estampille, et l'appel rend `true` sur un axe parqué.
- **Chemin lent** (changement de mode) : mettre `cmd_timeout_ms` **à zéro avant** la transition — un délai nul désactive le failsafe, donc la synchronisation, qui peut durer 50 ms, devient non expirable — puis estampiller après l'armement. Sans cela, le failsafe expire pendant la synchronisation, désarme, et **écrase le mode** que le cœur 1 vient d'écrire : on finit armé, en mode repos, alimentation prise, moteur jamais piloté.

Estampiller simplement « avant » ou « après » la délégation échange une fenêtre contre l'autre au lieu de les fermer toutes les deux. L'instantané est publié par la tâche FOC avec un compteur pair/impair :

```cpp
/* Publisher, FOC task only. */
void drivePublishState(const Axis &ax, uint8_t i) {
  s_pub[i].seq += 1;                 /* odd: write in progress */
  __sync_synchronize();
  s_pub[i].st.angle = ax.encoder.countAngle();
  s_pub[i].st.velocity = ax.motor.shaft_velocity;
  s_pub[i].st.uq = ax.motor.voltage.q;
  s_pub[i].st.armed = ax.armed;
  s_pub[i].st.calibrated = ax.calibrated;
  __sync_synchronize();
  s_pub[i].seq += 1;                 /* even: readable */
}

bool driveGetState(uint8_t axis, AxisState *out) {
  if (axis >= AXIS_COUNT || !axes[axis].present) {
    return false;
  }
  for (int tries = 0; tries < 4; ++tries) {
    const uint32_t s0 = s_pub[axis].seq;
    if ((s0 & 1u) != 0u) {
      continue;
    }
    __sync_synchronize();
    *out = s_pub[axis].st;
    __sync_synchronize();
    if (s_pub[axis].seq == s0) {
      return true;
    }
  }
  return false;
}
```

- [ ] **Step 6 : ajouter le failsafe à la tâche FOC**

Réintroduire dans `focTask`, avant le traitement du mode, le bloc décrit à la tâche 4 : si l'axe est armé et que `failsafeExpired(now_ms, ax.last_cmd_ms, ax.cmd_timeout_ms)`, mettre la cible à zéro, désarmer et repasser en `Mode::Off`. Le désarmement vient de la tâche elle-même, donc aucune synchronisation n'est nécessaire ici.

- [ ] **Step 7 : faire passer la CLI par l'API**

Remplacer `mainSetOpenloop`, `mainIdle`, `mainVoltageLimit` et `mainSetVoltageLimit` par des appels à `drive_api.h`, avec `timeout_ms = 0`. Supprimer les quatre déclarations de couture de `cli.cpp`.

**Ne pas oublier `net.cpp` :** son `onStart` appelle encore `mainIdle(0b11)`, la dernière couture. La remplacer par `for (uint8_t i = 0; i < AXIS_COUNT; ++i) { driveStop(i); }` et supprimer la déclaration de `mainIdle` en haut du fichier. Après cette étape, un `grep -rn "mainIdle\|mainSetOpenloop\|mainVoltageLimit\|mainSetVoltageLimit" src include` ne doit plus rien retourner. La CLI traduit son masque : `for (i in 0..1) if (mask & (1<<i)) driveSetVelocity(i, v, 0);`.

**Attention au signe :** `status` affiche des valeurs dans le repère moteur, alors que `vel 3` est maintenant une commande en repère robot. Afficher les deux : `tgt` (moteur) et `cmd` (robot).

- [ ] **Step 8 : valider au banc**

Attendu :
- `vel 3` fait tourner la roue gauche dans le sens qui fait **avancer** le robot, roue levée ;
- le comportement de la CLI est inchangé, sans coupure intempestive, puisque le timeout vaut 0 ;
- ajouter temporairement une commande de test `fs <ms>` qui envoie une consigne avec ce timeout, puis vérifier que la roue s'arrête seule au bout du délai, et que `status` montre `armed=0` et `MEN=0`. Garder cette commande : elle documente le failsafe et servira à l'intégration de l'équilibrage.

- [ ] **Step 9 : commit**

```bash
git add ESP32FOCDrive
git commit -m "feat(esp32focdrive): drive API with command failsafe and seqlock state snapshot"
```

---

### Task 8 : WiFi, OTA, outillage de flash

**Files:**
- Create: `ESP32FOCDrive/include/net.h`, `ESP32FOCDrive/src/net.cpp`
- Modify: `ESP32FOCDrive/src/main.cpp`, `ESP32FOCDrive/src/cli.cpp`
- Create: `ESP32FOCDrive/platformio.local.ini` (non commité)

**Interfaces:**
- Consomme : la couture `mainIdle(uint8_t axis_mask)` de la tâche 2. **Ni `driveStop` ni `focPauseTimer` n'existent encore** : cette tâche est exécutée en troisième position, voir « Ordre d'exécution ».
- Produit : `void netSetup()`, `void netLoop()`, `bool netOtaActive()`, `void netPrintInfo()`, `void netWifiOff()`.

- [ ] **Step 1 : écrire `net.cpp`**

Porté de `ESP32FOCHardwareCheck/src/main.cpp:187` : STA avec 8 s de timeout, repli en `softAP(OTA_AP_SSID)`, `ArduinoOTA.setHostname(OTA_HOSTNAME)`, `setTimeout(60000)`.

```cpp
ArduinoOTA.onStart([]() {
  s_ota_active = true;
  mainIdle(0b11);          /* task 7 swaps this for the driveStop loop */
  disableCore1WDT();
  Serial.println("ota: start - motors off");
  Serial.flush();
});
ArduinoOTA.onError([](ota_error_t err) {
  s_ota_active = false;
  Serial.printf("ota: err %u\n", (unsigned)err);
});
```

Il n'y a **pas** de mise en pause du timer FOC ici : à cette position dans l'ordre d'exécution, la tâche FOC n'existe pas encore. La tâche 4 l'ajoutera. Déclarer `void mainIdle(uint8_t axis_mask);` en haut de `net.cpp`, comme le fait déjà `cli.cpp`.

- [ ] **Step 2 : céder le CPU pendant l'upload**

Dans `loop()` :

```cpp
void loop() {
  netLoop();
  if (netOtaActive()) {
    /* A tight handle() loop starves IDLE1 and the WDT aborts mid-flash. */
    vTaskDelay(1);
    return;
  }
  cliPoll();
  ...
}
```

- [ ] **Step 3 : ajouter `ota` et `wifioff` à la CLI, et le WiFi à `status`**

- [ ] **Step 4 : créer `platformio.local.ini`**

```ini
[wifi]
build_flags =
    -DWIFI_SSID=\"ORBI44\"
    -DWIFI_PASS=\"<le mot de passe>\"
```

Vérifier qu'il est bien ignoré : `git status --short` ne doit pas le montrer, et `git check-ignore -v ESP32FOCDrive/platformio.local.ini` doit citer la règle `*.local.ini`.

- [ ] **Step 5 : valider au banc**

Attendu :
- au boot, `ota: STA ... ip=...` ; sans identifiants valides, repli en AP `ESP32FOCDrive` ;
- `pio run -e left_ota -t upload` avec la bonne `upload_port` termine à 100 % ;
- pendant le flash, les moteurs sont arrêtés et `M_EN` est bas ;
- après redémarrage, la CLI répond et `status` est cohérent ;
- `download` depuis la CLI, suivi de `pio run -e left -t upload` en USB, fonctionne aussi.

- [ ] **Step 6 : commit**

```bash
git add ESP32FOCDrive
git commit -m "feat(esp32focdrive): WiFi STA/AP fallback, OTA that parks the motors and pauses the FOC timer"
```

---

### Task 9 : deuxième axe

**Files:**
- Modify: `ESP32FOCDrive/src/axis.cpp`, `ESP32FOCDrive/src/cli.cpp`, `ESP32FOCDrive/src/foc_task.cpp`

**Interfaces:**
- Consomme : tout ce qui précède.
- Produit : commande `axes`, et le fonctionnement complet de l'env `dual`.

- [ ] **Step 1 : relire le code pour tout état resté global**

Chercher les variables qui supposent un seul axe :

```bash
cd H:/Projects/RobotRL/ESP32FOCDrive
grep -rn "kAxis\b\|static .*motor\|s_z_owner\|axes\[0\]" src include
```
Attendu : plus aucune occurrence de `kAxis` ni de `axes[0]` hors des boucles indexées.

- [ ] **Step 2 : ajouter la commande `axes`**

```
axes: mask=0b11  L pwm=32/33/25 abz=18/19/22 pcnt=0 present=1 cal=1
                 R pwm=26/27/14 abz=5/23/21  pcnt=1 present=1 cal=0
```

- [ ] **Step 3 : valider l'axe droit seul**

```bash
pio run -e right -t upload && pio device monitor -e right
```
Attendu, **avant tout** : le boot est propre et répétable, malgré GPIO5 qui est une broche de strap. Rebooter cinq fois de suite avec le MT6835 branché et vérifier qu'aucun boot ne part en mode download ni en boucle de reset. Puis refaire la séquence complète : `enc`, `cal R`, `save R`, reboot, `zsearch R`, `vel R 3`, `tq R 1`.

- [ ] **Step 4 : valider les deux axes ensemble**

```bash
pio run -e dual -t upload && pio device monitor -e dual
```
Attendu :
- `cal` sans préfixe calibre les deux axes **l'un après l'autre**, jamais en même temps ;
- `vel 3` fait tourner les deux roues dans le sens qui ferait avancer le robot ;
- `vel L 3` ne bouge que la gauche ;
- `idle L` arrête la gauche et **laisse la droite tourner**, ce qui valide le compteur `M_EN` ;
- `dtmax` avec deux axes reste largement sous 250 µs ; le noter.

- [ ] **Step 5 : commit**

```bash
git add ESP32FOCDrive
git commit -m "feat(esp32focdrive): both axes live, shared M_EN validated, axes command"
```

---

### Task 10 : diagnostics `vcap`, `vdump`, `jlog`

**Files:**
- Modify: `ESP32FOCDrive/src/foc_task.cpp`, `ESP32FOCDrive/include/foc_task.h`, `ESP32FOCDrive/src/cli.cpp`

**Interfaces:**
- Produit : `void vcapArm(uint8_t axis, uint32_t ms)`, `void vcapDump()`, `bool vcapDone()`.

- [ ] **Step 1 : porter la capture**

Repris de `ESP32FOCHardwareCheck/src/main.cpp:70` et `:1211`, avec `Iq` remplacé par `Uq` puisqu'il n'y a plus de courant mesuré :

```cpp
constexpr uint32_t kVCapMax = 2500;
struct VCapSample {
  int32_t dcount;   /* counts since the previous kept sample: exact, no filter */
  float vel;
  float uq;
};
```

La décimation reste : `decim = ceil(total_loops / kVCapMax)`, pour qu'une demande longue tienne dans le tampon. La capture ne concerne **qu'un seul axe**, choisi par `vcap [L|R] [ms]`, par défaut le premier axe présent et 300 ms.

`dcount` sert à distinguer une vraie oscillation mécanique d'un artefact d'estimation : si `vel` oscille alors que `dcount` reste lisse, le problème est dans l'estimation, pas dans le shaft.

- [ ] **Step 2 : ajouter `vcap`, `vdump` et `jlog` à la CLI**

`vdump` sort un CSV `idx,dcount,vel,uq`, précédé d'une ligne d'état. `jlog` affiche `jumps`, `overflowEvents` et le contenu du journal de sauts de l'encodeur, par axe.

- [ ] **Step 3 : valider au banc**

Attendu :
- `vel 3` puis `vcap 300` puis `vdump` : environ 1200 lignes, `dcount` régulier, `vel` proche de 3 ;
- `jlog` : `jumps=0` après plusieurs minutes de `vel 3` ;
- copier un `vdump` dans un fichier CSV et vérifier qu'il s'ouvre proprement.

- [ ] **Step 4 : commit**

```bash
git add ESP32FOCDrive
git commit -m "feat(esp32focdrive): per-iteration vcap capture, vdump CSV, PCNT jump log"
```

---

### Task 11 : mesure de gigue, README, clôture

**Files:**
- Create: `ESP32FOCDrive/README.md`
- Modify: `ESP32FOCDrive/include/config.h` si le banc impose d'autres gains

**Interfaces:**
- Produit : la documentation d'usage et les chiffres de référence.

- [ ] **Step 1 : mesurer la gigue, WiFi actif puis coupé**

Sur l'env `dual`, roues levées :

```
vel 3
dt                  (remet dtmax et late à zéro)
                    ... attendre 5 minutes ...
status              (noter dtmax et late)
wifioff
dt
                    ... attendre 5 minutes ...
status              (noter dtmax et late)
```

C'est la mesure qui décidera, pour l'équilibrage, s'il faut monter la priorité de la tâche FOC ou couper le WiFi en fonctionnement. La tâche WiFi est épinglée sur le core 0 et tourne à une priorité supérieure à 20.

- [ ] **Step 2 : écrire `README.md`**

En français, avec : le câblage des deux axes sous forme de tableau, la procédure de flash en USB et en OTA, la séquence de mise au point dans l'ordre (`enc`, `cal`, `save`, reboot, `zsearch`, `vel`, `tq`), la liste complète des commandes, les gains retenus au banc, les chiffres de `dtmax` et `late` de l'étape 1, et un renvoi vers la spec.

Y documenter aussi les deux pièges déjà payés une fois : `platformio.local.ini` ne surcharge que `[wifi]`, et COM1 n'est jamais la carte.

- [ ] **Step 3 : vérification finale**

```bash
cd H:/Projects/RobotRL/ESP32FOCDrive
pio run -e left && pio run -e right && pio run -e dual
```
Attendu : trois `SUCCESS`. Puis, par OTA, la commande `selftest` sur la carte : toutes les vérifications passent, `0 failed`.

- [ ] **Step 4 : commit**

```bash
git add ESP32FOCDrive
git commit -m "docs(esp32focdrive): bring-up guide, bench gains, scheduling jitter figures"
```

---

### Task 12 : commande `selftest`, pour valider sans câble

**Files:**
- Create: `ESP32FOCDrive/include/self_test.h`
- Modify: `ESP32FOCDrive/test/test_logic/test_main.cpp`
- Modify: `ESP32FOCDrive/src/cli.cpp`

**Interfaces:**
- Consomme : `cmd_parse.h`, `enc_math.h`, et plus tard `failsafe.h` et `cal_record.h` — chaque tâche qui ajoute un en-tête pur ajoute ses vérifications ici.
- Produit : `struct SelfTestResult { int run; int failed; }`, `typedef void (*SelfTestReport)(const char *name, bool ok, void *ctx)`, `SelfTestResult selfTestRun(SelfTestReport report, void *ctx)`.

**Pourquoi cette tâche existe.** Les tests Unity s'exécutent sur la cible, et cette carte impose un geste physique — maintenir BOOT, appuyer sur RESET — pour tout flash USB, parce que son auto-reset par DTR/RTS est cassé. Le binaire Unity n'ayant pas de serveur OTA, chaque exécution coûtait deux manipulations. Les vérifications déménagent donc dans un en-tête partagé, appelé par **deux** exécuteurs : la suite Unity, et une commande `selftest` de l'application, qui elle arrive par OTA. Un seul jeu d'assertions, aucune duplication.

- [ ] **Step 1 : écrire `include/self_test.h`**

En-tête autonome, sans dépendance Arduino, sur le modèle des autres en-têtes purs. Chaque vérification appelle le rapporteur avec son nom et son résultat ; le compteur d'échecs est tenu par `selfTestRun`.

```cpp
#pragma once

#include <math.h>
#include <stdint.h>
#include <string.h>

#include "cmd_parse.h"
#include "enc_math.h"

/**
 * Pure-logic checks, shared by two runners: the Unity suite on target, and
 * the firmware's own `selftest` command. This board cannot be flashed over
 * USB without a manual BOOT+RESET, so validation has to ride the OTA path.
 */
struct SelfTestResult {
  int run;
  int failed;
};

typedef void (*SelfTestReport)(const char *name, bool ok, void *ctx);

#define SELF_TEST_CHECK(name, cond)        \
  do {                                     \
    const bool ok_ = (cond);               \
    res.run += 1;                          \
    if (!ok_) {                            \
      res.failed += 1;                     \
    }                                      \
    if (report != nullptr) {               \
      report(name, ok_, ctx);              \
    }                                      \
  } while (0)

inline bool selfTestNear(float a, float b, float tol) { return fabsf(a - b) <= tol; }

inline SelfTestResult selfTestRun(SelfTestReport report, void *ctx) {
  SelfTestResult res{0, 0};
  ParsedCmd p{};

  SELF_TEST_CHECK("parse_bare", parseCmd("status", &p) && strcmp(p.cmd, "status") == 0 &&
                                    p.axis_mask == (uint8_t)FOC_AXIS_MASK && !p.has_value);
  SELF_TEST_CHECK("parse_value", parseCmd("  vel  3.5 ", &p) && strcmp(p.cmd, "vel") == 0 &&
                                     p.axis_mask == (uint8_t)FOC_AXIS_MASK && p.has_value &&
                                     selfTestNear(p.value, 3.5f, 1e-4f));
  SELF_TEST_CHECK("parse_axis_r", parseCmd("vel R -2", &p) && strcmp(p.cmd, "vel") == 0 &&
                                      p.axis_mask == 0b10 && p.has_value &&
                                      selfTestNear(p.value, -2.0f, 1e-4f));
  SELF_TEST_CHECK("parse_axis_l", parseCmd("cal l", &p) && strcmp(p.cmd, "cal") == 0 &&
                                      p.axis_mask == 0b01 && !p.has_value);
  SELF_TEST_CHECK("parse_empty", !parseCmd("   ", &p));
  SELF_TEST_CHECK("parse_truncate",
                  parseCmd("abcdefghijklmnopqrstuvwxyz 1", &p) && strlen(p.cmd) == 11 && p.has_value);

  SELF_TEST_CHECK("fold_fwd", encFoldDelta(100, 105, 16384) == 5);
  SELF_TEST_CHECK("fold_back", encFoldDelta(105, 100, 16384) == -5);
  /* The hardware counter RESETS to 0 at its limit instead of wrapping in
   * two's complement, so both arguments are successive counter readings, not
   * a delta: +10 counts from 16380 lands at 6, -10 from -16380 lands at -6. */
  SELF_TEST_CHECK("fold_hlim", encFoldDelta(16380, 6, 16384) == 10);
  SELF_TEST_CHECK("fold_llim", encFoldDelta(-16380, -6, 16384) == -10);

  int32_t rot = 0;
  float shaft = 0.0f;
  encAngleFromCount(65536 + 16384, 65536, &rot, &shaft);
  SELF_TEST_CHECK("angle_pos", rot == 1 && selfTestNear(shaft, 1.5708f, 1e-3f));
  encAngleFromCount(-16384, 65536, &rot, &shaft);
  SELF_TEST_CHECK("angle_neg", rot == -1 && selfTestNear(shaft, 4.7124f, 1e-3f));

  return res;
}
```

- [ ] **Step 2 : faire passer la suite Unity par ce même en-tête**

Dans `test/test_logic/test_main.cpp`, remplacer les douze tests existants par un test unique qui délègue, de sorte qu'il n'existe plus qu'un seul jeu d'assertions dans le dépôt :

```cpp
#include "self_test.h"

static void unityReport(const char *name, bool ok, void *) {
  TEST_ASSERT_TRUE_MESSAGE(ok, name);
}

void test_self_test_suite() {
  const SelfTestResult r = selfTestRun(unityReport, nullptr);
  TEST_ASSERT_GREATER_THAN_INT(0, r.run);
  TEST_ASSERT_EQUAL_INT(0, r.failed);
}
```

La granularité par test est perdue côté Unity, mais le nom de la vérification qui échoue est porté par le message d'assertion. C'est le prix de l'absence de duplication, et Unity devient de toute façon l'exécuteur secondaire.

- [ ] **Step 3 : ajouter la commande `selftest` à la ligne de commande**

Dans `cli.cpp`, un rapporteur qui imprime une ligne par vérification, puis un résumé :

```
selftest: parse_bare PASS
selftest: fold_hlim PASS
...
selftest: 12 run, 0 failed
```

L'ajouter au texte de `help`. La commande ne pilote aucun moteur et ne touche à aucune broche : elle est sûre à tout moment, moteur armé ou non.

- [ ] **Step 4 : vérifier la compilation**

```bash
cd H:/Projects/RobotRL/ESP32FOCDrive
pio run -e left && pio run -e right && pio run -e dual
```
Attendu : trois `SUCCESS`.

- [ ] **Step 5 : validation**

Par OTA, puis `selftest` dans le moniteur. Attendu : `12 run, 0 failed`.

- [ ] **Step 6 : commit**

```bash
git add ESP32FOCDrive
git commit -m "feat(esp32focdrive): shared pure-logic checks behind a selftest command, runnable over OTA"
```

**Règle pour la suite du plan :** toute tâche qui ajoute un en-tête pur — `failsafe.h` en tâche 7, `cal_record.h` en tâche 5 — ajoute ses vérifications à `self_test.h` au lieu d'écrire des tests Unity séparés.

---

## Ce que ce plan ne fait pas

Conformément à la section 15 de la spec, et à ne pas ajouter en cours de route :

- protection anti-blocage par `I_est`, **à faire avant** de monter `FOC_VOLTAGE_LIMIT` pour le robot ;
- tâche `ctrl` sur le core 1, IMU ICM45686, fusion de pitch, équilibrage ;
- recherche d'index automatique au boot ;
- liaison avec le STM32 ou la télémétrie ;
- passage éventuel en `SpaceVectorPWM`.
