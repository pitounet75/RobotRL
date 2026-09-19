# ESP32FOCHardwareCheck — plan

Outil de validation hardware : **un** MKS FS2804 + **un** MT6835 en **ABZ / PCNT** (Sensor SimpleFOC), voltage FOC. PWM = `ESP32FOC`. Pas de SPI encodeur, pas d’IMU. Le balancer (`ESP32FOC`) reste intact.

## Moteur

MKS FS2804 / YT2804 : **12N14P → 7 pole pairs**, ~220 kV, Rphase ~2.3 Ω. Voltage FOC, pas de current sense : le « couple » est **Uq** (V).

| Paramètre | Défaut | Pourquoi |
|-----------|--------|----------|
| `FOC_POLE_PAIRS` | 7 | 12N14P |
| `FOC_VBUS` | 8.4 | 2S |
| `FOC_VOLTAGE_LIMIT` | 2.0 | ~0.9 A court, sous le max 1–2 A |
| `FOC_VOLTAGE_ALIGN` | 1.0 | lock d-axis sans surchauffe |
| PWM | 20 kHz 3PWM | même que le balancer |

Un axe à la fois. ABZ : M0 A18/B19/Z22, M1 A5/B23/Z21. `ENC_SWAP_AB` inverse A/B dans le PCNT. L’autre PWM en LOW.

## Phase A — maintenant (1 / 2 / 3)

### 1. Offset électrique

Même grandeur que `encoder.config.offset` ODrive : angle électrique zéro + sens encodeur.

`cal` = Z search open-loop (`Z_SEARCH_RPS`, défaut 1 tr/s, max `Z_SEARCH_TURNS`) puis align électrique. `save` garde dir + zéro. Reboot : `zsearch` seul puis restore NVS. `M_EN` = GPIO12. NVS magic `HCK2`.

`vel` / `tq` refusés tant que pas calibré.

### 2. Vitesse

`MotionControlType::velocity`, `TorqueControlType::voltage`. Cible en rad/s. PID voltage conservateur (P=0.2, I=2, LPF 20 ms).

### 3. Couple

`MotionControlType::torque`, cible = Uq (V). Pas d’Iq : pas de shunt.

### CLI (115200)

| Commande | Rôle |
|----------|------|
| `help` / `status` | aide / dump |
| `cal` | offset (moteur bouge) |
| `save` / `forget` | NVS |
| `idle` | PWM off, cible 0 |
| `vel <rad/s>` | mode vitesse |
| `tq <V>` | mode couple (Uq) |
| `limit <V>` / `alignv <V>` | plafonds |
| `mon 0\|1` | télémétrie 5 Hz |

Critère OK : `cal` → PP check pass, sens stable ; `vel 3` tourne lisse ; `tq 0.4` couple au doigt, `tq 0` presque libre.

## Phase B — plus tard : anticogging (pas dans ce commit)

Comme ODrive v0.5.1, adapté au voltage FOC.

1. Prérequis : offset NVS + `vel` propre, roue libre.
2. Sweep position par bins (3600 comme ODrive, ou 360 pour un premier run).
3. Deux passes (horaire puis anti), moyenne, soustraction de la moyenne (offset DC ≠ cogging).
4. Appliquer `Uq += map[bin(θ)]` dans `loopFOC` (FF), pas dans le PID vitesse.
5. `anticog_valid` en RAM ; `pre_calibrated` seulement après moyenne. Interdire `save` pendant `calib_anticogging`.
6. Attendre `calib_anticogging == false`, **pas** `index != 0` (piège ODrive : index part et finit à 0).

Hors scope Phase B : carte Iq, dual-moteur, FreeRTOS.

## Fichiers Phase A

| Fichier | Rôle |
|---------|------|
| `platformio.ini` | envs `left` / `right` |
| `include/config.h` | PWM + ABZ A/B + PPR + PCNT |
| `include/pcnt_encoder.h` + `src/pcnt_encoder.cpp` | PCNT full-quad → `Sensor` |
| `src/main.cpp` | FOC + NVS + CLI |
| `README.md` | flash et séquence de test |
