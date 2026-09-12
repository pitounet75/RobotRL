# Design + faisabilité : 8 kHz, courant, anticogging

Un axe (`HardwareCheck`). Le balancer `ESP32FOC` ne change pas. Implémenté dans le firmware (timer FOC, ADC DMA, `foc_current`, `accal`).

## Verdict

| Objectif | Faisable ? | Condition |
|----------|------------|-----------|
| Boucle ≥ 8 kHz, 1 axe | **Oui, viser 10–16 kHz** | ADC **continu / DMA** (IDF), pas `analogRead` |
| `foc_current` | **Oui** | shunts 10 mΩ, ADC1 GPIO 39/36 (left) |
| Anticogging | **Oui, viser 3600 bins** | RAM ~28 ko, interpolé, 2 passes, FF **Iq** |
| Aussi fin que l’ODrive | **CPU oui, analogique non** | extra MHz ≠ ADC synchro PWM 12-bit propre |

L’ESP32 (2 × 240 MHz) a **plus de CPU** que le STM32F4 (~168 MHz). On s’en sert : DMA en fond, FOC plus vite, carte 3600 bins + interpolation. On ne s’en sert **pas** à faire 60 µs d’`analogRead` dans la boucle. Ce que le surplus de MHz **ne remplace pas** : échantillonner Iq au centre PWM (ODrive le fait en IRQ ADC). Ça, c’est du MCPWM + trigger, phase 2.

## Budget temps à 8 kHz (125 µs)

Mesures communautaires SimpleFOC / ESP32 classique :

| Travail | Durée |
|---------|--------|
| `analogRead` Arduino | ~60 µs **par** voie → 2 voies = 120 µs → **rate trop juste** |
| `adcRead()` SimpleFOC (registres SAR) | ~8–12 µs / voie → 2 voies ≈ 20 µs |
| `loopFOC` + Park + 2 PID Iq/Id + `move` | ~20–40 µs |
| Anticogging : `map[bin(θ)]` | < 1 µs |
| **Total cible** | **~50–80 µs / 125 µs** |

Cible ADC : **IDF continuous / DMA** — le SAR tourne tout seul, la tâche FOC lit le dernier échantillon (~1 µs). Budget FOC alors surtout du calcul (~30–50 µs) → **10–16 kHz** tenable sur un axe.

Repli si le DMA ADC1 est pénible (IDF 4.4 / Arduino 2.x) : `adcRead` SimpleFOC et 8 kHz. Jamais `analogRead`.

Mesure obligatoire : `dt_max` (µs). On **monte** la fréquence tant que `dt_max < 0,7 × période`. On ne descend que si on rate.

## Hardware courant (déjà sur la carte)

Makerbase V2.0, exemple officiel :

| Axe | Shunts | Gain | ADC |
|-----|--------|------|-----|
| left (M0) | 2 × R010 (10 mΩ) | 50 | GPIO **39**, **36** (ADC1) |
| right (M1) | 2 × R010 | 50 | GPIO **35**, **34** (ADC1) |

ADC1 = compatible Wi‑Fi. Pas de conflit avec l’ABZ (18/19/22 ou 5/23/21).

Ne **pas** lire le VIN Makerbase (GPIO13 = ADC2) pendant l’OTA.

### SNR à 0,5 A (2804)

`U_adc = I × 0,01 × 50 = 0,25 V` sur ~3,3 V (≈ 8 % de la pleine échelle).  
Bruit ADC ESP32 souvent 10–40 mV → **Iq bruité**. La boucle marchera ; elle sera moins propre que les shunts ODrive + ADC synchrone.

On **ne change pas** les R010 tant que `status` n’a pas montré un `Iq` utilisable. Si c’est inutilisable : 20–33 mΩ ensuite.

Le current sense est **inline**, pas low-side. SimpleFOC échantillonnera **asynchrone** au PWM 20 kHz → ripple. LPF Iq (`Tf` ~ 0,5–1 ms) obligatoire. L’ODrive échantillonne au centre PWM : on ne reproduira pas ça sans MCPWM + callback (phase 2, hors scope v1).

## Architecture tâches

```
Cœur 0  prio 17   foc_task     62–125 µs  loopFOC + move + ac FF (lit DMA)
Cœur 0  DMA       adc_cont     2 voies ADC1 en continu
Cœur 1  prio 1    loop()       USB, CLI, OTA, mon 1 Hz
```

Règles :

- Zéro `Serial` dans `foc_task`.
- `mode = Enc` **avant** `disable` (déjà le cas).
- `cal` / `accal` : `mode` dédié ou Enc + travail sur le cœur 1 **ou** machine d’état lue par `foc_task` (pas les deux qui appellent `loopFOC`).
- `vTaskDelayUntil`, pas un busy-loop (WDT / Wi‑Fi cœur 0).
- Si `dt_max` explose pendant un TX USB : le FOC ne doit plus être dans `loop()`.

## Plateforme : Arduino hors du chemin FOC

Le check actuel (cal, ABZ, `vel 3`) a tenu sur Arduino + `loop()`. Dès qu’on vise DMA + 8–16 kHz + Iq, **Arduino dans la boucle est le bug** : `analogRead` 60 µs, `Serial.printf` 15 ms, `loop()` prio 1, pas de timer isochrone.

Découpe :

| Reste Arduino (cœur 1) | Natif IDF (cœur 0) |
|------------------------|--------------------|
| USB CLI, echo, OTA, Wi‑Fi | `gptimer` ou `vTaskDelayUntil` FOC |
| `setup()` une fois | `adc_continuous` DMA ADC1 |
| | LEDC / MCPWM déjà sous SimpleFOC |

On n’écrit pas un nouveau stack moteur from scratch. SimpleFOC reste pour Park / PID / PWM. On lui **interdit** `analogRead` et on ne l’appelle plus depuis `loop()`.

Full `framework = espidf` (zéro Arduino) : seulement si Serial/OTA Arduino deviennent le prochain plafond. SimpleFOC parle encore `Print` / `delay` — un port IDF pur est un projet à part, pas le v1.

## Cascade (comme ODrive, unités SimpleFOC)

1. `vel` → PID vitesse → **consigne Iq** (A), pas Uq  
2. Anticogging : `Iq += map[bin(θ)]` (zero-mean)  
3. PID Iq / Id → Uq / Ud, plafonnés par `limit` (V)  
4. `ilim` = 0,6 A (2804). `tq <A>` = couple courant  

`tq` aujourd’hui en volts **change de sémantique** (breaking, check only).

PID v1 (départ conservateur, pas Makerbase P=1 I=500) :

- Iq/Id : P=0,5, I=50, LPF 0,8 ms  
- Vitesse : P un peu plus bas qu’aujourd’hui dès que la sortie est en ampères (à retuner)

## Anticogging (leçon ODrive)

Comme le firmware custom v0.5.1 :

- Sweep **horaire puis anti**, moyenne par bin, **soustraire la moyenne globale**  
- Enregistrer l’intégrateur **Iq** (ou Uq si on est encore en voltage le temps du proto) une fois **settled**  
- Attendre `calib_anticogging == false`, **jamais** `index == 0`  
- Interdire `save` / `ac save` pendant la cal  

v1 : **3600 bins** comme l’ODrive (14 ko + buffer reverse — rien pour l’ESP32). Interpolation linéaire entre bins. Cal plus longue (~2 min, 2 sens) ; on peut offrir `accal 360` pour un smoke test. NVS namespace `acog`, pas le blob `HCK2`.

CLI : `accal` `ac on` `ac off` `ac save` `ac forget`.

## Ce qu’on ne promet pas (v1)

- Dual-axe à 16 kHz  
- Synchro ADC ↔ centre PWM (MCPWM) — **le** levier « aussi fin qu’ODrive », phase 2  
- Changer les shunts avant d’avoir vu `Iq`  
- Portage dans `ESP32FOC`  

## Phases de vérif (go / no-go)

1. **Boucle voltage + ADC DMA**. `ol 20` + `mon 1` : plus de saut USB. Log `dt_us`. Monter 8 → 10 → 16 kHz tant que `dt_max < 0,7 T`.  
   - Fail CLI mort → prio / cœur, pas « remettre analogRead ».  
2. **Current**. `InlineCurrentSense(0.01f, 50.f, 39, 36)` left. `tq 0.2` (A), roue levée. `Iq` suit, pas d’emballement.  
   - Fail si `init` current / align foire, ou Iq bruit > ~0,15 A RMS au repos → LPF / skip_align / plus tard shunts.  
3. **Anticogging**. `vel` propre, puis `accal` roue **libre**. `ac on` : ripple à basse vitesse moindre.  
   - Fail si la cal ne termine pas (settle) → seuils, pas « index != 0 ».

## Fichiers prévus (quand tu dis d’implémenter)

| Fichier | Rôle |
|---------|------|
| `include/config.h` | `FOC_LOOP_HZ`, `I_LIM`, pins ADC |
| `include/anticog.h` + `src/anticog.cpp` | 360 bins, 2 passes, NVS |
| `src/main.cpp` | `foc_task` 125 µs, `InlineCurrentSense`, CLI |
| `src/pcnt_encoder.cpp` | inchangé si `getAngle` / `getVelocity` tiennent 8 kHz |

## Décisions figées (sauf ton veto)

- HardwareCheck, 1 axe  
- ADC **IDF continuous / DMA**, jamais `analogRead`  
- 8 kHz plancher, **16 kHz** si `dt` le permet  
- Current left = 39/36, 10 mΩ, ×50  
- `ilim` 0,6 A, `tq` en ampères  
- Anticogging **3600 bins**, interpolé, FF Iq, 2 directions, zero-mean  
