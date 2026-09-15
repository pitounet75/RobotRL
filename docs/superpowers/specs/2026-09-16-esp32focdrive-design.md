# ESP32FOCDrive — design

Firmware moteur pour la carte MKS ESP32 FOC V2.0 : deux MKS FS2804 en FOC
voltage, deux MT6835 en ABZ via PCNT. Reprise depuis SimpleFOC 2.3.3 natif,
augmentée des seules idées validées par `ESP32FOCHardwareCheck`.

Cible finale : robot équilibreur à deux roues. Ce firmware livre les roues
pilotables et l'interface de commande ; l'IMU et la boucle d'équilibrage sont
un sous-projet suivant.

## 1. Pourquoi repartir de zéro

`ESP32FOCHardwareCheck` a mélangé trois chantiers : la boucle temps réel, le
sense de courant et l'anticogging. Le mode courant (shunts 10 mΩ ×50, ADC1
échantillonné dans l'ISR MCPWM) n'a jamais donné un `Iq` exploitable, et
l'anticogging construit dessus n'a pas montré de gain. Le reste — PCNT, timer
plus tâche dédiée, calibration persistante, OTA — a fonctionné.

`ESP32FOCDrive` garde ce qui a marché, en mode voltage uniquement, avec deux
axes et une frontière nette entre le core 0 temps réel et le core 1 applicatif.

`ESP32FOCHardwareCheck` et `ESP32FOC` restent intacts comme référence.

## 2. Périmètre

Dans le périmètre :

- deux axes, avec le deuxième désactivable au build ;
- FOC voltage : `ol`, `vel`, `tq` en volts ;
- encodeurs MT6835 en ABZ par PCNT, index Z ;
- calibration persistante en NVS, par axe ;
- tâche FOC isochrone sur le core 0, avec métriques ;
- API de commande utilisable par la ligne de commande et, plus tard, par la
  boucle d'équilibrage, avec failsafe ;
- OTA, ligne de commande série, outillage de flash ;
- diagnostics `vcap` / `vdump` / `jlog` / `dt`.

Hors périmètre :

- sense de courant, couple en ampères mesurés, PID de courant ;
- anticogging ;
- IMU ICM45686, fusion de pitch, boucle d'équilibrage ;
- liaison avec le STM32 ou la télémétrie.

## 3. Repris de HardwareCheck

### 3.1 Encodeur PCNT

- Le compteur matériel est 16 bits et **se remet à zéro** aux limites, au lieu
  de déborder en complément à deux. Limites symétriques à ±16384, dépliage
  logiciel à chaque lecture dans un accumulateur 64 bits, sans ISR de limite ni
  `pcnt_counter_clear`.
- `getAngle()` et `getVelocity()` sont calculés depuis ce compteur, jamais
  depuis `full_rotations` de `Sensor`. Cette heuristique suppose un `update()` à
  cadence régulière ; un seul raté à une transition de mode fausse la vitesse
  définitivement.
- `getSensorAngle()` reste dans [0, 2π). SimpleFOC 2.3.3 traite une valeur
  négative comme une erreur et `update()` sort sans toucher à l'angle.
- Seul le **premier** front Z fixe l'origine. Rebaser le compteur sur les fronts
  suivants ferait sauter l'angle de π ou 2π et le PID de vitesse réagirait.
- Fenêtre de calcul de vitesse : `ENC_VEL_MIN_DT`, ramenée de 2 ms à **1 ms**
  (voir section 7).
- Filtre de glitch PCNT : 250 cycles APB, environ 3 µs.

### 3.2 Boucle temps réel

- Timer TG1_T0, ISR enregistrée **depuis la tâche** pour qu'elle soit allouée
  sur le core 0, puis `vTaskNotifyGiveFromISR`.
- Tâche `foc` épinglée sur le core 0, priorité 20, `disableCore0WDT`.
- Métriques `dt`, `dtmax`, `late`, et purge du backlog de notifications en fin
  d'itération. Sans cette purge, un dépassement fait que `ulTaskNotifyTake`
  rend la main immédiatement, la tâche IDLE0 ne tourne plus et le watchdog
  abat la tâche.
- `hz` réglable à chaud, borné entre 4 kHz et 16 kHz.
- Rien sur le port série depuis la tâche FOC.

### 3.3 Armement et consignes

- `enableIfNeeded` : `motor.enable()` de SimpleFOC fait toujours
  `setPwm(0,0,0)` et remet les PID à zéro. Réarmer à chaque commande arrête le
  shaft avant d'appliquer la nouvelle cible.
- Changement de consigne « live » : si l'axe est déjà dans le bon mode, on
  n'écrit que la cible.
- `primeEncoderMotion()` avant de passer en boucle fermée, pour que la vitesse
  ne parte pas d'un échantillon périmé.

### 3.4 Calibration

Séquence par axe : recherche Z en open-loop, détection du sens par rotation
open-loop, puis zéro électrique pris avec montée en rampe et slew doux jusqu'à
3π/2. L'`initFOC` natif fait des échelons de tension, ce qui secoue le shaft.
L'enregistrement NVS permet de ne refaire qu'un `zsearch` au boot.

### 3.5 Matériel et outillage

- `M_EN` (GPIO12) tenu bas avant tout le reste, parce que c'est une broche de
  strap.
- Broches PWM d'un axe absent tenues basses.
- `FOC_VBUS` en flag de build, affiché au boot. Une valeur fausse change le
  rapport tension demandée sur tension appliquée et déstabilise les boucles :
  c'est la cause racine du bug 8.4 V contre 12.6 V.
- `download` : hold RTC sur GPIO0 puis `esp_restart`, avec
  `scripts/esptool_nodtr.py` et `scripts/wrap_uploader.py`, parce que le CH340
  pulse DTR à l'ouverture du port et casse le hold.
- Identifiants WiFi dans un `platformio.local.ini` non commité, qui ne
  surcharge **que** la section `[wifi]`. PlatformIO **remplace** une clé
  `build_flags` redéfinie dans un fichier inclus, il ne la fusionne pas.
- OTA : arrêt des axes, pause du timer FOC, `disableCore1WDT`, et `loop()`
  réduit à `vTaskDelay(1)` pendant l'upload.

## 4. Abandonné

- `dma_adc.*`, `dma_current_sense.*`, `mcpwm_driver.*` et le hook ISR MCPWM ;
- `calCurrentOffsets`, `csflip`, `csoff`, `ilim`, `FOC_I_LIM`, PID `current_q`
  et `current_d` ;
- `anticog.*`, `accal`, `ac on|off|save|forget` ;
- `DESIGN_CURRENT_FOC.md` ;
- `tq` en ampères : l'ancien `tqv` devient `tq`, en volts ;
- le magic NVS `HCK2` : les calibrations de HardwareCheck ne sont pas reprises.

## 5. Architecture

### 5.1 Plan des tâches

| Core | Priorité | Tâche | Rôle |
|------|----------|-------|------|
| 0 | 20 | `foc` | encodeurs, `loopFOC`, `move`, failsafe, instantané d'état |
| 1 | 19 | `ctrl` | IMU et équilibrage — **pas dans ce firmware** |
| 1 | 1 | `loop()` Arduino | ligne de commande, OTA |

La boucle d'équilibrage aura sa propre tâche sur le core 1, pas `loop()` :
sinon un `Serial.printf` bloquant ou l'OTA la retardent. C'est la même raison
qui a sorti le FOC de `loop()`.

### 5.2 `Axis`

```cpp
enum class Mode  : uint8_t { Off, Openloop, Velocity, Torque };
enum class Owner : uint8_t { Task, Cli };

struct Axis {
  char name; uint8_t idx; int8_t cmd_sign;   // 'L'/'R', 0/1, -1/+1
  bool present;                              // bit de FOC_AXIS_MASK
  PcntEncoder encoder;
  BLDCDriver3PWM driver;                     // construit SANS enable pin
  BLDCMotor motor;
  volatile Mode mode;
  volatile Owner owner;
  volatile uint32_t cmd_deadline_ms;         // 0 = pas de failsafe
  bool calibrated;
  float voltage_limit, align_voltage;
};
extern Axis axes[2];
```

Les modes `Enc` et `Idle` de HardwareCheck fusionnent dans `Off`. L'affichage
de la ligne encodeur devient une option d'affichage de la ligne de commande,
pas un mode moteur.

`cmd_sign` est appliqué dans la couche de commande, une seule fois :
`FOC_CMD_SIGN_L = -1`, `FOC_CMD_SIGN_R = +1`, repris de `ESP32FOC`. Une
consigne positive fait avancer le robot. La cible SimpleFOC reste dans le
repère moteur.

### 5.3 Propriété d'un axe

Dans HardwareCheck, `goEnc()` écrit `mode` puis appelle `motor.disable()` tout
de suite, alors que la tâche peut être en plein `loopFOC()` et réécrire une
PWM après le disable. Avec `M_EN` partagé et l'autre axe actif, ce n'est pas
inoffensif.

Règle : **la tâche FOC est seule à appeler `encoder.update()`, `loopFOC()` et
`move()` sur un axe dont `owner == Task`.**

- La tâche incrémente `foc_seq` en fin d'itération.
- `syncWithTask()` attend que `foc_seq` ait avancé de 2, ce qui garantit
  qu'une itération complète a vu la nouvelle valeur. Timeout 50 ms, et retour
  immédiat si le timer est en pause, par exemple pendant un OTA.
- Tout changement de mode ou d'armement fait : écrire le champ, `syncWithTask()`,
  puis agir sur le moteur.
- `cal` et `zsearch` prennent l'axe (`owner = Cli`, puis sync), exécutent la
  séquence bloquante sur le core 1 en appelant eux-mêmes `update()`,
  `loopFOC()` et `move()`, puis rendent l'axe. L'autre axe continue de tourner
  pendant ce temps.

Le dépliage 16 bits impose une lecture avant 8192 counts, soit environ 10 ms à
80 rad/s. Comme la tâche appelle `encoder.update()` dans **tous** les modes, y
compris `Off`, cette contrainte est tenue sans dépendre de `loop()`.

### 5.4 Itération de la tâche FOC

```
n = ulTaskNotifyTake(portMAX_DELAY)      ; late += n - 1
t0 = esp_timer_get_time()
pour chaque axe présent :
    si owner != Task            → continuer
    si failsafe dépassé         → cible 0, désarmement
    mode Off                    → encoder.update()
    Openloop/Velocity/Torque    → motor.loopFOC() ; motor.move()
vcap : un échantillon (dcount, vel, uq) sur l'axe capturé
publication de l'instantané d'état
dt, dtmax ; foc_seq++ ; purge du backlog
```

Budget estimé : environ 40 µs par axe en voltage, soit à peu près 80 µs sur les
250 µs d'une période à 4 kHz. À vérifier avec `dtmax` au passage à deux axes.

## 6. Interface entre les cores

La ligne de commande ne parle pas au moteur directement. Elle passe par l'API
que la boucle d'équilibrage utilisera de la même façon.

```cpp
// Repère robot : cmd_sign est appliqué ici.
void driveSetVelocity(uint8_t axis, float rad_s, uint32_t timeout_ms);
void driveSetTorque  (uint8_t axis, float volts, uint32_t timeout_ms);
void driveStop       (uint8_t axis);

struct AxisState { float angle, velocity, uq; bool armed, calibrated; };
bool driveGetState(uint8_t axis, AxisState *out);
```

- Écrire une consigne ne bloque pas. Si l'axe est déjà armé dans le bon mode,
  l'appel se réduit à une écriture de `float`, atomique sur ESP32. Une boucle à
  1 kHz peut appeler ça à chaque cycle.
- `syncWithTask()` n'intervient qu'à l'armement et au désarmement.
- `timeout_ms` est le failsafe. La tâche compare l'horodatage de la dernière
  consigne à l'échéance ; au dépassement, cible à zéro et désarmement. La ligne
  de commande passe `0`, donc pas de timeout. La boucle d'équilibrage passera
  **10 ms**, soit 4 périodes à 400 Hz : assez pour tolérer une commande sautée,
  assez court pour qu'un blocage du core 1 coupe les roues.

Coût réel du flux de consignes, pour une boucle d'équilibrage à 400 Hz : 800
appels par seconde, chacun réduit à un bornage et deux écritures 32 bits
alignées, soit quelques centaines de nanosecondes. Moins de 0.1 % d'un core,
contre environ 32 % pour la tâche FOC elle-même. Le facteur limitant n'est
donc pas le débit d'ordres mais la gigue d'ordonnancement, traitée en
section 13.
- `driveGetState` rend un instantané cohérent, publié une fois par itération et
  protégé par un compteur pair/impair : le lecteur relit si le compteur a bougé.

## 7. Encodeurs

Fichier `pcnt_encoder.*` repris, avec quatre changements :

- ISR Z par instance, via `attachInterruptArg(pin_z, zIsr, this, ENC_Z_EDGE)`.
  Le spinlock devient un membre, `s_z_owner` disparaît.
- Unité PCNT égale à `idx`.
- La vitesse plausible du détecteur de sauts passe par le constructeur ;
  l'encodeur ne dépend plus de `FOC_VEL_LIMIT`.
- Diagnostics conservés : `zEdges`, `jumps`, journal des sauts, compteur de
  dépliages.
- Fenêtre de vitesse dans `config.h` : `ENC_VEL_MIN_DT = 1 ms`. À 2 ms, la
  vitesse n'est recalculée qu'à 500 Hz ; une boucle d'équilibrage à 400 Hz
  lirait alors une valeur dont l'âge varie de 0 à 2 ms, avec un battement entre
  les deux cadences, ce qui coûte de la marge de phase. À 1 ms, la résolution
  reste de 2π / 65536 / 1 ms, soit 0.096 rad/s, largement suffisante.

Câblage ABZ, confirmé au banc :

| Axe | A | B | Z | PCNT |
|-----|---|---|---|------|
| gauche, moteur 0 | IO18 | IO19 | IO22 | unit 0 |
| droite, moteur 1 | IO5 | IO23 | IO21 | unit 1 |

`ENC_PPR` doit correspondre au registre ABZ du MT6835, par défaut 16384. PCNT
en quadrature complète, donc CPR = 4 × PPR.

Point de contrôle à l'étape « deuxième axe » : GPIO5 est une broche de strap,
donc il faut vérifier que le boot reste propre avec le MT6835 branché.

## 8. Calibration et NVS

`cal [L|R]` traite un axe, ou les deux l'un après l'autre :

1. prise de l'axe (`owner = Cli`, puis sync) ;
2. `clearIndex`, puis recherche Z en open-loop à `Z_SEARCH_RPS`, échec au-delà
   de `Z_SEARCH_TURNS` ;
3. rotation open-loop à 3 rad/s pendant 500 ms, qui donne le sens ; échec si le
   shaft bouge de moins de 0.25 rad ;
4. contrôle de cohérence : ratio entre mouvement mesuré et mouvement attendu,
   environ 1.5 rad. Hors de l'intervalle 0.7 à 1.3, un **avertissement** signale
   un `ENC_PPR` différent du registre du chip, ou un nombre de paires de pôles
   faux. C'est le contrôle que fait `initFOC` et que HardwareCheck avait perdu ;
   il n'échoue pas la calibration, car un glissement en open-loop est possible ;
5. montée en rampe, slew jusqu'à 3π/2, stabilisation 400 ms, lecture du zéro
   électrique, descente en rampe ;
6. `initFOC()`, qui saute l'alignement puisque sens, zéro et index sont connus ;
7. `calibrated = true`, puis restitution de l'axe.

`zsearch [L|R]` fait la recherche Z, recharge le NVS et appelle `initFOC()`.

Aucun mouvement automatique au boot : `zsearch` est tapé à la main. La
recherche automatique viendra avec l'équilibreur.

`requireCal` refuse `vel` et `tq` tant que calibration et index ne sont pas
tous les deux présents. `ol` reste toujours permis.

NVS, namespace `drive`, clés `cal0` et `cal1` :

```cpp
struct CalRecord {
  uint32_t magic;              // 'DRV1'
  float zero_electric_angle;
  int8_t sensor_direction;
  uint8_t pole_pairs;
  uint16_t enc_ppr;
  char axis;
};
```

`enc_ppr` est vérifié au chargement : changer `ENC_PPR` invalide la
calibration au lieu de l'appliquer fausse. `save [L|R]` et `forget [L|R]`
agissent par axe.

## 9. Commandes moteur et limites

| Commande | Mode SimpleFOC | Cible | Calibration requise |
|---|---|---|---|
| `ol <rad/s>` | `velocity_openloop` | vitesse | non |
| `vel <rad/s>` | `velocity` + couple voltage | vitesse | oui |
| `tq <V>` | `torque` + couple voltage | Uq en volts | oui |

Le couple estimé en ampères est **affiché seulement** :
`I_est = (Uq − BEMF) / R`, avec `R = 5.2 Ω` et `KV = 220`. Renseigner
`phase_resistance` dans SimpleFOC changerait aussi le sens des gains du PID de
vitesse et ferait raisonner en ampères non mesurés ; on ne le fait pas.

`applyLimits(axis)` reste le point unique :

```
driver.voltage_limit = motor.voltage_limit = PID_velocity.limit = voltage_limit
motor.velocity_limit = FOC_VEL_LIMIT
```

La sortie du PID de vitesse est en volts, donc sa limite est `voltage_limit`.
`P_angle` n'est pas configuré : il n'y a pas de mode position dans ce
firmware. `FOC_VEL_LIMIT` sert aussi à borner les consignes `ol` et `vel`.

`M_EN` (GPIO12) est commun aux deux axes. Les drivers sont donc construits
**sans** enable pin — `motor.enable()` et `motor.disable()` ne touchent plus
qu'à la PWM — et un module carte tient un compteur d'axes armés : `M_EN` passe
à HIGH au premier axe armé, et retombe à LOW quand plus aucun ne l'est. Un axe
masqué garde ses trois broches PWM basses, sans PCNT ni ISR.

Les deux moteurs partagent MCPWM0 : SimpleFOC met le premier sur l'opérateur A
et le second sur l'opérateur B, avec les mêmes timers. La fréquence PWM est
donc commune aux deux axes.

## 10. Valeurs par défaut

| Paramètre | HardwareCheck | ESP32FOCDrive | Raison |
|---|---|---|---|
| `FOC_VBUS` | 12.6 | 12.6 | 3S, affiché au boot |
| `FOC_VOLTAGE_LIMIT` | 6.0 | **3.0** | 6 V sur 5.2 Ω font 1.15 A à l'arrêt ; 3 V en font 0.58 A. On remonte avec `limit`. |
| `FOC_VOLTAGE_ALIGN` | 4.0 | **2.0** | 0.38 A pendant l'alignement, sans chauffe |
| `FOC_VEL_P` | 0.12 | **0.2** | les anciens gains sortaient des ampères |
| `FOC_VEL_I` | 0.06 | **2.0** | idem |
| `FOC_VEL_LPF` | 3 ms | **5 ms** | idem |
| `FOC_VEL_RAMP` | 5 | **50** | 5 V/s mettrait une seconde pour atteindre la limite |
| `FOC_VEL_LIMIT` | 80 | 80 | inchangé |
| `FOC_PWM_HZ` | 20 k | 20 k | inchangé |
| `FOC_MODULATION` | SinePWM | SinePWM | en flag, voir section 13 |
| `FOC_LOOP_HZ` | 4 k | 4 k | plancher 4 k, plafond 16 k |
| `FOC_POLE_PAIRS` | 7 | 7 | 12N14P |
| `ENC_PPR` | 16384 | 16384 | registre ABZ du MT6835 |
| `ENC_VEL_MIN_DT` | 2 ms | **1 ms** | vitesse à 1 kHz, pour une boucle d'équilibrage à 400 Hz |

Les gains de vitesse sont un point de départ, à retuner au banc avec `vcap`.

## 11. Ligne de commande

Préfixe d'axe optionnel partout : `L`, `R`, ou rien pour les deux.

```
help  status  axes
enc [0|1]            ligne encodeur : une fois, ou en continu à 1 Hz
ol <rad/s>           vel <rad/s>       tq <V>        idle
cal  zsearch  save  forget
limit <V>  alignv <V>  hz <Hz>  mon 0|1
vcap [ms]  vdump  jlog  dt
ota  wifioff  download
```

- `status` : une ligne par axe — mode, armement, calibration, index, compteur,
  angle, vitesse, cible, Uq, `I_est`, zéro, sens. Puis une ligne globale — `hz`,
  `dt`, `dtmax`, `late`, `loops`, état de `M_EN`, état du WiFi.
- `vcap [L|R] [ms]` capture un seul axe, buffer de 2500 échantillons avec
  décimation automatique. `vdump` sort le CSV : index, dcount, vitesse, Uq.
- `axes` affiche le masque de build et les broches, pour savoir quel binaire
  tourne.
- Parsing repris de HardwareCheck : lecture caractère par caractère, écho,
  backspace, ligne de 96 octets.

## 12. Réseau, OTA, arborescence

STA avec 8 s de timeout, puis repli en AP `ESP32FOCDrive`. Hostname
`esp32focdrive`. Au démarrage d'un upload : `driveStop` sur les deux axes,
`M_EN` bas, pause du timer FOC, `disableCore1WDT`, et `loop()` réduit à
`vTaskDelay(1)`. Timeout de 60 s, reprise sur erreur.

```
ESP32FOCDrive/
  platformio.ini          [wifi] + envs left / right / dual, variantes _ota
  platformio.local.ini    non commité, identifiants WiFi
  scripts/                wrap_uploader.py, esptool_nodtr.py
  include/ + src/
    config.h              masque d'axes, table de broches, gains, modulation
    pcnt_encoder.*        PCNT, Z par instance, diagnostics
    axis.*                Axis, modes, cal, zsearch, NVS, applyLimits
    board.*               M_EN avec compteur, broches au repos, download
    foc_task.*            timer, tâche, dt/late, failsafe, vcap, état publié
    drive_api.h           driveSetVelocity / driveSetTorque / driveStop / driveGetState
    cli.*                 parsing avec préfixe d'axe, status, diagnostics
    net.*                 WiFi et OTA
    main.cpp              setup et loop
  README.md               câblage, séquence de mise au point, commandes
```

Envs PlatformIO : `left` vaut `FOC_AXIS_MASK=0b01`, `right` vaut `0b10`,
`dual` vaut `0b11`, chacun avec sa variante `_ota` par `extends`. Plateforme
`espressif32 @ 7.0.1`, framework Arduino, `askuric/Simple FOC @ 2.3.3`,
`lib_archive = false`.

## 13. Risques et points de vigilance

**Modulation contre limite de tension.** En SinePWM, la modulation est centrée
et le Uq exploitable plafonne vers `Vbus/2`, soit environ 6.3 V à 12.6 V de
bus. Au-delà, les rapports cycliques saturent et la forme d'onde se déforme.
La version robot vise `FOC_VOLTAGE_LIMIT = 12.6` : il faudra soit rester sous
6.3 V, soit passer en `SpaceVectorPWM`, qui monte à `Vbus/√3`, environ 7.3 V.
La modulation est donc un flag de `config.h` dès le départ.

**Thermique à limite haute.** 12.6 V sur 5.2 Ω font 2.4 A à rotor bloqué, au
delà de ce qu'encaisse un 2804. `I_est` est une alerte, pas une protection.

**GPIO5.** Broche de strap, utilisée pour A de l'axe droit. Boot à vérifier
avec le MT6835 branché.

**Charge à 4 kHz sur deux axes.** À valider par `dtmax` avant d'aller plus haut
en fréquence.

**Gigue d'ordonnancement, WiFi actif.** C'est la contrainte qui décidera de la
qualité de l'équilibrage, bien avant le débit de consignes. Relevé dans le SDK
de ce framework :

| Élément | Valeur |
|---|---|
| `configMAX_PRIORITIES` | 25 |
| Tick FreeRTOS | 1 kHz |
| Tâches WiFi | épinglées **core 0** |
| Tâche TCP/IP (lwIP) | priorité 18, **core 0** |
| `esp_timer` | priorité 22 |
| `loop()` et tâche d'événements Arduino | core 1 |

Sur le core 0, la tâche FOC à la priorité 20 peut donc être préemptée par
`esp_timer` (22) et par la tâche WiFi (priorité 23 d'après la documentation
IDF, valeur non vérifiable ici car elle vit dans les blobs précompilés). C'est
ce que `dtmax` et `late` mesurent, et c'est la raison d'être de `wifioff`.
Leviers, dans l'ordre : mesurer, puis monter la tâche FOC à 23 ou 24, puis
couper le WiFi pendant l'équilibrage.

Sur le core 1, la tâche d'événements Arduino tourne à la priorité 20. `ctrl`
est donc prévue à **19** : elle ne sera décalée que par un événement réseau,
par exemple une reconnexion WiFi.

## 14. Mise au point, étape par étape

Chaque étape a son critère de validation au banc, roue levée.

1. **SimpleFOC natif.** Un axe en open-loop, PWM et `M_EN` corrects. Le shaft
   tourne.
2. **PCNT.** `cnt` bouge à la main, `idx` passe à 1 au premier Z.
3. **Timer et tâche.** `dtmax` reste stable pendant `ol 20` avec l'USB actif.
4. **Calibration.** `cal`, puis `save`, puis reboot, puis `zsearch` : le zéro
   revient et le ratio du contrôle de cohérence est proche de 1.
5. **`vel` et `tq`.** `vel 3` tourne rond, `tq 1` donne un couple au doigt,
   `tq 0` laisse la roue presque libre.
6. **API et failsafe.** Une consigne avec `timeout_ms` non nul coupe l'axe si
   elle n'est pas rafraîchie.
7. **OTA.** Upload sans couper le port série, moteur arrêté pendant le flash.
8. **Deuxième axe.** Boot propre avec GPIO5, les deux axes calibrés, `dtmax`
   relevé à deux axes.
9. **Diagnostics.** `vcap` et `vdump` sur un `vel 3`, `jlog` à zéro saut.
10. **Gigue, avant de brancher l'équilibrage.** Relever `dtmax` et `late` sur
    plusieurs minutes de `vel 3` à deux axes, une fois WiFi actif et une fois
    après `wifioff`. L'écart entre les deux chiffres décide s'il faut monter la
    priorité de la tâche FOC ou couper le WiFi en fonctionnement.

## 15. Plus tard

- Protection anti-blocage : couper l'axe au-delà d'un seuil de `I_est` maintenu
  à vitesse quasi nulle. À faire **avant** de remonter `FOC_VOLTAGE_LIMIT` pour
  le robot.
- Tâche `ctrl` sur le core 1 : IMU ICM45686, fusion de pitch, équilibrage.
- Recherche d'index automatique au boot.
- Liaison avec le STM32 ou la télémétrie.
- Éventuel passage en `SpaceVectorPWM`.
