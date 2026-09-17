# ESP32FOCDrive

Firmware pour la carte MKS ESP32 FOC V2.0 : deux moteurs brushless MKS
FS2804 (12N14P, 7 paires de pôles) pilotés en FOC **voltage** (pas de sense
de courant), deux encodeurs MT6835 lus en ABZ via le PCNT matériel de
l'ESP32. Ce firmware livre les roues pilotables et l'interface de commande ;
l'IMU et la boucle d'équilibrage sont un sous-projet séparé, non présent ici.

Conception complète, choix et raisons : voir
[`docs/superpowers/specs/2026-09-16-esp32focdrive-design.md`](../docs/superpowers/specs/2026-09-16-esp32focdrive-design.md).
Ce README est le mode d'emploi ; la spec est le pourquoi.

## 1. Câblage des deux axes

| Axe | Nom | PWM (UH/VH/WH) | ABZ (A/B/Z) | Unité PCNT | Sens robot (`cmd_sign`) |
|---|---|---|---|---|---|
| Gauche | `L`, moteur 0 | IO32 / IO33 / IO25 | IO18 / IO19 / IO22 | 0 | -1 |
| Droite | `R`, moteur 1 | IO26 / IO27 / IO14 | IO5 / IO23 / IO21 | 1 | +1 |

`M_EN` (GPIO12) est le seul signal d'activation des gate drivers BL9342, et
il est **partagé par les deux axes** : les deux `BLDCDriver3PWM` sont
construits sans broche d'enable propre, et un compteur d'axes armés dans
`board.cpp` fait passer `M_EN` à `HIGH` dès qu'un axe s'arme et à `LOW`
seulement quand plus aucun n'est armé. Armer un seul axe alimente donc les
deux demi-ponts ; désarmer un seul axe ne coupe rien tant que l'autre tourne.

GPIO5 (A de l'axe droit) et GPIO12 (`M_EN`) sont des broches de strap au
boot : `boardInit()` tient `M_EN` bas avant toute autre initialisation, et le
boot a été vérifié propre avec le MT6835 branché sur GPIO5 (tâche 9).

Un axe absent du masque de build (`FOC_AXIS_MASK`, voir §6) a ses trois
broches PWM tenues basses et ne reçoit ni PCNT ni ISR.

## 2. Flash : l'USB est cassé, l'OTA est le chemin normal

**Défaut matériel de cette carte, à ne jamais oublier :** l'auto-reset par
DTR/RTS du CH340 ne fonctionne pas dessus. Un `pio run -t upload` classique
n'entraîne aucun redémarrage automatique en mode téléchargement, et **rien
ne redémarre non plus tout seul quand le flash est fini** — il n'y a pas de
message final évident sur le terminal série à observer, l'esptool wrappé
(`scripts/esptool_nodtr.py`) tente d'abord d'envoyer la commande `download`
sur le port série pour amorcer un firmware déjà présent, puis retombe sur un
geste manuel s'il n'y a pas d'application à l'écoute (premier flash, ou
application plantée) :

```
enter_download: <port> -> send 'download'
...
enter_download: first flash / dead app → R13 + Reset
```

C'est ce message, ou l'absence de toute activité série suivie d'un échec
`esptool` (timeout de synchronisation), qui dit qu'il faut faire le geste à
la main : **maintenir BOOT (GPIO0 à la masse), appuyer sur RESET, relâcher
RESET puis BOOT**, puis relancer l'upload. Une fois l'upload terminé (barre
de progression `esptool` à 100 % puis retour au prompt), la carte reste en
mode téléchargement — **il faut de nouveau presser RESET à la main** pour
relancer l'application ; rien ne le fait automatiquement. C'est le seul
repère fiable : pas de reboot spontané, ni au début ni à la fin d'un flash
USB.

Conséquence : chaque flash USB coûte deux manipulations physiques. **L'OTA
est donc le chemin normal d'itération, pas une commodité.** La séquence :

1. **Un seul flash USB**, pour amorcer un firmware qui embarque déjà l'OTA
   (n'importe quel firmware de ce dépôt, puisque `netSetup()` tourne dans
   tous les environnements) :
   ```bash
   ~/.platformio/penv/Scripts/pio.exe run -e dual -t upload
   ```
   avec le geste BOOT+RESET ci-dessus si besoin, à l'amorçage comme à la fin.
2. Ouvrir le moniteur série (115200 bauds) sur le bon port — voir le piège
   COM1 au §9 — et taper `ota` pour lire l'adresse retenue :
   ```
   ota: STA <ssid>  ip=192.168.x.y  host=esp32focdrive
   ```
   (repli en point d'accès `ESP32FOCDrive` si le WiFi configuré ne répond pas
   en 8 s, IP alors fixe `192.168.4.1` — voir §9 pour les identifiants).
3. Reporter cette IP dans `upload_port` de l'environnement `_ota` choisi
   (`env:left_ota`, `env:right_ota`, `env:dual_ota` dans `platformio.ini`,
   valeur `192.168.1.1` par défaut, à écraser).
4. Téléverser :
   ```bash
   ~/.platformio/penv/Scripts/pio.exe run -e dual_ota -t upload
   ```
   Plus jamais de câble ensuite, tant que la carte reste alimentée et que le
   WiFi répond. `ArduinoOTA` coupe les deux axes, met le timer FOC en pause
   et désactive le watchdog du cœur 1 le temps du transfert
   (`net.cpp:onStart`), et les réactive à la fin ou en cas d'erreur
   (`onError`).

## 3. `selftest` : valider sans câble

Cette carte ne peut pas être flashée en USB sans le geste BOOT+RESET
manuel décrit ci-dessus. Faire tourner la suite Unity sur cible coûterait
donc **deux** manipulations physiques par exécution (flasher le binaire de
test, puis reflasher l'application). Les vérifications de logique pure
(parsing de commande, dépliage d'encodeur, validité d'un enregistrement de
calibration, expiration du failsafe) vivent donc dans un en-tête partagé,
`include/self_test.h`, appelé par **deux** exécuteurs : la suite Unity
(`test/test_logic/`, toujours utilisable en local, sans cible) et la
commande `selftest` de l'application elle-même, qui elle arrive par OTA.

```
> selftest
selftest: parse_bare PASS
selftest: parse_value PASS
...
selftest: fs_wrap_expired PASS
selftest: 22 run, 0 failed
```

`selftest` ne pilote aucun moteur et ne touche à aucune broche : elle est
sûre à tout moment, axes armés ou non. Un `N failed` non nul après un
changement de code signale une régression sur la logique pure avant même de
toucher au banc.

## 4. Séquence de mise au point

Dans l'ordre, roues levées, sur l'environnement `dual` (ou `left`/`right`
pour un seul axe) :

| Étape | Commande | Ce qui vaut succès |
|---|---|---|
| 1 | `selftest` | `N run, 0 failed` — la logique pure (parsing, dépliage d'encodeur, validité de calibration, failsafe) est correcte sur ce binaire avant de toucher au moteur |
| 2 | `enc` (ou `enc L` / `enc R`) | `cnt` bouge quand on tourne l'arbre à la main ; `idx` passe à 1 au premier passage par l'index Z ; les niveaux `A`/`B`/`Z` affichés cohérents avec le sens de rotation |
| 3 | `cal` (ou `cal L` / `cal R`) | Séquence : recherche Z en open-loop, rotation courte pour détecter le sens, montée en rampe et alignement électrique — le shaft bouge à chaque étape. Message final `cal L: ok ...` (pas `FAIL`), et le ratio mouvement mesuré / attendu affiché reste proche de 1 (avertissement hors de 0.7–1.3, sans faire échouer la calibration) |
| 4 | `save` (ou `save L` / `save R`) | Message `save L: zero=... dir=... pp=7 ppr=16384 (reboot: zsearch then this zero)`, pas `nvs fail` ni `write fail` |
| 5 | redémarrage (reset physique, ou coupure/remise sous tension) | `status` affiche `cal=1` avant tout `zsearch` — c'est la calibration NVS rechargée depuis `axisInitAll()`, pas encore l'index Z du boot en cours |
| 6 | `zsearch` (ou `zsearch L` / `zsearch R`) | Message `zsearch L: ok  cnt=...` puis `zsearch L: electrical ok  initFOC=1` — l'index Z est retrouvé et le zéro électrique déjà connu est réappliqué sans nouvelle rotation d'alignement |
| 7 | `vel 3` (ou `vel L 3` / `vel R 3`) | La roue tourne rond, sans à-coups ni bruit de commutation excessif ; `status` montre `mode=VEL armed=1` et `vel≈3.0` qui se stabilise |
| 8 | `tq 1` puis `tq 0` | `tq 1` : couple perceptible au doigt qui résiste à un freinage léger. `tq 0` : la roue tourne presque librement à la main (couple résiduel de cogging/frottement seulement) |

`vel` et `tq` refusent de s'exécuter tant que l'axe n'est pas à la fois
calibré et indexé (`requireCal`, voir la spec §8) — `need cal L` sur la
console signale ce refus.

## 5. Commandes, par usage

Préfixe d'axe optionnel devant la valeur : `L`, `R`, ou rien pour tous les
axes présents dans le build (`FOC_AXIS_MASK`). Sortie complète disponible
avec `help`.

**État et diagnostic sans risque**
```
help / ?            liste des commandes
status               une ligne par axe (mode, armé, cal, index, angle,
                      vitesse, cible, Uq, I_est, zéro, sens) + une ligne
                      globale (hz, dt, dtmax, late, loops, M_EN, WiFi)
axes                 masque de build, broches PWM/ABZ, unité PCNT, cal
mon [0|1]            active/désactive le flux status en continu (1 Hz)
selftest             vérifications de logique pure (§3)
```

**Mouvement**
```
ol [L|R] <rad/s>     vitesse open-loop, sans calibration requise
vel [L|R] <rad/s>    vitesse en boucle fermée (PID voltage), calibration requise
tq [L|R] <V>         couple en volts (Uq direct), calibration requise
idle / stop [L|R]    désarme l'axe (ou les deux)
fs [L|R] <ms>        démo/test du failsafe : vel=3 rad/s qui expire après <ms>
```

**Calibration**
```
cal [L|R]            recherche Z, détection de sens, alignement électrique
zsearch [L|R]        recherche Z seule, recharge le zéro NVS existant
save [L|R]           écrit la calibration en NVS
forget [L|R]         efface la calibration NVS de l'axe
```

**Configuration d'axe**
```
limit [L|R] <V>      limite de tension appliquée (voltage_limit)
alignv [L|R] <V>      tension utilisée pendant l'alignement
hz [Hz]              cadence de la tâche FOC, bornée [4000, 16000]
```

**Encodeur**
```
enc [0|1]            ligne encodeur brute (cnt/idx/zEdges/pins/angle) ;
                      une fois, ou en continu à 1 Hz avec `enc 1`
```

**Diagnostics avancés**
```
vcap [L|R] [ms]      capture par échantillon (dcount, vitesse, Uq) d'un seul
                      axe, tampon 2500 échantillons, décimation automatique
vdump                sort la capture en CSV
jlog [L|R]           compteurs de sauts/débordements PCNT + journal des sauts
dt                    remet à zéro dtmax/late (métriques de la tâche FOC)
```

**Réseau et flash**
```
ota                   affiche le mode WiFi courant et l'IP à utiliser en OTA
wifioff               coupe le WiFi jusqu'au prochain reboot
download / dl         arrête les deux axes puis force le mode téléchargement
                      ROM (utilisé par le flash USB, pas un usage manuel courant)
```

## 6. Réglages retenus

Les valeurs par défaut de `include/config.h`, avec l'environnement de build
qui fixe `FOC_AXIS_MASK` (`left`=`0b01`, `right`=`0b10`, `dual`=`0b11`,
`platformio.ini`) :

| Paramètre | Valeur | Rôle |
|---|---|---|
| `FOC_VBUS` | 12.6 V | tension de bus déclarée (3S), affichée au boot |
| `FOC_VOLTAGE_LIMIT` | 3.0 V | limite de tension appliquée ; 6 V sur 5.2 Ω feraient 1.15 A à l'arrêt, 3 V en font 0.58 A |
| `FOC_VOLTAGE_ALIGN` | 2.0 V | tension pendant l'alignement de calibration |
| `FOC_VEL_P` / `FOC_VEL_I` / `FOC_VEL_D` | 0.2 / 2.0 / 0.0 | gains du PID de vitesse, sortie en volts (pas de boucle de courant) |
| `FOC_VEL_LPF` | 5 ms | filtre passe-bas sur la vitesse estimée |
| `FOC_VEL_RAMP` | 50 V/s | pente de montée en tension |
| `FOC_VEL_LIMIT` | 80 rad/s | borne haute des consignes `ol`/`vel` et de `motor.velocity_limit` |
| `FOC_PWM_HZ` | 20 kHz | fréquence PWM |
| `FOC_MODULATION` | SinePWM | Uq exploitable plafonne vers `Vbus/2` (~6.3 V) ; passer en SpaceVectorPWM si `FOC_VOLTAGE_LIMIT` doit dépasser cette borne |
| `FOC_LOOP_HZ` | 4000 Hz (borné 4000–16000) | cadence de la tâche FOC, réglable à chaud par `hz` |
| `FOC_POLE_PAIRS` | 7 | FS2804 12N14P |
| `ENC_PPR` | 16384 | registre ABZ du MT6835 ; CPR = 4×PPR en quadrature complète |
| `ENC_VEL_MIN_DT` | 1 ms | fenêtre de calcul de vitesse |

**Ces gains — en particulier `FOC_VEL_P`, `FOC_VEL_I`, `FOC_VEL_LPF` et
`FOC_VEL_RAMP` — sont un point de départ jamais validé au banc.** Ils
viennent d'un raisonnement sur les unités (sortie en volts, pas en ampères)
et non d'un essai roue levée. À retoucher en observant la roue, avec `vel`
et `vcap`/`vdump` pour visualiser une oscillation ou un dépassement, avant
tout montage sur le robot.

## 7. Gigue d'ordonnancement — à relever au banc

**Pas encore mesuré. Ne pas se fier à une valeur qui ne serait pas
relevée ici avec la date de la mesure.**

Procédure exacte (env `dual`, roues levées) :

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

Emplacements à compléter :

| Condition | `dtmax` (µs) | `late` |
|---|---|---|
| WiFi actif, 5 min à `vel 3` | *à relever au banc — voir la procédure ci-dessus* | *à relever au banc* |
| Après `wifioff`, 5 min à `vel 3` | *à relever au banc — voir la procédure ci-dessus* | *à relever au banc* |

Ce que ces deux chiffres décident : la tâche `foc` tourne à la priorité 20
sur le cœur 0, sous la tâche WiFi (épinglée sur le même cœur, priorité
documentée supérieure à 20 côté IDF, non vérifiable directement dans ce
projet car elle vit dans les blobs précompilés) et sous `esp_timer`
(priorité 22). Si l'écart entre les deux lignes du tableau est significatif,
deux leviers, dans l'ordre : monter la priorité de la tâche `foc` (20 → 23
ou 24), puis, si ça ne suffit pas, couper le WiFi pendant l'équilibrage
(`wifioff`, ou ne jamais l'activer sur le firmware embarqué). Voir la spec
§13 pour le détail de l'arbitrage de priorités FreeRTOS sur ce SDK.

## 8. Deux pièges déjà payés une fois

**`platformio.local.ini` ne doit surcharger QUE la section `[wifi]`.**
PlatformIO **remplace** une clé `build_flags` redéfinie dans un fichier
inclus, il ne la fusionne pas. Un `platformio.local.ini` qui écrirait
directement dans `[env]` `build_flags` ferait disparaître silencieusement
tous les flags déjà posés dans `platformio.ini` — c'est exactement ce qui a
fait sauter `FOC_VBUS` sur le projet précédent (`ESP32FOCHardwareCheck`),
sans erreur de compilation pour le signaler. La bonne forme, non commitée
(`**/platformio.local.ini` dans `.gitignore`) :
```ini
[wifi]
build_flags =
    -DWIFI_SSID=\"votre_ssid\"
    -DWIFI_PASS=\"votre_mot_de_passe\"
```
`platformio.ini` référence cette section via `${wifi.build_flags}`, qui
s'ajoute donc aux flags existants au lieu de les remplacer.

**Le port COM1 n'est jamais la carte.** C'est l'UART de la carte mère (port
série hérité du PC), pas le CH340 de la carte ESP32FOCDrive. Le CH340
apparaît sous un autre numéro (par exemple `COM6` au moment de ce chantier,
confirmé au Gestionnaire de périphériques sous « USB-SERIAL CH340K ») —
**à revérifier à chaque branchement**, Windows peut en changer. Envoyer un
flash ou ouvrir le moniteur sur COM1 échoue silencieusement ou parle à autre
chose ; ce n'est jamais un symptôme du défaut matériel décrit au §2.

## 9. Pour aller plus loin

Choix de conception, raisons de chaque écart avec `ESP32FOCHardwareCheck`,
architecture des deux cœurs, protocole de propriété d'axe, format NVS de la
calibration, risques connus (thermique à tension haute, modulation contre
limite de tension, gigue d'ordonnancement) : voir
[`docs/superpowers/specs/2026-09-16-esp32focdrive-design.md`](../docs/superpowers/specs/2026-09-16-esp32focdrive-design.md).
