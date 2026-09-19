# Algorithme d'équilibrage — analyse détaillée

Analyse ligne-à-ligne de la loi de commande d'équilibrage réellement embarquée sur le STM32H743,
telle qu'implémentée dans `control_strategy_ff_cascade.c` (stratégie **`ff_cascade`**, seule
stratégie enregistrée dans `control_strategy.c`). Ce document complète
[CONTROL_ARCHITECTURE.md](CONTROL_ARCHITECTURE.md) (qui décrit encore une loi conceptuelle en
mode vitesse, obsolète par rapport au code) et [ANTIPATINAGE.md](ANTIPATINAGE.md) (détection roue
en l'air, référencée mais pas dupliquée ici).

**Portée :** estimation d'état (IMU), boucle de commande 500 Hz, paramètres réglables, valeurs
validées au banc, et points d'attention relevés pendant l'analyse.

---

## 1. Vue d'ensemble

Le robot est un pendule inversé deux roues, commandé en **couple** (`SET_INPUT_TORQUE` CAN vers
deux ODrive, un par roue, cf. `task_motor_tx.c` / `odrive_torque_mode_startup.c` — pas de mode
vitesse en fonctionnement normal). La loi de commande tourne dans `task_control` à **500 Hz**
(`APP_CONTROL_PERIOD_MS = 2`), pilotée par un pas de temps fixe `dt_s = 2 ms` (pas de mesure de dt
réel dans la boucle de contrôle — cf. §7).

Chaîne : IMU (1 kHz, fusion complémentaire) → `task_control` (500 Hz) → `control_strategy_update`
→ `wheel_contact_update` (overlay antipatinage) → `app_motor_command` → `task_motor_tx` (500 Hz)
→ CAN `SET_INPUT_TORQUE` ×2.

```
IMU (1 kHz, SPI)          Encodeur ABZ (500 Hz)     ODrive ×2 (CAN, 100 Hz + RTR)
   │ complementary filter        │                          │
   ▼                             ▼                          ▼
pitch, pitch_rate ──────► control_strategy_ff_cascade_update() ◄── vel_motor_l/r, yaw_rate
   (IMU gyro Z)                  │
                                 ▼
                     wheel_contact_update()  (antipatinage overlay, désactivé par défaut)
                                 │
                                 ▼
                    torque_left_nm, torque_right_nm  ──► CAN SET_INPUT_TORQUE
```

---

## 2. Estimation d'état

### 2.1 Angle et vitesse angulaire (tangage)

`imu_fusion.c` — **filtre complémentaire** classique, mis à jour à chaque échantillon IMU (1 kHz,
`task_imu.c`) :

```
pitch_accel = atan2(-a[forward], a[up])                       // inclinaison "gravité"
pitch_rate  = sign * gyro[axis]                                 // vitesse angulaire brute gyro

si accel valide (‖a‖ ≈ g ± tol) :
    pitch = α·(pitch_prev + pitch_rate·dt) + (1-α)·pitch_accel   // α = 0.999 (APP_IMU_COMPLEMENTARY_ALPHA)
sinon (accélération linéaire suspecte — poussée sur roue) :
    pitch = pitch_prev + pitch_rate·dt                           // intégration gyro pure
```

- `accel_ok` teste `|‖a‖ - g| ≤ 3.0 m/s²` (`APP_IMU_ACCEL_NORM_TOL_MPS2`) : quand le robot
  accélère fort (roues qui poussent), la norme de l'accéléromètre s'écarte de *g* et la correction
  gravité est **coupée automatiquement** — seul le gyro intègre pendant cette fenêtre (protection
  contre le classique biais "accel corrompu par accélération linéaire" des filtres complémentaires
  sur véhicule).
- α = 0.999 à 1 kHz ⇒ constante de temps du filtre passe-haut gyro ≈ 1 s ; la correction gravité
  ne pèse que ≈0.1 %/échantillon — **correction lente**, dérive gyro corrigée sur plusieurs
  secondes seulement.
- Au premier échantillon (`sample_count == 0`), le filtre part de `pitch_accel` (ou 0 si
  l'accéléromètre est invalide) plutôt que 0 — évite un transitoire si le robot est déjà penché à
  la mise sous tension.
- Axes configurés en dur dans `app_config.h` : `APP_IMU_PITCH_ACCEL_FORWARD_AXIS=1`,
  `..._UP_AXIS=0`, `..._GYRO_AXIS=2`, `..._GYRO_SIGN=+1`. Le lacet (`yaw_rate`, utilisé pour le
  maintien de cap et l'antipatinage) est lu séparément dans `task_control.c` sur
  `APP_IMU_YAW_GYRO_AXIS=0` (même axe physique que "up"), signe `-1`.

### 2.2 Calibration du biais IMU (`task_bias.c`)

Séquence entièrement automatique, décorrélée de `task_control` (les deux communiquent via
`app_imu_offset_owns_motors()` — pendant la calibration, `task_control` délègue les moteurs à
`app_imu_offset_motor_torque()` et **saute** la loi d'équilibrage) :

1. **DETECT** — attend que le robot soit posé à plat (`|a_up| < sin(15°)·g` pendant 3 s).
2. **PULSE_IN** — 2 impulsions de couple (`APP_IMU_OFFSET_PULSE_NM = 0.02 Nm`, 100 ms ON / 500 ms
   OFF) pour signaler à l'opérateur "redresse le robot maintenant", puis prend la main sur les
   moteurs.
3. **WAIT_UPRIGHT → SETTLE** — attend `|a_up| > cos(15°)·g` (robot tenu à la verticale), 2 s
   d'immobilisation.
4. **SAMPLE** — moyenne 1000 échantillons (`APP_IMU_OFFSET_SAMPLE_COUNT`, ~20 s à 50 Hz
   d'échantillonnage) d'accel + gyro bruts.
5. Biais gyro = moyenne brute ; biais accel = **écart à la gravité normalisée**
   (`accel_bias_remove_gravity` — soustrait uniquement l'écart de direction/norme à *g*, pas la
   gravité elle-même).
6. **PULSE_OUT** (3 impulsions) puis sauvegarde en flash (secteur 7, `0x081E0000`) et
   `app_imu_offset_request_fusion_reset()` réinitialise le filtre complémentaire avec le nouveau
   biais actif.
7. Timeout global 90 s (`APP_IMU_OFFSET_CAL_TIMEOUT_MS`) — sinon `owns_motors` reste bloqué à
   `true` et `task_control` ne reprend jamais la main.

Cette procédure ne calibre **pas** un offset de tangage constant lié au montage mécanique — elle
compense seulement le biais capteur (accel/gyro), sous l'hypothèse que "vertical tenu à la main"
≈ "vertical réel".

---

## 3. Boucle de commande — `control_strategy_ff_cascade_update` (500 Hz)

### 3.1 Boucle externe : vitesse **ou** position (mutuellement exclusif, `outer_mode`)

- **`outer_mode = 0` (vitesse, défaut)** : `vel_ref = vel_ref_turns_s` (consigne directe, Jetson /
  téléop).
- **`outer_mode = 1` (position)** : asservissement de position **du robot** (pas des roues
  individuellement) via l'odométrie moyenne ABZ (`pos_wheel_turns`, moyenne L/R quand les deux
  encodeurs sont valides — `wheel_odometry()` dans `task_control.c`) :

  ```
  x_m      = (pos_wheel_turns - offset) · 2π · r_roue
  x_err    = x_ref - x_m
  x_err_ema = α·x_err_ema + (1-α)·x_err        // α = pos_err_ema_alpha = 0.99 → fuite lente
  vel_ref  = clamp(Kp_pos·x_err + Kema_pos·x_err_ema - Kd_pos·v_roue, ±v_max_pos)
  ```

  Par défaut `pos_kp = pos_kd = pos_ema_kp = 0` (mode position **implémenté mais non réglé**,
  cf. `BALANCE_BASELINE.md` "Optional next"). Le passage à `outer_mode=1` déclenche un
  re-ancrage de position (`s_pos_offset_valid=false` → capture au prochain échantillon
  d'odométrie valide).

### 3.2 Slew-limiter sur `vel_ref`

Avant d'entrer dans la cascade, `vel_ref` est limité en pente (`vel_ref_slew_turns_s2`,
défaut 80 turn/s²) pour éviter qu'un échelon de consigne (slider téléop, ou sortie brutale du mode
position) ne "cogne" la boucle de tangage. `vel_ref_dot` (dérivée de la rampe) sert de terme
feed-forward plus loin (§3.3).

### 3.3 Cascade vitesse → tangage de consigne

C'est le cœur du contrôle de vitesse d'un robot deux-roues (on ne peut pas commander la vitesse
directement — on incline le robot pour qu'il accélère) :

```
vel_err     = vel_ref - v_roue
vel_err_ema = α·vel_err_ema + (1-α)·vel_err       // α = cascade_vel_err_ema_alpha = 0.80, fuite (pas d'intégrateur pur)
v̇_roue      = LPF(d(v_roue)/dt, k=0.85)            // dérivée filtrée, PAS d(erreur)/dt (évite le kick sur consigne)

pitch_cmd  = Kp·vel_err + Kema·vel_err_ema + Kd·v̇_roue - Kaccel·vel_ref_dot
pitch_cmd  = clamp(pitch_cmd, ±cascade_pitch_ref_max_rad)   // ±0.26 rad ≈ 15°
pitch_trim = -pitch_cmd
```

- **Pas d'intégrateur pur** : `vel_err_ema` est un filtre passe-bas (EMA), pas un ∑e·dt — il ne
  peut pas s'accumuler indéfiniment (« leaky I »), donc pas de windup classique, mais aussi pas
  d'annulation garantie de l'erreur statique en régime permanent (une pente constante ou un
  frottement asymétrique laisse un `vel_err` résiduel non nul).
- `Kaccel·vel_ref_dot` est un **feed-forward négatif** : quand la consigne accélère, le robot doit
  *anticiper* en se penchant plus fort dans le sens du mouvement voulu — terme qui précède la
  réaction de l'erreur elle-même.
- `pitch_ref_eff = clamp(pitch_ref_rad + pitch_trim, ±cascade_pitch_ref_max_rad)` — le trim de la
  cascade s'ajoute à la consigne de tangage "manuelle" (`pitch_ref_rad`, typ. 0).

### 3.4 Boucle interne d'équilibrage

```
θ_err = pitch_ref_eff - pitch

u_ff  = -K_ff · sin(clamp(pitch, ±pitch_failsafe))       // feed-forward gravité
u_fb  = Kθ·θ_err - Kω·pitch_rate + u_vel
u_vel = clamp(Kv·(vel_ref - v_roue), ±Kv_max)             // amortissement additionnel direct sur v (Kv < 0, compile-time)

u_raw = u_ff + u_fb  [+ friction_comp]                    // §3.5
u     = α_lpf·u_prev + (1-α_lpf)·u_raw                    // ff_output_alpha = 0.50, LPF simple
```

- `K_ff` (feed-forward gravité) linéarise le couple nécessaire pour contrer la gravité à
  l'inclinaison courante : `-K_ff·sin(pitch)`. Au banc, `ff_grav_k = 0.14` contre un plein
  feed-forward théorique `K_ff_full ≈ 0.178 Nm/moteur` (plante mesurée : m=1.335 kg, h=0.145 m,
  réducteur 3:16) — soit **~80 % du FF théorique**, le reste étant repris par le PD. Le FF utilise
  `pitch` (mesure brute), pas `pitch_ref_eff` — l'écrêtage à `pitch_failsafe_rad` (45°) protège
  seulement contre un `sin()` qui diverge en cas de chute, pas une action de contrôle.
- Le PD (`Kθ`, `Kω`) régit la **dynamique locale** autour du point de fonctionnement `pitch_ref_eff`
  fourni par la cascade vitesse — c'est une architecture en cascade classique (boucle vitesse
  lente → génère la consigne d'angle de la boucle angle rapide), et non un simple LQR à deux
  gains.
- `u_vel` est un troisième terme d'amortissement sur la vitesse **directement dans la boucle
  interne** (en plus de celui déjà injecté indirectement via `pitch_trim`) — `Kv` est négatif et
  n'est **pas exposé en paramètre live** (`APP_CTRL_FF_FB_K_VEL`, compile-time), contrairement à
  quasiment tous les autres gains. Le clamp `Kv_max = 0.010 Nm` le limite à une contribution mineure.
- Le low-pass de sortie (`ff_output_alpha = 0.50`) lisse `u_raw` d'un facteur ~2 — filtre simple,
  pas synchronisé à un dt variable (§7.2).

### 3.5 Compensation de frottement sec (Coulomb), gated

Deux modèles au choix (`friction_mode`, param live) :

- **`friction_mode = 0` (legacy)** : deadband simple — si `|u| > 0`, ajoute `sign(u)·torque_deadband_nm`.
- **`friction_mode = 1` (par défaut sur `hypothesis_lab`, v11)** : modèle **statique + cinétique**
  à deux niveaux :
  ```
  si |ω_roue| ≤ ε (friction_vel_eps_turns_s = 0.05 turn/s) :
      comp = sign(u) · friction_static_nm      // 0.0045 Nm — vainc le collage, dans le sens de la commande
  sinon :
      comp = sign(ω_roue) · friction_kinetic_nm // 0.003 Nm — s'oppose au glissement, dans le sens du mouvement
  ```

Dans les deux modes, la compensation est **gated** (`friction_gates_ok`) : désactivée si
`|pitch_rate| > torque_deadband_rate_max_rads` (0.30 rad/s) ou `|pitch| >
torque_deadband_pitch_max_rad` (0.05 rad ≈ 3°, actif seulement en build `APP_HYPOTHESIS_LAB`) —
n'agit donc qu'à l'arrêt/quasi-vertical, pas en mouvement franc où elle ferait plus de mal que de
bien. Le journal de test (`BALANCE_BASELINE.md`) documente qu'une compensation Coulomb **non
gated** (~0.0035 Nm) produit un cycle limite ~13 Hz — point de vigilance si ce gate est
désactivé (`torque_deadband_pitch_max_rad = 0`).

### 3.6 Correction accélération moteur (shelved, inactive par défaut)

`alpha_est_update` estime `α_meas = dω/dt` par moteur (filtré, `alpha_lpf = 0.80`) à partir de la
vitesse ODrive CAN (`vel_motor_l/r_turns_s`, pas de l'encodeur ABZ). `alpha_torque_correction`
compare à un modèle roue libre `α_ref = (u - c·sign(ω)) / J` et ajoute `Δτ = Kα·(α_ref - α_meas)`,
gated sur pitch/rate/vitesse.

**Statu quo : `alpha_kp = 0`** (§ *Shelved* de `BALANCE_BASELINE.md`, décision 2026-08) — le code
est complet et exercé au runtime (les deux `alpha_est_update` tournent toujours pour alimenter
`wheel_contact` avec `alpha_l/r`, cf. §4) mais `Δτ` reste nul tant que `alpha_kp = 0`. Le choix
retenu pour compenser le cogging résiduel est l'anticogging bidirectionnel côté ODrive, pas cette
boucle STM32.

### 3.7 Maintien de cap (yaw rate hold)

```
yaw_rate_lpf = LPF(yaw_rate_imu, α = 0.99)              // quasi-intégral, lissage fort
heading = wrap(heading + yaw_rate_lpf · dt)              // cap intégré, debug seulement (g_ctrl_heading_rad)

si (heading_kp ≠ 0 ou heading_kd > 0) et heading_torque_max_nm > 0 et pas WC_MODE_BOTH_AIR :
    rate_err = heading_ref_rad - yaw_rate_lpf             // "heading_ref_rad" est en réalité une consigne de VITESSE de lacet (ψ̇_ref), pas un cap
    u_yaw = clamp(Kp_h·rate_err - Kd_h·d(yaw_rate)/dt, ±heading_torque_max_nm)
```

Par défaut `heading_kp = 0.08`, `heading_kd = 0.005`, `heading_torque_max_nm = 0.04`
(saturates ~0.5 rad/s ; half of `APP_CTRL_CMD_MAX_TORQUE_NM`). Le nommage `heading_ref_rad` / `heading_kp` est trompeur : ce
n'est pas un asservissement de cap absolu (P sur `ψ` intégré) mais un **régulateur de vitesse de
lacet** (P/D sur `ψ̇`), cohérent avec le commentaire de `app_ctrl_params.h`
(« ψ̇_ref = value »). L'intégration de cap (`heading_rad`) n'est calculée que pour le debug/télémétrie.

### 3.8 Répartition gauche/droite et antipatinage

```
τ_L_pré = u + Δτ_L - u_yaw
τ_R_pré = u + Δτ_R + u_yaw
```

passés à `wheel_contact_update()` qui peut **surcharger** ces couples selon l'état de contact
détecté (voir [ANTIPATINAGE.md](ANTIPATINAGE.md) pour le détail complet : NORMAL / SYNC_L / SYNC_R
/ BOTH_AIR / RECOVERY, détection par `η = |α|/max(|τ|, τ_min)` et décorrélation cap
cinématique/gyro). En sortie, clamp final `±cmd_max_torque_nm` (0.08 Nm par défaut — le
commentaire code indique une valeur volontairement réduite « less violent while hunting gains »,
vs. 0.04 Nm dans la baseline validée `BALANCE_BASELINE.md`).

**Point d'analyse important :** `APP_CTRL_ANTIPATINAGE_ENABLE = 0` et `APP_ANTIPAT_SYNC_ENABLE = 0`
par défaut dans `app_config.h` actuel — le mécanisme d'antipatinage est **implémenté et exercé
dans le flux de données** (les champs `alpha_l/r`, `tau_l/r_pre_nm` sont toujours calculés) mais
**inactif** tant que ces deux flags ne sont pas mis à 1 : `wheel_contact_update` retourne alors
directement `u_cmd_nm = in->u` sans aucune détection ni overlay (voir le bloc
`if (!antipat_enabled())` en tête de fonction). À vérifier avant tout test « roue en l'air ».

### 3.9 Sécurités (`task_control.c`)

- **IMU invalide** → estop immédiat (couple nul aux deux roues), sans même appeler la stratégie.
- **`|pitch| > pitch_failsafe_rad`** (0.785 rad = 45°, `control_failsafe_angle`) → estop, évalué
  *après* le calcul de la stratégie (le couple calculé est ignoré et mis à 0).
- Pendant la calibration IMU (`app_imu_offset_owns_motors()`), `task_control` **court-circuite**
  entièrement `control_strategy_update` — aucune protection pitch pendant cette phase, seule la
  séquence d'impulsions de `task_bias` pilote les moteurs.

---

## 4. Paramètres réglables (télémétrie, `app_ctrl_params.c`)

Tous les gains ci-dessous sont modifiables à chaud via le protocole de télémétrie
(`app_ctrl_params_set`, snapshot version 12) sauf mention « compile-time ». Valeurs = défauts
`app_config.h` (branche `hypothesis_lab`, `APP_HYPOTHESIS_LAB=1`).

| Groupe | Param | Défaut | Rôle |
|---|---|---|---|
| Sécurité | `pitch_failsafe_rad` | 0.785 (45°) | Seuil estop |
| Sécurité | `cmd_max_torque_nm` | 0.08 | Clamp couple final par roue |
| Équilibrage | `ff_grav_k` | 0.14 | Feed-forward gravité (`-K_ff·sinθ`) |
| Équilibrage | `ff_fb_k_pitch` | 0.055 | Gain P sur `θ_err` |
| Équilibrage | `ff_fb_k_rate` | 0.013 | Gain D sur `pitch_rate` |
| Équilibrage | `ff_output_alpha` | 0.50 | LPF sortie `u` |
| Cascade | `cascade_vel_kp` | 0.08 | P sur erreur de vitesse → tangage |
| Cascade | `cascade_vel_kd` | 0.008 | D sur `v̇` filtrée |
| Cascade | `cascade_vel_err_ema_alpha` | 0.80 | Fuite de l'intégrateur (EMA) |
| Cascade | `cascade_vel_ema_kp` | 0.03 | Gain sur l'EMA (quasi-I) |
| Cascade | `cascade_vel_accel_kp` | 0.04 | FF sur `v̇_ref` |
| Cascade | `cascade_pitch_ref_max_rad` | 0.26 (~15°) | Clamp tangage de consigne |
| Cascade | `vel_ref_slew_turns_s2` | 80 | Pente max de `vel_ref` |
| Frottement | `friction_mode` | 1 (hyp. lab) | 0 legacy / 1 statique+cinétique |
| Frottement | `friction_static_nm` | 0.0045 | Coulomb à l'arrêt |
| Frottement | `friction_kinetic_nm` | 0.003 | Coulomb en glissement |
| Frottement | `friction_vel_eps_turns_s` | 0.05 | Seuil arrêt/mouvement |
| Frottement (legacy) | `torque_deadband_nm` | 0.004 | Coulomb mode 0 |
| Frottement (gate) | `torque_deadband_pitch_max_rad` | 0.05 | Gate |pitch| |
| Frottement (gate) | `torque_deadband_rate_max_rads` | 0.30 | Gate |pitch_rate| |
| Accel moteur (shelved) | `alpha_kp` | **0** | Désactivé (voir §3.6) |
| Accel moteur (shelved) | `alpha_max_nm`, `motor_J`, `motor_friction_c`, `alpha_*_max_*`, `alpha_lpf` | — | Sans effet tant que `alpha_kp=0` |
| Position (non réglé) | `pos_kp`, `pos_kd`, `pos_ema_kp` | 0 | Mode position implémenté, gains à zéro |
| Position | `pos_v_max_turns_s` | 0.5 | Clamp `vel_ref` en mode position |
| Position | `pos_err_ema_alpha` | 0.99 | Fuite erreur position |
| Cap | `heading_kp`, `heading_kd` | 0.08 / 0.005 | Yaw-rate hold actif |
| Cap | `heading_torque_max_nm` | 0.04 | Plafond (½ du couple balance) |
| Outer | `outer_mode` | 0 (vitesse) | 0=vitesse / 1=position |
| Outer | `vel_ref_turns_s` | 0 | Consigne vitesse (mode 0) |

**Compile-time seulement** (pas dans le snapshot télémétrie) :
`APP_CTRL_FF_FB_K_VEL` (−0.0005), `APP_CTRL_FF_FB_K_VEL_MAX_NM` (0.010),
`APP_IMU_COMPLEMENTARY_ALPHA` (0.999), `APP_CONTROL_PERIOD_MS` (2 ms).

---

## 5. Réglage validé au banc (référence)

`BALANCE_BASELINE.md` (2026-08-09, log `krate013_db0017.csv`, strategy `ff_cascade`) — meilleur
compromis obtenu à date (résistant à la poussée, peu de vibration, rattrapage précoce) :
`ff_grav_k=0.14`, `ff_fb_k_pitch=0.055`, `ff_fb_k_rate=0.013`, `ff_output_alpha=0.50`,
`cmd_max_torque_nm=0.04` (≠ défaut compile-time 0.08 — **flashé différemment de la baseline
courante**, à re-vérifier avant tout test), `cascade_vel_kp=0.001`,
`cascade_pitch_ref_max_rad=0.262`, `torque_deadband_pitch_max_rad=0.05`. Résultats : tangage moyen
0.59° / max 1.88°, ~10 flips/s de commande, |v| moyenne ~4.7 turn/s.

Plante identifiée (roue libre, USB, moyenne L/R) : `J = 1.12e-5 kg·m²`, `c = 0.0052 Nm` (Coulomb),
`b ≈ 0` (visqueux négligeable) — script `identify_motor_inertia.py`. Robot : `m = 1.335 kg`,
`h = 0.145 m` (CoM), réducteur `3:16`, rayon roue `0.04 m` → `K_ff_full ≈ 0.178 Nm/moteur`.

**Ne pas régresser** (leçons du journal de tuning) : Coulomb non-gated ≈ 0.0035 Nm → cycle limite
~13 Hz ; deadband 0.002 + `k_rate=0.013` améliore le tangage mais augmente les flips/s (~17) ;
deadband 0.0015 + `k_rate=0.017` trop bruyant. Changer **un seul** paramètre à la fois, log CSV à
chaque changement.

---

## 6. Séquence de démarrage moteur

`odrive_torque_mode_startup.c`, exécuté **avant** `osKernelStart()` (donc avant que
`task_control` ne tourne) : reset erreurs → calibration moteur (état 4) → passage en
`ODRIVE_CONTROL_MODE_TORQUE` (input mode passthrough) → `CLOSED_LOOP_CONTROL` (état 8), pour les
deux nœuds CAN (0 = gauche, 1 = droite, bus 500 kbit/s). Si l'échec est `IN_PROGRESS` au premier
essai (flash ancienne ou branchement au bureau), `app_drivers_rtos_init()` retente une fois. Build
`DEBUG_DESK_NO_ODRIVE` saute complètement cette séquence pour développer sans CAN branché.

---

## 7. Constats d'analyse — points d'attention

1. **Antipatinage désactivé par défaut** (§3.8) : `APP_CTRL_ANTIPATINAGE_ENABLE=0`,
   `APP_ANTIPAT_SYNC_ENABLE=0`. Le firmware actuellement flashable en l'état ne protège **pas**
   contre un emballement moteur si une roue quitte le sol — seul le failsafe pitch (45°) reste
   actif, ce qui est un seuil large pour ce cas de figure (le tangage ne diverge pas forcément vite
   pendant un emballement de roue).
2. **`dt_s` fixe, pas mesuré** : `task_control.c` passe `dt_s = APP_CONTROL_PERIOD_MS / 1000` codé
   en dur à `control_strategy_input_t`, jamais le delta réel entre deux réveils
   `vTaskDelayUntil`. Tous les gains (`cascade_vel_kd`, LPF, EMA) sont donc calibrés pour un
   scheduler FreeRTOS jamais en retard à 500 Hz ; un jitter RTOS non détecté fausserait
   silencieusement les dérivées/filtres sans lever d'alarme.
3. **EMA/LPF non normalisés au temps** : `ff_output_alpha`, `cascade_vel_err_ema_alpha`,
   `alpha_lpf`, `k_vel_dot_lpf`, `yaw_rate` LPF (`APP_CTRL_YAW_RATE_LPF_ALPHA=0.99`) sont des
   constantes de mélange par **itération**, pas des constantes de temps indépendantes de
   `APP_CONTROL_PERIOD_MS`. Un changement de fréquence de boucle imposerait de retuner tous ces
   coefficients (contrairement à `vel_ref_slew_turns_s2`, exprimé en unité/s² et donc déjà
   normalisé par `dt_s`).
4. **`Kv` (`APP_CTRL_FF_FB_K_VEL`) et sa borne sont compile-time**, seul gain de la boucle interne
   non exposé en télémétrie — asymétrie avec le reste de l'architecture (tout le reste est
   live-tunable). À signaler si un réglage terrain de ce terme s'avère nécessaire.
5. **`cmd_max_torque_nm` par défaut (0.08 Nm) diverge de la valeur baseline validée (0.04 Nm)** —
   le commentaire source (`app_config.h:220`) l'indique explicitement comme provisoire ("less
   violent while hunting gains"). Un flash à partir des défauts actuels ne reproduit donc pas le
   comportement documenté dans `BALANCE_BASELINE.md` sans réappliquer les paramètres listés en §5.
6. **Nommage `heading_*` trompeur** (§3.7) : ce sont des paramètres de régulation de **vitesse**
   de lacet (ψ̇), pas de cap absolu (ψ), bien que `heading_rad`/`g_ctrl_heading_rad` calculent un
   cap intégré (debug uniquement, non utilisé en boucle fermée).
7. **`alpha_kp` (correction accel moteur) et le mode position (`pos_kp/kd`) sont du code mort actif** :
   entièrement exécuté à chaque cycle (calcul d'`alpha_est`, calcul de `x_m`/`x_err_ema`) mais sans
   effet tant que les gains associés restent à 0 — coût CPU mineur, mais toute réactivation
   nécessite de revalider les gates (`alpha_pitch_max_rad`, `alpha_rate_max_rads`,
   `alpha_vel_max_turns_s`) qui n'ont jamais été éprouvées en conditions réelles depuis la mise en
   sommeil (2026-08).
8. **Aucune protection sur `pitch_ref_rad` externe** : si Jetson/télémétrie pousse une valeur hors
   plage, elle n'est écrêtée qu'indirectement via `pitch_ref_eff = clamp(pitch_ref_rad +
   pitch_trim, ±cascade_pitch_ref_max_rad)` — un `pitch_ref_rad` proche de `pitch_failsafe_rad`
   combiné à un trim positif peut déclencher un estop immédiat en sortie de cascade.

---

## 8. Fichiers sources de référence

| Rôle | Fichier |
|---|---|
| Loi d'équilibrage | `Core/Src/control_strategy_ff_cascade.c` |
| Sélecteur de stratégie | `Core/Src/control_strategy.c`, `Core/Inc/control_strategy.h` |
| Fusion IMU (tangage) | `Core/Src/imu_fusion.c` |
| Calibration biais IMU | `Core/Src/tasks/task_bias.c`, `Core/Src/app_imu_offset.c` |
| Boucle de contrôle 500 Hz | `Core/Src/tasks/task_control.c` |
| Antipatinage (contact roue) | `Core/Src/wheel_contact.c` — spec [ANTIPATINAGE.md](ANTIPATINAGE.md) |
| Paramètres réglables | `Core/Src/app_ctrl_params.c`, `Core/Inc/app_ctrl_params.h` |
| Défauts compile-time | `Core/Inc/app_config.h` |
| Démarrage ODrive (couple) | `Core/Src/odrive_torque_mode_startup.c` |
| Envoi couple CAN | `Core/Src/tasks/task_motor_tx.c` |
| Réglage validé au banc | `../../ODrive/OdriveTool/Commands/BALANCE_BASELINE.md` |
| Architecture RTOS globale | [CONTROL_ARCHITECTURE.md](CONTROL_ARCHITECTURE.md) *(section "loi conceptuelle" obsolète — voir ce document à la place)* |

---

## Historique

| Date | Note |
|---|---|
| 2026-09-11 | Analyse et documentation initiale de l'algorithme `ff_cascade` réellement implémenté |
