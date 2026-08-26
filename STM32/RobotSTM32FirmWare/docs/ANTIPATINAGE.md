# Antipatinage — détection roue en l'air et mode de repli

Spécification **figée** pour le firmware STM32 (`control_strategy_ff_cascade`, `task_control`).  
**Statut :** design validé, **non implémenté** (2026-08).

Voir aussi : [CONTROL_ARCHITECTURE.md](CONTROL_ARCHITECTURE.md), [BALANCE_BASELINE.md](../../../ODrive/OdriveTool/Commands/BALANCE_BASELINE.md), `control_strategy_ff_cascade.c`, `app_config.h`.

---

## Objectif

Détecter qu'une roue **quitte le sol** (décollage, marche, obstacle) sans confondre avec :

- bruit gyro ;
- imperfections mécaniques (courroie, anticog, asymétrie L/R) ;
- **micro-glissements** transitoires au sol.

Puis **changer de mode** pour la roue concernée :

1. **Désenclencher le contrôle de cap φ** (couple différentiel `u_yaw`) sur cette roue.
2. **Aligner la vitesse moteur** de la roue en l'air sur celle qui touche encore le sol.

Le nom **antipatinage** couvre à la fois la détection (roue qui patine / ne transmet plus le couple) et la réponse (resynchronisation, pas un simple estop).

### Synthèse des modes

| Mode | Déclenchement | `u` balance | Couple roues |
|------|---------------|-------------|--------------|
| **NORMAL** | contact | actif | `u ± u_yaw ± Δτ` |
| **SYNC_L / SYNC_R** | η + decorrelated + `k_dom` | actif (roue au sol) | air : `u + τ_sync` ; sol : `u ± u_yaw` |
| **BOTH_AIR** | η_L + η_R + pitch_mismatch | **coupé** | P → **`v_good`** ; **`integrator_trust=0`** |
| **RECOVERY** | post-contact | rampe **`u`**, **`pitch_trim`** | re-ancrage **`x_m`**, reset fuite pitch |

### Cas d'usage visés

| Situation | Mode attendu |
|-----------|--------------|
| Ramassage / pose (deux roues en l'air) | **BOTH_AIR** — pas de runaway, **`ω → v_good`** |
| Marche, obstacle **une roue** | **SYNC** — balance sur l'autre roue |
| Micro-slip, virage | **NORMAL** (debounce + `ε_abs`) |
| Chute / pitch diverge | **BOTH_AIR** + failsafe pitch |

---

## Signaux déjà disponibles (500 Hz)

| Signal | Symbole | Source firmware |
|--------|---------|-----------------|
| Vitesse moteur gauche / droite | `ω_L`, `ω_R` | ODrive CAN → `vel_motor_l/r_turns_s` |
| Accélération angulaire moteur | `α_L`, `α_R` | `alpha_est_update()` → `s_alpha_l/r` |
| Couple balance (commun) | `u` | `ff_cascade` avant split L/R |
| Pitch / taux | `θ`, `θ̇` | IMU → `pitch_rad`, `pitch_rate_rads` |
| Couple commandé L/R | `τ_L`, `τ_R` | Sortie cascade avant clamp |
| Vitesse de lacet | `ψ̇` | Gyro yaw → `yaw_rate_rads` |
| Cap intégré | `ψ` | `∫ ψ̇ dt` (heading hold) |

Constantes géométrie (à paramétrer) :

- `r` = `APP_WHEEL_RADIUS_M` (0,04 m)
- Rapport moteur:roue = **3:16** → vitesse roue `v_i = ω_i · (3/16) · 2π · r`
- `D` = **empattement** (`track_width_m`, paramètre à ajouter)

---

## Plant mesuré au bench (roue libre)

Valeurs **identifiées** (pas calculées théoriquement) — robot **hors sol**, roues libres, pas de couple **τ** ≈ 0,008–0,012 N·m + coast. Script : `ODrive/OdriveTool/Commands/identify_motor_inertia.py`. Détail : [BALANCE_BASELINE.md](../../../ODrive/OdriveTool/Commands/BALANCE_BASELINE.md) § *Identified plant*.

| Symbole | Valeur mesurée | Firmware / télémétrie |
|---------|----------------|------------------------|
| **J** (axe moteur, roue libre) | **1,12×10⁻⁵ kg·m²** | `APP_CTRL_MOTOR_J_KG_M2`, param `motor_J` |
| **c** Coulomb (axe moteur) | **0,0052 N·m** | `APP_CTRL_MOTOR_FRICTION_C_NM`, `motor_friction_c` |
| **b** visqueux | ≈ 0 (négligeable) | — |
| Moyenne L/R | proches | une valeur commune retenue |

**Contenu de J :** rotor + roue + train poulie/courroie, **refléété à l’axe moteur**, **sans** masse du corps du robot. C’est l’inertie pertinente pour **roue en l’air** / **BOTH_AIR**.

**Contraste au sol :** plant balance `m ≈ 1,335 kg`, COM `h ≈ 0,145 m` (`app_config.h`) → même **τ** produit une **α** bien plus faible → **η** reste bas (base du critère 1).

**Ordre de grandeur accélération roue libre** (justifie **`T_on_both`** court), avec **`τ ≈ APP_CTRL_CMD_MAX_TORQUE_NM = 0,04 N·m`** :

```text
α ≈ τ / J           ≈ 3570 rad/s²  ≈ 570 turn/s² (moteur), |ω| ≈ 0
α ≈ (τ − c) / J     ≈ 3100 rad/s²  ≈ 490 turn/s² (moteur), avec friction identifiée
```

| Durée **u** actif | Δω moteur (≈ α·Δt, roue libre) |
|-------------------|--------------------------------|
| 10 ms | ~5–6 turn/s |
| 20 ms (`T_on_both`) | ~10–11 turn/s |
| 50 ms (`T_on`) | ~25–28 turn/s |

**Re-identification :** relancer `identify_motor_inertia.py` si masse roue, courroie ou moteur 2836 change ; mettre à jour `motor_J` / `motor_friction_c` en télémétrie et cette section.

---

## Critères de détection (figés)

Deux **voies** distinctes : lift **unilatéral** (SYNC) et **BOTH_AIR** (voie rapide).

### Critère 1 — roue « déchargée » (inertie seule)

```text
η_i = |α_i| / max(|τ_i|, τ_min)
```

- `η_i` **élevé** : forte accélération moteur pour peu de couple utile → roue ne charge plus le sol.
- `τ_min` évite la division par zéro (~0,001 N·m).

**Seuils (hystérésis) :**

```text
lift_i  ←  η_i > η_on   pendant T_on  (ex. 50–80 ms)
contact ←  η_i < η_off  pendant T_off (η_off < η_on)
```

### Critère 2 — lacet gyro découplé de la cinématique roues

Vitesse linéaire roue estimée à partir des moteurs :

```text
v_i = ω_i · (3/16) · 2π · r
ψ_kin = (v_R − v_L) / D
e_ψ = |ψ̇ − ψ_kin|
```

Seuil (le terme relatif scale **uniquement sur le gyro**, pas sur `ψ_kin` — les vitesses roues ne sont pas fiables en slip / en l'air) :

```text
decorrelated  ←  e_ψ > max(ε_abs, k_rel · |ψ̇|)
```

**Rôle de `ε_abs` :**

- absorber le bruit gyro ;
- tolérer les imperfections mécaniques et erreurs de `D`, `r`, ratio poulies ;
- **absorber les micro-glissements temporaires** au sol (pics courts sur `e_ψ` sans `η` élevé).

**Seuils sortie (hystérésis) :** `e_ψ` repasse sous `k_off · seuil` (ou seuil fixe réduit) pendant `T_off`.

### Critère 3 — pitch ne suit pas le couple balance (BOTH_AIR)

Quand **au moins une roue** charge le sol, **`u`** produit une réaction au sol → le pitch répond.  
Quand **les deux** roues sont en l'air, **`u`** accélère surtout les rotors ; le pitch suit la gravité → **`u` ne « prend » plus**.

```text
e_θ = θ_ref − θ

pitch_mismatch ←  |u| > u_min
              AND  e_θ · θ̇ < 0          // erreur et vitesse de bascule s'éloignent
              AND  |θ̇| > θ̇_min          // filtre bruit gyro à faible inclinaison
```

- **`u_min`** (~0,005–0,01 N·m) : ignore le régime quasi neutre.
- **`θ̇_min`** : tune au bench (typ. faible, ex. 0,05–0,15 rad/s).
- Variante optionnelle : `d/dt(|e_θ|) > pitch_div_min` si la forme produit-scalaire est insuffisante.

**Rôle :** distinguer **deux η hauts au sol** (grip faible, grosse accélération) d'un **vrai double décollage** (pitch diverge malgré **`u`**).

### Lift unilatéral — η + decorrelated + k_dom

| Signal | Rôle |
|--------|------|
| **decorrelated** | Preuve globale : lacet gyro ≠ cinématique roues |
| **η_L / η_R** | Latéralisation : quelle roue est déchargée |
| **k_dom** | Filtre si les deux η passent `η_on` mais une seule roue est en l'air |

Critères 1 **et** 2 simultanés, debounce **`T_on`** (~50–80 ms) :

```text
cand_L ←  decorrelated  AND  η_L > η_on  AND  η_L > k_dom · η_R
cand_R ←  decorrelated  AND  η_R > η_on  AND  η_R > k_dom · η_L

wheel_lift_L ←  cand_L  (debounce T_on)
wheel_lift_R ←  cand_R  (debounce T_on)
```

**Cas typiques :**

- **η_L** haut, **η_R** bas → `cand_L` → **SYNC_L**.
- **η_R** haut, **η_L** bas → `cand_R` → **SYNC_R**.

**`k_dom`** (défaut ~**1,4**) : la roue suspecte doit dominer nettement l'autre.

La combinaison **η + décorrélation + debounce** est conservative : un micro-slip seul ne suffit pas ; une roue en l'air durable oui.

### BOTH_AIR — voie rapide (triplet figé)

Signature : les deux roues déchargées **et** le robot ne répond plus au couple de balance → accélération moteur quasi immédiate (inertie **roue libre** mesurée, voir § *Plant mesuré au bench*).

**Critères (les trois requis) :**

```text
both_cand ←  η_L > η_on  AND  η_R > η_on
         AND  NOT cand_L  AND  NOT cand_R    // pas de lift unilatéral (k_dom)
         AND  pitch_mismatch

BOTH_AIR  ←  both_cand  (debounce T_on_both)
```

| Signal | Rôle BOTH_AIR |
|--------|----------------|
| **`η_L`, `η_R`** | les deux roues tournent « libres » |
| **`pitch_mismatch`** | plus de contact sol → **`u`** inefficace sur θ |
| **`NOT cand_L/R`** | pas de cas unilatéral masqué |

**`decorrelated`** : renfort optionnel, **non requis** pour BOTH_AIR si le triplet ci-dessus est vrai.

**`T_on_both`** : **10–20 ms** (pas `T_on`) — voir tableau Δω § *Plant mesuré au bench* (~10 turn/s en 20 ms à τ≈0,04 N·m).

**Faux positifs évités :**

- Deux η hauts **sans** `pitch_mismatch` → patinage / charge forte au sol, **pas** BOTH_AIR.
- `pitch_mismatch` **sans** les deux η → chute / failsafe pitch classique, pas antipatinage BOTH_AIR.

---

## Machine d'états par roue

```text
CONTACT ──(critères + debounce)──► AIRBORNE ──(hystérésis + debounce)──► CONTACT
```

États globaux dérivés :

| `lift_L` | `lift_R` | Mode global |
|----------|----------|-------------|
| 0 | 0 | **NORMAL** — cascade + heading φ |
| 1 | 0 | **SYNC_L** — roue G en l'air |
| 0 | 1 | **SYNC_R** — roue D en l'air |
| 1 | 1 | **BOTH_AIR** — vol / repli couple |
| * | * | **RECOVERY** — post-contact (avant NORMAL) |

(`RECOVERY` = état global transitoire ; **`lift_L/R`** peuvent encore être vrais au début.)

---

## Mode NORMAL (contact)

Loi actuelle (`ff_cascade`) :

```text
u_yaw = clamp(Kp·wrap(φ_ref − ψ) − Kd·ψ̇, ±τ_yaw_max)
τ_L = clamp(u + Δτ_L − u_yaw, ±τ_max)
τ_R = clamp(u + Δτ_R + u_yaw, ±τ_max)
```

(`φ` / `ψ` = cap intégré depuis le gyro ; « contrôle de φ » = terme **`u_yaw`** sur chaque roue.)

---

## Mode SYNC — une roue en l'air

Pour la roue **i** en l'air, roue **j** au sol :

### 1. Désenclencher φ sur la roue en l'air

- **`u_yaw` gelé à 0** pour toute la durée du lift (au moins une roue en l'air).
- Sur la roue **j** (sol) : conserver `± u_yaw` comme en NORMAL.
- Sur la roue **i** (air) : **pas** de terme `u_yaw` — évite d'accélérer une roue qui ne freine plus le yaw.

Exemple lift gauche :

```text
τ_L = u + Δτ_L + τ_sync_L          (pas de −u_yaw)
τ_R = u + Δτ_R + u_yaw               (heading actif côté sol)
```

### 2. Aligner la vitesse moteur en l'air sur la roue au sol

Référence : **`ω_ref_i = ω_j`** (même sens moteur, signes ODrive déjà appliqués dans `vel_motor_*`).

En mode **couple** ODrive, overlay P sur l'écart de vitesse :

```text
τ_sync_i = K_sync · (ω_j − ω_i)
τ_sync_i = clamp(τ_sync_i, ±τ_sync_max)
```

- `K_sync`, `τ_sync_max` : paramètres télémetrie.
- Le couple balance **`u`** reste **commun** (équilibre pitch) ; seul le différentiel heading + sync différencie L/R.
- Option : rampe `K_sync` sur **50–100 ms** à la reprise contact pour éviter à-coup.

### Intention physique

La roue en l'air est ramenée à la vitesse de celle qui roule encore au sol → à la **repose**, les vitesses moteur sont déjà proches → moins de choc, moins de faux yaw qu'un `τ = 0` brutal.

---

## Mode BOTH_AIR (les deux roues en l'air)

Objectif : **couper `u`** (inutile sur le pitch en vol) sans **`τ = 0`** brutal — ramener **`ω`** vers la **dernière vitesse connue au sol** pour limiter le runaway et adoucir l'atterrissage.

### Réaction immédiate (dès `both_cand` ou fin `T_on_both`)

```text
u ← 0,  u_yaw ← 0,  Δτ_L ← 0,  Δτ_R ← 0     // balance / α inopérants en vol
vel_ref ← 0
```

**Couper `u` sans délai** : **J** roue libre faible → runaway si **`u`** persiste (§ *Plant mesuré au bench*).

### Vitesse « bonne » — moving average + lookback

Pendant **NORMAL** (contact des deux roues), entretien continu :

```text
// EMA courte (T_ma ≈ 80–150 ms) ou ring buffer @ 500 Hz
si contact_L && contact_R :
    v_ma_L ← v_ma_L + α·(ω_L − v_ma_L)
    v_ma_R ← v_ma_R + α·(ω_R − v_ma_R)
    buffer ← (t, ω_L, ω_R, contact_ok=1)
```

À l'entrée **BOTH_AIR** (`t₀`), **geler** la référence depuis le passé — pas la MA instantanée (contaminée par le décollage / debounce) :

```text
T_excl = 2 · T_on_both          // figé : zone exclue ≥ 2× durée détection BOTH_AIR

v_good_L = moyenne( ω_L sur [t₀ − T_excl − T_ma , t₀ − T_excl] , contact_ok )
v_good_R = moyenne( ω_R sur [t₀ − T_excl − T_ma , t₀ − T_excl] , contact_ok )
```

**Pourquoi `T_excl ≥ 2·T_on_both` :**

- les **`T_on_both`** ms avant le déclenchement peuvent déjà voir **`η`** monter et **`ω`** dériver ;
- en remontant **au-delà de 2×** cette fenêtre, on lit une vitesse enregistrée **quand le contact était fiable** ;
- pas assez d'échantillons `contact_ok` → fallback : dernier **`v_ma`** mémorisé en **NORMAL**, ou **`v_good ← 0`** (conservateur).

Exemple : `T_on_both = 20 ms` → `T_excl = 40 ms` ; `T_ma = 100 ms` → moyenne sur **[t₀−140 ms, t₀−40 ms]**.

**Chronologie (exemple numérique) :**

```text
      contact fiable          zone exclue T_excl        BOTH_AIR
◄──────────────────────────►◄────────────────────►      t₀
 t₀−140 ms              t₀−40 ms                  t₀−20 ms   t₀
 │◄──── T_ma = 100 ms ────►│                         │◄T_on_both►│
 │     v_good = moy(ω)     │  η monte, ω dérive      │ debounce │
 │     contact_ok = 1      │  (non utilisé)          │          │
```

**Implémentation ring buffer (500 Hz) :**

```c
// Chaque tick NORMAL (les deux roues en contact) :
ring_push(t, ω_L, ω_R, contact_ok=1);

// À BOTH_AIR (t₀) :
v_good_L = ring_mean(ω_L, t ∈ [t₀ − T_excl − T_ma, t₀ − T_excl], contact_ok);
// idem ω_R ; si count < min_samples → fallback v_ma_last ou 0
```

### Couple en BOTH_AIR — tendre vers `v_good`

```text
τ_L = clamp( K_both_v · (v_good_L − ω_L) , ±τ_both_max )
τ_R = clamp( K_both_v · (v_good_R − ω_R) , ±τ_both_max )
```

- **`K_both_v`**, **`τ_both_max`** : couple **réduit** — P vitesse borné, pas un second contrôleur balance.
- Si **`ω`** a déjà dépassé **`v_good`** pendant le debounce : le P **freine** vers **`v_good`**.
- Option : **`v_good ← 0.5·(v_good_L + v_good_R)`** sur les deux roues (mode commun strict en vol).

**`pitch_failsafe`** reste le filet parallèle ; BOTH_AIR doit couper **`u`** plus vite que le failsafe angle.

---

## Reprise contact

À la repose, **`ω_moteur`**, **`ω_ABZ`** et la **distance réelle** ne coïncident plus → risque de **discontinuités** sur les intégrateurs **vitesse / distance** et sur toute la **chaîne pitch** qui en dépend.

### Problème (état actuel `ff_cascade`)

**Odometrie / vitesse :**

| Consommateur | Source | Effet au recontact |
|--------------|--------|-------------------|
| **`x_m`** (mode pos) | `pos_wheel` ABZ moy. L/R | intégration **fantôme** → **`x_err`** énorme → **`vel_ref`** coup de fouet |
| **`s_x_err_ema`** | fuite sur **`x_err`** | mémoire fausse pendant le vol |
| **`s_vel_err_ema`** | fuite sur **`vel_ref − vel_wheel`** | pic **`pitch_trim`** |
| **`vel_dot`** (cascade D) | Δ**`vel_wheel`** / Δt | step → pic D sur **`pitch_trim`** |
| **`u_vel`** | **`vel_ref − vel_wheel`** | même discontinuité dans **`u_fb`** |
| **`s_vel_ref_slew`** | rampe **`vel_ref`** | **`vel_ref`** incohérent après vol |
| **Bias roues** (futur) | ODrive vs ABZ | à geler si lift |

**Chaîne pitch** (via cascade + filtres — pas de **`pitch_ki`** dans `ff_cascade`, mais **fuite / mémoire** équivalente) :

| Consommateur | Source | Effet au recontact |
|--------------|--------|-------------------|
| **`pitch_trim`** | P + **leaky I** **`s_vel_err_ema`** + D **`vel_dot`** | trim **faux** accumulé → **`pitch_ref_eff`** décalé → **`u_fb`** coup de fouet |
| **`pitch_ref_eff`** | **`pitch_ref + pitch_trim`** | consigne pitch effective **sautée** |
| **`theta_err`** | **`pitch_ref_eff − θ`** | PD pitch réagit à un **faux** **`pitch_ref_eff`** |
| **`s_u_prev`** | LPF sur **`u_raw`** | mémoire de **`u`** d'avant vol / BOTH_AIR → reprise **`u`** en rampe **désynchronisée** |
| **`u_ff`** | **`sin(θ)`** | OK — ne dépend pas des roues |
| **`s_heading_rad`** | ∫ gyro yaw | **peu affecté** — ne pas geler ; **`u_yaw`** géré à part |
| **`θ`, `θ̇`** (IMU) | gyro + fusion | **mesure fiable** ; ne pas geler — seulement ne pas **corriger** avec des trim falsifiés |

Tant que **`vel_wheel`** / **`pos_wheel`** restent **valides** côté encodeur pendant un lift, la loi **croit** que le robot avance et **pousse le pitch** via **`pitch_trim`** et **`u_vel`**.

### Principe : `integrator_trust` + sous-état RECOVERY

Flag unique **`integrator_trust`** (alias doc **`odom_trust`** — même signal) :

```text
integrator_trust ←  (mode == NORMAL)  AND  NOT recovering
```

Couvre **odometrie** et **chaîne pitch alimentée par la vitesse roue**.

Dès **`wheel_lift_*`** ou **`BOTH_AIR`** : **`integrator_trust ← false`**.  
Sortie lift confirmée → **RECOVERY** pendant **`T_recover`** (ex. **100–200 ms**) avant **`integrator_trust ← true`**.

```text
NORMAL ──lift──► LIFT (SYNC / BOTH_AIR) ──contact──► RECOVERY ──T_recover──► NORMAL
```

### Pendant LIFT (SYNC ou BOTH_AIR)

**Geler** intégrateurs vitesse-distance **et** mémoire pitch liée à **`vel_wheel`** :

```text
// Snapshot à l'entrée lift (t_lift) :
x_m_frozen           ← x_m courant (si pos valide)
pos_offset_snap      ← s_pos_offset_turns
pitch_trim_frozen    ← pitch_trim courant (ou 0 en BOTH_AIR)
pitch_ref_eff_frozen ← pitch_ref + pitch_trim_frozen
s_u_prev_snap        ← s_u_prev
vel_ref_slew_snap    ← s_vel_ref_slew

// Chaque tick lift (si !integrator_trust) :
NE PAS mettre à jour s_x_err_ema, s_vel_err_ema depuis vel_wheel / pos_wheel
NE PAS mettre à jour vel_dot ; s_vel_prev gelé
NE PAS recalculer pitch_trim depuis cascade vel — utiliser pitch_trim_frozen
pitch_ref_eff ← pitch_ref_eff_frozen   (PD pitch IMU reste actif en SYNC si u ≠ 0)
NE PAS mettre à jour s_u_prev depuis u_raw pollué (BOTH_AIR : u=0 ; SYNC : u filtré ou gelé)
NE PAS avancer s_vel_ref_slew depuis vel_ref pollué (mode pos)
```

**BOTH_AIR** : **`u ← 0`** → **`pitch_trim_frozen`** peu importe pour **`u`**, mais **snapshot** quand même pour **RECOVERY**.  
**SYNC** : **`u`** actif — **`pitch_ref_eff_frozen`** évite un trim vitesse **faux** pendant le lift unilatéral ; le PD **`θ`** reste sur IMU.

**Ne pas** geler **`θ` / `θ̇`** IMU — seulement la branche **vitesse → pitch**.

### À l'entrée RECOVERY (premier contact fiable)

**Re-ancrage odometrie** sans saut de **`x_err`** :

```text
// pos_wheel_touch = lecture ABZ au moment contact (moy. L/R si les deux valides)
pos_offset_new = pos_wheel_touch − x_m_frozen / (2π · r)

s_pos_offset_turns ← pos_offset_new
s_x_err_ema        ← 0
s_vel_err_ema      ← 0
s_vel_prev_turns   ← vel_wheel_touch
s_vel_dot_valid    ← false
pitch_trim_frozen  ← 0                    // repartir neutre côté vitesse→pitch
s_u_prev           ← u @ contact (ou 0)   // LPF u cohérente avec reprise u
s_vel_ref_slew     ← vel_ref demandé @ contact (mode pos / slew)
```

**Rampe pitch / cascade** pendant **`T_recover`** :

```text
pitch_trim_ctrl ← pitch_trim_frozen · (1 − recover_ramp) + pitch_trim_nom · recover_ramp
// recover_ramp : 0 → 1 sur T_recover
u_fb pitch path  : utiliser pitch_ref_eff dérivé de pitch_trim_ctrl
s_u_prev         : laisser LPF converger (ff_output_alpha) — pas de step u_raw
```

Ainsi **`x_m`** repart de **`x_m_frozen`** et **`pitch_ref_eff`** ne **saute** pas au recontact.

**Vitesses pour la cascade** pendant **`T_recover`** :

```text
vel_wheel_ctrl ← blend( v_good ou vel @ contact , vel_wheel_meas , ramp over T_recover )
// ou : clamp rate of change |d vel_wheel_ctrl/dt| ≤ vel_recover_slew
```

**Reprise des gains** (rampe, pas step) :

```text
K_both_v, K_sync     → 0
u, u_yaw             : rampe 0 → nominal sur T_recover
pitch_trim (cascade) : recover_ramp (voir ci-dessus)
pos_kp effective     : pos_kp · recover_ramp
cascade_vel_* eff.   : idem si besoin
```

### Sortie lift (conditions — inchangées)

**AIRBORNE** unilatéral :

- `η_i < η_off` **et** `e_ψ` sous seuil **et** debounce **`T_off`**

**BOTH_AIR** :

- `η_L`, `η_R` < `η_off` **et** pas **`pitch_mismatch`** **et** debounce **`T_off_both`**

→ déclenche **RECOVERY**, pas retour direct **NORMAL**.

### SYNC unilatéral — cas particulier

Une roue au sol : **`integrator_trust`** reste **false** tant qu'une roue est en lift (même logique pitch + odom).

- **`vel_wheel`** : préférer la roue **au sol** seule (pas la moyenne L/R) si signes connus ;
- **`x_m`** : geler tant que **une** roue est en lift (`lift_L || lift_R`) — la roue en l'air pollue la moyenne.

### Paramètres reprise

| Param | Rôle | Défaut |
|-------|------|--------|
| `T_recover` | durée sous-état RECOVERY | 100–200 ms |
| `vel_recover_slew` | max \|d vel_wheel_ctrl/dt\| | tune bench |
| `pos_kp_recover_ramp` | rampe gains mode pos | 0→1 sur `T_recover` |
| `odom_freeze_on_lift` | geler intégrateurs si lift | 1 (figé) |
| `pitch_trim_recover_ramp` | rampe 0→1 **`pitch_trim`** post-contact | sur `T_recover` |

### Télémétrie debug (reprise)

| Champ | Usage |
|-------|--------|
| `odom_trust` | 0/1 |
| `x_m_frozen` | distance logique pendant lift |
| `recovery_ms` | compteur RECOVERY |
| `pos_offset_reanchor` | offset après repose |

### Intégration firmware (complément)

- Exposer **`odom_trust`** depuis le module antipatinage → **`ff_cascade`** gate **`s_x_err_ema`**, **`s_vel_err_ema`**, **`vel_dot`**, mode pos.
- **`task_control`** : option **`vel_wheel_ctrl`** distinct de **`g_ctrl_vel_wheel`** mesuré.
- Entrée lift : snapshot ; sortie contact : re-ancrage **`pos_offset`** comme ci-dessus.

---

## Paramètres (télémetrie / `app_config`)

| Param | Rôle | Calibration initiale |
|-------|------|----------------------|
| `motor_J` | **J** axe moteur (roue libre) | **1,12×10⁻⁵ kg·m²** — **mesuré bench** |
| `motor_friction_c` | Coulomb axe moteur | **0,0052 N·m** — **mesuré bench** |
| `track_width_m` | `D` dans `ψ_kin` | mesure chassis |
| `τ_min` | plancher critère 1 | ~0,001 N·m |
| `η_on`, `η_off` | seuils lift η | bench : roue levée vs roulage |
| `k_dom` | dominance L/R si les deux η > η_on | ~1,4 ; tune si faux BOTH_AIR |
| `u_min` | plancher \|u\| pour pitch_mismatch | ~0,005–0,01 N·m |
| `θ̇_min` | plancher \|θ̇\| pour pitch_mismatch | tune bench |
| `T_on_both`, `T_off_both` | debounce BOTH_AIR | 10–20 ms / 80–120 ms |
| `T_ma` | fenêtre MA vitesse sol | 80–150 ms |
| `T_excl` | lookback exclu avant BOTH_AIR | **2·T_on_both** (figé) |
| `K_both_v`, `τ_both_max` | P vitesse → `v_good` en BOTH_AIR | tune bench ; `τ_both_max` ≪ `τ_max` |
| `ε_abs` | zone morte décorrélation | max `e_ψ` au sol (ligne droite + petits virages) × 1,5–2 |
| `k_rel` | marge relative en virage | `e_ψ` vs `\|ψ̇\|` en virage nominal |
| `k_off` | hystérésis sortie décorrélation | ~0,5–0,7 |
| `T_on`, `T_off` | debounce ms | 50–80 ms / 80–120 ms |
| `K_sync`, `τ_sync_max` | resynchro vitesse | tune bench lift L puis R |
| `T_recover` | sous-état RECOVERY post-contact | 100–200 ms |
| `vel_recover_slew` | rampe `vel_wheel_ctrl` | tune reprise |
| `pos_kp_recover_ramp` | rampe mode pos après lift | 0→1 sur `T_recover` |

---

## Télémétrie debug (à ajouter)

| Champ | Usage |
|-------|--------|
| `η_l`, `η_r` | tune critère 1 |
| `pitch_mismatch` | tune critère 3 / BOTH_AIR |
| `psi_kin`, `e_psi` | tune critère 2 |
| `wheel_lift_l`, `wheel_lift_r` | état |
| `v_good_l`, `v_good_r` | ref BOTH_AIR (lookback) | debug |
| `tau_both_l`, `tau_both_r` | overlay P → `v_good` | tune `K_both_v` |
| `tau_sync_l`, `tau_sync_r` | overlay SYNC unilatéral | debug |
| `antipatinage_mode` | 0=NORMAL, 1=SYNC_L, 2=SYNC_R, 3=BOTH_AIR, 4=RECOVERY |
| `odom_trust` / `integrator_trust` | intégrateurs vitesse + pitch actifs | debug reprise |
| `x_m_frozen` | distance logique gelée en lift | debug |
| `pitch_trim_frozen` | trim cascade gelé en lift | debug |

---

## Intégration firmware (plan)

1. Module `wheel_contact.c` : calcul `η`, `ψ_kin`, `e_ψ`, `pitch_mismatch`, debounce ; ring buffer **`v_good`** ; FSM **RECOVERY** ; **`odom_trust`** ; **`x_m_frozen`** + re-ancrage **`pos_offset`**.
2. Gate **`ff_cascade`** : si **`!integrator_trust`**, geler **`s_vel_err_ema`**, **`s_x_err_ema`**, **`vel_dot`**, **`pitch_trim`**, **`s_u_prev`**, **`s_vel_ref_slew`**.
3. Geler `u_yaw` si lift unilatéral ; couper **`u`** + **`Δτ`** en BOTH_AIR ( **`τ`** via **`K_both_v`** seulement ).
4. Params + télémétrie vN.
5. Bench : lever roue G/D ; saut / double décollage ; virage + micro-slip.

**Ne pas** utiliser décorrélation ou η seuls. **Ne pas** couper **`u`** en lift unilatéral — seulement φ + sync. **Couper `u`** en BOTH_AIR.

---

## Révision

| Date | Notes |
|------|--------|
| 2026-08 | Spec initiale : critères figés, mode SYNC + désactivation φ par roue |
| 2026-08 | Ajout `k_dom` : latéralisation si les deux η dépassent `η_on` |
| 2026-08 | BOTH_AIR : triplet η_L+η_R+pitch_mismatch, `T_on_both` court, coupure immédiate `u` |
| 2026-08 | § Plant mesuré : J, c bench + ordres de grandeur α / Δω |
| 2026-08 | BOTH_AIR : `v_good` lookback `T_excl≥2·T_on_both`, P vitesse borné vs τ=0 |
| 2026-08 | Doc : synthèse modes, cas d'usage, chronologie lookback, ring buffer |
| 2026-08 | Reprise contact : `integrator_trust`, RECOVERY, gel odom + **chaîne pitch** |
