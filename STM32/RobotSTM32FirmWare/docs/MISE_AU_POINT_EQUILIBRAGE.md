# Mise au point équilibrage — punch list post-revue physique

Suite à la revue physique/mécanique de `ff_cascade` (2026-09-11, voir
[balance_algorithm.html](../../../docs/balance_algorithm.html) §07 et
`BALANCE_ALGORITHM.md`). La loi de commande elle-même (cascade + FF gravité +
PD + frottement gated) est jugée saine ; ce qui reste à fermer avant un essai
terrain sans filet est listé ici, dans l'ordre où le faire.

Cocher au fur et à mesure. Une case cochée = testé sur le matériel réel, pas
juste "le code compile".

---

## Phase 1 — Sécuriser le cas "roue en l'air" (bloquant avant tout essai terrain)

- [ ] **1. Identifier le matériel réellement monté.** Pour les deux ODrive
      effectivement câblés sur le robot (pas un banc de bring-up) : relever
      `serial_number`, `axis?.motor.config.motor_type`, `pole_pairs`,
      `torque_constant`, `motor.config.current_lim` (celui qui est **sauvé en
      NVM**, pas une valeur volatile de session). Le dépôt contient plusieurs
      configs candidates (SF2804 220kv, "2836", "4108") avec des valeurs
      différentes — ne pas supposer, dumper `odrivetool` sur le matériel monté
      et comparer à `app_config.h` (`APP_ODRIVE_LEFT/RIGHT_NODE_ID`,
      `cmd_max_torque_nm`).
- [ ] **2. Vérifier que `current_lim × torque_constant ≥ cmd_max_torque_nm`**
      sur les deux axes, avec la valeur relevée en (1). Si non, soit remonter
      `current_lim` (avec marge thermique), soit baisser `cmd_max_torque_nm`
      pour que le plafond logiciel STM32 corresponde au couple réellement
      livrable.
- [ ] **3. Revalider `limitVel()` / `enable_current_mode_vel_limit` en
      isolation**, banc, roue libre : appliquer un couple typique
      d'équilibrage (0.04–0.08 Nm) sans charge sur la roue et vérifier qu'elle
      s'arrête au `vel_limit` configuré **sans** `CONTROLLER_ERROR_OVERSPEED`.
      Si le trip survient, augmenter `controller.config.vel_gain` (défaut
      1/6 Nm/(tr/s), probablement trop faible pour l'inertie de rotor mesurée
      `J≈1.12e-5 kg·m²`) jusqu'à arrêt propre avec marge, puis sauver en NVM.
- [ ] **4. Activer `APP_CTRL_ANTIPATINAGE_ENABLE=1`, garder
      `APP_ANTIPAT_SYNC_ENABLE=0`** (seul BOTH_AIR actif). Tenir le robot en
      l'air (aucune roue au sol), vérifier : coupure de couple, transition
      NORMAL→BOTH_AIR→RECOVERY→NORMAL propre à la repose, pas de saut de
      commande au réancrage (`x_m_frozen`, `pos_offset_turns_new`).
- [ ] **5. Activer `APP_ANTIPAT_SYNC_ENABLE=1`.** Lever une seule roue à la
      fois (gauche puis droite), vérifier passage en `SYNC_L`/`SYNC_R`, pas
      d'emballement du côté en l'air, retour `NORMAL` propre à la repose.
- [ ] **6. Si le comportement mesuré diverge de `ANTIPATINAGE.md`** (détection
      trop tardive/trop précoce, oscillation à la frontière), ajuster
      `APP_ANTIPAT_ETA_ON/OFF`, `K_DOM`, `T_ON/OFF_MS` un paramètre à la fois,
      logger à chaque changement (même discipline que `BALANCE_BASELINE.md`).

## Phase 2 — Cohérence budget de couple

- [ ] **7. Trancher `cmd_max_torque_nm` : 0.08 (défaut actuel) ou 0.04 (banc
      validé)**, en connaissance de la marge PD résultante à 15° de gîte
      (~55 % vs ~9 % du plafond, voir §07 de `balance_algorithm.html`).
      Documenter le choix dans `app_config.h` (le commentaire actuel
      "less violent while hunting gains" laisse penser que 0.08 est provisoire).
- [ ] **8. Si `cmd_max_torque_nm` change, revalider `BALANCE_BASELINE.md`** —
      les gains FF/PD ont été validés avec 0.04 ; un plafond différent change
      la dynamique de saturation, pas seulement l'amplitude max.

## Phase 3 — Rigueur dynamique (second temps, pas bloquant)

- [ ] **9. Identifier l'inertie du corps** (pendule bifilaire du châssis, ou
      bascule chronométrée en roue libre) pour compléter le triplet
      `m / h / réducteur` déjà connu et permettre un vrai calcul de marge de
      stabilité (root locus / placement de pôles) plutôt qu'un réglage
      empirique seul.
- [ ] **10. Mesurer le `dt` réel de `task_control`** (au lieu du
      `dt_s = 2 ms` codé en dur) sur quelques minutes de run pour quantifier
      le jitter FreeRTOS réel et confirmer qu'il reste négligeable devant les
      constantes de temps des filtres (EMA, LPF) — cf. point d'attention §7
      de `BALANCE_ALGORITHM.md`.

---

## Historique

| Date | Note |
|---|---|
| 2026-09-11 | Punch list initiale, issue de la revue physique de `ff_cascade` |
