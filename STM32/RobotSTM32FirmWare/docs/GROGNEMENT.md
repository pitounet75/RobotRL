# Grognement ~38 Hz — mesures et état de l'enquête

Le robot vibre en permanence autour de 38 Hz en équilibre. Ce document consigne
ce qui a été **mesuré**, avec les chiffres, pour ne pas refaire les mêmes
balayages. Les hypothèses fausses sont gardées : elles évitent d'y revenir.

Outils : `TelemetryServer/scripts/growl_sweep.py` (balayage d'un paramètre,
amplitude et fréquence du pic) et `scripts/ringdown.py` (réponse libre, couple
coupé). Capture de référence : `logs/fit_vs_ema.csv`, 14,2 s, 500 Hz.

---

## 1. Le phénomène

Sur une capture en équilibre :

| Signal | Pic | Amplitude dans la bande |
|--------|-----|-------------------------|
| Vitesse roue gauche | 41,9 Hz | ±0,66 turn/s |
| Vitesse roue droite | 41,9 Hz | ±0,63 turn/s |
| Vitesse de tangage | 42,2 Hz | 0,045 rad/s |
| **Couple commandé** | 41,9 Hz | **80 % de sa variance** |

- Les deux roues sont **en phase**, corrélation 0,98 : mode commun, pas du lacet.
- Le couple **suit** la vitesse avec 2 ms de retard, soit un cycle de commande.
- L'amplitude croît avec la vitesse : 0,58 turn/s à l'arrêt, 0,83 au-dessus de
  0,8 turn/s.
- L'accélération de roue impliquée est de **130 à 175 turn/s²**.

**Conséquence pour l'antipatinage** : la signature de roue libre à détecter vaut
14 turn/s². Rapport signal sur bruit 0,08. Aucun critère fondé sur η ou sur
l'accélération ne peut fonctionner tant que le grognement est là.

---

## 2. Ce que chaque paramètre fait (mesuré)

Toutes les valeurs viennent de balayages entrelacés `--repeat 3` sauf mention.
**Ne jamais comparer deux exécutions entre elles** : le point de fonctionnement
et la batterie dérivent. Comparer les médianes d'une même exécution à leur
étendue.

| Paramètre | Effet sur la fréquence | Effet sur l'amplitude |
|-----------|------------------------|-----------------------|
| `cascade_vel_kd` | la déplace, 41 → 22 Hz | la réduit, mais double le mouvement lent |
| `cascade_vel_dot_lpf` | la déplace, 38 → 29 Hz | **aucun** |
| `balance_output_alpha` | la déplace, 40 → 31,5 Hz | **aucun** (le couple baisse de 2×) |
| `friction_kinetic_nm` | aucun | **optimum déjà atteint** à 0,003 |
| `meca_k_pitch_damp` | aucun | aucun |

### friction_kinetic_nm — courbe en U, creux sur la valeur actuelle

| Valeur | 0 | 0,0015 | **0,003** | 0,0045 | 0,006 |
|--------|---|--------|-----------|--------|-------|
| Amplitude | 1,100 | 0,925 | **0,48–0,55** | 0,672 | 0,683 |
| Mouvement lent | 0,229 | 0,367 | **0,35–0,49** | 0,754 | 0,845 |

La compensation de frottement **amortit** le grognement, elle ne le crée pas :
la supprimer le multiplie par 2,3. Monter dégrade les deux critères. Ne pas y
toucher.

---

## 3. La résonance à 27 Hz

Réponse libre, couple coupé par `cmd_max_torque_nm = 0.0001` :

| Configuration | Signal | Pic | Amortissement 1/e | Q |
|---------------|--------|-----|-------------------|---|
| **Au sol** | `pitch_rate` (IMU) | **25–28 Hz** | 36 ms | **3,1** |
| En l'air | `vel_l` | 18–22 Hz | 24 ms | 1,5 |
| En l'air | `vel_r` | 15–20 Hz | 44 ms | 2,1 |

Au sol c'est **le corps** qui sonne, vu par l'IMU et pas par les roues. En l'air
c'est la chaîne cinématique, vue par les roues et pas par l'IMU.

**Vérification quantitative.** Le balayage de `balance_output_alpha` montre que
le couple est divisé par 2,03 entre 40 et 31,5 Hz sans que la vitesse bouge :
l'admittance mécanique double. Un mode du second ordre à 27 Hz prédit :

| Modèle | Admittance 31,5 Hz / 40 Hz |
|--------|----------------------------|
| 27 Hz, Q = 2,0 | 2,05 |
| 27 Hz, Q = 3,1 | 2,47 |
| **Mesure** | **~2,0** |

La prédiction encadre la mesure.

---

## 4. Le modèle actuel

Le grognement est un **cycle limite de la boucle qui s'installe juste au-dessus
d'une résonance du corps à 27 Hz**. La boucle fixe *où* elle croise, la
mécanique fixe *avec quelle amplitude*.

Ça explique les observations qui semblaient contradictoires :

- les gains déplacent la fréquence, mais pas l'amplitude ;
- filtrer descend le croisement **vers** la résonance, où l'admittance monte et
  compense exactement la baisse de couple ;
- descendre encore serait pire avant d'être mieux, il faudrait traverser 27 Hz.

**Direction non testée** : monter le croisement pour s'éloigner de la résonance
par le haut, c'est-à-dire réduire le retard dans la boucle. `cascade_vel_dot_src
= 1` fait exactement ça — l'ajustement d'ordre 2 rend l'accélération avec 10 ms
de retard contre 17. Deux effets s'opposent alors (moins d'admittance, mais plus
de gain à haute fréquence), donc seule la mesure tranchera.

---

## 5. Hypothèses testées et écartées

| Hypothèse | Verdict |
|-----------|---------|
| La compensation de frottement fait relais et entretient le cycle | **Faux** : la supprimer empire ×2,3 |
| Filtrer le terme D davantage réduira l'amplitude | **Faux** : déplace la fréquence, amplitude inchangée |
| La courroie résonne | **Non** : une GT2 6 mm sur 5–10 cm donne 300–500 Hz en torsion, le mode transversal ~350 Hz. Deux courroies indépendantes n'expliqueraient pas non plus la corrélation 0,98 entre roues |
| L'ajustement d'ordre 2 réduira le grognement (moins de retard) | **Contredit sur le papier** : à 40 Hz il passe 0,57 de la dérivée idéale contre 0,15 pour la chaîne EMA. Reste à mesurer, voir §4 |

---

## 6. Réglages retenus

| Paramètre | Valeur | Raison |
|-----------|--------|--------|
| `friction_kinetic_nm` | 0,003 | creux de la courbe en U |
| `cascade_vel_dot_lpf` | 0,85 | 0,94 ne gagne rien et coûte 2,7× de mouvement lent |
| `meca_k_pitch_damp` | 0,013 | sans effet sur le grognement, baisser coûte de la tenue |
| `balance_output_alpha` | 0,70 | −25 % de couple à la fréquence du grognement, coût faible |

`cascade_vel_kd = 0,008` reste en place faute de mieux : baisser réduit le
grognement mais double le mouvement lent.

---

## 7. Méthode — pièges rencontrés

- **Une seule passe ne prouve rien.** La dispersion entre essais vaut autant que
  l'effet, et la batterie faiblit pendant un balayage, ce qui pénalise
  systématiquement les dernières valeurs. Toujours `--repeat 3`, entrelacé.
- **Surveiller le coût, pas seulement le grognement.** Un balayage qui ne
  regarde que l'amplitude récompense la suppression de l'amortissement : le
  robot cesse de vibrer et se met à vaciller. D'où les colonnes `pitch rms` et
  `vel <5Hz`.
- **Couper le couple avec `cmd_max_torque_nm`, pas avec `pitch_failsafe_rad`.**
  L'estop est recalculé à chaque cycle, donc il se relâche quand le tangage
  croise le seuil : 0,026 N·m ont été mesurés pendant une capture censée être
  libre.
- **L'excitation doit être brève.** Une poussée à la main n'a pas d'énergie
  au-dessus de 20 Hz ; l'absence de pic à 35 Hz ne prouve alors rien. Le script
  mesure et refuse de conclure sous 2 % d'énergie dans la bande 25–60 Hz.
- **Les paramètres ne sont pas en mémoire non volatile.** Couper l'alimentation
  restaure les défauts — c'est le filet de sécurité si une restauration échoue.
