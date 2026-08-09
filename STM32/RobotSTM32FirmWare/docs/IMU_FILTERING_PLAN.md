# Plan de filtrage IMU pour le self-balancing

## Objectif

Améliorer le rapport signal/bruit du retour gyroscopique sans dégrader
sensiblement la marge de phase de la boucle d'équilibre à 500 Hz.

Le principe retenu est :

- faire davantage confiance au gyroscope pour la dynamique rapide ;
- utiliser l'accéléromètre uniquement pour corriger lentement la dérive ;
- filtrer le bruit gyroscopique hors de la bande passante mécanique utile ;
- mesurer le compromis bruit/retard au lieu de supposer que la mesure la plus
  fraîche est systématiquement la meilleure.

## État actuel

- ICM45686 configuré avec un ODR accéléromètre et gyroscope de **800 Hz**.
- Tâche IMU exécutée avec une cible de **1 kHz**.
- Boucle de contrôle exécutée à **500 Hz**.
- Filtre complémentaire :

  \[
  \theta_k =
  \alpha(\theta_{k-1}+\dot{\theta}_k\Delta t)
  +(1-\alpha)\theta_{acc,k}
  \]

- Valeur actuelle : `APP_IMU_COMPLEMENTARY_ALPHA = 0.92`.
- `pitch_rate_rads` est le gyroscope brut signé, sans filtre logiciel.
- Le terme de damping du contrôleur utilise directement cette vitesse :

  \[
  u_D=-K_\omega\dot{\theta}
  \]

À 1 kHz, `alpha=0.92` correspond approximativement à une correction
accéléromètre autour de 13 Hz. Cette correction est trop rapide pour un robot
mobile : une accélération longitudinale peut être interprétée comme une
variation d'inclinaison.

## Cible initiale proposée

### Filtre complémentaire

Commencer avec :

```c
#define APP_IMU_COMPLEMENTARY_ALPHA 0.994f
```

À 1 kHz, cette valeur place la correction accéléromètre autour de 1 Hz.
Une variante plus conservatrice pourra utiliser `0.997` (environ 0,5 Hz).

Pour un biais gyro de `0.02 rad/s`, l'erreur statique approximative après
correction à 1 Hz est :

\[
\frac{0.02}{2\pi\cdot1}\approx0.0032\ rad\approx0.18^\circ
\]

Le gyro reste ainsi dominant sur la dynamique d'équilibre, tandis que
l'accéléromètre empêche une dérive illimitée.

### Filtre de vitesse angulaire

Ajouter un passe-bas causal du premier ordre sur `pitch_rate_rads` :

\[
y_k=y_{k-1}+\beta(x_k-y_{k-1})
\]

avec :

\[
\beta=1-\exp(-2\pi f_c\Delta t)
\]

Point de départ recommandé :

- fréquence de coupure `f_c = 100 Hz` ;
- calcul à chaque nouvel échantillon IMU ;
- utilisation du dernier résultat filtré par la boucle de contrôle à 500 Hz.

À 100 Hz, le filtre ajoute environ 11 degrés de phase à 20 Hz et une constante
de temps proche de 1,6 ms. Ce compromis doit être confirmé sur le robot.

Éviter initialement une moyenne glissante : elle impose un retard fixe plus
important et introduit des zéros dans la réponse fréquentielle.

## Fréquence d'acquisition

La tâche IMU vise actuellement 1 kHz, mais le capteur produit des données à
800 Hz. Lire les registres à 1 kHz ne crée donc pas 1 000 échantillons
indépendants par seconde.

Ordre recommandé :

1. conserver l'ODR à 800 Hz pour établir une baseline ;
2. filtrer chaque nouvel échantillon à sa fréquence réelle ;
3. vérifier le filtre numérique et l'anti-aliasing internes de l'ICM45686 ;
4. augmenter éventuellement l'ODR du capteur avant d'augmenter davantage la
   fréquence de lecture logicielle.

## Correction accéléromètre adaptative

Après validation de la solution fixe, rendre la confiance accéléromètre
dépendante de la dynamique :

- calculer l'écart entre la norme de l'accélération et `g` ;
- conserver la correction lente lorsque la norme est proche de `g` ;
- augmenter `alpha` vers 1 pendant les accélérations importantes ;
- revenir progressivement à la valeur nominale afin d'éviter une discontinuité.

Une première règle simple peut utiliser deux seuils avec hystérésis. Cette
évolution doit rester postérieure à la validation du filtre fixe afin de ne pas
mélanger plusieurs effets pendant le réglage.

## Instrumentation nécessaire

Ajouter à la télémétrie ou aux variables de debug :

- vitesse angulaire brute ;
- vitesse angulaire filtrée ;
- angle accéléromètre ;
- angle fusionné ;
- valeur effective de `alpha` ;
- norme de l'accélération ;
- contributions `u_ff`, `u_fb` et commande finale ;
- timestamps ou âge de l'échantillon IMU lu par la boucle de contrôle.

## Protocole de validation

### 1. Capteur immobile

- Enregistrer au moins 60 secondes.
- Mesurer moyenne, écart-type et dérive intégrée du gyro brut et filtré.
- Vérifier l'absence de retard pertinent sur les petites perturbations.

### 2. Mouvement manuel, moteurs désactivés

- Appliquer plusieurs impulsions angulaires rapides.
- Comparer amplitude et retard du gyro brut et filtré.
- Vérifier que l'angle fusionné revient correctement à sa référence.

### 3. Robot en équilibre

Tester successivement :

1. baseline actuelle ;
2. `alpha=0.994`, gyro brut ;
3. `alpha=0.994`, gyro filtré à 100 Hz ;
4. coupures 80 Hz et 120 Hz si nécessaire.

Pour chaque configuration, comparer :

- bruit RMS de la commande de couple ;
- oscillation RMS de l'angle ;
- dépassement après impulsion ;
- temps de retour ;
- couple crête et temps passé en saturation ;
- bruit et température des moteurs ;
- chutes ou déclenchements du failsafe.

Ne modifier aucun gain du contrôleur pendant cette comparaison.

### 4. Retuning

Une fois le filtre retenu :

- retuner le gain de vitesse angulaire `ff_fb_k_rate` ;
- vérifier ensuite le gain angulaire `ff_fb_k_pitch` ;
- seulement après, activer et régler la boucle vitesse externe.

## Critères d'acceptation

La configuration filtrée est retenue si :

- elle réduit clairement le bruit RMS de couple ;
- elle ne dégrade pas significativement le temps de récupération ;
- elle ne crée pas d'oscillation supplémentaire ;
- le retard estimé reste compatible avec la bande passante du robot ;
- aucune saturation ou chute supplémentaire n'apparaît ;
- le résultat reste reproductible sur plusieurs démarrages et niveaux de
  batterie.

## Repli

Tous les paramètres doivent rester configurables à la compilation, puis
éventuellement par télémétrie :

- `APP_IMU_COMPLEMENTARY_ALPHA`;
- activation du LPF gyro ;
- fréquence de coupure du LPF gyro ;
- activation de la confiance accéléromètre adaptative.

La désactivation du LPF doit restituer exactement le comportement antérieur
pour permettre une comparaison A/B rapide.
