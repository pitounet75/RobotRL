# ESP32FOCHardwareCheck

Validation d’**un** MKS FS2804 + MT6835 **ABZ** : voltage open-loop, puis FOC **courant** (shunts 10 mΩ ×50, ADC1 DMA). Roue **levée**.

Arduino ne tourne plus la boucle moteur. FOC = timer matériel + tâche prio 20. USB / OTA restent dans `loop()`.

## Câblage ABZ

| Axe | A | B | Z |
|-----|---|---|---|
| moteur 0 (`left`) | 30 / IO18 | 31 / IO19 | 36 / IO22 |
| moteur 1 (`right`) | 29 / IO5 | 37 / IO23 | 33 / IO21 |

Courant (Makerbase) : left GPIO **39 / 36**, right **35 / 34**. PWM left 32/33/25, right 26/27/14. `M_EN` = GPIO12.

`ENC_PPR` = registre ABZ du chip (défaut **16384**). PCNT ×4 → CPR = 4×PPR.

## Flash

```powershell
cd H:\Projects\RobotRL\ESP32FOCHardwareCheck
pio run -e left -t upload
pio device monitor -e left
```

CH340 = souvent **COM6**, jamais COM1 (UART ACPI). Axe droit : `-e right`. 115200.

## Séquence

1. Boot = **`enc`**. `cnt` doit bouger à la main. Boot log : `ADC DMA ok` + offsets ~1.6 V.
2. `ol 20` + `mon 1` : plus de saut USB. Regarder `dt` / `dtmax`. Monter `hz 8000` → `10000` → `16000` tant que `dtmax < 0.7 × (1e6/hz)`.
3. `cal` puis `save`. Reboot : `zsearch`.
4. `tq 0.2` (ampères). `Iq` suit, pas d’emballement. Si le couple combat la consigne : `csflip`.
5. `vel 3` propre, roue **libre**, puis `accal` (3600 bins, ~2 min) ou `accal 360` smoke. Attendre `calib=0`, **pas** `index=0`. `ac save` puis `ac on`.

`vel` / `tq` / `accal` refusés sans index + cal + ADC. `tq` est en **ampères** (plus en volts).

## Commandes

```
help status enc
ol <rad/s>
cal save forget zsearch
vel <rad/s>
tq  <A>
limit <V>  ilim <A>  hz <Hz>  alignv <V>
accal [bins]   ac on|off|save|forget
dt   csflip   mon 0|1   ota   download
```

2S (`FOC_VBUS=8.4`). 3S : `-DFOC_VBUS=12.6`.
