# PKE : note d'état pour les agents (branche `nrf52840/main`, mise à jour du 28/10/2026)

Entrée sans clé DIY pour une Lexus CT200h 2018. Deux applications Zephyr / nRF Connect SDK sur XIAO nRF52840 Plus, **sans UWB** sur cette branche :
- `car/` : unité voiture sur la prise OBD (BLE, CAN MCP2515 + SN65HVD230, relais de réveil, micro INMP441 pour la double tape dans `car/src/tap/`).
- `fob/` : clé (BLE, ADXL362, LiPo).
- `common/` : protocole et crypto partagés. Build et flash : `scripts/flash.sh car|fob [-p]` (bootloader UF2, pas le bouton Flash de l'extension VS Code).

Le `README.md` (anglais) détaille le fonctionnement ; ne le lire qu'au besoin.

## Branches
| Branche | Matériel | Brochage dans |
| --- | --- | --- |
| `nrf52840/main` (celle-ci) | XIAO câblé à la main, sans UWB | `car/app.overlay`, `fob/app.overlay` |
| `nrf52840/UWB` | XIAO soudé sur la carte support `carte_uwb` (UWB, CAN commuté par P-MOS) | `car/app.overlay` |
| `ESP32/main` | ancienne unité ESP32 (Arduino), sans UWB | `car/src/board.h` |

Pas d'UWB ici : toutes les broches du bord sont prises. Le travail UWB et la coupure du module CAN se font sur `nrf52840/UWB`.

## Brochage
- Voiture : D0 relais, D1 CS MCP2515, D2 INT MCP2515, D3/D4/D5 INMP441 SCK/WS/SD, D6 entrée de réveil (vers la masse), D8/D9/D10 SPI du MCP2515.
- Fob : ADXL362 CS D1, INT1 D2, SPI D8/D9/D10.

## Matériel
- Module CAN toujours alimenté, MCP2515 en mode normal (Sleep, Rs et `door_sleep()` retirés).
- **Jamais sur la broche 3V3 du XIAO** : elle ne tient pas les pointes du SN65HVD230 à l'émission (erreurs de bit puis bus-off). Régulateur 3,3 V dédié (MCP1700), masse commune, 47 à 100 µF + 100 nF au module.
- Relais à bobine 3,3 V (SRD-03VDC), actif à l'état haut, LED dessoudée.

## Conventions
- Code et commentaires en anglais, style Zephyr (tabulations, `LOG_*`), réglages dans `*_config.h`.
- Le CAN ne réveille pas la voiture (ECU carrosserie Toyota) : réveil par relais sur le contacteur de montant (`RELAY_WAKE_ENABLED`).
- Parking : scan BLE à 50 % pendant 2 jours, 1 s toutes les 60 s jusqu'à 4 jours, puis System OFF. Seule l'entrée D6 en sort.
- Les sources de `car/src/tap/` viennent du dépôt tapDetector : les recopier plutôt que les modifier ici.

## À faire (firmware)
- [ ] Câbler l'entrée de réveil D6 (bouton vers la masse et/ou ligne du contacteur par une BAT54, cathode côté ligne).
- [ ] Quartz du MCP2515 : lire la valeur sur le module et la reporter dans `osc-freq` (`car/app.overlay`). 8 MHz par défaut.
- [ ] Rapport engagé : trouver `GEAR_PID`, `GEAR_BYTE`, `GEAR_P_VALUE` (`car/src/vehicle_config.h`) puis `VEHICLE_GEAR_ENABLED` à 1.
- [ ] Déverrouillage automatique désactivé (`AUTO_UNLOCK_ENABLED` 0 dans `car/src/main.c`) jusqu'à ce qu'on lui fasse confiance.
- [ ] `car/src/ranging.c` est un factice (pas d'UWB) : `-ENOTSUP` compte comme contrôle réussi.
- [ ] Micro : comparer `floor` et `peak` des lignes `EVT` avec l'ESP32, puis refaire la calibration et le gabarit FFT des tapes.
- [ ] Fob : INT1 de l'ADXL362 bloqué à l'état haut fob posé (le fob ne dort plus) ; remettre `CONFIG_BT_RPA_TIMEOUT` à 900 s après les tests.

## Tests sur la voiture
Protocole dans `tests/reveil-ecu-relais.md` (réveil par le contacteur, test A puis test B relais sur D0).
