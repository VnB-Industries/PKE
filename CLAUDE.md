# PKE : note d'état pour les agents (mise à jour du 28/10/2026)

Entrée sans clé DIY pour une Lexus CT200h 2018. Deux applications Zephyr / nRF Connect SDK sur XIAO nRF52840 Plus :
- `car/` : unité voiture sur la prise OBD (BLE, CAN MCP2515 + SN65HVD230, relais de réveil, micro INMP441 pour la double tape, UWB DW3000).
- `fob/` : clé (BLE, UWB DW3000, ADXL362, LiPo).
- `common/` : protocole et crypto partagés. Build et flash : `scripts/flash.sh car|fob [-p]` (bootloader UF2, pas le bouton Flash de l'extension VS Code).

Le `README.md` (anglais) détaille le fonctionnement ; ne le lire qu'au besoin.

## Branches
| Branche | Matériel | Brochage dans |
| --- | --- | --- |
| `nrf52840/UWB` (celle-ci) | XIAO soudé sur la carte support `carte_uwb` | `car/app.overlay`, `fob/app.overlay` |
| `nrf52840/main` | XIAO câblé à la main, sans UWB | `car/app.overlay` |
| `ESP32/main` | ancienne unité ESP32 (Arduino), sans UWB | `car/src/board.h` |

Le brochage de `nrf52840/UWB` suit la carte : toute modification passe par une nouvelle révision de `carte_uwb`.

## Matériel retenu : `carte_uwb` rév. E-PB-stock (montée à la main)
- D7 commande la grille du P-MOS Q1 (IRLML6402, 100 kΩ de tirage, **bas = module CAN alimenté**). 3,3 V commuté vers J1.
- **C4 = 1 µF 25 V 0805** (démarrage progressif de Q1, ~4 ms, appel de courant ~0,3 A au lieu de 0,5 à 1,1 A).
- **C3 = 22 µF 1206**, validé par simulation pour les salves UWB.
- DW3000 sur SPIM3 (D0 à D6, IRQ sur D6), MCP2515 sur D17 à D19 + CS D8 + INT D9, relais D10, réveil D14.

## Conventions
- Code et commentaires en anglais, style Zephyr (tabulations, `LOG_*`), réglages dans `*_config.h`.
- Le module CAN n'est alimenté que pendant une session fob (`mcp_power_on()` / `mcp_power_off()` dans `car/src/door.c`). Avant de le couper, relâcher INT, CS, SCK et MOSI, sinon il est alimenté par ses diodes de protection.
- Le CAN ne réveille pas la voiture (ECU carrosserie Toyota) : réveil par relais sur le contacteur de montant.

## À faire (firmware)
- [x] **Délai d'allumage du MCP2515** : attendre ≥ 10 ms après D7 bas avant la première commande SPI (`MCP_POWER_UP_MS`, appliqué au boot dans `can_power_boot()` et à chaque session dans `mcp_power_on()`). Ne pas le réduire sous ~10 ms avec C4 = 1 µF.
- [ ] Compiler et tester la coupure du module CAN sur la carte : courant hors session (~0 µA), réponse à la première requête OBD-II après rallumage.
- [ ] Pilote DW3000 : `car/src/ranging.c` est un factice qui passe toujours ; côté fob, `RANGE_REQ` ne fait que dériver la clé STS.
- [ ] Quartz du MCP2515 : lire la valeur sur le module et la reporter dans `osc-freq` (`car/app.overlay`). 8 MHz par défaut.
- [ ] Rapport engagé : trouver `GEAR_PID`, `GEAR_BYTE`, `GEAR_P_VALUE` puis `VEHICLE_GEAR_ENABLED` à 1.
- [ ] Déverrouillage automatique désactivé (`AUTO_UNLOCK_ENABLED` 0) jusqu'à ce qu'on lui fasse confiance.
- [ ] Fob : INT1 de l'ADXL362 bloqué à l'état haut fob posé (le fob ne dort plus) ; remettre `CONFIG_BT_RPA_TIMEOUT` à 900 s après les tests.

## Tests sur la voiture
Protocoles dans `tests/` : `reveil-ecu-relais.md` (réveil par le contacteur), `reveil-unite-bus-can.md`.
