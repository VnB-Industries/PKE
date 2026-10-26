# Protocole de test — réveil de l'unité voiture par le bus CAN

> **Abandonné le 26/10 sur `nrf52840/UWB`.** Le module CAN n'est plus alimenté hors session BLE (P-MOS sur D7) : le bus ne peut plus réveiller l'unité, et `door_arm_bus_wake()` a été retiré. Seule l'entrée de réveil D14 sort l'unité du System OFF. Ce protocole est gardé pour mémoire.

**Véhicule :** Lexus CT200h 2018 · **Cible :** vérifier que l'activité du bus CAN de la prise OBD sort l'unité voiture du System OFF, pour se passer du bouton caché et de la diode sur le contacteur de montant.

## Principe

C'est l'inverse du réveil par relais (`reveil-ecu-relais.md`) : ici, c'est la voiture qui réveille l'unité. Juste avant le System OFF, `door_arm_bus_wake()` (`car/src/door.c`) démarre le MCP2515 en mode écoute seule (Listen-Only : il n'acquitte ni n'émet rien) et donne le SENSE à sa broche INT. La première trame reçue lève l'interruption de réception, INT passe à l'état bas et le nRF52840 redémarre. Le SN65HVD230 repasse en veille (Rs haut), où son récepteur reste actif. Le mode Sleep du MCP2515 n'est pas utilisé, parce que son quartz ne redémarre pas sur ce module (`MCP_SLEEP_ENABLED 0`). Rien à modifier sur le matériel.

Ce qui est déjà mesuré (README, « Deep sleep is real ») : en sommeil profond, le bus est totalement silencieux ; une ouverture de porte le réveille, et `0x45A` tique à ~1 Hz pendant quelques minutes. Ce qui ne l'est pas : quels gestes réveillent le segment OBD (clé mécanique, télécommande d'origine, poignée intérieure), et si le bus se réveille aussi tout seul (contrôles périodiques de l'hybride, par exemple).

## Préparation

1. Dans `car/src/power.h`, raccourcir les délais pour le test, par exemple `POWER_DEEP_IDLE_AFTER_S` à 60 et `POWER_OFF_AFTER_S` à 120. **À remettre ensuite.**
2. Laisser l'entrée de réveil (D14, J1 broche 11, sur la carte `carte_uwb`) non câblée pour ce test, afin que seul le bus puisse réveiller l'unité.
3. Flasher (`scripts/flash.sh car`) et lire les logs sur l'USB. Pendant le test, alimenter l'unité par l'USB d'un portable **sans** le 12 V (jamais les deux à la fois), en reliant seulement la masse de la prise OBD (broche 4 ou 5) à la masse de l'unité, en plus de CAN-H et CAN-L.
4. Garder le fob loin de la voiture (ou le poser immobile pour qu'il dorme).

## Test C1 — l'unité entre en System OFF

- Attendre `entering System OFF until the wake input or CAN activity` dans les logs.
- Si le log dit `until the wake input` seul, le MCP2515 n'a pas démarré au boot : vérifier le câblage CAN et `osc-freq`.

## Test C2 — gestes qui réveillent l'unité

Pour chaque geste, attendre d'abord que le bus soit en sommeil profond (au moins 15 min sans toucher la voiture, vérifiable au sniffer), puis noter si l'unité redémarre (bannière de boot, `car ready`) et au bout de combien de temps.

| Geste | Bus réveillé ? | Unité réveillée ? | Délai |
| --- | --- | --- | --- |
| Télécommande d'origine : déverrouiller | | | |
| Clé mécanique dans la serrure | | | |
| Ouvrir la porte conducteur (déjà déverrouillée) | | | |
| Approcher avec le fob d'origine (Smart Access) sans toucher | | | |

## Test C3 — réveils parasites

Laisser la voiture une nuit, unité en System OFF, et compter les redémarrages dans les logs (ou avec un compteur en RAM retenue si l'USB n'est pas branché). Un réveil parasite n'est pas grave : l'unité scanne 2 jours puis revient en System OFF. Mais s'il y en a souvent, la consommation moyenne monte.

## Décision

- Si C2 montre que le premier geste normal d'arrivée (télécommande d'origine ou porte) réveille l'unité : le bouton caché et la diode sur le contacteur de montant deviennent inutiles. On garde l'entrée de réveil dans le firmware et sur la carte (D6 sur `nrf52840/main`, D14 sur `carte_uwb`), mais sans rien y câbler.
- Sinon, garder le câblage de l'entrée de réveil.

Après le test, remettre les délais de `power.h`.
