# Protocole de test — réveil du body ECU par relais piloté

**Véhicule :** Lexus CT200h 2018 · **Cible :** réveiller le body ECU en sommeil profond (> ~10 min d'inactivité) pour que la commande d'unlock passe sur le CAN.

## Rappel du problème

Deux régimes mesurés (voir `PKE/README.md`) :

| Régime | Bus | Trames porte 0x750 |
| --- | --- | --- |
| Réactif (quelques min après un événement physique) | 0x45A tique à 1 Hz, trous de 36–48 s | **passent** — ACK + réponse sur 0x758 |
| Sommeil profond (~10+ min) | silence total | échouent, rien n'ACK |

L'injection CAN ne réveille pas la voiture en sommeil profond (`16 sent, 0 acked`). Le body ECU se réveille sur ses **entrées câblées** — les contacteurs de montant de porte (contacteur montant côté conducteur, Toyota **84231-60070**), pas sur l'activité CAN. Le plan : piloter cette ligne avec un relais pulsé.

## Ce qui est établi / ce qui ne l'est pas

- **Établi :** une ouverture de porte réveille l'ECU, voiture verrouillée au fob, **sans déclencher l'alarme**. L'antivol lit ce même contacteur et ne s'oppose donc pas à l'approche.
- **Pas encore isolé :** l'observation portait sur une porte ouverte *de l'intérieur*, ce qui ferme le contacteur de montant **et** tire la tringlerie de serrure (premier tir de poignée intérieure = déverrouillage). Donc on ne sait pas encore si c'est bien le contacteur seul qui gouverne le sommeil.

Le **Test A** ci-dessous lève cette ambiguïté **sans aucun accès électrique** et doit être fait avant de câbler quoi que ce soit.

---

## Sécurité et préalables

- Voiture en stationnement sûr, moteur coupé (bouton OFF), frein de parking.
- Batterie auxiliaire 45 Ah : ne pas laisser le bus maintenu éveillé des heures (le keepalive la vide).
- Ne rien épisser dans le faisceau tant que le Test A n'a pas confirmé le contacteur comme entrée de sommeil.
- Le relais, quand il viendra, doit être **isolé** du 12 V du véhicule (contacts secs) : on commute uniquement la ligne du contacteur, on n'y injecte pas de tension.

## Matériel

- Logger CAN : montage **odb-tool** (ESP32 + MCP2515 + SN65HVD230) branché OBD-II broches 6/14 (CAN-H/CAN-L) + masse 4/5. Crystal du MCP2515 bien réglé dans `config.h` (`MCP_OSC_FREQ`).
- Serial Monitor à 115200 baud, Newline.
- Du ruban adhésif résistant (pour le Test A).
- Pour le Test B : un relais (bobine 5 V pilotée par GPIO via transistor + diode de roue libre, ou module relais opto-isolé), contacts **en parallèle** du contacteur de montant.

---

## Montage du logger et ce qu'on doit voir

Démarrer odb-tool en mode **normal** (défaut — indispensable : en listen-only il n'ACK pas et fait avorter le réveil). Repères dans le Serial Monitor :

- `*** BUS SILENT ***` : la voiture s'endort ; le compteur de silence démarre.
- `*** BUS AWAKE after <ms> of silence ***` : réveil détecté + table des premières trames.
- Trames **0x750** qu'on envoie (`unlock`) et surtout la réponse **0x758** : octet 2 = `0x70` → le body ECU a bien **traité** la commande (preuve réelle, pas un simple ACK).

Laisser `status` sous la main : il donne l'état du bus, le mode et les compteurs d'erreur (`EFLG`, `TEC`).

---

## Test A — plongeur scotché (baseline, sans câblage)

But : confirmer que le contacteur de montant seul gouverne le sommeil, et que « relâcher le plongeur » (ce que fera le relais) réveille l'ECU — sans toucher à la serrure ni au faisceau.

1. Ouvrir la porte conducteur, **scotcher le plongeur du contacteur enfoncé** (la voiture croit la porte fermée, porte physiquement ouverte).
2. Verrouiller au fob. Logger en marche.
3. Attendre le **sommeil profond** : `*** BUS SILENT ***` puis au moins **10–13 min** de silence continu (confirmer qu'un `unlock` échoue bien à ce stade).
4. **Relâcher le plongeur** (le laisser ressortir). La serrure ne bouge pas, aucune poignée n'est touchée : c'est électriquement identique à ce que fera le relais.
5. Observer le logger.

**Lecture du résultat :**

| Observation | Conclusion |
| --- | --- |
| `BUS AWAKE` apparaît au relâchement, et un `unlock` passe ensuite (758 → octet 2 `0x70`) | contacteur de montant = l'entrée de réveil ; feu vert pour le relais |
| Rien au relâchement | le réveil observé venait de la tringlerie/poignée, pas du contacteur → revoir l'approche avant de câbler |

Noter aussi : **l'alarme sonne-t-elle ?** Elle ne doit pas (l'observation précédente était silencieuse). Si elle sonne, s'arrêter.

---

## Test B — relais sur la ligne du contacteur

À faire seulement si le Test A est concluant.

**Câblage :** contacts du relais **en parallèle** du contacteur de montant conducteur. La ligne est à un état haut via le pull-up interne du body ECU (porte fermée) et mise à la masse par le corps métallique du contacteur (porte ouverte). Fermer le relais met la ligne à la masse = « porte ouverte » ; l'ouvrir la laisse remonter au pull-up = « porte fermée ». (Détails fil/topologie/contrôles carte : `Pin-diagram.md`.)

**Séquence d'impulsions** — ne pas pulser une seule fois. La procédure Toyota pour un body ECU endormi est : ouvrir la porte conducteur, puis ouvrir/fermer une porte **plusieurs fois à ~1,5 s d'intervalle**. Donc un **train** :

- Fermer le relais 1,5 s (porte « ouverte »), ouvrir 1,5 s (porte « fermée »), répéter 4–6 cycles.
- Durées d'impulsion à balayer si le premier essai ne réveille pas : **300 ms**, **800 ms**, **1,5 s**, **3 s** par état.

**Déroulé du test :**

1. Verrouiller au fob, logger en marche, attendre le sommeil profond (`BUS SILENT` + 10–13 min).
2. Lancer le train d'impulsions relais.
3. Vérifier `BUS AWAKE` dans le logger, puis envoyer `unlock` et confirmer la réponse 0x758.
4. Mesurer le **délai** entre la dernière impulsion et le moment où l'`unlock` passe (combien de temps le réveil tient).
5. Répéter à froid (plusieurs heures de sommeil) pour confirmer la robustesse.

**Critères de réussite :**

- `BUS AWAKE` déclenché par le relais seul (aucune porte réellement ouverte, aucun fob).
- `unlock` 0x750 répondu sur 0x758 (octet 2 `0x70`) après le réveil.
- Aucune alarme.
- Réveil reproductible après sommeil profond prolongé.

---

## Journalisation

Pour chaque essai, garder la sortie Serial (copier-coller depuis le Serial Monitor) avec : durée de silence avant réveil, trames de la table `BUS AWAKE`, trame 0x758, délai réveil→unlock. C'est ce qui permettra de figer la durée d'impulsion finale dans le firmware.

## Risques / points d'attention

- Ne pas maintenir le bus éveillé longuement (batterie aux 45 Ah).
- Relais à contacts secs, isolé du 12 V ; diode de roue libre si bobine pilotée par GPIO.
- Tant que le Test A n'a pas isolé le contacteur, ne rien épisser dans le montant A.
