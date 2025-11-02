# LandingGear Moto - Système de Béquillage Central Motorisé (Arduino Nano R4)

Ce projet open source est un système électronique avancé de pilotage pour béquillage central motorisé à double bras (Landing Gear) pour moto. Il permet un stationnement sécurisé et le déplacement à très faible vitesse. Le système est basé sur la plateforme **Arduino Nano R4** et intègre de multiples niveaux de sécurité, de synchronisation et de journalisation.

## Contexte et Philosophie du Projet

Ce projet est une réponse directe à une défaillance du marché. Suite à l'acquisition d'un système de béquillage motorisé coûteux (même chez un revendeur local [TAMOTO.fr]), des **défauts de conception critiques** ont été identifiés dans l'électronique de puissance et la gestion de la sécurité. En l'absence totale de service après-vente ou de support fiable, l'achat de tels produits représente un risque injustifié, rendant l'achat direct en Chine souvent plus logique et bien moins cher.

L'objectif de ce dépôt est de fournir une **solution alternative, open source, fiable et auditable** par la communauté, corrigeant ces défauts par une ingénierie plus rigoureuse (double surveillance de courant, synchronisation active, sécurité GPS absolue).

**Ce code est mon don à la communauté** pour que d'autres puissent bénéficier d'un système de sécurité de haute qualité, construit sur des bases saines.

Ce dépôt a pour but de fournir une **alternative open source** :
1.  **Fiable et Auditable :** Le code est public, permettant à la communauté de vérifier et d'assurer la rigueur des sécurités implémentées (notamment le double contrôle de courant et la gestion GPS).
2.  **Qualitative :** Il corrige les lacunes constatées en intégrant des mécanismes avancés (synchronisation P-only, sécurité immédiate à 30 km/h).

**Ceci est un partage communautaire.** Nous rendons la technologie de pilotage de systèmes critiques accessible et sûre, sans dépendre d'intermédiaires défaillants.

---

##  Fonctionnalités Clés

* **Plateforme :** Développé sur Arduino Nano R4 (ABX00143) et son Module d'Extension (Connector Carrier ASX00061).
* **Pilotage Moteur :** Contrôle de deux moteurs indépendants via drivers haute puissance **BTS7960**.
* **Sécurité et Surveillance :**
    * **Surveillance du Courant :** Capteurs **ACS712 30A** pour la détection de surcharge, le déploiement virtuel et la synchronisation.
    * **Fins de Course Hybrides :** Mécaniques pour la rétraction (TZ-8122), virtuelles (seuil de courant) pour le déploiement.
    * **Sécurité GPS :** Blocage automatique de toute commande à $\ge 25 \text{ km/h}$ et **rétraction immédiate et verrouillage** du système à $\ge 30 \text{ km/h}$ si les bras sont déployés.
* **Précision & Robustesse :**
    * **Synchronisation Dynamique :** Rétroaction de courant (logique P-only) pour maintenir l'équilibre des bras.
    * **Journalisation des Données :** Écriture des événements critiques, des défauts et des données de cycle sur carte **microSD** (fichier `LOG.TXT`) pour analyse post-mortem.
* **Interface Utilisateur :** Armement/désarmement et contrôle de l'action via boutons dédiés et LED d'état.

##  Démarrage Rapide

### Matériel Requis (BOM)

| Composant | Référence/Type | Rôle |
| :--- | :--- | :--- |
| Microcontrôleur | Arduino Nano R4 (ABX00143) | Cœur du système |
| Drivers Moteur (x2) | BTS7960 (43A) | Pilotage Moteurs Gauche/Droit |
| Capteurs de Courant (x2) | ACS712 30A | Rétroaction et Sécurité |
| GPS | Module compatible TinyGPS++ (ex: NEO-M8N) | Vitesse et position |
| Stockage | Module Carte microSD | Journalisation des données |

### Câblage et Assemblage

Le fichier **`LandingGearMoto_ArduinoNanoR4.pdf`** contient le schéma de câblage détaillé pour connecter tous les composants au **Nano Connector Carrier**.

* Consultez également le tableau de câblage dans la documentation technique [Doc_LandingGearNanoR4.pdf](Doc_LandingGearNanoR4.pdf) pour les affectations des broches.

### Installation du Code

1.  Téléchargez le fichier `LandingGear.ino`.
2.  Installez les librairies suivantes dans l'IDE Arduino : `TinyGPS++`, `SD`, et `EEPROM`.
3.  Compilez et téléversez le code sur votre Arduino Nano R4.

##  Licences

| Élément | Licence | Description |
| :--- | :--- | :--- |
| **Code Source** (`.ino`) | **[MIT License](LICENSE)** | Permissive, permet l'utilisation et la modification par tous (y compris à des fins commerciales). |
| **Documentation et Schémas** (`.pdf`, images) | **CC BY-SA 4.0** | Permet le partage et la modification à condition de citer l'auteur et de redistribuer les modifications sous la même licence. |

---

## Contribuer

Les contributions sont les bienvenues ! Si vous trouvez un bug, proposez une amélioration ou ajoutez de nouvelles fonctionnalités, n'hésitez pas à soumettre une *Pull Request* ou à ouvrir une *Issue* sur ce dépôt.