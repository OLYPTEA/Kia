#  Kia Arm Controller (ESP32-S3) By OlypTea
 (Baser sur : https://www.youtube.com/watch?v=AIsVlgopqJc )

<img width="1537" height="813" alt="CarteKia" src="https://github.com/user-attachments/assets/dafd473e-ae1b-489c-9e00-e7207d854a1b" />

<img width="1656" height="872" alt="Kia-Arm-Ctrl22" src="https://github.com/user-attachments/assets/cb482844-e9b0-4ea2-b0ad-96000f585973" />

<img width="1918" height="1018" alt="KiaStudio" src="https://github.com/user-attachments/assets/8ba67ce8-2601-4a23-b78b-73d092ff01e2" />

https://github.com/user-attachments/assets/333e42ec-3789-4127-9b2e-19c26ccea488



Refonte complète (Hardware, Firmware & Software) d'un bras robotique imprimé en 3D (4 axes + pince), inspirée initialement du projet de la chaîne *Build Some Stuff*. 

Cette version abandonne le combo limité Arduino Uno + PCA9685 au profit d'une **carte sur mesure basée sur l'ESP32-S3**, capable de gérer la cinématique et l'alimentation de puissance de manière autonome.


---

##  Améliorations majeures

* **Calcul haute performance :** ESP32-S3 (Dual-core 240MHz, FPU, 16MB Flash + 8MB PSRAM Octal). Gère la cinématique directe/inverse (FK/IK) et l'interpolation par spline cubique directement en local.
* **Alimentation intégrée (6V/10A) :** Régulateur Buck synchrone TI TPS56C215 (pic à 15A) capable d'alimenter directement les servos de forte puissance (Miuzei 15 à 35kg).
* **Sécurité matérielle :** Surveillance de la puissance via INA219 et circuit d'arrêt d'urgence physique (**E-STOP** par MOSFETs) coupant la puissance sans affecter la logique du MCU.
* **Signal propre :** Abandon du PCA9685. Utilisation du périphérique natif **MCPWM** de l'ESP32-S3 avec conversion de niveau logique 3.3V vers 5V (SN74AHCT244).
* **Interface PC dédiée (Kia Studio) :** Application de contrôle et calibration 3D temps réel développée en Python (PySide6 + PyOpenGL).

---

##  Spécifications du Hardware & PCB

* **Dimensions :** 100 × 70 mm (conçu pour s'intégrer parfaitement dans l'espace de 113.5 × 80.7 mm de la base du bras / plaque MB2).
* **Stackup :** PCB 4 couches avec couches internes en cuivre épais de 2 oz (70 µm).
* **Routage :** Pistes d'alimentation VSERVO $\ge$ 3 mm et isolation stricte des masses (**Star Ground**) : connexions séparées pour le GND logique, puissance et analogique reliées en un seul point (Net tie) sous l'INA219.
* **Conception :** Réalisé sous **KiCad 10** (Schémas divisés en modules : Power, MCU, IO, Connectors).

 **Piège de routage évité :** Les GPIO 26 à 37 sont réservés à la PSRAM Octal de l'ESP32-S3 et sont strictement inutilisables.

---

##  Architecture du Firmware (Dual-Core)

Développé directement sous **ESP-IDF + FreeRTOS** pour maximiser les performances en temps réel.

* **Core 0 (PRO_CPU) : Gestion & Comms**
  * Parser USB CDC & Serveur BLE GATT (futurs services WiFi/HTTP).
  * Machine d'état principale de tout le système.
* **Core 1 (APP_CPU) : Temps Réel Strict**
  * Tâche ADC (100 Hz) : Suréchantillonnage x64, filtre EMA et hystérésis pour les 5 potentiomètres.
  * Solveur cinématique (FK/IK) et interpolation de trajectoire (Spline cubique) pour des mouvements fluides sans à-coups.
  * Driver MCPWM (50 Hz) pour le signal précis des servos (impulsions de 1 à 2 ms).
  * Supervision de sécurité (Lecture I2C en temps réel de l'INA219 & gestion de l'interruption E-STOP).

---

##  Logiciel PC : Kia Studio

Interface 3D légère basée sur une **architecture hexagonale** en Python 3.11 + PySide6 et un rendu graphique de bas niveau (pyqtgraph + PyOpenGL).

* **`proto/`** : Protocole de communication USB CDC partagé à 100 % avec le contrôleur.
* **`core/`** : Modèle cinématique pur Python et chargement des fichiers STL.
* **`ui/`** : Vue Qt, panneau de calibration 3D avec manipulation des axes XYZ (Pivot Gizmo) et sauvegarde des offsets dans un fichier de configuration JSON.

*(Note : L'application est générique. Il suffit de remplacer les modèles 3D et de rebuild l'app pour l'adapter à un autre bras possédant la même architecture).*

---

##  Modes de Fonctionnement

| Mode | Déclencheur | Logique principale | Utilisation |
| :--- | :--- | :--- | :--- |
| **Manual** | Interrupteur physique | Lecture potentiomètres $\rightarrow$ Filtre ADC $\rightarrow$ Angles $\rightarrow$ MCPWM | Test initial de chaque articulation et manipulation simple sans PC. |
| **App** | Commande PC | Cible via USB/BLE $\rightarrow$ Solveur IK $\rightarrow$ Spline cubique $\rightarrow$ MCPWM | Contrôle spatial complexe via Kia Studio, exécution de trajectoires. |
| **Calibration**| Commande / Bouton | Désactivation spline $\rightarrow$ Ajustement fin des neutres (largeur d'impulsion) $\rightarrow$ Sauvegarde NVS | Premier assemblage, remplacement de servo ou recentrage suite à un saut de dent. |
| **Fault** | E-STOP / INA219 | Coupure matérielle immédiate des servos + LED Alerte jaune & WS2812 rouge | Protection en cas de collision, blocage mécanique ou court-circuit. |



---

##  Remerciements & Références
* **Build Some Stuff** Fondateur du projet initiale
* **Espressif ESP-IDF** - Framework.
* **KiCad EDA** - Suite de conception électronique open-source.
* **PySide / Qt** - Framework UI pour Kia Studio.
* Merci aux membres de la communauté open-source pour les inspirations structurelles des bras robotiques 3D.
