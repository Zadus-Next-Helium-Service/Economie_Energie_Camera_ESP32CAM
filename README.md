
# ESP32-CAM — Surveillance, contrôle à distance et gestion intelligente de l'éclairage

![Arduino](https://img.shields.io/badge/Arduino-ESP32-blue)
![Edge Impulse](https://img.shields.io/badge/IA-Edge%20Impulse-orange)
![FreeRTOS](https://img.shields.io/badge/OS-FreeRTOS-green)
![Licence](https://img.shields.io/badge/Licence-MIT-yellow)
![Plateforme](https://img.shields.io/badge/Plateforme-ESP32--CAM-red)

> Système embarqué intelligent qui surveille un espace en temps réel, contrôle des équipements 
> électriques à distance via WiFi, et gère automatiquement l'éclairage grâce à la détection de 
> présence par intelligence artificielle — sans cloud, sans abonnement, depuis n'importe quel navigateur.

---

## Démonstration

> Je mettrais l image plustard de l interface web

---

## Fonctionnalités

- **Surveillance vidéo en temps réel** — flux vidéo live accessible depuis tout navigateur WiFi
- **Caméra orientable à distance** — servo panoramique contrôlable depuis l'interface web
- **Contrôle à distance d'équipements** — allumer/éteindre comme un interrupteur connecté
- **Compatibilité interrupteur physique** — l'interrupteur normal continue de fonctionner en parallèle
- **Détection de personne par IA embarquée** — modèle Edge Impulse, seuil 80%, sans cloud
- **Extinction automatique** — la lumière s'éteint seule après 3 minutes sans présence
- **Mode nuit automatique** — synchronisation NTP, système en veille entre 22h et 6h
- **Mode manuel / automatique** — basculement depuis l'interface web
- **Diagnostics temps réel** — mémoire, uptime, erreurs, état complet du système
- **Mémoire persistante** — configuration sauvegardée après redémarrage

---

## Matériel nécessaire

| Composant | Quantité | Broche ESP32-CAM |
|-----------|----------|-----------------|
| ESP32-CAM AI Thinker | 1 | — |
| Servo moteur 5V | 1 | GPIO 13 |
| Module relais 5V | 1 | GPIO 14 |
| Capteur LDR (numérique) | 1 | GPIO 15 |
| Adaptateur FTDI | 1 | TX / RX (flash uniquement) |
| Alimentation 5V / 2A | 1 | — |

---

## Installation

L'installation se fait en **trois grandes étapes** : préparer Arduino IDE, installer les bibliothèques, 
puis créer et importer le modèle IA Edge Impulse.

---

### Étape 1 — Installer Arduino IDE et le support ESP32

1. Télécharger et installer [Arduino IDE 2.x](https://www.arduino.cc/en/software)
2. Ouvrir Arduino IDE → `Fichier → Préférences`
3. Dans le champ **URL de gestionnaire de cartes supplémentaires**, ajouter :
https://dl.espressif.com/dl/package_esp32_index.json
4. Aller dans `Outils → Type de carte → Gestionnaire de cartes`
5. Rechercher `esp32` et installer le package **ESP32 by Espressif Systems**
6. Sélectionner la carte : `Outils → Type de carte → ESP32 Arduino → AI Thinker ESP32-CAM`

> Cette étape installe automatiquement toutes les bibliothèques système du projet :
> `esp_camera`, `WiFi`, `esp_http_server`, `esp_heap_caps`, `esp_task_wdt`, `Preferences`, `time`

---

### Étape 2 — Installer les bibliothèques Arduino

#### ESP32Servo — à installer depuis le gestionnaire Arduino

1. Aller dans `Outils → Gérer les bibliothèques`
2. Rechercher `ESP32Servo`
3. Installer **ESP32Servo by Kevin Harrington**

> Toutes les autres bibliothèques (`esp_camera.h`, `WiFi.h`, `Preferences.h`, etc.) sont déjà 
> incluses automatiquement avec le package ESP32 installé à l'étape 1. Aucune installation 
> supplémentaire n'est nécessaire pour elles.

---

### Étape 3 — Créer et importer le modèle IA Edge Impulse

C'est l'étape la plus importante et la plus spécifique à ce projet. Le fichier 
`EconomieEnergieBBITEST_inferencing.h` n'est pas une bibliothèque que l'on télécharge — 
c'est un modèle d'intelligence artificielle que **tu dois entraîner toi-même** sur la plateforme 
Edge Impulse, puis exporter pour Arduino.

#### Pourquoi entraîner son propre modèle ?

Le modèle IA doit être capable de reconnaître des personnes dans les conditions spécifiques 
de ton environnement : l'angle de ta caméra, l'éclairage de ta pièce, la distance de détection 
souhaitée. Un modèle générique ne donnera pas de bons résultats. C'est pourquoi chaque 
déploiement nécessite son propre modèle entraîné.

#### Vidéo de référence

Il n'existe pas encore de vidéo officielle montrant exactement comment créer un modèle de 
détection de personnes pour ESP32-CAM avec Edge Impulse. Cependant, la méthode est 
identique à la détection d'autres objets. Cette vidéo montre le processus complet et peut 
être suivie en adaptant les données d'entraînement à des personnes :

> 📺 **[Tutoriel Edge Impulse pour ESP32-CAM — Object Detection](https://youtu.be/L2w160DwM2k?si=M9mHNzR2tXxbDf5D)**

#### Étapes pour créer le modèle

1. Créer un compte gratuit sur [edgeimpulse.com](https://edgeimpulse.com)
2. Créer un nouveau projet
3. Aller dans **Data acquisition** → collecter des images avec ta caméra ESP32-CAM :
   - Des images avec une personne présente dans le champ → étiqueter `personne`
   - Des images sans personne → étiqueter `rien`
   - Collecter au minimum 50 à 100 images par catégorie pour un bon résultat
4. Aller dans **Impulse design** → créer le pipeline :
   - Input : Image 96x96 pixels (ou 160x120)
   - Processing block : Image
   - Learning block : Transfer Learning (Images)
5. Aller dans **Training** → lancer l'entraînement
6. Vérifier les performances dans **Model testing**
7. Aller dans **Deployment** → sélectionner **Arduino library**
8. Cliquer sur **Build** → télécharger le fichier `.zip`

#### Importer la bibliothèque dans Arduino IDE

1. Dans Arduino IDE : `Croquis → Inclure une bibliothèque → Ajouter la bibliothèque .ZIP`
2. Sélectionner le fichier `.zip` téléchargé depuis Edge Impulse
3. La bibliothèque apparaît maintenant dans la liste — le projet peut être compilé

#### Cloner le modèle utilisé dans ce projet

Si tu veux partir directement du modèle entraîné pour ce projet comme point de départ :

> 🔗 **[Cloner le projet Edge Impulse — EconomieEnergieBBITEST](https://studio.edgeimpulse.com/studio/944878/impulse/1/learning/keras-object-detection/7)**



Pour rendre ton projet Edge Impulse public et obtenir ce lien :
`Edge Impulse → Ton projet → Dashboard → Make public`

---

### Étape 4 — Configurer et flasher

1. Ouvrir `src/main.ino` dans Arduino IDE
2. Modifier les identifiants WiFi :
```cpp
   const char* wifi_ssid     = "VOTRE_RESEAU_WIFI";
   const char* wifi_password = "VOTRE_MOT_DE_PASSE";
```
3. Connecter l'ESP32-CAM via l'adaptateur FTDI avec **GPIO0 relié à GND** (mode flash)
4. Sélectionner le bon port : `Outils → Port`
5. Téléverser le code
6. Débrancher GPIO0 de GND, appuyer sur le bouton Reset
7. Ouvrir le moniteur série à **115200 bauds** pour voir l'adresse IP

---

### Étape 5 — Accéder à l'interface
Interface principale  →  http://[IP_ESP32]/
Diagnostics système   →  http://[IP_ESP32]/health
Flux vidéo direct     →  http://[IP_ESP32]:81/stream

Accessible depuis tout navigateur (téléphone ou ordinateur) connecté au même réseau WiFi.

---

## API disponible

| Endpoint | Action |
|----------|--------|
| `GET /` | Interface web principale |
| `GET /light/on` | Allumer la lumière (mode manuel) |
| `GET /light/off` | Éteindre la lumière (mode manuel) |
| `GET /light/auto` | Retour au mode automatique IA |
| `GET /servo/left` | Tourner le servo à gauche (−10°) |
| `GET /servo/right` | Tourner le servo à droite (+10°) |
| `GET /status` | État complet du système |
| `GET /health` | Diagnostics détaillés |
| `GET :81/stream` | Flux vidéo MJPEG en direct |

---

## Structure du dépôt
esp32cam-detection-lumiere/
├── README.md               ← Ce fichier
├── RAPPORT.md              ← Contexte, impact et présentation du projet
├── LICENSE
├── src/
│   └── main.ino            ← Code source principal
├── hardware/
│   ├── schema_cablage.png  ← Schéma de câblage
│   └── composants.md       ← Liste complète du matériel
└── assets/
├── demo.gif            ← Démonstration animée
└── screenshots/        ← Captures de l'interface web

---

## Auteur

Développé par **[ZADUS]**
Centre d'incubation — Afrique de l'Ouest, 2025

- Contact : [nexteliumservice@gmail.com]
-whatsapp[+226 67250288]

---

## Licence

Ce projet est distribué sous licence MIT.
Libre de l'utiliser, le modifier et le redistribuer avec attribution.





# Economie_Energie_Camera_ESP32CAM
“Système IoT basé sur ESP32-CAM pour une maison intelligente : surveillance vidéo avec détection de personnes, contrôle d’éclairage à distance, optimisation énergétique et automatisation domotique. Intègre capteurs LDR pour une gestion intelligente de la présence et de la luminosité.
