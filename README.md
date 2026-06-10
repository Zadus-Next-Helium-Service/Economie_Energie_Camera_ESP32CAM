# 🏠 Système Domotique Intelligent — ESP32-CAM + IA Embarquée

> **Conception et réalisation d'un système domotique intelligent de surveillance vidéo et de contrôle d'équipements électriques par intelligence artificielle embarquée**  
> Réalisé au FabLab du **Burkina Business Incubator (BBI)** — Ouagadougou, Burkina Faso  
> Stage de fin de cycle — Licence Génie Électrique, option Électronique et Informatique Industrielle — UTM 2025-2026

---

## 📋 Table des matières

- [À propos du projet](#-à-propos-du-projet)
- [Fonctionnalités](#-fonctionnalités)
- [Matériel requis](#-matériel-requis)
- [Architecture du système](#-architecture-du-système)
- [Modèle IA embarquée](#-modèle-ia-embarquée)
- [Modes de fonctionnement](#-modes-de-fonctionnement)
- [Installation et configuration](#️-installation-et-configuration)
- [Interface web](#-interface-web)
- [Alertes Telegram](#-alertes-telegram)
- [Résultats obtenus](#-résultats-obtenus)
- [Perspectives d'amélioration](#-perspectives-damélioration)
- [Démonstrations](#-démonstrations)
- [Auteur](#-auteur)

---

## 📖 À propos du projet

Ce projet est né d'une observation simple : la salle de réunion du FabLab du BBI restait régulièrement avec ses lumières et ses équipements allumés alors qu'elle était vide. Ce qui a commencé comme une idée d'économie d'énergie s'est transformé en un système domotique complet, intelligent et autonome.

**Le problème central :**  
Les systèmes de surveillance et de domotique disponibles au Burkina Faso sont coûteux, entièrement importés et dépendants de services cloud étrangers — alors que les besoins locaux sont réels : surveiller des locaux la nuit, éteindre automatiquement les équipements en l'absence d'occupants, recevoir une alerte en cas d'intrusion.

**La réponse apportée :**  
Un système qui fait tourner l'intelligence artificielle **directement sur le microcontrôleur**, sans connexion internet obligatoire, sans cloud étranger, avec des composants disponibles localement à Ouagadougou pour environ **13 000 FCFA** pour le composant principal.

---

## ✅ Fonctionnalités

### Surveillance vidéo
- Flux vidéo **MJPEG en temps réel** accessible depuis tout navigateur WiFi (port 81)
- Balayage panoramique de **0° à 100°** par pas de 10° grâce au servomoteur
- Enregistrement automatique de **photos horodatées** sur carte microSD à chaque détection suspecte

### Détection de présence par IA embarquée
- Modèle **FOMO (Faster Objects More Objects)** entraîné sur 740 images du FabLab du BBI
- Inférence directement sur l'ESP32-CAM, **sans cloud, sans internet obligatoire**
- Score F1 de **83,1%** obtenu après optimisation
- **Seuil de confiance à 90%** pour éviter les faux positifs
- **Double détection** : une présence est validée seulement si détectée 2 fois en moins de 10 secondes

### Contrôle automatique des équipements
- Allumage automatique des équipements dès qu'une présence est détectée
- Extinction automatique après **4 minutes d'inactivité**
- Retour d'état en temps réel via capteur **LDR** (vérification que l'état réel correspond à la commande)
- Montage en **va-et-vient** avec l'interrupteur mural existant (qui continue de fonctionner normalement)

### Interface web embarquée
- Accessible depuis tout navigateur connecté au même réseau WiFi, **sans installation**
- Adresse IP fixe configurable depuis le routeur
- Contrôle en temps réel : allumage/extinction, déplacement du servomoteur, changement de mode
- Gestion des **jours fériés** directement depuis l'interface (ajout/modification/suppression)
- Page `/health` pour surveiller l'uptime, la mémoire heap, le nombre de détections et l'état NTP

### Alertes intelligentes
- Envoi automatique via **API Telegram** d'une photo + lien direct vers l'interface web
- Alertes actives la **nuit (22h–6h)**, les **weekends** et les **jours fériés**
- **Mode absence** : alertes permanentes quelle que soit l'heure

### Robustesse et fiabilité
- Architecture **FreeRTOS** avec répartition des tâches sur les **deux cœurs** de l'ESP32
- **Watchdog** : redémarrage automatique en cas de blocage (délai 60 secondes)
- Synchronisation de l'heure via **NTP** pour une gestion précise des plages horaires
- Accès à distance via **tunnel ngrok**

---

## 🔧 Matériel requis

| Composant | Rôle | Broche |
|---|---|---|
| **ESP32-CAM AI Thinker** | Cerveau du système — capture, IA, WiFi | — |
| **OV2640** (intégrée) | Caméra 2 mégapixels | — |
| **ESP32-CAM-MB** | Programmateur USB (port Micro-USB + boutons BOOT/RESET) | — |
| **Servomoteur** | Rotation panoramique de la caméra (0°–100°) | GPIO 13 |
| **Module double relais** | Contrôle des équipements électriques + va-et-vient | GPIO 14 |
| **Capteur LDR** | Vérification de l'état réel de la lumière | GPIO 15 |
| **Résistance 5 kΩ** | Pont diviseur pour lecture numérique de la LDR (ADC désactivé par la caméra) | — |
| **Carte microSD** | Enregistrement des photos horodatées | Slot intégré |
| **2 chargeurs USB 5V/2A** | Alimentation séparée (ESP32+LDR / servomoteur+relais) | — |

> **Note alimentation :** Deux alimentations séparées aux masses reliées sont indispensables pour éviter que les pics de consommation lors des commutations du relais ne perturbent l'ESP32-CAM.

---

## 🏗️ Architecture du système

Le système s'articule autour de trois grandes parties :

```
┌─────────────────────────────────────────────────────────┐
│                     ACQUISITION                         │
│  Caméra OV2640 sur servomoteur  +  Capteur LDR          │
└─────────────────────────┬───────────────────────────────┘
                          │
┌─────────────────────────▼───────────────────────────────┐
│              TRAITEMENT ET DÉCISION (ESP32-CAM)          │
│  Modèle IA FOMO  →  Seuil 90%  →  Double détection      │
│  FreeRTOS : Cœur 1 (capture) / Cœur 0 (IA + WiFi)       │
└──────────┬──────────────────────────────┬───────────────┘
           │                              │
┌──────────▼──────────┐      ┌────────────▼───────────────┐
│  ACTION PHYSIQUE    │      │  COMMUNICATION              │
│  Relais → équipem.  │      │  Interface web (MJPEG)      │
│  Servomoteur 0-100° │      │  Alertes Telegram           │
│  Enregistrement SD  │      │  Tunnel ngrok               │
└─────────────────────┘      └────────────────────────────┘
```

### Répartition FreeRTOS sur les deux cœurs

| Cœur | Tâche | Rôle |
|---|---|---|
| **Cœur 1** | Capture rapide | Capture et préparation des images en continu |
| **Cœur 0** | Inférence IA | Analyse IA, contrôle du relais et du servomoteur |
| **Cœur 0** | WiFi et serveurs | Connexion réseau, interface web, alertes Telegram |

Les accès aux ressources partagées (tampon d'images, état de la lumière, position servomoteur) sont protégés par des **mutex** et des **sémaphores** FreeRTOS.

---

## 🤖 Modèle IA embarquée

Le modèle est entraîné sur **Edge Impulse**, une plateforme gratuite spécialisée pour l'IA sur microcontrôleurs.

### Données d'entraînement
- **740 images** capturées directement dans la salle du FabLab du BBI
- Conditions variées : personnes assises, debout, partiellement visibles, lumière allumée ou éteinte
- Étiquetage manuel avec bounding boxes autour de chaque personne

### Pipeline Edge Impulse
1. **Collecte** : capture depuis navigateur via la bibliothèque EloquentESPCam
2. **Étiquetage** : bounding boxes autour de chaque personne (images vides sans rectangle)
3. **Impulsion** : images 96×96 pixels — modèle FOMO (optimisé pour microcontrôleurs)
4. **Entraînement** : score F1 = **83,1%** après ajout d'images ciblées et révision des étiquettes
5. **Déploiement** : bibliothèque Arduino ZIP → importée directement dans l'IDE Arduino

### Mécanisme de fiabilisation

```
Image capturée toutes les 2 secondes
         │
         ▼
Confiance IA > 90% ?
    NON → Ignoré
    OUI → Compteur ++
         │
         ▼
2 détections positives en moins de 10s ?
    NON → Pas de validation
    OUI → Présence confirmée → Équipements allumés
```

---

## ⚙️ Modes de fonctionnement

| Mode | Description |
|---|---|
| **Automatique** (défaut) | L'IA prend toutes les décisions — allumage sur détection, extinction après 4 min d'inactivité |
| **Manuel physique** | Contrôle via l'interrupteur mural existant (cohabite naturellement avec le système) |
| **Manuel web** | Contrôle à distance depuis l'interface web — retour automatique en mode automatique après 4 min |
| **Mode absence** | Alertes Telegram permanentes quelle que soit l'heure dès qu'une présence est détectée |

---

## 🛠️ Installation et configuration

### Prérequis logiciels
- [Arduino IDE](https://www.arduino.cc/en/software) avec le package **ESP32 by Espressif Systems**
- Bibliothèque **Edge Impulse** exportée depuis votre projet Edge Impulse (format ZIP Arduino)
- Compte [Telegram](https://telegram.org) pour les alertes (API gratuite)
- [ngrok](https://ngrok.com) pour l'accès à distance (optionnel)

### Bibliothèques Arduino nécessaires
```
- ESP32 by Espressif Systems (via gestionnaire de cartes)
- EloquentESPCam
- FreeRTOS (inclus dans le package ESP32)
- Bibliothèque Edge Impulse exportée (EconomieEnergieBBITEST.zip)
```

### Étapes d'installation

**1. Cloner le dépôt**
```bash
git clone https://github.com/Zadus-Next-Helium-Service/Economie_Energie_Camera_ESP32CAM.git
cd Economie_Energie_Camera_ESP32CAM
```

**2. Configurer les paramètres réseau et Telegram**

Dans le fichier de configuration, renseigner :
```cpp
// Réseau WiFi
const char* ssid     = "VOTRE_SSID";
const char* password = "VOTRE_MOT_DE_PASSE";

// Adresse IP fixe (à configurer aussi côté routeur)
IPAddress local_IP(192, 168, X, X);

// Telegram
const String BOT_TOKEN = "VOTRE_TOKEN_BOT";
const String CHAT_ID   = "VOTRE_CHAT_ID";
```

**3. Importer la bibliothèque Edge Impulse**

Dans Arduino IDE : `Croquis → Inclure une bibliothèque → Ajouter une bibliothèque .ZIP`  
→ Sélectionner `EconomieEnergieBBITEST.zip`

**4. Sélectionner la carte et téléverser**

```
Outils → Type de carte → AI Thinker ESP32-CAM
Outils → Port → (port COM de votre ESP32-CAM-MB)
```
Appuyer sur **BOOT** pendant le téléversement si nécessaire, puis **RESET** pour démarrer.

### Câblage

```
ESP32-CAM GPIO 13  →  Signal servomoteur
ESP32-CAM GPIO 14  →  Signal module double relais
ESP32-CAM GPIO 15  →  Point milieu pont diviseur LDR (résistance 5kΩ en série)
GND commun entre les deux alimentations
```

> ⚠️ **Important :** Utiliser deux alimentations 5V/2A séparées avec masses reliées. Ne pas alimenter le servomoteur et le relais depuis la même source que l'ESP32-CAM.

---

## 🌐 Interface web

Une fois le système démarré et connecté au WiFi, l'interface est accessible depuis tout navigateur :

```
http://192.168.X.X       → Interface principale + flux vidéo
http://192.168.X.X:81    → Flux MJPEG seul
http://192.168.X.X/health → Diagnostics (uptime, mémoire heap, NTP, détections)
```

**Fonctionnalités de l'interface :**
- Visualisation du flux vidéo en temps réel
- Contrôle du relais (allumage/extinction)
- Déplacement du servomoteur (pas de 10°)
- Basculement entre mode automatique et mode manuel
- Activation/désactivation du mode absence
- Gestion de la liste des jours fériés

Pour un accès depuis l'extérieur du réseau local, utiliser ngrok :
```bash
ngrok http 80
```

---

## 📬 Alertes Telegram

Le système envoie automatiquement une alerte Telegram dans les cas suivants :

- **La nuit** (22h–6h) : dès qu'une présence est détectée
- **Les weekends** : dès qu'une présence est détectée
- **Les jours fériés** : liste configurable depuis l'interface web
- **Mode absence** : toute détection, 24h/24

Chaque alerte contient :
- Une **photo** de la détection
- Le **lien direct** vers l'interface web

L'heure est synchronisée automatiquement via **NTP** au démarrage.

---

## 📊 Résultats obtenus

| Fonctionnalité | Statut |
|---|---|
| Flux MJPEG en temps réel | ✅ Réalisé |
| Détection IA embarquée (FOMO, score F1 83,1%) | ✅ Réalisé |
| Double validation (2 détections / 10s) | ✅ Réalisé |
| Contrôle automatique via relais | ✅ Réalisé |
| Extinction automatique après 4 min | ✅ Réalisé |
| Balayage servomoteur 0°–100° | ✅ Réalisé |
| Interface web embarquée | ✅ Réalisé |
| Mode manuel physique (va-et-vient) | ✅ Réalisé |
| Mode manuel web | ✅ Réalisé |
| Mode nuit automatique (NTP 22h–6h) | ✅ Réalisé |
| Mode absence | ✅ Réalisé |
| Alertes Telegram (photo + lien) | ✅ Réalisé |
| Enregistrement SD horodaté | ✅ Réalisé |
| Gestion jours fériés (interface web) | ✅ Réalisé |
| Accès à distance (tunnel ngrok) | ✅ Réalisé |
| Watchdog (redémarrage automatique 60s) | ✅ Réalisé |
| Boîtier 3D (FreeCAD + impression FabLab) | ✅ Réalisé |

---

## 🔭 Perspectives d'amélioration

- Amélioration du modèle IA par un jeu de données plus varié (score F1 cible > 90%)
- Intégration d'une **batterie de secours** pour les coupures fréquentes au Burkina Faso
- Déploiement de ngrok sur **Raspberry Pi** pour un accès à distance totalement autonome
- Développement d'une **application mobile** dédiée
- Conception d'un **PCB dédié** pour un système plus compact et commercialisable
- Extension du contrôle à d'autres équipements via capteur de courant **ACS712**
- Déploiement dans d'autres structures burkinabè (écoles, bureaux, commerces)

---

## 🎥 Démonstrations

Les vidéos de démonstration du système en conditions réelles sont disponibles ici :  
👉 [Voir les vidéos de démonstration](https://github.com/Zadus-Next-Helium-Service/Economie_Energie_Camera_ESP32CAM/tree/main/videos)

---

## 👤 Auteur

**ZANRE Abasse DIMITRI**  
Licence Professionnelle en Génie Électrique — Option Électronique et Informatique Industrielle  
Université de Technologies et de Management (UTM) — Ouagadougou, Burkina Faso  
Stage réalisé au **FabLab du Burkina Business Incubator (BBI)**  
Professeur de Suivi : **M.COULIBALY Souleymane**,Enseignant à UTM

Maître de stage : **OUEDRAOGO Mohamed Bassirou**, FabLab Manager

---

*Réalisé entièrement depuis le Burkina Faso, avec des composants disponibles localement, sans dépendance à des services cloud étrangers.*
