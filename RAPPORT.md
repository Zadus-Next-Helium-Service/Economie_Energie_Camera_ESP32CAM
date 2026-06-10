# 📄 Rapport Technique — Système Domotique Intelligent ESP32-CAM

> Documentation technique complète du projet.  
> Pour la présentation générale, les fonctionnalités et le matériel requis, voir le [README](./README.md).

---

## 📋 Table des matières

1. [Contexte et genèse du projet](#1-contexte-et-genèse-du-projet)
2. [Évolution technique : les tentatives abandonnées](#2-évolution-technique--les-tentatives-abandonnées)
3. [Architecture logicielle détaillée](#3-architecture-logicielle-détaillée)
4. [Entraînement du modèle IA sur Edge Impulse](#4-entraînement-du-modèle-ia-sur-edge-impulse)
5. [Solutions aux problèmes techniques majeurs](#5-solutions-aux-problèmes-techniques-majeurs)
6. [Guide d'installation complet](#6-guide-dinstallation-complet)
7. [Guide d'utilisation](#7-guide-dutilisation)
8. [Structure du dépôt](#8-structure-du-dépôt)
9. [Références](#9-références)

---

## 1. Contexte et genèse du projet

Ce système a été conçu et réalisé au **FabLab du Burkina Business Incubator (BBI)** à Ouagadougou, dans le cadre d'un stage de fin de cycle de Licence en Génie Électrique (option Électronique et Informatique Industrielle) à l'**Université de Technologies et de Management (UTM)**.

Le point de départ était concret : la salle de réunion du FabLab restait régulièrement avec lumières et équipements allumés alors qu'elle était vide. Personne ne le faisait exprès — c'est simplement ce qui arrive quand plusieurs personnes partagent un espace commun.

**Contraintes spécifiques au contexte burkinabè :**

- Connexion internet instable → le système doit fonctionner **sans cloud**
- Budget limité → composants disponibles localement à Ouagadougou (~13 000 FCFA pour l'ESP32-CAM)
- Coupures de courant fréquentes → système devant redémarrer proprement et automatiquement
- Besoin de reproductibilité → tout le code et la doc sont ouverts et documentés

---

## 2. Évolution technique : les tentatives abandonnées

Comprendre pourquoi les premières approches ont été abandonnées est essentiel pour comprendre les choix finaux.

### 2.1 Tentative 1 — Capteur PIR

**Principe :** détection des variations de chaleur corporelle.

**Problèmes rencontrés :**
- Calibration fastidieuse (trop ou pas assez sensible selon la température ambiante)
- Portée limitée : une personne assise dans un coin de la salle n'était pas détectée
- Impossible de compter le nombre de personnes présentes

**Conclusion :** abandonné. Le PIR est utile pour les couloirs et petits espaces, pas pour une grande salle.

---

### 2.2 Tentative 2 — Capteurs à ultrasons HC-SR04

**Principe :** deux capteurs à l'entrée pour compter les entrées et sorties, déduire le sens du mouvement selon lequel se déclenche en premier.

**Problèmes rencontrés :**
- Une personne s'arrêtant devant la porte → fausse détection
- Deux personnes entrant simultanément → compteur perd le fil
- Coupure de courant → compteur réinitialisé à zéro, toutes les données perdues
- Plusieurs jours de corrections dans le code insuffisants → l'approche elle-même était fragile

**Conclusion :** abandonné. La logique de comptage entrées/sorties est trop fragile dans un environnement réel.

---

### 2.3 Tentative 3 — OV7670 + Python + YOLO sur ordinateur

**Principe :** caméra OV7670 couplée au modèle YOLO tournant sur un ordinateur connecté en permanence.

**Problèmes rencontrés :**
- Qualité d'image de l'OV7670 insuffisante pour une détection fiable
- Nécessité d'un ordinateur allumé en permanence → consommation énergétique absurde pour un système censé résoudre un problème d'économie d'énergie

**Conclusion :** abandonné. La dépendance à un ordinateur externe contredit l'objectif d'autonomie.

---

### 2.4 Solution finale — ESP32-CAM + Edge Impulse ✅

La nette supériorité de la qualité d'image de l'ESP32-CAM par rapport à l'OV7670 a été le déclic. Un module qui détecte déjà des visages avec son code de base peut faire tourner un modèle de détection de présence personnalisé.

Edge Impulse, découvert lors de recherches complémentaires, a apporté la pièce manquante : entraîner un modèle IA et le déployer directement sur microcontrôleur, sans écrire une seule ligne de code d'apprentissage automatique.

---

## 3. Architecture logicielle détaillée

### 3.1 Vue d'ensemble du code

Le code principal tourne sur l'ESP32-CAM sous **Arduino IDE** en C/C++, structuré autour de **FreeRTOS** pour la gestion multitâche sur les deux cœurs du processeur.

```
main.ino
├── setup()                    ← Initialisation unique au démarrage
│   ├── Initialisation caméra
│   ├── Connexion WiFi + IP fixe
│   ├── Synchronisation NTP
│   ├── Montage carte SD
│   ├── Configuration GPIO (relais, servo, LDR)
│   └── Création des tâches FreeRTOS
│
└── Tâches FreeRTOS (loop() non utilisé)
    ├── taskCapture()          ← Cœur 1 : capture image → tampon partagé
    ├── taskInference()        ← Cœur 0 : analyse IA + contrôle équipements
    └── taskWifi()             ← Cœur 0 : serveur web + alertes Telegram
```

### 3.2 Gestion des ressources partagées

Plusieurs tâches accèdent aux mêmes données. Les conflits sont évités par deux mécanismes FreeRTOS :

| Ressource partagée | Mécanisme de protection |
|---|---|
| Tampon d'images (capture → inférence) | Sémaphore binaire |
| État du relais (inférence ↔ web) | Mutex |
| Position du servomoteur (inférence ↔ web) | Mutex |
| Compteur de détections (inférence ↔ web) | Mutex |
| État LDR (inférence → web) | Variable volatile |

### 3.3 Logique de détection complète

```
┌─────────────────────────────────────────────────────────────┐
│  taskInference() — toutes les 2 secondes                    │
│                                                             │
│  1. Récupère image depuis tampon (sémaphore)                │
│  2. Lance inférence FOMO sur image 96×96px                  │
│  3. Confiance > 90% ?                                       │
│       NON  → reset compteur si > 10s sans détection         │
│       OUI  → compteur_détections++                          │
│  4. compteur >= 2 ET temps_écoulé < 10s ?                   │
│       NON  → attendre                                       │
│       OUI  → PRÉSENCE CONFIRMÉE                             │
│              → Allumer relais (GPIO 14)                     │
│              → Reset chrono extinction (4 min)              │
│              → Si plage alerte → envoyer Telegram + SD      │
│  5. Chrono extinction écoulé (4 min sans détection) ?       │
│       OUI  → Éteindre relais                                │
│  6. Vérifier LDR (GPIO 15) → état réel == commande ?        │
│       NON  → corriger commande                              │
└─────────────────────────────────────────────────────────────┘
```

### 3.4 Logique des plages d'alerte

```
Est-ce une heure d'alerte ?
│
├─ Mode absence activé ?       → OUI → Alerte
│
├─ Heure entre 22h et 6h ?     → OUI → Alerte
│
├─ Jour = samedi ou dimanche ? → OUI → Alerte
│
└─ Date = jour férié (liste interface web) ? → OUI → Alerte

Sinon → Pas d'alerte (journée normale de semaine)
```

### 3.5 Lecture numérique de la LDR (sans ADC)

L'activation de la caméra désactive l'ADC de l'ESP32-CAM sur toutes les broches, rendant impossible une lecture analogique classique. Solution adoptée :

```
        VCC (3.3V)
            │
         [LDR]          ← résistance variable selon lumière
            │
         GPIO 15  ←── lecture numérique HIGH/LOW
            │
        [5 kΩ fixe]     ← fixe le seuil de détection
            │
          GND

Lumière allumée → LDR faible résistance → tension au point milieu → HIGH
Lumière éteinte → LDR haute résistance → tension chute → LOW
```

La résistance fixe de 5 kΩ joue le même rôle qu'un potentiomètre de sensibilité, en filtrant la lumière ambiante et en évitant les faux positifs.

---

## 4. Entraînement du modèle IA sur Edge Impulse

### 4.1 Créer un compte et un projet Edge Impulse

1. Aller sur [studio.edgeimpulse.com](https://studio.edgeimpulse.com)
2. Créer un compte gratuit
3. Créer un nouveau projet → lui donner un nom (ex: `EconomieEnergieBBI`)

### 4.2 Collecter les images

Flasher la bibliothèque **EloquentESPCam** sur l'ESP32-CAM pour accéder à l'outil de capture :

```
Croquis → Exemples → EloquentESPCam → collect_images_for_EdgeImpulse
```

Accéder à l'interface de capture depuis le navigateur → déclencher des rafales → télécharger le ZIP.

**Recommandations pour la collecte :**
- Capturer dans l'espace réel où le système sera déployé
- Varier les positions : personnes debout, assises, partiellement visibles, dos tourné
- Varier les conditions : lumière allumée, lumière éteinte, mi-journée, soir
- Viser au minimum 300 images avec personnes et 300 images sans personne
- Ratio conseillé : 80% entraînement / 20% test

### 4.3 Étiqueter les images

Sur Edge Impulse → **Data acquisition** → **Labeling queue** :

- Dessiner une **bounding box** autour de chaque personne visible
- Images sans personne → laisser sans bounding box
- Ne pas étiqueter les parties de corps partielles comme des personnes complètes

> ⚠️ La qualité du modèle dépend directement de la qualité des étiquettes. Un mauvais étiquetage produit un mauvais modèle. Prendre le temps de vérifier chaque image.

### 4.4 Créer l'impulsion (pipeline)

**Create impulse :**
- Image width: `96`, Image height: `96`, Resize mode: `Fit shortest axis`
- Processing block: **Image**
- Learning block: **Object Detection (Images) — FOMO**

**Image block :**
- Color depth: `Grayscale` (réduit la mémoire utilisée)
- Sauvegarder → **Generate features**

### 4.5 Entraîner le modèle

**Object Detection → Training settings :**
- Number of training cycles: `60` (augmenter si le score F1 est insuffisant)
- Learning rate: `0.001`
- Data augmentation: activée

Lancer l'entraînement → observer le score F1.

**Interprétation du score F1 :**

| Score F1 | Interprétation |
|---|---|
| < 70% | Insuffisant — revoir les étiquettes ou ajouter des images |
| 70–80% | Acceptable — peut être amélioré |
| 80–85% | Bon pour la détection de personnes en conditions réelles |
| > 85% | Excellent |

> Le score F1 obtenu sur ce projet est **83,1%** après correction des étiquettes et ajout d'images ciblées aux angles problématiques.

### 4.6 Déployer sur l'ESP32-CAM

**Deployment → Arduino library → Build** → télécharger le fichier ZIP.

Dans Arduino IDE :
```
Croquis → Inclure une bibliothèque → Ajouter une bibliothèque .ZIP
→ Sélectionner le fichier ZIP téléchargé
```

---

## 5. Solutions aux problèmes techniques majeurs

### 5.1 Perturbations du relais simple (back-EMF)

**Symptôme :** l'ESP32-CAM redémarrait aléatoirement lors des commutations du relais.

**Cause :** lorsqu'un relais commute, la bobine génère une surtension inverse (back-EMF) qui se propage dans le circuit et perturbe les composants voisins.

**Solution :** remplacer le module relais simple par un **module double relais**, mieux protégé contre ces phénomènes. Ses deux contacts NO et NF ont également permis le montage en va-et-vient avec l'interrupteur mural existant.

---

### 5.2 Instabilité de l'alimentation

**Symptôme :** comportements erratiques lors des pics de consommation (commutation relais + inférence IA + WiFi simultanés).

**Solution :** deux alimentations 5V/2A séparées, masses reliées :
- Alimentation 1 → ESP32-CAM + LDR
- Alimentation 2 → servomoteur + module relais

---

### 5.3 Conflits mémoire entre bibliothèques

**Symptôme :** plantages aléatoires, mémoire heap tombant à des valeurs critiques.

**Diagnostic :** surveiller en temps réel via la page `/health` de l'interface web.

**Solutions appliquées :**
- Réduction de la résolution IA à 96×96 pixels (QVGA pour le flux vidéo, résolution réduite pour l'IA)
- Utilisation du mode Grayscale pour les features Edge Impulse
- Libération explicite des buffers après chaque inférence
- Watchdog à 60 secondes pour redémarrage automatique en cas de blocage

---

### 5.4 Faux positifs du modèle IA

**Symptôme :** le modèle détectait des présences alors que la salle était vide (score F1 initial : 80%).

**Causes identifiées :**
- Manque d'images de salle vide à certains angles de balayage du servomoteur
- Erreurs d'étiquetage fournissant de fausses données au modèle

**Solutions appliquées :**
- Ajout d'images supplémentaires aux angles problématiques (via QR code Edge Impulse depuis smartphone)
- Révision complète des étiquettes
- Mise en place du mécanisme de double détection + seuil de confiance à 90%
- Résultat : score F1 porté à **83,1%**

---

## 6. Guide d'installation complet

### 6.1 Prérequis

**Logiciels :**
- [Arduino IDE 2.x](https://www.arduino.cc/en/software)
- Compte [Edge Impulse](https://studio.edgeimpulse.com) (gratuit)
- Compte [Telegram](https://telegram.org) pour les alertes

**Package Arduino :**

Dans Arduino IDE → `Fichier → Préférences → URL de gestionnaire de cartes supplémentaires` :
```
https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
```
Puis `Outils → Type de carte → Gestionnaire de cartes` → rechercher **esp32** → installer **ESP32 by Espressif Systems**.

---

### 6.2 Créer le bot Telegram

1. Ouvrir Telegram → rechercher **@BotFather**
2. Envoyer `/newbot` → suivre les instructions → noter le **token**
3. Rechercher **@userinfobot** → envoyer n'importe quel message → noter le **Chat ID**

---

### 6.3 Cloner le dépôt et configurer

```bash
git clone https://github.com/Zadus-Next-Helium-Service/Economie_Energie_Camera_ESP32CAM.git
cd Economie_Energie_Camera_ESP32CAM
```

Ouvrir le fichier principal dans Arduino IDE et modifier les paramètres :

```cpp
// ── Réseau WiFi ──────────────────────────────────────────
const char* WIFI_SSID     = "VOTRE_NOM_WIFI";
const char* WIFI_PASSWORD = "VOTRE_MOT_DE_PASSE";

// ── Adresse IP fixe ──────────────────────────────────────
// Configurer aussi une réservation d'adresse côté routeur
IPAddress LOCAL_IP(192, 168, X, X);
IPAddress GATEWAY(192, 168, X, 1);
IPAddress SUBNET(255, 255, 255, 0);

// ── Telegram ─────────────────────────────────────────────
const String BOT_TOKEN = "XXXXXXXXXX:XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX";
const String CHAT_ID   = "XXXXXXXXXX";

// ── NTP (fuseau horaire Ouagadougou = UTC+0) ─────────────
const long  GMT_OFFSET_SEC    = 0;
const int   DAYLIGHT_OFFSET   = 0;
const char* NTP_SERVER        = "pool.ntp.org";

// ── Paramètres de détection ───────────────────────────────
const float CONFIDENCE_THRESHOLD = 0.90;  // Seuil de confiance IA
const int   DETECTIONS_REQUIRED  = 2;     // Détections pour validation
const int   DETECTION_WINDOW_MS  = 10000; // Fenêtre de temps (ms)
const int   AUTO_OFF_DELAY_MS    = 240000;// Délai extinction (4 min)

// ── Plages d'alerte nocturne ──────────────────────────────
const int   NIGHT_HOUR_START = 22;
const int   NIGHT_HOUR_END   = 6;
```

---

### 6.4 Importer la bibliothèque Edge Impulse

```
Arduino IDE → Croquis → Inclure une bibliothèque → Ajouter une bibliothèque .ZIP
→ Sélectionner votre fichier ZIP exporté depuis Edge Impulse
```

---

### 6.5 Câblage

```
ESP32-CAM               Composant
─────────────────────────────────────────────────────
GPIO 13          →      Signal servomoteur (fil jaune/orange)
GPIO 14          →      Signal module double relais (IN1)
GPIO 15          →      Point milieu pont diviseur LDR
GND              →      GND servomoteur + GND relais + GND LDR
─────────────────────────────────────────────────────
Alimentation 1 (5V/2A)  →  VCC ESP32-CAM-MB + VCC LDR
Alimentation 2 (5V/2A)  →  VCC servomoteur + VCC relais
GND des deux alimentations reliés ensemble
─────────────────────────────────────────────────────
Pont diviseur LDR :
  3.3V → [LDR] → GPIO 15 → [5kΩ] → GND
─────────────────────────────────────────────────────
Module double relais (montage va-et-vient) :
  Contact NO  → Phase vers ampoule
  Contact NF  → Phase vers interrupteur mural existant
  COM         → Phase secteur
```

> ⚠️ Travailler uniquement hors tension lors du câblage du relais sur le circuit 220V.

---

### 6.6 Téléverser le code

1. Connecter l'ESP32-CAM-MB via câble Micro-USB
2. Dans Arduino IDE :
   ```
   Outils → Type de carte → ESP32 Arduino → AI Thinker ESP32-CAM
   Outils → Port → (sélectionner le bon port COM)
   Outils → Vitesse de téléversement → 115200
   ```
3. Cliquer sur **Téléverser**
4. Si le téléversement échoue → maintenir le bouton **BOOT** pendant l'upload
5. Une fois terminé → appuyer sur **RESET**

---

### 6.7 Vérifier le démarrage

Ouvrir le **Moniteur série** (115200 baud) — vous devriez voir :

```
Initialisation de la caméra... OK
Connexion WiFi....... OK
Adresse IP : 192.168.X.X
Synchronisation NTP... OK
Carte SD... OK
Démarrage des tâches FreeRTOS...
Système prêt.
```

Si la connexion WiFi échoue → vérifier les identifiants et redémarrer.

---

### 6.8 Accès à distance avec ngrok (optionnel)

Installer ngrok sur un PC connecté au même réseau :

```bash
# Installation (Linux/Mac)
wget https://bin.equinox.io/c/bNyj1mQVY4c/ngrok-v3-stable-linux-amd64.tgz
tar xvzf ngrok-v3-stable-linux-amd64.tgz

# Authentification (compte gratuit sur ngrok.com)
./ngrok authtoken VOTRE_TOKEN_NGROK

# Lancer le tunnel
./ngrok http 192.168.X.X:80
```

L'URL publique générée (ex: `https://xxxx.ngrok.io`) donne accès à l'interface depuis n'importe où.

> Pour une solution autonome sans PC, déployer ngrok sur un **Raspberry Pi** connecté au même réseau.

---

## 7. Guide d'utilisation

### 7.1 Interface web — pages disponibles

| URL | Contenu |
|---|---|
| `http://192.168.X.X` | Interface principale |
| `http://192.168.X.X:81/stream` | Flux MJPEG seul |
| `http://192.168.X.X/health` | Diagnostics système |
| `http://192.168.X.X/status` | État JSON du système |

### 7.2 Interface principale — commandes

| Commande | Effet |
|---|---|
| Bouton ON/OFF | Allumer/éteindre manuellement le relais |
| Slider servo | Déplacer la caméra (0° à 100° par pas de 10°) |
| Toggle Auto/Manuel | Basculer le mode de contrôle |
| Toggle Mode absence | Activer/désactiver les alertes permanentes |
| Champ jours fériés | Ajouter/supprimer des dates d'alerte |

### 7.3 Page /health — indicateurs à surveiller

```json
{
  "uptime_s": 3600,
  "heap_free": 45000,       ← Doit rester > 20 000 bytes
  "heap_min": 38000,        ← Doit rester > 15 000 bytes
  "detections": 12,
  "ntp_sync": true,
  "sd_ok": true,
  "mode": "auto",
  "light_state": "on"
}
```

Si `heap_free` descend régulièrement sous 20 000 bytes → redémarrer le système et surveiller les fuites mémoire.

### 7.4 Consultation des photos SD

Retirer la carte microSD → la lire depuis un ordinateur. Les fichiers sont nommés :
```
YYYYMMDD_HHMMSS.jpg
ex: 20260315_224532.jpg  → 15 mars 2026 à 22h45
```

---

## 8. Structure du dépôt

```
Economie_Energie_Camera_ESP32CAM/
│
├── README.md                    ← Présentation générale du projet
├── RAPPORT_TECHNIQUE.md         ← Ce document
│
├── src/
│   └── main/
│       └── main.ino             ← Code source principal
│
├── model/
│   └── EconomieEnergieBBITEST.zip  ← Bibliothèque Edge Impulse exportée
│
├── hardware/
│   ├── schema_cablage.png       ← Schéma de câblage complet
│   └── boitier_freecad.FCStd   ← Fichier source FreeCAD du boîtier
│
├── boitier_3d/
│   └── boitier.stl              ← Fichier STL pour impression 3D
│
└── videos/
    ├── demo_detection.mp4       ← Démonstration de la détection IA
    ├── demo_interface_web.mp4   ← Démonstration de l'interface web
    └── demo_alerte_telegram.mp4 ← Démonstration des alertes
```

---

## 9. Références

| Référence | Lien |
|---|---|
| Edge Impulse Documentation | https://docs.edgeimpulse.com |
| ESP32 by Espressif Systems | https://docs.espressif.com |
| FreeRTOS Documentation | https://www.freertos.org/Documentation/RTOS_book.html |
| Arduino IDE | https://www.arduino.cc/en/software |
| FreeCAD | https://www.freecad.org |
| ngrok | https://ngrok.com |
| Telegram Bot API | https://core.telegram.org/bots/api |

---

*Réalisé au FabLab du Burkina Business Incubator — Ouagadougou, Burkina Faso — 2026*
