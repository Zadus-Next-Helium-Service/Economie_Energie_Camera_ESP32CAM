# Rapport de présentation du projet
# ESP32-CAM — Système intelligent de surveillance, de contrôle à distance et de gestion de l'éclairage

---

## 1. Présentation générale

Ce projet est un système embarqué intelligent multi-fonctions conçu autour d'une caméra connectée (ESP32-CAM AI Thinker). Il combine trois grandes capacités en un seul dispositif compact et peu coûteux :

1. **La surveillance vidéo en temps réel** — visualisation du flux caméra depuis n'importe quel navigateur
2. **Le contrôle à distance d'équipements** — allumer, éteindre ou piloter des appareils électriques via WiFi
3. **La gestion intelligente de l'éclairage** — automatisation basée sur la détection de présence par intelligence artificielle

Dans le cadre de ce projet, le test concret a été réalisé avec le **contrôle d'une lumière** : le système peut allumer ou éteindre une lampe automatiquement selon la présence détectée, ou manuellement depuis l'interface web, exactement comme le ferait un interrupteur physique — mais accessible depuis un téléphone ou un ordinateur, n'importe où dans le bâtiment, sans se déplacer.

Le système a été entièrement conçu, développé et testé dans le laboratiore Fablab de BURKINA BUISINESS INCUBATOR, avec des composants électroniques standards et accessibles, dans l'objectif de produire une solution réplicable et adaptée au contexte africain.

---

## 2. Contexte et problèmes identifiés

### 2.1 Un manque criant de solutions intelligentes accessibles en Afrique

L'Afrique fait face à plusieurs défis technologiques simultanés dans la gestion des bâtiments et des espaces :

- **L'éclairage non contrôlé** : les lumières restent allumées en permanence dans les bureaux, couloirs, salles de classe et espaces communs, même en l'absence totale d'occupants
- **L'absence de surveillance abordable** : de nombreux espaces (entrepôts, bureaux, locaux techniques) ne sont pas surveillés faute de systèmes accessibles financièrement
- **Le contrôle manuel limité** : pour allumer ou éteindre un équipement, il faut physiquement se déplacer jusqu'à l'interrupteur ou au tableau électrique
- **Le coût prohibitif des solutions professionnelles** : les systèmes domotiques et de vidéosurveillance du marché mondial sont conçus pour des marchés à hauts revenus et restent inaccessibles les deux ne sont pas fait ensemble donc les entreprises sont obligé de les acheter ensemble

### 2.2 La réalité énergétique

L'Afrique subsaharienne fait face à une crise énergétique structurelle. Les coupures de courant sont fréquentes, les coûts élevés, et l'infrastructure fragile. Dans ce contexte, chaque kilowattheure compte. Une ampoule de 60W laissée allumée inutilement 8h/jour représente environ 175 kWh gaspillés par an — par pièce, par an — soit plusieurs dizaines de milliers de francs CFA en factures inutiles.

### 2.3 La surveillance : un besoin réel sans solution locale abordable

Les caméras de surveillance professionnelles (CCTV, caméras IP) nécessitent une infrastructure réseau dédiée, un enregistreur numérique, et souvent un abonnement à un service cloud étranger. Pour une petite entreprise, une école ou une association en Afrique, ces coûts sont souvent insupportables. Il n'existait pas de solution simple, locale et peu coûteuse permettant à la fois de surveiller un espace et de contrôler ses équipements depuis un téléphone.

---

## 3. La solution développée

### 3.1 Un seul dispositif, trois fonctions majeures

Ce système répond simultanément aux trois problèmes identifiés avec un seul dispositif installé dans un espace :

```
┌─────────────────────────────────────────────────────────────┐
│                                                             │
│   SURVEILLANCE VIDÉO            CONTRÔLE À DISTANCE        │
│   ─────────────────             ─────────────────          │
│   Flux vidéo live en            Allumer / éteindre         │
│   temps réel depuis             équipements depuis         │
│   tout navigateur WiFi          téléphone ou PC            │
│                                                             │
│               ┌──────────────────────┐                     │
│               │   ESP32-CAM          │                     │
│               │   + Servo + Relais   │                     │
│               └──────────────────────┘                     │
│                                                             │
│   GESTION INTELLIGENTE          DÉTECTION IA               │
│   ─────────────────             ──────────                 │
│   Éclairage automatique         Analyse d'image            │
│   selon présence humaine        embarquée, sans cloud      │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### 3.2 La surveillance vidéo en temps réel

Le système diffuse un flux vidéo continu (format MJPEG) accessible depuis n'importe quel navigateur web connecté au réseau WiFi local. Aucune application à installer, aucun compte à créer, aucun abonnement nécessaire. Il suffit d'ouvrir un navigateur et de saisir l'adresse IP du dispositif.

La caméra est montée sur un servo moteur qui effectue un balayage panoramique automatique de 0° à 100°, ce qui élargit considérablement le champ de surveillance par rapport à une caméra fixe. L'opérateur peut également orienter manuellement la caméra depuis l'interface web pour inspecter un angle précis à la demande.

Cette fonctionnalité répond directement au besoin de surveillance d'espaces (bureaux, salles, entrepôts, entrées de bâtiments) sans investir dans une infrastructure de vidéosurveillance coûteuse.

### 3.3 Le contrôle à distance d'équipements — le test avec la lumière

Le cœur de la démonstration pratique de ce projet est le **contrôle à distance d'une lumière via un relais électronique**. Le relais fonctionne comme un interrupteur commandé électroniquement : il peut couper ou rétablir le courant dans un circuit électrique sur ordre du microcontrôleur.

Depuis l'interface web, l'utilisateur dispose de trois modes de contrôle :

- **Mode manuel — Allumer** : équivaut à appuyer sur l'interrupteur physique pour allumer la lumière, depuis n'importe où via WiFi
- **Mode manuel — Éteindre** : équivaut à appuyer sur l'interrupteur physique pour éteindre la lumière, à distance
- **Mode automatique (IA)** : confier entièrement le contrôle au système intelligent qui gère la lumière selon la présence détectée

Un point important : **la compatibilité avec l'interrupteur physique est maintenue**. Il est toujours possible d'allumer ou d'éteindre la lumière avec l'interrupteur normal installé dans la pièce. Le système détecte automatiquement ce changement d'état grâce au capteur LDR intégré et met à jour l'interface en conséquence. L'utilisateur n'a donc pas à choisir entre le contrôle physique et le contrôle à distance — les deux fonctionnent ensemble, de façon complémentaire.

Ce principe est fondamental et extensible : bien que le test ait été réalisé avec une lumière, **le même relais peut contrôler n'importe quel équipement électrique** — un ventilateur, une pompe à eau, une sirène d'alarme, un système d'arrosage, un portail électrique, ou tout autre appareil fonctionnant sur secteur. Le projet pose ainsi les bases d'un système de contrôle à distance universel pour équipements électriques.

### 3.4 La gestion intelligente par IA embarquée

Un modèle d'intelligence artificielle entraîné sur la plateforme Edge Impulse est déployé directement sur le microcontrôleur, sans connexion à un serveur distant ni cloud. Ce modèle analyse chaque image capturée et détermine si une personne est présente ou non dans le champ de vision, avec un niveau de confiance calculé.

- Si une personne est détectée avec plus de 80% de confiance → la lumière s'allume automatiquement
- Si aucune présence n'est détectée pendant 3 minutes consécutives → la lumière s'éteint seule
- Entre 22h et 6h → le système entre en mode nuit, réduit son activité et économise l'énergie

Le traitement étant entièrement local, **aucune image n'est envoyée à l'extérieur**, ce qui garantit la confidentialité des occupants et le fonctionnement même sans accès internet.

---

## 4. Les apports concrets du projet

### 4.1 Apport pour la surveillance et la sécurité

Pour toute structure qui ne peut pas se permettre un système de vidéosurveillance professionnel, ce dispositif offre une alternative concrète et immédiatement opérationnelle. Il permet de :

- Surveiller visuellement un espace en temps réel depuis un téléphone ou un ordinateur
- Détecter automatiquement une présence inattendue dans un espace
- Orienter la caméra à la demande pour inspecter n'importe quel angle de la pièce
- Combiner surveillance vidéo et contrôle d'équipements dans un seul appareil
- Fonctionner sans infrastructure réseau dédiée, sans enregistreur, sans abonnement

C'est une réponse directe au besoin de sécurité basique qui existe dans des milliers de structures africaines (bureaux, écoles, boutiques, entrepôts) qui n'ont aujourd'hui aucun moyen de surveillance.

### 4.2 Apport pour le contrôle à distance

La capacité à contrôler un équipement électrique à distance sans se déplacer représente un gain de confort, de temps et d'efficacité opérationnelle. Dans un contexte où un responsable peut gérer plusieurs espaces simultanément, pouvoir allumer ou éteindre un équipement depuis son téléphone — sans se lever, sans se déplacer dans le couloir ou dans une autre salle — est un apport concret et mesurable.

Cette capacité prend encore plus de valeur dans des contextes spécifiques :
- **Sécurité** : pouvoir éteindre à distance un équipement oublié allumé en quittant les locaux
- **Urgence** : couper rapidement l'alimentation d'un appareil en cas de problème détecté à distance
- **Gestion multi-sites** : un responsable peut contrôler des équipements dans plusieurs bâtiments depuis un seul endroit

### 4.3 Apport pour l'efficacité énergétique

L'automatisation de l'extinction de la lumière dès qu'un espace est vide élimine quasi totalement le gaspillage lié aux oublis, sans changer les habitudes des occupants ni nécessiter leur intervention. Pour un espace de bureau standard :

| Scénario | Consommation gaspillée/an | Coût estimé (FCFA) |
|----------|--------------------------|-------------------|
| Sans système (oublis fréquents) | ~175 kWh | ~35 000 FCFA perdus |
| Avec ce système | ~0 kWh | Économie totale |
| Coût du dispositif | — | ~15 000 FCFA |
| Retour sur investissement | — | Moins de 6 mois |

### 4.4 Apport technologique

Ce projet démontre qu'il est possible de faire fonctionner de l'intelligence artificielle embarquée sur un microcontrôleur qui coûte moins de 5 euros. C'est une rupture importante par rapport à l'idée reçue que l'IA nécessite des serveurs puissants et des infrastructures coûteuses. Ici, l'IA tourne localement, en temps réel, sur un composant de la taille d'une carte de visite, analysant des images à raison de 2 fois par seconde.

Le système utilise FreeRTOS, un système d'exploitation temps réel, pour faire tourner trois tâches simultanément sur les deux cœurs du processeur : la capture vidéo, l'analyse IA, et la gestion du réseau WiFi. Cette architecture garantit la fluidité du streaming vidéo même pendant que l'IA analyse les images en arrière-plan.

### 4.5 Apport pour l'autonomie et la souveraineté numérique

En Afrique, la dépendance aux services cloud étrangers est un frein majeur : coûts en devises, latence réseau, dépendance à une connexion internet stable, risques de confidentialité des données. Ce projet propose une approche radicalement différente : **tout fonctionne localement**, sur le réseau WiFi interne du bâtiment, sans aucune dépendance externe. Les données, les images, les décisions — tout reste sur place.

### 4.6 Apport pédagogique et de transfert de compétences

Ce projet est entièrement documenté, open source, et conçu pour être reproductible. Il constitue une ressource pédagogique complète couvrant simultanément la programmation embarquée, l'intelligence artificielle, les systèmes temps réel, le développement web, et l'électronique pratique. N'importe quel technicien ou développeur peut s'en inspirer, le reproduire, ou l'adapter à un nouveau besoin.

---

## 5. Applications et cas d'usage

### 5.1 Bâtiments publics et administratifs
Bureaux, mairies, préfectures, couloirs — surveillance visuelle, extinction automatique de l'éclairage, contrôle d'équipements à distance.

### 5.2 Établissements scolaires et universitaires
Salles de classe, amphithéâtres, bibliothèques — gestion de l'éclairage en dehors des heures de cours, surveillance des espaces sensibles, réduction des factures d'électricité.

### 5.3 Établissements de santé
Hôpitaux, centres de santé — économie d'énergie dans les couloirs et espaces administratifs, surveillance des zones d'accès restreint, contrôle d'équipements à distance.

### 5.4 Petites et moyennes entreprises
Entrepôts, ateliers, boutiques — contrôle à distance des équipements, surveillance vidéo sans infrastructure coûteuse, gestion automatique de l'éclairage.

### 5.5 Logements et résidences
Contrôle de l'éclairage depuis son téléphone, surveillance de l'entrée ou du séjour, gestion à distance lors d'une absence prolongée.

### 5.6 Agriculture et espaces extérieurs
En remplaçant la lumière par une pompe d'irrigation ou un système d'arrosage, le même dispositif peut contrôler des équipements agricoles à distance — ouvrant des perspectives importantes pour l'agriculture connectée en Afrique.

---

## 6. Limites actuelles et perspectives d'évolution

Tout projet honnête doit mentionner ses limites actuelles :

- La détection IA fonctionne mieux en conditions d'éclairage normales ; les performances peuvent baisser dans l'obscurité totale ou en contre-jour fort
- Le flux vidéo et le contrôle sont accessibles uniquement sur le réseau WiFi local ; un accès depuis internet nécessiterait une configuration réseau supplémentaire
- Le modèle IA actuel détecte la présence de façon binaire (présent / absent) ; il ne compte pas encore le nombre de personnes
- La qualité vidéo (2MP) est suffisante pour la détection de présence, mais limitée pour une identification précise à grande distance

Ces limites sont des axes d'amélioration identifiés pour les versions futures :

- **Accès internet sécurisé** via VPN pour le contrôle hors du réseau local
- **Comptage de personnes** et adaptation de l'intensité lumineuse
- **Alertes push** sur téléphone lors de détections inattendues
- **Enregistrement vidéo** sur carte SD lors de détections
- **Contrôle multi-équipements** avec plusieurs relais indépendants
- **Intégration solaire** pour une autonomie totale hors réseau électrique

---

## 7. Conclusion

Ce projet représente bien plus qu'un système d'éclairage automatique. C'est une démonstration concrète qu'un seul dispositif compact, fabriqué avec des composants accessibles et développé localement, peut simultanément assurer la **surveillance vidéo** d'un espace, permettre le **contrôle à distance d'équipements électriques** comme s'il s'agissait d'un interrupteur connecté, et automatiser la **gestion intelligente de l'éclairage** grâce à l'intelligence artificielle embarquée.

Il répond à des besoins réels et quotidiens : voir ce qui se passe dans un espace sans y être physiquement, contrôler un équipement sans se déplacer, ne plus payer pour de l'électricité gaspillée. Ces trois besoins existent dans des milliers de structures à travers toute l'Afrique, et ce projet prouve qu'il est possible d'y répondre  de matériel et un développement entièrement local.

L'innovation technologique pertinente pour l'Afrique peut venir d'Afrique, avec des ressources locales, pour résoudre des problèmes locaux — et ce projet en est la preuve concrète.

---

*Projet développé au sein d'un centre d'incubation BURKINA BUISINNESS NCUBATOR, Afrique de l'Ouest.*
*Auteur : ZANRE Abasse Dimitri — +226 67250288*
*Année : 23/04/2026*
