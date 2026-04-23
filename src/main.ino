// ============================================================
// ESP32-CAM AI THINKER — MODE CLIENT WIFI (Centre d'incubation)
// ============================================================
// Ce programme permet à l'ESP32-CAM de :
// - se connecter au WiFi "Centre d'incubation 2.4G"
// - récupérer l'heure par NTP
// - diffuser un flux vidéo (serveur web)
// - analyser les images avec un modèle IA (Edge Impulse)
// - contrôler un relais (lumière), un servo-moteur, une LED
// - gérer automatiquement l'éclairage selon détection de personne
// ============================================================

#include "esp_camera.h"
#include <WiFi.h>
#include "esp_http_server.h"
#include "esp_heap_caps.h"
#include <ESP32Servo.h>
#include <Preferences.h>
#include "esp_task_wdt.h"
#include <EconomieEnergieBBITEST_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <time.h>

// Vérification que le modèle IA est bien de type caméra
#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_CAMERA
#error "Modèle IA invalide — vérifier l'export Edge Impulse"
#endif

// ============================================================
// CONFIGURATION CAMÉRA (broches pour AI THINKER)
// ============================================================
#define CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// ============================================================
// BROCHES PÉRIPHÉRIQUES
// ============================================================
#define BROCHE_RELAIS   14     // commande de la lumière (relais)
#define BROCHE_LDR      15     // capteur de luminosité (tout ou rien)
#define BROCHE_SERVO    13     // servo-moteur
#define BROCHE_LED_CAM   4     // LED d'éclairage de la caméra (PWM)

// ============================================================
// CONFIGURATION WIFI (mode client uniquement)
// ============================================================
const char* nom_wifi     = "Nom_wifi";        // à remplacer par votre SSID
const char* mot_passe_wifi = "Mot_de_passe_wifi"; // et votre mot de passe

// ============================================================
// TAILLE DE L'IMAGE POUR L'IA (320x240 pixels, RGB888)
// ============================================================
#define LARGEUR_IMAGE_IA   320
#define HAUTEUR_IMAGE_IA   240
#define TAILLE_PIXEL_IA      3   // 3 octets par pixel (R,V,B)

// ============================================================
// CONFIGURATION DE LA LUMIÈRE (LDR + temporisation)
// ============================================================
#define SEUIL_LDR           3000   // valeur LDR pour considérer qu'il fait clair (0-4095)
#define DUREE_EXTINCTION_MS 180000 // 3 minutes d'extinction auto après dernière détection
#define DUREE_MANUEL_MS     240000 // 4 minutes sans activité web → retour mode auto
#define ANTI_REBOND_RELAIS_MS 2000 // délai entre deux basculements du relais

// ============================================================
// CONFIGURATION DU SERVO-MOTEUR
// ============================================================
#define PAS_SERVO           10    // incrément lors du balayage auto
#define ANGLE_MAX_SERVO     100   // angle max en mode auto (limité à 100°)
#define PAS_RETOUR_SERVO     7    // pas de retour du servo
#define PAS_SERVO_WEB       10    // pas de déplacement via commandes web
#define TOUS_LES_CYCLES      3    // scan du servo toutes les 3 itérations IA

// ============================================================
// WATCHDOG (surveillance du système)
// ============================================================
#define TIMEOUT_WATCHDOG_SEC  30

// ============================================================
// GESTION DES ERREURS CAMÉRA
// ============================================================
#define MAX_ERREURS_CAMERA       10
#define MAX_REINITIALISATIONS_CAMERA  3

// ============================================================
// SURVEILLANCE MÉMOIRE HEAP
// ============================================================
#define SEUIL_AVERTISSEMENT_HEAP  30000 // alerte si moins de 30 Ko libres

// ============================================================
// INTERVALLE ENTRE DEUX ANALYSES IA
// ============================================================
#define DELAI_IA_MS  2000   // 2 secondes

// ============================================================
/// CONFIGURATION NTP (heure internet)
// ============================================================
#define SERVEUR_NTP            "pool.ntp.org"
#define DECALAGE_HORAIRE_SEC    0      // à ajuster (ex: 3600 pour UTC+1)
#define DECALAGE_ETE_SEC        0

// ============================================================
// CONFIGURATION PWM POUR LED DE LA CAMÉRA
// ============================================================
#define CANAL_PWM_LED     0
#define FREQ_PWM_LED      5000
#define RESOLUTION_PWM_LED 8
#define DUTY_MAX          255
#define DUTY_DIM          128   // luminosité réduite pour clignotements

// ============================================================
// MACROS DE LOG (affichage sur le port série)
// ============================================================
#define LOG_INFO(tag, fmt, ...)    Serial.printf("[INFO][%s] " fmt "\n",    tag, ##__VA_ARGS__)
#define LOG_WARNING(tag, fmt, ...) Serial.printf("[WARNING][%s] " fmt "\n", tag, ##__VA_ARGS__)
#define LOG_ERROR(tag, fmt, ...)   Serial.printf("[ERROR][%s] " fmt "\n",   tag, ##__VA_ARGS__)
#define LOG_FATAL(tag, fmt, ...)   Serial.printf("[FATAL][%s] " fmt "\n",   tag, ##__VA_ARGS__)

// ============================================================
// STRUCTURE CONTENANT LES INDICATEURS DE SANTÉ DU SYSTÈME
// ============================================================
struct SanteSysteme {
    uint32_t redemarrages;        // nombre de redémarrages
    uint32_t erreursCamera;       // échecs de capture
    uint32_t reinitialisationsCamera;
    uint32_t compteurDetections;  // nombre de fois où une personne a été vue
    uint32_t compteurAllumages;   // combien de fois la lumière a été allumée
    uint32_t basculementsRelais;  // nombre de changements d'état du relais
    uint32_t timeoutsMutex;       // échecs de prise de mutex
    uint32_t heapMin;             // plus petite mémoire libre constatée
    unsigned long heureDemarrage; // millis() au démarrage
};
SanteSysteme sante = { 0, 0, 0, 0, 0, 0, 0, UINT32_MAX, 0 };

// ============================================================
// STOCKAGE NON VOLATILE (NVS) pour conserver le mode (manuel/auto)
// ============================================================
Preferences preferences;
volatile bool modeManuel = false;               // si true, l'IA ne commande pas la lumière
volatile unsigned long debutModeManuel = 0;     // timestamp du passage en mode manuel
static bool dernierModeSauvegarde = false;      // pour éviter d'écrire trop souvent

// Sauvegarde l'état actuel dans la mémoire NVS (reboots et mode)
void sauvegarderEtatNVS() {
    if (modeManuel == dernierModeSauvegarde) return;
    dernierModeSauvegarde = modeManuel;
    preferences.begin("esp32cam", false);
    preferences.putBool("modeManuel", modeManuel);
    preferences.putUInt("redemarrages", sante.redemarrages);
    preferences.end();
    LOG_INFO("NVS", "État sauvegardé (modeManuel=%s)", modeManuel ? "MANUEL" : "AUTO");
}

// ============================================================
// GESTION DE L'HEURE (NTP)
// ============================================================
static bool heureDisponible = false;
static time_t epochDemarrage = 0;          // heure absolue au démarrage
static unsigned long millisDemarrage = 0;  // millis() au démarrage

// Retourne l'heure actuelle (0-23) ou -1 si non disponible
int obtenirHeureActuelle() {
    if (heureDisponible) {
        unsigned long ecoule = (millis() - millisDemarrage) / 1000;
        time_t maintenant = epochDemarrage + (time_t)ecoule;
        struct tm* temps = localtime(&maintenant);
        return temps->tm_hour;
    }
    return -1;
}

// Détermine si on est en mode nuit (22h - 6h) ou simulation basée sur uptime
bool estModeNuit() {
    if (heureDisponible) {
        int h = obtenirHeureActuelle();
        return (h >= 22 || h < 6);
    }
    // Fallback : alternance jour/nuit factice de 16h jour / 8h nuit
    unsigned long dureeFonct = millis() - sante.heureDemarrage;
    unsigned long phase = dureeFonct % (16*3600*1000UL + 8*3600*1000UL);
    return (phase >= 16*3600*1000UL);
}

// ============================================================
// BUFFERS ET SÉMAPHORES POUR LE FLUX VIDÉO ET L'IA
// ============================================================
uint8_t* tamponFlux = NULL;      // image JPEG pour le streaming
size_t   longueurFlux = 0;
SemaphoreHandle_t mutexFlux = NULL;

uint8_t* tamponRgbIa = NULL;     // image décodée en RGB pour l'IA
SemaphoreHandle_t semRequeteIa = NULL;   // signal pour demander une nouvelle image IA
SemaphoreHandle_t semPretIa = NULL;      // signal quand l'image est prête

// Autres sémaphores pour protéger les ressources partagées
SemaphoreHandle_t mutexDetection = NULL;
SemaphoreHandle_t mutexLumiere = NULL;
SemaphoreHandle_t mutexServo = NULL;
SemaphoreHandle_t mutexRelais = NULL;
SemaphoreHandle_t mutexClient = NULL;

volatile bool clientConnecte = false;   // un client est-il connecté au flux vidéo ?

char  labelDetection[64] = "En attente...";
float scoreDetection = 0.0f;

bool lumiereAllumee = false;        // état logique (reflète la réalité physique)
unsigned long derniereDetection = 0;
bool chronoActif = false;

Servo monServo;
int angleServo = 0;          // angle courant du servo (0-180)
int angleManuel = -1;        // angle demandé par l'utilisateur (-1 = pas de demande)
static bool servoRetour = false;
static int compteurCyclesServo = 0;
static bool servoModeNuit = false;

int compteurErreursCam = 0;
int compteurReinitCam = 0;

httpd_handle_t serveur80 = NULL, serveur81 = NULL;

// ============================================================
// CONFIGURATION DE LA CAMÉRA (paramètres de capture)
// ============================================================
static camera_config_t configCamera = {
    .pin_pwdn = PWDN_GPIO_NUM, .pin_reset = RESET_GPIO_NUM, .pin_xclk = XCLK_GPIO_NUM,
    .pin_sscb_sda = SIOD_GPIO_NUM, .pin_sscb_scl = SIOC_GPIO_NUM,
    .pin_d7 = Y9_GPIO_NUM, .pin_d6 = Y8_GPIO_NUM, .pin_d5 = Y7_GPIO_NUM, .pin_d4 = Y6_GPIO_NUM,
    .pin_d3 = Y5_GPIO_NUM, .pin_d2 = Y4_GPIO_NUM, .pin_d1 = Y3_GPIO_NUM, .pin_d0 = Y2_GPIO_NUM,
    .pin_vsync = VSYNC_GPIO_NUM, .pin_href = HREF_GPIO_NUM, .pin_pclk = PCLK_GPIO_NUM,
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,
    .pixel_format = PIXFORMAT_JPEG,
    .frame_size = FRAMESIZE_QVGA,   // 320x240
    .jpeg_quality = 12,             // qualité JPEG (0-63, plus bas = meilleure qualité)
    .fb_count = 1,
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

// ============================================================
// FONCTIONS DE SURVEILLANCE DE LA MÉMOIRE
// ============================================================
void verifierTas() {
    uint32_t libre = esp_get_free_heap_size();
    if (libre < sante.heapMin) sante.heapMin = libre;
    if (libre < SEUIL_AVERTISSEMENT_HEAP) LOG_WARNING("MEM", "Tas faible : %u octets !", libre);
}

// ============================================================
// GESTION DU CAPTEUR LDR (binaire : HIGH = très lumineux)
// ============================================================
int lireLDR() { return (digitalRead(BROCHE_LDR) == HIGH) ? 4095 : 0; }

// Indique si la lumière physique est allumée (via relais et LDR)
bool estLumiereAllumeePhysique() { return (lireLDR() > SEUIL_LDR); }

// ============================================================
// BASCULEMENT DU RELAIS (anti-rebond)
// ============================================================
static unsigned long dernierBasculementRelais = 0;
bool basculerRelais() {
    if (xSemaphoreTake(mutexRelais, pdMS_TO_TICKS(1000)) != pdTRUE) {
        sante.timeoutsMutex++;
        return false;
    }
    unsigned long maintenant = millis();
    if (maintenant - dernierBasculementRelais < ANTI_REBOND_RELAIS_MS) {
        xSemaphoreGive(mutexRelais);
        return false;
    }
    int etat = digitalRead(BROCHE_RELAIS);
    digitalWrite(BROCHE_RELAIS, !etat);
    dernierBasculementRelais = maintenant;
    sante.basculementsRelais++;
    xSemaphoreGive(mutexRelais);
    vTaskDelay(pdMS_TO_TICKS(500));
    return true;
}

// ============================================================
// ALLUMER LA LUMIÈRE (via relais)
// ============================================================
void allumerLumiere() {
    if (estLumiereAllumeePhysique()) {
        if (xSemaphoreTake(mutexLumiere, pdMS_TO_TICKS(200)) == pdTRUE) {
            lumiereAllumee = true;
            xSemaphoreGive(mutexLumiere);
        } else sante.timeoutsMutex++;
        LOG_INFO("ALLUMER", "Déjà allumée");
        return;
    }
    if (!basculerRelais()) {
        LOG_ERROR("ALLUMER", "Basculement refusé");
        return;
    }
    if (estLumiereAllumeePhysique()) {
        if (xSemaphoreTake(mutexLumiere, pdMS_TO_TICKS(200)) == pdTRUE) {
            lumiereAllumee = true;
            xSemaphoreGive(mutexLumiere);
        } else sante.timeoutsMutex++;
        sante.compteurAllumages++;
        LOG_INFO("ALLUMER", "Succès");
    } else {
        // Échec : on rebascule pour revenir à l'état précédent
        basculerRelais();
        if (xSemaphoreTake(mutexLumiere, pdMS_TO_TICKS(200)) == pdTRUE) {
            lumiereAllumee = false;
            xSemaphoreGive(mutexLumiere);
        } else sante.timeoutsMutex++;
        LOG_ERROR("ALLUMER", "Échec (LDR n'a pas détecté la lumière)");
    }
}

// ============================================================
// ÉTEINDRE LA LUMIÈRE
// ============================================================
void eteindreLumiere() {
    if (!estLumiereAllumeePhysique()) {
        if (xSemaphoreTake(mutexLumiere, pdMS_TO_TICKS(200)) == pdTRUE) {
            lumiereAllumee = false;
            chronoActif = false;
            xSemaphoreGive(mutexLumiere);
        } else sante.timeoutsMutex++;
        LOG_INFO("ETEINDRE", "Déjà éteinte");
        return;
    }
    if (!basculerRelais()) {
        LOG_ERROR("ETEINDRE", "Basculement refusé");
        return;
    }
    if (!estLumiereAllumeePhysique()) {
        if (xSemaphoreTake(mutexLumiere, pdMS_TO_TICKS(200)) == pdTRUE) {
            lumiereAllumee = false;
            chronoActif = false;
            xSemaphoreGive(mutexLumiere);
        } else sante.timeoutsMutex++;
        LOG_INFO("ETEINDRE", "Succès");
    } else {
        basculerRelais(); // annulation
        if (xSemaphoreTake(mutexLumiere, pdMS_TO_TICKS(200)) == pdTRUE) {
            lumiereAllumee = true;
            xSemaphoreGive(mutexLumiere);
        } else sante.timeoutsMutex++;
        LOG_ERROR("ETEINDRE", "Échec (LDR reste élevé)");
    }
}

// ============================================================
// INITIALISATION DE L'ÉTAT DE LA LUMIÈRE AU DÉMARRAGE
// ============================================================
void initLumiere() {
    int valLdr = lireLDR();
    LOG_INFO("INIT", "LDR = %d", valLdr);
    if (valLdr > SEUIL_LDR) {
        lumiereAllumee = true;
        derniereDetection = millis();
        chronoActif = true;
    } else {
        lumiereAllumee = false;
    }
}

// ============================================================
// GÉRER LA LUMIÈRE EN FONCTION DE LA DÉTECTION IA
// appelée à chaque analyse IA
// ============================================================
void gererLumiere(bool personneDetectee) {
    bool physique = estLumiereAllumeePhysique();
    // Mise à jour de l'état logique
    if (xSemaphoreTake(mutexLumiere, pdMS_TO_TICKS(200)) == pdTRUE) {
        lumiereAllumee = physique;
        xSemaphoreGive(mutexLumiere);
    } else sante.timeoutsMutex++;

    if (personneDetectee) {
        sante.compteurDetections++;
        if (xSemaphoreTake(mutexLumiere, pdMS_TO_TICKS(200)) == pdTRUE) {
            derniereDetection = millis();
            chronoActif = true;
            xSemaphoreGive(mutexLumiere);
        } else sante.timeoutsMutex++;
        if (!physique) allumerLumiere();
        else LOG_INFO("LUMIERE", "Personne détectée — chrono reset");
    } else {
        LOG_INFO("LUMIERE", "Aucune présence");
    }
}

// ============================================================
// FONCTION APPELÉE RÉGULIÈREMENT POUR GÉRER LE TIMER D'EXTINCTION
// et le retour automatique après mode manuel
// ============================================================
void gererChronoLumiere() {
    bool physique = estLumiereAllumeePhysique();
    bool actifChrono = false;
    unsigned long dernier = 0;

    if (xSemaphoreTake(mutexLumiere, pdMS_TO_TICKS(200)) == pdTRUE) {
        actifChrono = chronoActif;
        dernier = derniereDetection;
        xSemaphoreGive(mutexLumiere);
    } else { sante.timeoutsMutex++; return; }

    // Détection d'un allumage externe (par exemple via un interrupteur physique)
    if (physique && !actifChrono) {
        if (xSemaphoreTake(mutexLumiere, pdMS_TO_TICKS(200)) == pdTRUE) {
            derniereDetection = millis();
            chronoActif = true;
            xSemaphoreGive(mutexLumiere);
        }
        LOG_INFO("LUMIERE", "Allumage externe détecté — chrono démarré");
        actifChrono = true;
        dernier = millis();
    }

    // Extinction externe
    if (!physique && actifChrono) {
        if (xSemaphoreTake(mutexLumiere, pdMS_TO_TICKS(200)) == pdTRUE) {
            chronoActif = false;
            xSemaphoreGive(mutexLumiere);
        }
        LOG_INFO("LUMIERE", "Extinction externe — chrono arrêté");
        actifChrono = false;
    }

    // En mode manuel, la lumière ne s'éteint pas automatiquement
    if (modeManuel) {
        // Vérifier le timeout du mode manuel (absence de client web)
        unsigned long ecoule = millis() - debutModeManuel;
        bool clientPresent = false;
        if (xSemaphoreTake(mutexClient, pdMS_TO_TICKS(200)) == pdTRUE) {
            clientPresent = clientConnecte;
            xSemaphoreGive(mutexClient);
        }
        if (!clientPresent && ecoule >= DUREE_MANUEL_MS) {
            modeManuel = false;
            sauvegarderEtatNVS();
            LOG_INFO("MODE", "Timeout web — retour mode automatique IA");
        }
        return; // ne pas éteindre automatiquement
    }

    // Mode auto : éteindre si le délai est dépassé
    if (actifChrono && physique) {
        unsigned long ecoule = millis() - dernier;
        if (ecoule >= DUREE_EXTINCTION_MS) {
            LOG_INFO("TIMER", "Délai %d ms atteint — extinction", DUREE_EXTINCTION_MS);
            eteindreLumiere();
        }
    }

    // Si la lumière est éteinte, on désactive le chrono
    if (actifChrono && !physique) {
        if (xSemaphoreTake(mutexLumiere, pdMS_TO_TICKS(200)) == pdTRUE) {
            chronoActif = false;
            xSemaphoreGive(mutexLumiere);
        }
    }

    verifierTas();
}

// ============================================================
// RÉINITIALISATION DE LA CAMÉRA EN CAS D'ERREUR
// ============================================================
bool reinitialiserCamera() {
    LOG_WARNING("CAM", "Réinitialisation (%d/%d)", compteurReinitCam+1, MAX_REINITIALISATIONS_CAMERA);
    esp_camera_deinit();
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_err_t err = esp_camera_init(&configCamera);
    if (err != ESP_OK) {
        LOG_ERROR("CAM", "Réinit échouée : 0x%x", err);
        sante.reinitialisationsCamera++;
        compteurReinitCam++;
        return false;
    }
    compteurErreursCam = 0;
    compteurReinitCam = 0;
    sante.reinitialisationsCamera++;
    LOG_INFO("CAM", "Réinit réussie");
    return true;
}

// ============================================================
// GESTION AUTOMATIQUE DU SERVO (balayage jour/nuit)
// ============================================================
void gererServoAuto() {
    // Nuit : servo à 0° et arrêt
    if (estModeNuit()) {
        if (!servoModeNuit) {
            servoModeNuit = true;
            if (xSemaphoreTake(mutexServo, pdMS_TO_TICKS(100)) == pdTRUE) {
                angleServo = 0;
                xSemaphoreGive(mutexServo);
            }
            monServo.write(0);
            LOG_INFO("SERVO", "Mode nuit — servo à 0");
        }
        return;
    }

    if (servoModeNuit) {
        servoModeNuit = false;
        servoRetour = false;
        LOG_INFO("SERVO", "Mode jour — reprise auto");
    }

    compteurCyclesServo++;
    if (compteurCyclesServo < TOUS_LES_CYCLES) return;
    compteurCyclesServo = 0;

    int angleActuel = 0;
    if (xSemaphoreTake(mutexServo, pdMS_TO_TICKS(100)) == pdTRUE) {
        angleActuel = angleServo;
        xSemaphoreGive(mutexServo);
    } else { sante.timeoutsMutex++; return; }

    int nouvelAngle = angleActuel;
    if (angleActuel >= ANGLE_MAX_SERVO) servoRetour = true;
    if (servoRetour) {
        nouvelAngle = angleActuel - PAS_RETOUR_SERVO;
        if (nouvelAngle <= 0) { nouvelAngle = 0; servoRetour = false; }
    } else {
        nouvelAngle = angleActuel + PAS_SERVO;
        if (nouvelAngle > ANGLE_MAX_SERVO) nouvelAngle = ANGLE_MAX_SERVO;
    }

    if (xSemaphoreTake(mutexServo, pdMS_TO_TICKS(100)) == pdTRUE) {
        angleServo = nouvelAngle;
        xSemaphoreGive(mutexServo);
    } else { sante.timeoutsMutex++; return; }

    monServo.write(nouvelAngle);
}

// ============================================================
// SYNCHRONISATION HEURE VIA NTP
// ============================================================
bool synchroniserHeureNTP() {
    LOG_INFO("NTP", "Synchronisation heure via NTP...");
    configTime(DECALAGE_HORAIRE_SEC, DECALAGE_ETE_SEC, SERVEUR_NTP);
    struct tm infoTemps;
    unsigned long debut = millis();
    while (millis() - debut < 15000) {
        if (getLocalTime(&infoTemps)) {
            epochDemarrage = mktime(&infoTemps);
            millisDemarrage = millis();
            heureDisponible = true;
            LOG_INFO("NTP", "Heure NTP : %02d:%02d:%02d", infoTemps.tm_hour, infoTemps.tm_min, infoTemps.tm_sec);
            return true;
        }
        delay(500);
    }
    LOG_WARNING("NTP", "Impossible d'obtenir l'heure NTP");
    return false;
}

// ============================================================
// HANDLERS POUR LES REQUÊTES HTTP (serveur web)
// ============================================================

// Allumer la lumière (commande manuelle)
static esp_err_t gestionAllumerLumiere(httpd_req_t *req) {
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    modeManuel = true;
    debutModeManuel = millis();
    allumerLumiere();
    sauvegarderEtatNVS();
    LOG_INFO("WEB", "Commande manuelle : ALLUMER");
    httpd_resp_send(req, "OK", 2);
    return ESP_OK;
}

// Éteindre la lumière (commande manuelle)
static esp_err_t gestionEteindreLumiere(httpd_req_t *req) {
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    modeManuel = true;
    debutModeManuel = millis();
    eteindreLumiere();
    sauvegarderEtatNVS();
    LOG_INFO("WEB", "Commande manuelle : ÉTEINDRE");
    httpd_resp_send(req, "OK", 2);
    return ESP_OK;
}

// Repasser en mode automatique (IA)
static esp_err_t gestionModeAuto(httpd_req_t *req) {
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    modeManuel = false;
    sauvegarderEtatNVS();
    // Relancer le chrono si la lumière est allumée
    if (estLumiereAllumeePhysique()) {
        if (xSemaphoreTake(mutexLumiere, pdMS_TO_TICKS(200)) == pdTRUE) {
            derniereDetection = millis();
            chronoActif = true;
            xSemaphoreGive(mutexLumiere);
        }
    }
    LOG_INFO("WEB", "Retour mode automatique IA");
    httpd_resp_send(req, "OK", 2);
    return ESP_OK;
}

// Déplacer le servo à gauche (commande manuelle)
static esp_err_t gestionServoGauche(httpd_req_t *req) {
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    modeManuel = true;
    debutModeManuel = millis();
    int angleActuel = 0;
    if (xSemaphoreTake(mutexServo, pdMS_TO_TICKS(200)) == pdTRUE) {
        angleActuel = angleServo;
        xSemaphoreGive(mutexServo);
    } else sante.timeoutsMutex++;
    int nouvelAngle = angleActuel - PAS_SERVO_WEB;
    if (nouvelAngle < 0) nouvelAngle = 0;
    if (xSemaphoreTake(mutexServo, pdMS_TO_TICKS(200)) == pdTRUE) {
        angleManuel = nouvelAngle;
        xSemaphoreGive(mutexServo);
    } else sante.timeoutsMutex++;
    char buf[8];
    snprintf(buf, sizeof(buf), "%d", nouvelAngle);
    LOG_INFO("WEB", "Servo gauche -> %d deg", nouvelAngle);
    httpd_resp_send(req, buf, strlen(buf));
    return ESP_OK;
}

// Déplacer le servo à droite
static esp_err_t gestionServoDroite(httpd_req_t *req) {
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    modeManuel = true;
    debutModeManuel = millis();
    int angleActuel = 0;
    if (xSemaphoreTake(mutexServo, pdMS_TO_TICKS(200)) == pdTRUE) {
        angleActuel = angleServo;
        xSemaphoreGive(mutexServo);
    } else sante.timeoutsMutex++;
    int nouvelAngle = angleActuel + PAS_SERVO_WEB;
    if (nouvelAngle > 180) nouvelAngle = 180;
    if (xSemaphoreTake(mutexServo, pdMS_TO_TICKS(200)) == pdTRUE) {
        angleManuel = nouvelAngle;
        xSemaphoreGive(mutexServo);
    } else sante.timeoutsMutex++;
    char buf[8];
    snprintf(buf, sizeof(buf), "%d", nouvelAngle);
    LOG_INFO("WEB", "Servo droite -> %d deg", nouvelAngle);
    httpd_resp_send(req, buf, strlen(buf));
    return ESP_OK;
}

// Renvoie l'état système pour la page web (format pipe-separated)
static esp_err_t gestionStatut(httpd_req_t *req) {
    char buf[320];
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache, no-store, must-revalidate");

    char label[64] = "En attente...";
    if (xSemaphoreTake(mutexDetection, pdMS_TO_TICKS(200)) == pdTRUE) {
        strncpy(label, labelDetection, 63);
        xSemaphoreGive(mutexDetection);
    } else sante.timeoutsMutex++;

    bool lumPhys = false, chrono = false;
    unsigned long dernier = 0;
    if (xSemaphoreTake(mutexLumiere, pdMS_TO_TICKS(200)) == pdTRUE) {
        lumPhys = lumiereAllumee;
        chrono = chronoActif;
        dernier = derniereDetection;
        xSemaphoreGive(mutexLumiere);
    } else sante.timeoutsMutex++;

    unsigned long secondesRestantes = 0;
    if (chrono && lumPhys) {
        unsigned long ecoule = millis() - dernier;
        if (ecoule < DUREE_EXTINCTION_MS)
            secondesRestantes = (DUREE_EXTINCTION_MS - ecoule) / 1000;
    }

    int angle = 0;
    if (xSemaphoreTake(mutexServo, pdMS_TO_TICKS(100)) == pdTRUE) {
        angle = angleServo;
        xSemaphoreGive(mutexServo);
    } else sante.timeoutsMutex++;

    char heureStr[12] = "??:??";
    if (heureDisponible) {
        unsigned long ecoule = (millis()-millisDemarrage)/1000;
        time_t maintenant = epochDemarrage + ecoule;
        struct tm* t = localtime(&maintenant);
        snprintf(heureStr, 12, "%02d:%02d", t->tm_hour, t->tm_min);
    }

    snprintf(buf, sizeof(buf), "%s|%s|%lu|%s|%d|%s|%s",
             label,
             lumPhys ? "ON" : "OFF",
             secondesRestantes,
             modeManuel ? "MANUEL" : "AUTO",
             angle,
             estModeNuit() ? "NUIT" : "JOUR",
             heureStr);
    httpd_resp_send(req, buf, strlen(buf));
    return ESP_OK;
}

// Diagnostic système complet (format texte)
static esp_err_t gestionSante(httpd_req_t *req) {
    char buf[768];
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    unsigned long uptime = (millis() - sante.heureDemarrage) / 1000;
    unsigned long heures = uptime / 3600, minutes = (uptime % 3600) / 60, secondes = uptime % 60;
    uint32_t tasLibre = esp_get_free_heap_size();
    int angle = 0;
    if (xSemaphoreTake(mutexServo, pdMS_TO_TICKS(100)) == pdTRUE) {
        angle = angleServo;
        xSemaphoreGive(mutexServo);
    }
    char heureStr[12] = "Non dispo";
    if (heureDisponible) {
        unsigned long ecoule = (millis()-millisDemarrage)/1000;
        time_t maintenant = epochDemarrage + ecoule;
        struct tm* t = localtime(&maintenant);
        snprintf(heureStr, 12, "%02d:%02d:%02d", t->tm_hour, t->tm_min, t->tm_sec);
    }
    snprintf(buf, sizeof(buf),
        "Uptime          : %luh %lum %lus\n"
        "Heure actuelle  : %s\n"
        "Heap libre      : %u octets\n"
        "Heap minimum    : %u octets\n"
        "Redémarrages    : %u\n"
        "Erreurs caméra  : %u\n"
        "Réinit caméra   : %u\n"
        "Détections      : %u\n"
        "Allumages       : %u\n"
        "Basculements    : %u\n"
        "Timeouts mutex  : %u\n"
        "Mode            : %s\n"
        "Mode nuit       : %s\n"
        "Servo angle     : %d deg\n"
        "Servo retour    : %s\n"
        "Lumière physique: %s\n"
        "--- Réglages ---\n"
        "Délai IA        : %d ms\n"
        "Scan servo tous : 1/%d cycles\n"
        "Heure NTP       : %s\n",
        heures, minutes, secondes, heureStr,
        tasLibre,
        (sante.heapMin == UINT32_MAX) ? tasLibre : sante.heapMin,
        sante.redemarrages,
        sante.erreursCamera,
        sante.reinitialisationsCamera,
        sante.compteurDetections,
        sante.compteurAllumages,
        sante.basculementsRelais,
        sante.timeoutsMutex,
        modeManuel ? "MANUEL" : "AUTO",
        estModeNuit() ? "OUI" : "NON",
        angle,
        servoRetour ? "OUI" : "NON",
        estLumiereAllumeePhysique() ? "ALLUMEE" : "ETEINTE",
        DELAI_IA_MS,
        TOUS_LES_CYCLES,
        heureDisponible ? "OK" : "NON"
    );
    httpd_resp_send(req, buf, strlen(buf));
    return ESP_OK;
}

// Page web principale (interface utilisateur)
static esp_err_t gestionPageIndex(httpd_req_t *req) {
    const char* html = R"rawliteral(
<!DOCTYPE html><html><head><title>ESP32-CAM Détection</title><meta name="viewport" content="width=device-width, initial-scale=1"><style>
*{box-sizing:border-box;margin:0;padding:0;}body{font-family:Arial;background:#1c1c1c;color:#fff;padding:16px;text-align:center;}h1{color:#0f0;font-size:18px;margin-bottom:14px;}img{max-width:100%;border:2px solid #0f0;border-radius:6px;display:block;margin:0 auto 12px;}#result-box{display:block;padding:10px 20px;border-radius:6px;font-size:1.1em;font-weight:bold;margin:0 auto 10px;max-width:480px;}#result-box.oui{background:#1a5c1a;border:2px solid #0f0;color:#0f0;}#result-box.non{background:#5c1a1a;border:2px solid #f44;color:#f44;}#result-box.attente{background:#333;border:2px solid #888;color:#aaa;}.badge-row{display:flex;justify-content:space-between;max-width:480px;margin:0 auto 10px;flex-wrap:wrap;gap:6px;}.badge{display:inline-flex;align-items:center;gap:6px;padding:5px 12px;border-radius:20px;font-size:12px;font-weight:bold;}.badge-light-on{background:#3d3d00;border:1.5px solid #ff0;color:#ff0;}.badge-light-off{background:#222;border:1.5px solid #555;color:#555;}.badge-auto{background:#002244;border:1.5px solid #08f;color:#08f;}.badge-manual{background:#3d1a00;border:1.5px solid #f80;color:#f80;}.badge-nuit{background:#1a1a3d;border:1.5px solid #88f;color:#88f;}.badge-heure{background:#003300;border:1.5px solid #0f0;color:#0f0;}.dot{width:8px;height:8px;border-radius:50%;display:inline-block;}.dot-on{background:#ff0;}.dot-off{background:#444;}.dot-auto{background:#08f;}.dot-manual{background:#f80;animation:blink 1s step-start infinite;}.dot-nuit{background:#88f;}@keyframes blink{50%{opacity:0;}}.section{background:#252525;border-radius:8px;border:1px solid #444;padding:12px 14px;margin:8px auto;max-width:480px;}.sec-title{font-size:11px;color:#777;text-transform:uppercase;letter-spacing:1px;margin-bottom:10px;}.btn-row{display:flex;gap:8px;}.btn{flex:1;padding:10px 0;border-radius:6px;font-size:13px;font-weight:bold;border:none;cursor:pointer;}.btn:active{opacity:0.7;}.btn-on{background:#1a5c1a;color:#0f0;border:2px solid #0f0;}.btn-off{background:#5c1a1a;color:#f44;border:2px solid #f44;}.btn-auto{background:#222;color:#08f;border:2px solid #08f;font-size:12px;width:100%;padding:8px 0;margin-top:8px;cursor:pointer;border-radius:6px;}.btn-health{background:#222;color:#8f8;border:2px solid #8f8;font-size:11px;width:100%;padding:6px 0;margin-top:6px;cursor:pointer;border-radius:6px;}.info-row{display:flex;justify-content:space-between;font-size:11px;color:#666;margin-top:8px;}.servo-ctrl{display:flex;align-items:center;justify-content:center;gap:20px;margin:8px 0;}.arrow-btn{width:56px;height:56px;border-radius:50%;border:2px solid #0af;background:#002233;color:#0af;font-size:26px;cursor:pointer;display:flex;align-items:center;justify-content:center;}.arrow-btn:hover{background:#003344;}.arrow-btn:active{background:#0af;color:#000;}.angle-display{text-align:center;min-width:70px;}.angle-val{font-size:30px;font-weight:bold;color:#0af;display:block;}.angle-lbl{font-size:11px;color:#555;}.bar-bg{background:#333;border-radius:4px;height:6px;width:100%;margin-top:8px;}.bar-fill{background:#0af;border-radius:4px;height:6px;width:0%;transition:width 0.2s;}.bar-labels{display:flex;justify-content:space-between;font-size:10px;color:#555;margin-top:3px;}#health-box{background:#111;border:1px solid #3a3a3a;border-radius:6px;padding:10px;text-align:left;font-family:monospace;font-size:11px;color:#8f8;white-space:pre;margin-top:8px;display:none;}#update-info{color:#555;font-size:11px;margin-top:10px;}.night-banner{display:none;background:#1a1a3d;border:1px solid #88f;border-radius:6px;padding:8px;color:#88f;font-size:12px;margin:0 auto 8px;max-width:480px;}</style>
</head><body><h1>ESP32-CAM — Détection de personne</h1><div class="night-banner" id="night-banner">&#128315; Mode nuit actif — Servo en pause</div>
<img id="streamImg" src="" /><div id="result-box" class="attente">En attente...</div>
<div class="badge-row"><span class="badge badge-light-off" id="light-badge"><span class="dot dot-off" id="light-dot"></span><span id="light-text">Lumière : ÉTEINTE</span></span><span class="badge badge-auto" id="mode-badge"><span class="dot dot-auto" id="mode-dot"></span><span id="mode-text">Mode : AUTO</span></span><span class="badge badge-nuit" id="night-badge" style="display:none;"><span class="dot dot-nuit"></span><span>Nuit</span></span><span class="badge badge-heure" id="heure-badge" style="display:none;"><span id="heure-text">--:--</span></span></div>
<div class="section"><div class="sec-title">Contrôle lumière</div><div class="btn-row"><button class="btn btn-on" onclick="setLight('on')">Allumer</button><button class="btn btn-off" onclick="setLight('off')">Éteindre</button></div><button class="btn-auto" onclick="setLight('auto')">Repasser en mode automatique (IA)</button><div class="info-row"><span id="timer-txt"></span><span id="mode-info-txt">Mode : AUTO</span></div></div>
<div class="section"><div class="sec-title">Contrôle servo</div><div class="servo-ctrl"><button class="arrow-btn" onclick="moveServo('left')">&#8592;</button><div class="angle-display"><span class="angle-val" id="angle-val">0&deg;</span><span class="angle-lbl">position</span></div><button class="arrow-btn" onclick="moveServo('right')">&#8594;</button></div><div class="bar-bg"><div class="bar-fill" id="servo-bar"></div></div><div class="bar-labels"><span>0&deg;</span><span>90&deg;</span><span>180&deg;</span></div><div class="info-row"><span>Pas : 10&deg; par clic</span><span>Max : 180&deg;</span></div></div>
<div class="section"><div class="sec-title">Santé du système</div><button class="btn-health" onclick="toggleHealth()">Afficher / Masquer les diagnostics</button><div id="health-box">Chargement...</div></div>
<p id="update-info">Connexion en cours...</p>
<script>
var errorCount=0,healthVisible=false;
var streamUrl = '//' + window.location.hostname + ':81/stream';
document.getElementById('streamImg').src = streamUrl;
function updateBar(angle){document.getElementById('angle-val').textContent=angle+'\u00b0';document.getElementById('servo-bar').style.width=(angle/180*100).toFixed(1)+'%';}
function setLight(action){fetch('/light/'+action,{cache:'no-store'}).catch(function(e){});}
function moveServo(dir){fetch('/servo/'+dir,{cache:'no-store'}).then(function(r){return r.text();}).then(function(txt){var a=parseInt(txt);if(!isNaN(a))updateBar(a);}).catch(function(e){});}
function toggleHealth(){healthVisible=!healthVisible;document.getElementById('health-box').style.display=healthVisible?'block':'none';if(healthVisible)fetchHealth();}
function fetchHealth(){if(!healthVisible)return;fetch('/health?'+Date.now(),{cache:'no-store'}).then(function(r){return r.text();}).then(function(t){document.getElementById('health-box').textContent=t;}).catch(function(e){document.getElementById('health-box').textContent='Erreur : '+e.message;});}
function updateStatus(){fetch('/status?'+Date.now(),{cache:'no-store'}).then(function(r){if(!r.ok)throw new Error('HTTP '+r.status);return r.text();}).then(function(data){errorCount=0;var parts=data.trim().split('|');var label=parts[0]||'En attente...';var light=parts[1]||'OFF';var seconds=parseInt(parts[2])||0;var mode=parts[3]||'AUTO';var angle=parseInt(parts[4])||0;var phase=parts[5]||'JOUR';var heure=parts[6]||'??:??';var box=document.getElementById('result-box');box.textContent=label;var ll=label.toLowerCase();if(ll.indexOf('person')!==-1||ll.indexOf('personne')!==-1)box.className='oui';else if(ll.indexOf('rien')!==-1||ll.indexOf('attente')!==-1)box.className='attente';else box.className='non';
var lb=document.getElementById('light-badge'),ld=document.getElementById('light-dot'),lt=document.getElementById('light-text');if(light==='ON'){lb.className='badge badge-light-on';ld.className='dot dot-on';lt.textContent='Lumière : ALLUMÉE';}else{lb.className='badge badge-light-off';ld.className='dot dot-off';lt.textContent='Lumière : ÉTEINTE';}
var mb=document.getElementById('mode-badge'),md=document.getElementById('mode-dot'),mt=document.getElementById('mode-text');if(mode==='MANUEL'){mb.className='badge badge-manual';md.className='dot dot-manual';mt.textContent='Mode : MANUEL';document.getElementById('mode-info-txt').textContent='Mode : Manuel';}else{mb.className='badge badge-auto';md.className='dot dot-auto';mt.textContent='Mode : AUTO';document.getElementById('mode-info-txt').textContent='Mode : Auto (IA)';}
var nb=document.getElementById('night-badge'),nb2=document.getElementById('night-banner');if(phase==='NUIT'){nb.style.display='inline-flex';nb2.style.display='block';}else{nb.style.display='none';nb2.style.display='none';}
var hb=document.getElementById('heure-badge'),ht=document.getElementById('heure-text');if(heure!=='??:??'){hb.style.display='inline-flex';ht.textContent=heure;}else hb.style.display='none';
var tb=document.getElementById('timer-txt');if(light==='ON'&&seconds>0){var min=Math.floor(seconds/60),sec=seconds%60;tb.textContent='Extinction dans : '+min+'min '+sec+'s';}else tb.textContent='';
updateBar(angle);var now=new Date();var h=now.getHours().toString().padStart(2,'0'),m=now.getMinutes().toString().padStart(2,'0'),s=now.getSeconds().toString().padStart(2,'0');document.getElementById('update-info').textContent='Dernière mise à jour : '+h+':'+m+':'+s;if(healthVisible)fetchHealth();}).catch(function(e){errorCount++;document.getElementById('update-info').textContent='Reconnexion... (tentative '+errorCount+')';});}
updateStatus();setInterval(updateStatus,1000);
</script></body></html>
)rawliteral";
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_send(req, html, strlen(html));
    return ESP_OK;
}

// ============================================================
// STREAMING VIDEO (flux MJPEG) sur le port 81
// ============================================================
static esp_err_t gestionFluxVideo(httpd_req_t *req) {
    esp_err_t res = ESP_OK;
    char part_buf[64];
    if (xSemaphoreTake(mutexClient, pdMS_TO_TICKS(200)) == pdTRUE) {
        clientConnecte = true;
        xSemaphoreGive(mutexClient);
    }
    LOG_INFO("STREAM", "Client connecté");
    res = httpd_resp_set_type(req, "multipart/x-mixed-replace;boundary=123456789000000000000987654321");
    if (res != ESP_OK) {
        if (xSemaphoreTake(mutexClient, pdMS_TO_TICKS(200)) == pdTRUE) {
            clientConnecte = false;
            xSemaphoreGive(mutexClient);
        }
        return res;
    }
    uint32_t compteurTimeout = 0;
    while (true) {
        bool imageEnvoyee = false;
        if (xSemaphoreTake(mutexFlux, pdMS_TO_TICKS(150)) == pdTRUE) {
            if (tamponFlux != NULL && longueurFlux > 0) {
                res = httpd_resp_send_chunk(req, "\r\n--123456789000000000000987654321\r\n", strlen("\r\n--123456789000000000000987654321\r\n"));
                if (res == ESP_OK) {
                    size_t hlen = snprintf(part_buf, sizeof(part_buf), "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n", longueurFlux);
                    res = httpd_resp_send_chunk(req, part_buf, hlen);
                }
                if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char*)tamponFlux, longueurFlux);
                if (res == ESP_OK) imageEnvoyee = true;
            }
            xSemaphoreGive(mutexFlux);
        }
        if (res != ESP_OK) break;
        if (!imageEnvoyee) {
            compteurTimeout += 50;
            if (compteurTimeout >= 5000) break;
        } else compteurTimeout = 0;
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (xSemaphoreTake(mutexClient, pdMS_TO_TICKS(200)) == pdTRUE) {
        clientConnecte = false;
        xSemaphoreGive(mutexClient);
    }
    LOG_INFO("STREAM", "Client déconnecté");
    return res;
}

// ============================================================
// DÉMARRAGE DES SERVEURS WEB (port 80 pour l'interface, 81 pour le flux)
// ============================================================
bool demarrerServeurs() {
    httpd_config_t config80 = HTTPD_DEFAULT_CONFIG();
    config80.server_port = 80;
    config80.max_open_sockets = 7;
    config80.lru_purge_enable = true;

    // Enregistrement des URIs pour le serveur 80
    httpd_uri_t uri_index = { "/", HTTP_GET, gestionPageIndex, NULL };
    httpd_uri_t uri_statut = { "/status", HTTP_GET, gestionStatut, NULL };
    httpd_uri_t uri_sante = { "/health", HTTP_GET, gestionSante, NULL };
    httpd_uri_t uri_allumer = { "/light/on", HTTP_GET, gestionAllumerLumiere, NULL };
    httpd_uri_t uri_eteindre = { "/light/off", HTTP_GET, gestionEteindreLumiere, NULL };
    httpd_uri_t uri_auto = { "/light/auto", HTTP_GET, gestionModeAuto, NULL };
    httpd_uri_t uri_servo_gauche = { "/servo/left", HTTP_GET, gestionServoGauche, NULL };
    httpd_uri_t uri_servo_droite = { "/servo/right", HTTP_GET, gestionServoDroite, NULL };

    if (httpd_start(&serveur80, &config80) != ESP_OK) {
        LOG_ERROR("SERVER", "Échec port 80");
        return false;
    }
    httpd_register_uri_handler(serveur80, &uri_index);
    httpd_register_uri_handler(serveur80, &uri_statut);
    httpd_register_uri_handler(serveur80, &uri_sante);
    httpd_register_uri_handler(serveur80, &uri_allumer);
    httpd_register_uri_handler(serveur80, &uri_eteindre);
    httpd_register_uri_handler(serveur80, &uri_auto);
    httpd_register_uri_handler(serveur80, &uri_servo_gauche);
    httpd_register_uri_handler(serveur80, &uri_servo_droite);
    LOG_INFO("SERVER", "Port 80 OK");

    httpd_config_t config81 = HTTPD_DEFAULT_CONFIG();
    config81.server_port = 81;
    config81.max_open_sockets = 3;
    config81.lru_purge_enable = true;
    config81.ctrl_port = 32769;
    httpd_uri_t uri_flux = { "/stream", HTTP_GET, gestionFluxVideo, NULL };
    if (httpd_start(&serveur81, &config81) != ESP_OK) {
        LOG_ERROR("SERVER", "Échec port 81");
        return false;
    }
    httpd_register_uri_handler(serveur81, &uri_flux);
    LOG_INFO("SERVER", "Port 81 OK (stream)");
    return true;
}

// ============================================================
// FONCTION DE LECTURE DES DONNÉES POUR LE MODÈLE EDGE IMPULSE
// Convertit le buffer RGB en un signal lisible par l'IA (entier 24 bits)
// ============================================================
static int ei_camera_obtenir_donnees(size_t offset, size_t longueur, float *ptrSortie) {
    size_t indexPixel = offset * 3;       // chaque pixel = 3 octets
    size_t pixelsRestants = longueur;
    size_t indexSortie = 0;
    while (pixelsRestants != 0) {
        // Construction d'un entier 24 bits : R<<16 | G<<8 | B
        ptrSortie[indexSortie] = (tamponRgbIa[indexPixel + 2] << 16) +
                                 (tamponRgbIa[indexPixel + 1] << 8) +
                                 tamponRgbIa[indexPixel];
        indexSortie++;
        indexPixel += 3;
        pixelsRestants--;
    }
    return 0;
}

// ============================================================
// TÂCHE DE CAPTURE RAPIDE (exécutée sur le cœur 1)
// Récupère une image, la stocke pour le streaming et la prépare pour l'IA
// ============================================================
void tacheCaptureRapide(void* parametres) {
    LOG_INFO("TACHE", "Capture rapide démarrée (Cœur 1)");
    esp_task_wdt_add(NULL);
    while (true) {
        esp_task_wdt_reset();
        camera_fb_t* fb = esp_camera_fb_get();  // capture JPEG
        if (!fb) {
            sante.erreursCamera++;
            compteurErreursCam++;
            LOG_ERROR("CAM", "Erreur capture (%d/%d)", compteurErreursCam, MAX_ERREURS_CAMERA);
            if (compteurErreursCam >= MAX_ERREURS_CAMERA) {
                if (reinitialiserCamera()) {
                    compteurErreursCam = 0;
                } else if (compteurReinitCam >= MAX_REINITIALISATIONS_CAMERA) {
                    esp_restart(); // redémarrage complet si trop d'échecs
                }
            }
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }
        compteurErreursCam = 0;

        // Mise à jour du tampon de streaming (JPEG)
        if (xSemaphoreTake(mutexFlux, pdMS_TO_TICKS(100)) == pdTRUE) {
            if (fb->len <= LARGEUR_IMAGE_IA * HAUTEUR_IMAGE_IA * 2) { // sécurité
                memcpy(tamponFlux, fb->buf, fb->len);
                longueurFlux = fb->len;
            } else {
                longueurFlux = 0;
            }
            xSemaphoreGive(mutexFlux);
        }

        // Demande en attente de la part de la tâche IA ?
        if (xSemaphoreTake(semRequeteIa, 0) == pdTRUE) {
            // Décodage JPEG -> RGB888 dans tamponRgbIa
            bool conversionOk = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, tamponRgbIa);
            if (conversionOk) {
                // Si la taille exigée par l'IA est différente de 320x240, on redimensionne
                if ((EI_CLASSIFIER_INPUT_WIDTH != LARGEUR_IMAGE_IA) ||
                    (EI_CLASSIFIER_INPUT_HEIGHT != HAUTEUR_IMAGE_IA)) {
                    ei::image::processing::crop_and_interpolate_rgb888(
                        tamponRgbIa,
                        LARGEUR_IMAGE_IA, HAUTEUR_IMAGE_IA,
                        tamponRgbIa,
                        EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT
                    );
                }
                xSemaphoreGive(semPretIa); // prévient l'IA que l'image est prête
            } else {
                xSemaphoreGive(semRequeteIa); // réessaiera plus tard
            }
        }

        esp_camera_fb_return(fb);
    }
}

// ============================================================
// TÂCHE IA (exécutée sur le cœur 0)
// Attend une image, exécute le classifieur Edge Impulse, puis agit
// ============================================================
void tacheIa(void* parametres) {
    LOG_INFO("TACHE", "Tâche IA démarrée (Cœur 0)");
    esp_task_wdt_add(NULL);
    while (true) {
        esp_task_wdt_reset();

        // Signaler qu'on veut une nouvelle image
        xSemaphoreGive(semRequeteIa);

        // Attendre que l'image soit décodée (avec timeout)
        if (xSemaphoreTake(semPretIa, pdMS_TO_TICKS(2000)) != pdTRUE) {
            LOG_WARNING("IA", "Timeout attente image");
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        // Préparer le signal pour Edge Impulse
        ei::signal_t signal;
        signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
        signal.get_data = &ei_camera_obtenir_donnees;

        ei_impulse_result_t resultat = {0};
        EI_IMPULSE_ERROR erreur = run_classifier(&signal, &resultat, false);
        if (erreur != EI_IMPULSE_OK) {
            LOG_ERROR("IA", "Erreur classifieur : %d", erreur);
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        float meilleurScore = 0.0f;
        char nouveauLabel[64] = "Rien détecté";
        bool personneDetectee = false;

#if EI_CLASSIFIER_OBJECT_DETECTION == 1
        // Mode détection d'objets (bounding boxes)
        for (uint32_t i = 0; i < resultat.bounding_boxes_count; i++) {
            ei_impulse_result_bounding_box_t bb = resultat.bounding_boxes[i];
            if (bb.value == 0) continue;
            if (bb.value > meilleurScore) {
                meilleurScore = bb.value;
                snprintf(nouveauLabel, sizeof(nouveauLabel), "%s : %.1f%%", bb.label, bb.value * 100);
                personneDetectee = ((strstr(bb.label, "person") != NULL || strstr(bb.label, "personne") != NULL) && meilleurScore > 0.80f);
            }
        }
#else
        // Mode classification simple
        for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
            if (resultat.classification[i].value > meilleurScore) {
                meilleurScore = resultat.classification[i].value;
                snprintf(nouveauLabel, sizeof(nouveauLabel), "%s : %.1f%%", ei_classifier_inferencing_categories[i], meilleurScore * 100);
                personneDetectee = ((strstr(ei_classifier_inferencing_categories[i], "person") != NULL ||
                                    strstr(ei_classifier_inferencing_categories[i], "personne") != NULL) &&
                                    meilleurScore > 0.80f);
            }
        }
#endif

        // Mettre à jour le label de détection (pour l'interface web)
        if (xSemaphoreTake(mutexDetection, pdMS_TO_TICKS(200)) == pdTRUE) {
            strncpy(labelDetection, nouveauLabel, 63);
            labelDetection[63] = 0;
            scoreDetection = meilleurScore;
            xSemaphoreGive(mutexDetection);
        } else sante.timeoutsMutex++;

        // Commander la lumière si mode automatique
        if (!modeManuel) {
            gererLumiere(personneDetectee);
        } else {
            LOG_INFO("LUMIERE", "Mode manuel — IA suspendue");
        }

        // Afficher un log récapitulatif
        int angleLog = 0;
        if (xSemaphoreTake(mutexServo, pdMS_TO_TICKS(50)) == pdTRUE) {
            angleLog = angleServo;
            xSemaphoreGive(mutexServo);
        }
        LOG_INFO("IA", "%s | Personne:%s | Lum:%s | Mode:%s | Nuit:%s | Servo:%d° | Heap:%u",
                 nouveauLabel, personneDetectee ? "OUI" : "NON",
                 estLumiereAllumeePhysique() ? "ON" : "OFF",
                 modeManuel ? "MAN" : "AUTO",
                 estModeNuit() ? "OUI" : "NON",
                 angleLog, esp_get_free_heap_size());

        // Appliquer éventuellement un angle manuel demandé par le web
        bool angleManuelApplique = false;
        if (xSemaphoreTake(mutexServo, pdMS_TO_TICKS(50)) == pdTRUE) {
            if (angleManuel >= 0) {
                angleServo = angleManuel;
                angleManuel = -1;
                angleManuelApplique = true;
                int angleTemp = angleServo;
                xSemaphoreGive(mutexServo);
                monServo.write(angleTemp);
                LOG_INFO("SERVO", "Commande manuelle : %d deg", angleTemp);
            } else {
                xSemaphoreGive(mutexServo);
            }
        } else sante.timeoutsMutex++;

        // Si aucun mouvement manuel, et que nous sommes en mode auto, on gère le servo
        if (!angleManuelApplique && !modeManuel) {
            gererServoAuto();
        }

        // Pause avant la prochaine analyse
        vTaskDelay(pdMS_TO_TICKS(DELAI_IA_MS));
    }
}

// ============================================================
// TÂCHE WIFI + SERVEUR (exécutée sur le cœur 0)
// Maintenir la connexion WiFi et les serveurs HTTP
// ============================================================
void tacheWifi(void* parametres) {
    LOG_INFO("TACHE", "WiFi client démarrée (Cœur 0)");
    esp_task_wdt_add(NULL);

    // Vérifier que le WiFi est connecté (déjà fait dans setup, mais on s'assure)
    if (WiFi.status() != WL_CONNECTED) {
        LOG_WARNING("WIFI", "Non connecté, tentative de connexion...");
        WiFi.begin(nom_wifi, mot_passe_wifi);
        int essais = 0;
        while (WiFi.status() != WL_CONNECTED && essais < 20) {
            delay(500);
            essais++;
        }
    }

    if (WiFi.status() == WL_CONNECTED) {
        LOG_INFO("WIFI", "Connecté, IP : %s", WiFi.localIP().toString().c_str());
    } else {
        LOG_ERROR("WIFI", "Impossible de se connecter au WiFi, redémarrage...");
        esp_restart();
    }

    if (!demarrerServeurs()) {
        LOG_FATAL("SERVER", "Serveurs non démarrés — reboot");
        vTaskDelay(pdMS_TO_TICKS(3000));
        esp_restart();
    }

    while (true) {
        esp_task_wdt_reset();
        gererChronoLumiere();     // Gère l'extinction automatique et le retour du mode manuel

        // Surveillance de la connexion WiFi
        if (WiFi.status() != WL_CONNECTED) {
            LOG_WARNING("WIFI", "Connexion perdue, reconnexion...");
            WiFi.reconnect();
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

// ============================================================
// SETUP – INITIALISATION GÉNÉRALE
// ============================================================
void setup() {
    // Désactiver le détecteur de sous-tension (brownout) pour éviter les redémarrages intempestifs
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

    Serial.begin(115200);
    LOG_INFO("INIT", "=== ESP32-CAM CLIENT WIFI (Centre d'incubation) ===");

    // Initialiser le watchdog avec un timeout de 30 secondes
    esp_task_wdt_init(TIMEOUT_WATCHDOG_SEC, true);

    // Configurer la PWM pour la LED de la caméra (indicateur)
    ledcSetup(CANAL_PWM_LED, FREQ_PWM_LED, RESOLUTION_PWM_LED);
    ledcAttachPin(BROCHE_LED_CAM, CANAL_PWM_LED);
    ledcWrite(CANAL_PWM_LED, 0);   // éteinte au départ

    // Initialiser les sorties
    pinMode(BROCHE_RELAIS, OUTPUT);
    pinMode(BROCHE_LDR, INPUT);
    digitalWrite(BROCHE_RELAIS, LOW);

    // === CONNEXION WIFI ===
    LOG_INFO("WIFI", "Connexion à %s...", nom_wifi);
    WiFi.mode(WIFI_STA);
    WiFi.begin(nom_wifi, mot_passe_wifi);
    unsigned long debutWifi = millis();
    bool wifiOk = false;
    while (millis() - debutWifi < 20000) {
        if (WiFi.status() == WL_CONNECTED) {
            wifiOk = true;
            break;
        }
        delay(500);
        // Clignotement de la LED caméra pour signaler la tentative
        ledcWrite(CANAL_PWM_LED, DUTY_DIM);
        delay(100);
        ledcWrite(CANAL_PWM_LED, 0);
        delay(400);
    }
    if (!wifiOk) {
        LOG_ERROR("WIFI", "Échec connexion WiFi, redémarrage...");
        esp_restart();
    }
    LOG_INFO("WIFI", "Connecté, IP : %s", WiFi.localIP().toString().c_str());

    // === SYNCHRONISATION NTP ===
    if (!synchroniserHeureNTP()) {
        LOG_WARNING("NTP", "Heure non disponible, mode nuit basé sur uptime");
    }

    // === INITIALISATION LUMIÈRE ===
    initLumiere();

    // === SERVO ===
    ESP32PWM::allocateTimer(3);           // alloue un timer pour le servo
    monServo.setPeriodHertz(50);          // 50 Hz pour servo standard
    monServo.attach(BROCHE_SERVO, 500, 2400); // largeur d'impulsion (500-2400 µs)
    monServo.write(0);
    LOG_INFO("SERVO", "Initialisé à 0 deg");

    // === CAMÉRA ===
    esp_err_t err = esp_camera_init(&configCamera);
    if (err != ESP_OK) {
        LOG_FATAL("CAM", "Erreur init caméra : 0x%x", err);
        while (1) delay(1000);
    }
    // Ajustements spécifiques selon le capteur
    sensor_t* capteur = esp_camera_sensor_get();
    if (capteur->id.PID == OV3660_PID) {
        capteur->set_vflip(capteur, 1);
        capteur->set_brightness(capteur, 1);
        capteur->set_saturation(capteur, 0);
    } else {
        capteur->set_vflip(capteur, 1);
        capteur->set_hmirror(capteur, 1);
    }
    LOG_INFO("CAM", "Initialisée");

    // === CRÉATION DES SÉMAPHORES ===
    mutexFlux = xSemaphoreCreateMutex();
    mutexDetection = xSemaphoreCreateMutex();
    mutexLumiere = xSemaphoreCreateMutex();
    mutexServo = xSemaphoreCreateMutex();
    mutexRelais = xSemaphoreCreateMutex();
    mutexClient = xSemaphoreCreateMutex();
    semRequeteIa = xSemaphoreCreateBinary();
    semPretIa = xSemaphoreCreateBinary();

    if (!mutexFlux || !mutexDetection || !mutexLumiere || !mutexServo || !mutexRelais ||
        !mutexClient || !semRequeteIa || !semPretIa) {
        LOG_FATAL("INIT", "Échec création mutex — reboot");
        esp_restart();
    }

    // === ALLOCATION DES BUFFERS (PSRAM ou SRAM) ===
    // Tampon pour le flux JPEG : taille max environ 320*240*2 = 153600 octets
    tamponFlux = (uint8_t*)malloc(LARGEUR_IMAGE_IA * HAUTEUR_IMAGE_IA * 2);
    // Tampon RGB pour l'IA : 320*240*3 = 230400 octets
    tamponRgbIa = (uint8_t*)malloc(LARGEUR_IMAGE_IA * HAUTEUR_IMAGE_IA * TAILLE_PIXEL_IA);
    if (!tamponFlux || !tamponRgbIa) {
        LOG_FATAL("MEM", "Allocation buffers échouée");
        esp_restart();
    }
    LOG_INFO("MEM", "Buffers alloués");

    // === CHARGEMENT DE L'ÉTAT NVS ===
    preferences.begin("esp32cam", true);
    modeManuel = preferences.getBool("modeManuel", false);
    sante.redemarrages = preferences.getUInt("redemarrages", 0) + 1;
    preferences.end();
    dernierModeSauvegarde = modeManuel;
    preferences.begin("esp32cam", false);
    preferences.putUInt("redemarrages", sante.redemarrages);
    preferences.end();
    LOG_INFO("NVS", "Redémarrages: %u | Mode: %s", sante.redemarrages, modeManuel ? "MANUEL" : "AUTO");

    sante.heureDemarrage = millis();
    initLumiere();  // rafraîchit l'état de la lumière

    // === CRÉATION DES TÂCHES FREERTOS ===
    // Capture rapide sur cœur 1 (priorité 2)
    xTaskCreatePinnedToCore(tacheCaptureRapide, "CaptureRapide", 8192, NULL, 2, NULL, 1);
    // IA sur cœur 0 (priorité 1)
    xTaskCreatePinnedToCore(tacheIa, "IATask", 16384, NULL, 1, NULL, 0);
    // WiFi + serveurs sur cœur 0 (priorité 1)
    xTaskCreatePinnedToCore(tacheWifi, "WiFi", 8192, NULL, 1, NULL, 0);

    LOG_INFO("INIT", "Système prêt → http://%s", WiFi.localIP().toString().c_str());
    LOG_INFO("INIT", "Diagnostics → http://%s/health", WiFi.localIP().toString().c_str());
}

// ============================================================
// LOOP (inutilisé car tout se passe dans les tâches)
// ============================================================
void loop() {
    vTaskDelay(pdMS_TO_TICKS(10000));
}
