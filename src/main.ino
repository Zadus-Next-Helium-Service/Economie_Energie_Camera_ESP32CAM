/**
 * @file main.cpp
 * @brief ESP32-CAM AI Thinker – Surveillance intelligente complète
 * 
 * Ce système combine :
 * - Détection de personnes par IA (Edge Impulse)
 * - Gestion automatique de l'éclairage via relais + capteur LDR
 * - Contrôle d'un servo-moteur pour orienter la caméra
 * - Notifications Telegram avec photos (nuit / jours fériés)
 * - Serveurs web : interface de contrôle, dashboard santé, gestion jours fériés
 * - Stockage persistant des réglages (NVS)
 * - Watchdog et surveillance mémoire pour robustesse
 * 
 * Matériel : ESP32-CAM AI Thinker + relais + LDR + servo + LED intégrée
 */

// ============================================================
//  LIBRAIRIES SYSTÈME
// ============================================================
#include "esp_camera.h"
#include <WiFi.h>
#include "esp_http_server.h"
#include "esp_heap_caps.h"
#include <ESP32Servo.h>
#include <Preferences.h>
#include "esp_task_wdt.h"
#include <EconomieEnergieBBITEST_inferencing.h>   // Modèle Edge Impulse (personne / rien)
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <time.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <cstring>

// Vérification que le modèle a bien été exporté pour une caméra
#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_CAMERA
#error "Ce firmware nécessite un modèle Edge Impulse entraîné avec des images (format 'camera')"
#endif

// ============================================================
//  BROCHES CAMÉRA – Carte AI Thinker
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
//  BROCHES PÉRIPHÉRIQUES
// ============================================================
#define BROCHE_RELAIS   14   // Commande de la lampe
#define BROCHE_LDR      15   // Capteur de lumière (présence physique de la lampe)
#define BROCHE_SERVO    13   // Servo-moteur
#define BROCHE_LED_CAM   4   // LED témoin (flash + indicateur d'état)

// ============================================================
//  IDENTIFIANTS RÉSEAU (À MODIFIER SELON VOTRE RÉSEAU)
// ============================================================
const char* wifi_ssid     = "VOTRE_SSID_WIFI";      // Remplacer par votre SSID
const char* wifi_password = "VOTRE_MOT_DE_PASSE";   // Remplacer par votre mot de passe

// ============================================================
//  NOTIFICATIONS TELEGRAM (À CONFIGURER)
// ============================================================
#define TELEGRAM_BOT_TOKEN   "VOTRE_BOT_TOKEN_TELEGRAM"     // Ex: "123456:ABC-DEF..."
#define TELEGRAM_CHAT_ID     "VOTRE_CHAT_ID_TELEGRAM"       // Ex: "123456789"

// ============================================================
//  RÈGLES DE NOTIFICATION
// ============================================================
#define HEURE_DEBUT_NUIT          20   // 20h -> début période de nuit
#define HEURE_FIN_NUIT             6   //  6h -> fin période de nuit
#define DELAI_ABSENCE_MS    7200000UL // 2h sans personne avant alerte "retour"
#define ANTI_SPAM_NUIT_MS    600000UL // 10 min entre deux alertes nuit
#define ANTI_SPAM_ABSENCE_MS  300000UL // 5 min entre deux alertes absence

// ============================================================
//  STOCKAGE JOURS FÉRIÉS (NVS)
// ============================================================
#define NVS_NAMESPACE_HOLIDAYS  "holidays"
#define NVS_KEY_LIST            "holiday_list"
#define NVS_KEY_YEAR            "hol_year"

// Jours fériés fixes par défaut (JJ/MM)
const char* defaultHolidays[] = {
  "01/01", "03/01", "08/03", "01/05", "15/08", "01/11", "25/12"
};
const int defaultHolidaysCount = 7;

// ============================================================
//  FIABILITÉ DÉTECTION – validation temporelle
// ============================================================
#define FENETRE_VALIDATION_MS   10000   // 10 secondes
#define SEUIL_DETECTIONS        2       // Au moins 2 détections dans la fenêtre
#define HISTORIQUE_MAX          10      // Taille de l'historique glissant

// ============================================================
//  CONFIGURATION IMAGE POUR L'IA
// ============================================================
#define LARGEUR_IMAGE_IA   320
#define HAUTEUR_IMAGE_IA   240
#define TAILLE_PIXEL_IA      3   // RGB888

// ============================================================
//  GESTION DE LA LUMIÈRE (relais + LDR)
// ============================================================
#define SEUIL_LDR             3000      // Seuil de luminosité pour considérer lampe allumée
#define DUREE_EXTINCTION_MS  240000     // Éteindre après 4 min sans détection
#define DUREE_MANUEL_MS      240000     // Mode manuel → retour auto après 4 min
#define ANTI_REBOND_RELAIS_MS  2000     // Évite les basculements trop rapides

// ============================================================
//  SERVO-MOTEUR
// ============================================================
#define PAS_SERVO           10          // Incrément en mode auto
#define ANGLE_MAX_SERVO     100         // Course max (évite butées)
#define PAS_RETOUR_SERVO    10          // Vitesse de retour
#define PAS_SERVO_WEB       10          // Incrément depuis l'interface web
#define TOUS_LES_CYCLES      2          // Mouvement tous les 2 cycles IA

// ============================================================
//  WATCHDOG
// ============================================================
#define TIMEOUT_WATCHDOG_SEC  30        // Si une tâche se bloque 30s → reboot

// ============================================================
//  RÉSILIENCE CAMÉRA
// ============================================================
#define MAX_ERREURS_CAMERA            10   // Avant de tenter une réinit
#define MAX_REINITIALISATIONS_CAMERA   3   // Avant de rebooter

// ============================================================
//  SURVEILLANCE MÉMOIRE (heap)
// ============================================================
#define SEUIL_AVERTISSEMENT_HEAP  30000   // Alerte si < 30 Ko libres

// ============================================================
//  CADENCE DE L'IA
// ============================================================
#define DELAI_IA_MS  2000               // Une analyse toutes les 2 secondes

// ============================================================
//  NTP (heure)
// ============================================================
#define SERVEUR_NTP            "pool.ntp.org"
#define DECALAGE_HORAIRE_SEC    0        // UTC+0, ajustez si besoin
#define DECALAGE_ETE_SEC        0        // Pas d'heure d'été

// ============================================================
//  PWM POUR LED TÉMOIN
// ============================================================
#define FREQ_PWM_LED       5000
#define RESOLUTION_PWM_LED 8
#define DUTY_MAX           255
#define DUTY_DIM           128

// ============================================================
//  LIMITE REDÉMARRAGES WIFI
// ============================================================
#define MAX_REDEMARRAGES_WIFI  5         // Évite les boucles infinies

// ============================================================
//  MACROS DE LOG (colorées dans la console série)
// ============================================================
#define LOG_INFO(tag, fmt, ...)    Serial.printf("[INFO][%s] " fmt "\n",    tag, ##__VA_ARGS__)
#define LOG_WARNING(tag, fmt, ...) Serial.printf("[WARNING][%s] " fmt "\n", tag, ##__VA_ARGS__)
#define LOG_ERROR(tag, fmt, ...)   Serial.printf("[ERROR][%s] " fmt "\n",   tag, ##__VA_ARGS__)
#define LOG_FATAL(tag, fmt, ...)   Serial.printf("[FATAL][%s] " fmt "\n",   tag, ##__VA_ARGS__)

// ============================================================
//  STRUCTURE – SUIVI SANTÉ / STATISTIQUES
// ============================================================
struct SanteSysteme {
    uint32_t redemarrages;            // Compteur de redémarrages
    uint32_t erreursCamera;           // Échecs de capture
    uint32_t reinitialisationsCamera; // Nombre de réinits de la caméra
    uint32_t compteurDetections;      // Détections IA (personne)
    uint32_t compteurAllumages;       // Allumages de la lampe
    uint32_t basculementsRelais;      // Changements d'état du relais
    uint32_t timeoutsMutex;           // Mutex non pris à temps
    uint32_t heapMin;                 // Plus petite heap libre depuis démarrage
    unsigned long heureDemarrage;     // millis() au lancement
    uint32_t notificationsTelegram;   // Messages envoyés
};
SanteSysteme sante = { 0, 0, 0, 0, 0, 0, 0, UINT32_MAX, 0, 0 };

// ============================================================
//  VARIABLES TEMPS (référence absolue)
// ============================================================
static bool heureDisponible = false;
static time_t epochDemarrage = 0;
static unsigned long millisDemarrage = 0;

// ============================================================
//  NVS – ÉTAT GÉNÉRAL (mode manuel / auto)
// ============================================================
Preferences preferences;
volatile bool modeManuel = false;
volatile unsigned long debutModeManuel = 0;
static bool dernierModeSauvegarde = false;

/**
 * @brief Sauvegarde l'état du mode (manuel/auto) et les compteurs dans NVS
 */
void sauvegarderEtatNVS() {
    if (modeManuel == dernierModeSauvegarde) return;  // Pas de changement
    dernierModeSauvegarde = modeManuel;
    preferences.begin("esp32cam", false);
    preferences.putBool("modeManuel", modeManuel);
    preferences.putUInt("redemarrages", sante.redemarrages);
    preferences.end();
    LOG_INFO("NVS", "État sauvegardé (modeManuel=%s)", modeManuel ? "MANUEL" : "AUTO");
}

// ============================================================
//  GESTION JOURS FÉRIÉS
// ============================================================
String holidayList = "";          // Liste stockée ex: "01/01,25/12,15/08*15/08/2025"
int holidayCurrentYear = 0;

/**
 * @brief Calcule la date de Pâques (algorithme de Conway)
 * @return String au format "JJ/MM"
 */
String calculerPaques(int year) {
    int a = year % 19;
    int b = year / 100;
    int c = year % 100;
    int d = b / 4;
    int e = b % 4;
    int f = (b + 8) / 25;
    int g = (b - f + 1) / 3;
    int h = (19 * a + b - d - g + 15) % 30;
    int i = c / 4;
    int k = c % 4;
    int l = (32 + 2 * e + 2 * i - h - k) % 7;
    int m = (a + 11 * h + 22 * l) / 451;
    int month = (h + l - 7 * m + 114) / 31;
    int day   = ((h + l - 7 * m + 114) % 31) + 1;
    char buf[6];
    sprintf(buf, "%02d/%02d", day, month);
    return String(buf);
}

/**
 * @brief Vérifie si une date donnée (JJ/MM) est présente dans la liste des fériés
 */
bool holidayExists(const String& date) {
    if (holidayList.length() == 0) return false;
    int index = 0;
    while (index < (int)holidayList.length()) {
        int comma = holidayList.indexOf(',', index);
        if (comma == -1) comma = holidayList.length();
        String current = holidayList.substring(index, comma);
        current.trim();
        if (current == date) return true;
        // Si la date est stockée avec un suffixe d'année (JJ/MM*JJ/MM/AAAA)
        int star = current.indexOf('*');
        if (star != -1 && current.substring(0, star) == date) return true;
        index = comma + 1;
    }
    return false;
}

/**
 * @brief Ajoute une date (peut contenir '*' pour une année spécifique) et maintient la liste triée
 */
void ajouterJourFerie(const String& date) {
    if (holidayList.length() > 0) holidayList += ",";
    holidayList += date;

    // Extraction de tous les éléments pour trier par ordre calendaire
    String dates[50];
    int idx = 0, start = 0;
    String tmp = holidayList;
    for (int i = 0; i <= (int)tmp.length(); i++) {
        if (i == (int)tmp.length() || tmp.charAt(i) == ',') {
            dates[idx++] = tmp.substring(start, i);
            start = i + 1;
            if (idx >= 50) break;
        }
    }

    // Tri à bulles basé sur le numéro du jour (MMJJ)
    for (int i = 0; i < idx - 1; i++) {
        for (int j = 0; j < idx - i - 1; j++) {
            String d1 = dates[j], d2 = dates[j+1];
            int s1 = d1.indexOf('*'), s2 = d2.indexOf('*');
            if (s1 != -1) d1 = d1.substring(0, s1);
            if (s2 != -1) d2 = d2.substring(0, s2);
            int v1 = d1.substring(3,5).toInt()*100 + d1.substring(0,2).toInt();
            int v2 = d2.substring(3,5).toInt()*100 + d2.substring(0,2).toInt();
            if (v1 > v2) { String t = dates[j]; dates[j] = dates[j+1]; dates[j+1] = t; }
        }
    }

    // Reconstruction de la liste triée
    holidayList = "";
    for (int i = 0; i < idx; i++) {
        if (i > 0) holidayList += ",";
        holidayList += dates[i];
    }
}

/**
 * @brief Supprime une date (JJ/MM) de la liste
 */
void supprimerJourFerie(const String& date) {
    String newList = "";
    int index = 0; bool first = true;
    while (index < (int)holidayList.length()) {
        int comma = holidayList.indexOf(',', index);
        if (comma == -1) comma = holidayList.length();
        String current = holidayList.substring(index, comma);
        current.trim();
        if (current != date) {
            if (!first) newList += ",";
            newList += current;
            first = false;
        }
        index = comma + 1;
    }
    holidayList = newList;
}

/**
 * @brief Charge les jours fériés depuis la NVS
 */
void chargerJoursFeries() {
    prefsHolidays.begin(NVS_NAMESPACE_HOLIDAYS, true);
    holidayList = prefsHolidays.getString(NVS_KEY_LIST, "");
    holidayCurrentYear = prefsHolidays.getInt(NVS_KEY_YEAR, 0);
    prefsHolidays.end();
}

/**
 * @brief Sauvegarde la liste des jours fériés dans la NVS
 */
void sauvegarderJoursFeries() {
    prefsHolidays.begin(NVS_NAMESPACE_HOLIDAYS, false);
    prefsHolidays.putString(NVS_KEY_LIST, holidayList);
    prefsHolidays.putInt(NVS_KEY_YEAR, holidayCurrentYear);
    prefsHolidays.end();
}

/**
 * @brief Initialise la liste par défaut (incluant Pâques de l'année donnée)
 */
void initJoursFeriesDefaut(int year) {
    holidayList = "";
    for (int i = 0; i < defaultHolidaysCount; i++) {
        if (i > 0) holidayList += ",";
        holidayList += String(defaultHolidays[i]);
    }
    String paques = calculerPaques(year);
    if (!holidayExists(paques)) ajouterJourFerie(paques);
    holidayCurrentYear = year;
    sauvegarderJoursFeries();
    LOG_INFO("HOLIDAYS", "Jours fériés par défaut initialisés (année %d)", year);
}

/**
 * @brief Vérifie si l'année a changé (au 1er janvier) et met à jour la liste
 *        (supprime les dates avec année spécifique obsolète, ajoute Pâques)
 */
void verifierChangementAnnee(int anneeActuelle) {
    if (anneeActuelle <= 0 || holidayCurrentYear <= 0) return;
    if (anneeActuelle == holidayCurrentYear) return;
    LOG_INFO("HOLIDAYS", "Changement d'année %d → %d", holidayCurrentYear, anneeActuelle);

    String newList = "";
    int index = 0; bool first = true;
    while (index < (int)holidayList.length()) {
        int comma = holidayList.indexOf(',', index);
        if (comma == -1) comma = holidayList.length();
        String entry = holidayList.substring(index, comma);
        entry.trim();
        bool keep = true;
        int thirdSlash = entry.indexOf('/', 3);
        if (thirdSlash != -1) {
            int yearEntry = entry.substring(thirdSlash + 1).toInt();
            if (yearEntry != anneeActuelle) keep = false;
        }
        if (keep) {
            if (!first) newList += ",";
            newList += entry;
            first = false;
        }
        index = comma + 1;
    }
    holidayList = newList;
    holidayCurrentYear = anneeActuelle;
    String paques = calculerPaques(anneeActuelle);
    if (!holidayExists(paques)) ajouterJourFerie(paques);
    sauvegarderJoursFeries();
}

/**
 * @brief Détermine si aujourd'hui est un jour férié
 */
bool estJourFerie() {
    if (holidayList.length() == 0) return false;
    unsigned long ecoule = (millis() - millisDemarrage) / 1000;
    time_t maintenant = epochDemarrage + (time_t)ecoule;
    struct tm* t = localtime(&maintenant);
    char dateDuJour[6];
    sprintf(dateDuJour, "%02d/%02d", t->tm_mday, t->tm_mon + 1);
    return holidayExists(String(dateDuJour));
}

/**
 * @brief Retourne la liste des jours fériés au format JSON (pour l'API)
 */
String getHolidayArrayJson() {
    String json = "[";
    int index = 0; bool first = true;
    while (index < (int)holidayList.length()) {
        int comma = holidayList.indexOf(',', index);
        if (comma == -1) comma = holidayList.length();
        String current = holidayList.substring(index, comma);
        current.trim();
        if (current.length() > 0) {
            if (!first) json += ",";
            int star = current.indexOf('*');
            if (star != -1) json += "\"" + current.substring(0, star) + "\"";
            else json += "\"" + current + "\"";
            first = false;
        }
        index = comma + 1;
    }
    json += "]";
    return json;
}

// ============================================================
//  GESTION DE L'HEURE (NTP)
// ============================================================

/**
 * @brief Renvoie l'heure actuelle (0-23) si disponible, sinon -1
 */
int obtenirHeureActuelle() {
    if (heureDisponible) {
        unsigned long ecoule = (millis() - millisDemarrage) / 1000;
        time_t maintenant = epochDemarrage + (time_t)ecoule;
        struct tm* temps = localtime(&maintenant);
        return temps->tm_hour;
    }
    return -1;
}

/**
 * @brief Dit si on est en période de nuit (définie par HEURE_DEBUT_NUIT et HEURE_FIN_NUIT)
 *        Si l'heure NTP n'est pas dispo, on simule une alternance 16h jour / 8h nuit
 */
bool estModeNuit() {
    if (heureDisponible) {
        int h = obtenirHeureActuelle();
        return (h >= HEURE_DEBUT_NUIT || h < HEURE_FIN_NUIT);
    }
    // Fallback : cycle de 24h fictif (début à 0, 16h jour, 8h nuit)
    unsigned long dureeFonct = millis() - sante.heureDemarrage;
    unsigned long phase = dureeFonct % (16*3600*1000UL + 8*3600*1000UL);
    return (phase >= 16*3600*1000UL);
}

// ============================================================
//  NOTIFICATIONS TELEGRAM (avec anti-spam)
// ============================================================
static unsigned long derniereAlertNuit        = 0;
static unsigned long derniereAlerteFerie      = 0;
static unsigned long derniereAlerteAbsence    = 0;
static bool          absenceLongueSignalee    = false;
static unsigned long derniereMomentPresence   = 0;
static bool          presenceActive           = false;

/**
 * @brief Envoie un message texte simple via le bot Telegram
 */
void envoyerTelegram(const String& message) {
    if (WiFi.status() != WL_CONNECTED) {
        LOG_WARNING("TELEGRAM", "WiFi non connecté — message non envoyé");
        return;
    }
    WiFiClientSecure client;
    client.setInsecure();   // Accepte tout certificat (pour simplifier)
    HTTPClient http;
    String url = "https://api.telegram.org/bot";
    url += TELEGRAM_BOT_TOKEN;
    url += "/sendMessage";
    http.begin(client, url);
    http.addHeader("Content-Type", "application/x-www-form-urlencoded");
    String body = "chat_id=";
    body += TELEGRAM_CHAT_ID;
    body += "&text=";
    body += message;
    body += "&parse_mode=HTML";
    int code = http.POST(body);
    if (code == 200) {
        sante.notificationsTelegram++;
        LOG_INFO("TELEGRAM", "Message envoyé (total: %lu)", (unsigned long)sante.notificationsTelegram);
    } else {
        LOG_ERROR("TELEGRAM", "Échec envoi message (code: %d)", code);
    }
    http.end();
}

/**
 * @brief Envoie une photo avec légende via le bot Telegram (multipart/form-data)
 */
void envoyerTelegramPhoto(const uint8_t* imageData, size_t imageLen, const String& legende) {
    if (WiFi.status() != WL_CONNECTED || imageData == NULL || imageLen == 0) {
        envoyerTelegram(legende);
        return;
    }
    WiFiClientSecure client;
    client.setInsecure();
    String host = "api.telegram.org";
    if (!client.connect(host.c_str(), 443)) {
        LOG_ERROR("TELEGRAM", "Connexion HTTPS échouée pour photo");
        envoyerTelegram(legende);
        return;
    }

    String boundary = "----ESP32CamBoundary";
    String bodyStart = "--" + boundary + "\r\n";
    bodyStart += "Content-Disposition: form-data; name=\"chat_id\"\r\n\r\n";
    bodyStart += String(TELEGRAM_CHAT_ID) + "\r\n";
    bodyStart += "--" + boundary + "\r\n";
    bodyStart += "Content-Disposition: form-data; name=\"caption\"\r\n\r\n";
    bodyStart += legende + "\r\n";
    bodyStart += "--" + boundary + "\r\n";
    bodyStart += "Content-Disposition: form-data; name=\"photo\"; filename=\"esp32cam.jpg\"\r\n";
    bodyStart += "Content-Type: image/jpeg\r\n\r\n";
    String bodyEnd = "\r\n--" + boundary + "--\r\n";

    int totalLen = bodyStart.length() + imageLen + bodyEnd.length();

    String request = "POST /bot";
    request += TELEGRAM_BOT_TOKEN;
    request += "/sendPhoto HTTP/1.1\r\n";
    request += "Host: api.telegram.org\r\n";
    request += "Content-Type: multipart/form-data; boundary=" + boundary + "\r\n";
    request += "Content-Length: " + String(totalLen) + "\r\n";
    request += "Connection: close\r\n\r\n";

    client.print(request);
    client.print(bodyStart);

    size_t sent = 0;
    while (sent < imageLen) {
        size_t chunk = min((size_t)1024, imageLen - sent);
        client.write(imageData + sent, chunk);
        sent += chunk;
    }
    client.print(bodyEnd);

    unsigned long debut = millis();
    while (client.connected() && millis() - debut < 5000) {
        if (client.available()) {
            String line = client.readStringUntil('\n');
            if (line.indexOf("200 OK") != -1) {
                sante.notificationsTelegram++;
                LOG_INFO("TELEGRAM", "Photo envoyée (total: %lu)", (unsigned long)sante.notificationsTelegram);
                break;
            }
        }
    }
    client.stop();
}

/**
 * @brief Logique centrale des alertes : nuit / jour férié / retour après absence
 *        Gère l'anti-spam et l'envoi photo si nécessaire
 */
void gererNotificationsTelegram(bool personneDetectee, const uint8_t* imageData, size_t imageLen) {
    if (!heureDisponible) return;

    unsigned long maintenant = millis();
    bool nuit   = estModeNuit();
    bool ferie  = estJourFerie();

    char heureStr[9] = "??:??:??";
    if (heureDisponible) {
        unsigned long ecoule = (maintenant - millisDemarrage) / 1000;
        time_t ts = epochDemarrage + (time_t)ecoule;
        struct tm* t = localtime(&ts);
        sprintf(heureStr, "%02d:%02d:%02d", t->tm_hour, t->tm_min, t->tm_sec);
    }

    // Alerte nocturne (détection pendant la plage nuit)
    if (personneDetectee && nuit) {
        if (maintenant - derniereAlertNuit >= ANTI_SPAM_NUIT_MS) {
            derniereAlertNuit = maintenant;
            String msg = "⚠️ <b>Présence détectée — Heure inhabituelle</b>\n";
            msg += "🕐 Heure : " + String(heureStr) + "\n";
            msg += "🌙 Plage nuit active (" + String(HEURE_DEBUT_NUIT) + "h → " + String(HEURE_FIN_NUIT) + "h)";
            LOG_WARNING("TELEGRAM", "Alerte nuit envoyée");
            envoyerTelegramPhoto(imageData, imageLen, msg);
        }
    }

    // Alerte jour férié
    if (personneDetectee && ferie) {
        if (maintenant - derniereAlerteFerie >= ANTI_SPAM_NUIT_MS) {
            derniereAlerteFerie = maintenant;
            unsigned long ecoule = (maintenant - millisDemarrage) / 1000;
            time_t ts = epochDemarrage + (time_t)ecoule;
            struct tm* t = localtime(&ts);
            char dateDuJour[6];
            sprintf(dateDuJour, "%02d/%02d", t->tm_mday, t->tm_mon + 1);
            String msg = "🚨 <b>Présence détectée un jour férié !</b>\n";
            msg += "📅 Date : " + String(dateDuJour) + "\n";
            msg += "🕐 Heure : " + String(heureStr) + "\n";
            msg += "⚠️ Présence non autorisée aujourd'hui";
            LOG_WARNING("TELEGRAM", "Alerte jour férié envoyée");
            envoyerTelegramPhoto(imageData, imageLen, msg);
        }
    }

    // Détection de retour après longue absence (uniquement en journée)
    if (!nuit) {
        if (personneDetectee) {
            if (!presenceActive && derniereMomentPresence > 0) {
                unsigned long dureeAbsence = maintenant - derniereMomentPresence;
                if (dureeAbsence >= DELAI_ABSENCE_MS && !absenceLongueSignalee) {
                    if (maintenant - derniereAlerteAbsence >= ANTI_SPAM_ABSENCE_MS) {
                        derniereAlerteAbsence = maintenant;
                        absenceLongueSignalee = true;
                        unsigned long heuresAbs = dureeAbsence / 3600000UL;
                        unsigned long minutesAbs = (dureeAbsence % 3600000UL) / 60000UL;
                        String msg = ferie
                            ? "🚨 <b>Présence détectée — Jour férié !</b>\n"
                            : "👋 <b>Personne présente dans le lab</b>\n";
                        msg += "⏱️ Après " + String(heuresAbs) + "h" + String(minutesAbs) + "min d'absence\n";
                        msg += "🕐 Heure : " + String(heureStr);
                        if (ferie) msg += "\n⚠️ Présence non autorisée aujourd'hui";
                        LOG_INFO("TELEGRAM", "Alerte retour après absence envoyée (%luh%lumin)", heuresAbs, minutesAbs);
                        envoyerTelegramPhoto(imageData, imageLen, msg);
                    }
                }
            }
            presenceActive = true;
            derniereMomentPresence = maintenant;
            absenceLongueSignalee = false;
        } else {
            presenceActive = false;
        }
    }
}

// ============================================================
//  BUFFERS PARTAGÉS ET SÉMAPHORES (multitâche)
// ============================================================
uint8_t* tamponFlux    = NULL;   // Image JPEG pour le streaming
size_t   longueurFlux  = 0;
SemaphoreHandle_t mutexFlux = NULL;

uint8_t* tamponRgbIa   = NULL;   // Image au format RGB888 pour l'IA
SemaphoreHandle_t semRequeteIa  = NULL;   // Signal pour demander une conversion
SemaphoreHandle_t semPretIa     = NULL;   // Signal indiquant que l'image RGB est prête

SemaphoreHandle_t mutexDetection = NULL;  // Protection de labelDetection
SemaphoreHandle_t mutexLumiere   = NULL;  // Protection lumiereAllumee/chrono
SemaphoreHandle_t mutexServo     = NULL;
SemaphoreHandle_t mutexRelais    = NULL;
SemaphoreHandle_t mutexClient    = NULL;  // Indique si un client regarde le flux
SemaphoreHandle_t mutexTelegram  = NULL;  // Évite chevauchement d'envois

volatile bool clientConnecte = false;

char  labelDetection[64] = "En attente...";
float scoreDetection = 0.0f;

// Historique des détections (pour validation temporelle)
struct DetectionRecord {
    unsigned long timestamp;
    bool estValide;
};
DetectionRecord historiqueDetections[HISTORIQUE_MAX];
int indexHistorique = 0;
int tailleHistorique = 0;

bool lumiereAllumee = false;
unsigned long derniereDetection = 0;
bool chronoActif = false;

uint8_t* tamponSnapshot   = NULL;   // Copie de la dernière image pour les alertes
size_t   longueurSnapshot = 0;
SemaphoreHandle_t mutexSnapshot = NULL;

Servo monServo;
int angleServo  = 0;
int angleManuel = -1;        // -1 = pas de consigne manuelle
static bool servoRetour        = false;
static int  compteurCyclesServo = 0;
static bool servoModeNuit       = false;

int compteurErreursCam = 0;
int compteurReinitCam  = 0;

httpd_handle_t serveur80 = NULL, serveur81 = NULL;

// ============================================================
//  CONFIGURATION DE LA CAMÉRA
// ============================================================
static camera_config_t configCamera = {
    .pin_pwdn  = PWDN_GPIO_NUM,  .pin_reset   = RESET_GPIO_NUM, .pin_xclk = XCLK_GPIO_NUM,
    .pin_sscb_sda = SIOD_GPIO_NUM, .pin_sscb_scl = SIOC_GPIO_NUM,
    .pin_d7 = Y9_GPIO_NUM, .pin_d6 = Y8_GPIO_NUM, .pin_d5 = Y7_GPIO_NUM, .pin_d4 = Y6_GPIO_NUM,
    .pin_d3 = Y5_GPIO_NUM, .pin_d2 = Y4_GPIO_NUM, .pin_d1 = Y3_GPIO_NUM, .pin_d0 = Y2_GPIO_NUM,
    .pin_vsync = VSYNC_GPIO_NUM, .pin_href = HREF_GPIO_NUM, .pin_pclk = PCLK_GPIO_NUM,
    .xclk_freq_hz  = 20000000,
    .ledc_timer    = LEDC_TIMER_0,
    .ledc_channel  = LEDC_CHANNEL_0,
    .pixel_format  = PIXFORMAT_JPEG,
    .frame_size    = FRAMESIZE_QVGA,   // 320x240
    .jpeg_quality  = 12,               // Qualité JPEG (0-63, plus bas = meilleur)
    .fb_count      = 1,
    .fb_location   = CAMERA_FB_IN_PSRAM,
    .grab_mode     = CAMERA_GRAB_WHEN_EMPTY,
};

// ============================================================
//  SURVEILLANCE MÉMOIRE
// ============================================================
void verifierTas() {
    uint32_t libre = esp_get_free_heap_size();
    if (libre < sante.heapMin) sante.heapMin = libre;
    if (libre < SEUIL_AVERTISSEMENT_HEAP) LOG_WARNING("MEM", "Tas faible : %lu octets !", (unsigned long)libre);
}

// ============================================================
//  LECTURE DU CAPTEUR LDR (binaire : lampe allumée ou éteinte)
// ============================================================
int  lireLDR()                  { return (digitalRead(BROCHE_LDR) == HIGH) ? 4095 : 0; }
bool estLumiereAllumeePhysique(){ return (lireLDR() > SEUIL_LDR); }

// ============================================================
//  BASCULEMENT DU RELAIS (avec anti-rebond)
// ============================================================
static unsigned long dernierBasculementRelais = 0;
bool basculerRelais() {
    if (xSemaphoreTake(mutexRelais, pdMS_TO_TICKS(1000)) != pdTRUE) { sante.timeoutsMutex++; return false; }
    unsigned long maintenant = millis();
    if (maintenant - dernierBasculementRelais < ANTI_REBOND_RELAIS_MS) { xSemaphoreGive(mutexRelais); return false; }
    digitalWrite(BROCHE_RELAIS, !digitalRead(BROCHE_RELAIS));
    dernierBasculementRelais = maintenant;
    sante.basculementsRelais++;
    xSemaphoreGive(mutexRelais);
    vTaskDelay(pdMS_TO_TICKS(500)); // Laisser le temps au relais de s'établir
    return true;
}

/**
 * @brief Allume la lampe (si elle ne l'est pas déjà) en basculant le relais.
 *        Met à jour l'état logique lumiereAllumee.
 */
void allumerLumiere() {
    if (estLumiereAllumeePhysique()) {
        if (xSemaphoreTake(mutexLumiere, pdMS_TO_TICKS(200)) == pdTRUE) { lumiereAllumee = true; xSemaphoreGive(mutexLumiere); }
        else sante.timeoutsMutex++;
        return;
    }
    if (!basculerRelais()) { LOG_ERROR("ALLUMER", "Basculement refusé"); return; }
    if (estLumiereAllumeePhysique()) {
        if (xSemaphoreTake(mutexLumiere, pdMS_TO_TICKS(200)) == pdTRUE) { lumiereAllumee = true; xSemaphoreGive(mutexLumiere); }
        else sante.timeoutsMutex++;
        sante.compteurAllumages++;
        LOG_INFO("ALLUMER", "Succès");
    } else {
        // Si la lampe ne s'est pas allumée, on rebascule pour revenir à l'état initial
        basculerRelais();
        if (xSemaphoreTake(mutexLumiere, pdMS_TO_TICKS(200)) == pdTRUE) { lumiereAllumee = false; xSemaphoreGive(mutexLumiere); }
        else sante.timeoutsMutex++;
        LOG_ERROR("ALLUMER", "Échec LDR");
    }
}

/**
 * @brief Éteint la lampe (si elle est allumée)
 */
void eteindreLumiere() {
    if (!estLumiereAllumeePhysique()) {
        if (xSemaphoreTake(mutexLumiere, pdMS_TO_TICKS(200)) == pdTRUE) { lumiereAllumee = false; chronoActif = false; xSemaphoreGive(mutexLumiere); }
        else sante.timeoutsMutex++;
        return;
    }
    if (!basculerRelais()) { LOG_ERROR("ETEINDRE", "Basculement refusé"); return; }
    if (!estLumiereAllumeePhysique()) {
        if (xSemaphoreTake(mutexLumiere, pdMS_TO_TICKS(200)) == pdTRUE) { lumiereAllumee = false; chronoActif = false; xSemaphoreGive(mutexLumiere); }
        else sante.timeoutsMutex++;
        LOG_INFO("ETEINDRE", "Succès");
    } else {
        basculerRelais();
        if (xSemaphoreTake(mutexLumiere, pdMS_TO_TICKS(200)) == pdTRUE) { lumiereAllumee = true; xSemaphoreGive(mutexLumiere); }
        else sante.timeoutsMutex++;
        LOG_ERROR("ETEINDRE", "Échec LDR reste élevé");
    }
}

/**
 * @brief Décide d'allumer la lampe si une personne est détectée,
 *        et réinitialise le timer d'extinction.
 *        Le reset du timer peut être désactivé en fonction du seuil de fiabilité.
 */
void gererLumiere(bool personneDetectee, bool resetTimerAutorise = true) {
    bool physique = estLumiereAllumeePhysique();
    if (xSemaphoreTake(mutexLumiere, pdMS_TO_TICKS(200)) == pdTRUE) { lumiereAllumee = physique; xSemaphoreGive(mutexLumiere); }
    else sante.timeoutsMutex++;

    if (personneDetectee) {
        sante.compteurDetections++;
        if (resetTimerAutorise) {
            if (xSemaphoreTake(mutexLumiere, pdMS_TO_TICKS(200)) == pdTRUE) {
                derniereDetection = millis();
                chronoActif = true;
                xSemaphoreGive(mutexLumiere);
            } else sante.timeoutsMutex++;
        }
        if (!physique) allumerLumiere();
    }
}

/**
 * @brief Gère le chrono d'extinction automatique.
 *        Si mode manuel, vérifie la durée avant de repasser en auto.
 *        Sinon, éteint après DUREE_EXTINCTION_MS sans détection.
 */
void gererChronoLumiere() {
    bool physique = estLumiereAllumeePhysique();
    bool actifChrono = false; unsigned long dernier = 0;
    if (xSemaphoreTake(mutexLumiere, pdMS_TO_TICKS(200)) == pdTRUE) {
        actifChrono = chronoActif;
        dernier = derniereDetection;
        xSemaphoreGive(mutexLumiere);
    } else { sante.timeoutsMutex++; return; }

    // Si la lampe est allumée sans que le chrono soit actif, on l'active (cas d'allumage manuel)
    if (physique && !actifChrono) {
        if (xSemaphoreTake(mutexLumiere, pdMS_TO_TICKS(200)) == pdTRUE) {
            derniereDetection = millis();
            chronoActif = true;
            xSemaphoreGive(mutexLumiere);
        }
        actifChrono = true;
        dernier = millis();
    }
    if (!physique && actifChrono) {
        if (xSemaphoreTake(mutexLumiere, pdMS_TO_TICKS(200)) == pdTRUE) { chronoActif = false; xSemaphoreGive(mutexLumiere); }
    }

    // Mode manuel : retour automatique après délai si aucun client connecté
    if (modeManuel) {
        unsigned long ecoule = millis() - debutModeManuel;
        bool clientPresent = false;
        if (xSemaphoreTake(mutexClient, pdMS_TO_TICKS(200)) == pdTRUE) { clientPresent = clientConnecte; xSemaphoreGive(mutexClient); }
        if (!clientPresent && ecoule >= DUREE_MANUEL_MS) {
            modeManuel = false;
            sauvegarderEtatNVS();
        }
        return;
    }

    // Extinction automatique
    if (actifChrono && physique) {
        if (millis() - dernier >= DUREE_EXTINCTION_MS) eteindreLumiere();
    }
    if (actifChrono && !physique) {
        if (xSemaphoreTake(mutexLumiere, pdMS_TO_TICKS(200)) == pdTRUE) { chronoActif = false; xSemaphoreGive(mutexLumiere); }
    }

    verifierTas();
}

// ============================================================
//  RÉINITIALISATION DE LA CAMÉRA (en cas d'erreur)
// ============================================================
bool reinitialiserCamera() {
    esp_camera_deinit();
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_err_t err = esp_camera_init(&configCamera);
    if (err != ESP_OK) {
        sante.reinitialisationsCamera++;
        compteurReinitCam++;
        return false;
    }
    compteurErreursCam = 0;
    compteurReinitCam = 0;
    sante.reinitialisationsCamera++;
    return true;
}

// ============================================================
//  GESTION AUTOMATIQUE DU SERVO (va-et-vient)
// ============================================================
void gererServoAuto() {
    // La nuit, on bloque le servo en position 0° pour économiser
    if (estModeNuit()) {
        if (!servoModeNuit) {
            servoModeNuit = true;
            if (xSemaphoreTake(mutexServo, pdMS_TO_TICKS(100)) == pdTRUE) { angleServo = 0; xSemaphoreGive(mutexServo); }
            monServo.write(0);
        }
        return;
    }

    if (servoModeNuit) {
        servoModeNuit = false;
        servoRetour = false;
    }

    if (++compteurCyclesServo < TOUS_LES_CYCLES) return;
    compteurCyclesServo = 0;

    int angleActuel = 0;
    if (xSemaphoreTake(mutexServo, pdMS_TO_TICKS(100)) == pdTRUE) { angleActuel = angleServo; xSemaphoreGive(mutexServo); }
    else { sante.timeoutsMutex++; return; }

    int nouvelAngle = angleActuel;
    if (angleActuel >= ANGLE_MAX_SERVO) servoRetour = true;
    if (servoRetour) {
        nouvelAngle = max(0, angleActuel - PAS_RETOUR_SERVO);
        if (nouvelAngle <= 0) servoRetour = false;
    } else {
        nouvelAngle = min(ANGLE_MAX_SERVO, angleActuel + PAS_SERVO);
    }

    if (xSemaphoreTake(mutexServo, pdMS_TO_TICKS(100)) == pdTRUE) { angleServo = nouvelAngle; xSemaphoreGive(mutexServo); }
    monServo.write(nouvelAngle);
}

// ============================================================
//  HISTORIQUE DES DÉTECTIONS (pour validation)
// ============================================================
int compterDetectionsRecentes(unsigned long fenetre_ms) {
    unsigned long maintenant = millis();
    int compteur = 0;
    for (int i = 0; i < tailleHistorique; i++)
        if (maintenant - historiqueDetections[i].timestamp <= fenetre_ms && historiqueDetections[i].estValide)
            compteur++;
    return compteur;
}

// ============================================================
//  SYNCHRONISATION NTP
// ============================================================
bool synchroniserHeureNTP() {
    configTime(DECALAGE_HORAIRE_SEC, DECALAGE_ETE_SEC, SERVEUR_NTP);
    struct tm infoTemps;
    unsigned long debut = millis();
    while (millis() - debut < 15000) {
        if (getLocalTime(&infoTemps)) {
            epochDemarrage  = mktime(&infoTemps);
            millisDemarrage = millis();
            heureDisponible = true;
            holidayCurrentYear = infoTemps.tm_year + 1900;
            LOG_INFO("NTP", "Heure : %02d:%02d:%02d — Année %d", infoTemps.tm_hour, infoTemps.tm_min, infoTemps.tm_sec, holidayCurrentYear);
            return true;
        }
        delay(500);
    }
    return false;
}

// ============================================================
//  HANDLERS HTTP (API REST, pages web)
// ============================================================

static esp_err_t gestionAllumerLumiere(httpd_req_t *req) {
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    modeManuel = true; debutModeManuel = millis();
    allumerLumiere(); sauvegarderEtatNVS();
    httpd_resp_send(req, "OK", 2); return ESP_OK;
}

static esp_err_t gestionEteindreLumiere(httpd_req_t *req) {
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    modeManuel = true; debutModeManuel = millis();
    eteindreLumiere(); sauvegarderEtatNVS();
    httpd_resp_send(req, "OK", 2); return ESP_OK;
}

static esp_err_t gestionModeAuto(httpd_req_t *req) {
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    modeManuel = false; sauvegarderEtatNVS();
    if (estLumiereAllumeePhysique()) {
        if (xSemaphoreTake(mutexLumiere, pdMS_TO_TICKS(200)) == pdTRUE) { derniereDetection = millis(); chronoActif = true; xSemaphoreGive(mutexLumiere); }
    }
    httpd_resp_send(req, "OK", 2); return ESP_OK;
}

static esp_err_t gestionServoGauche(httpd_req_t *req) {
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    modeManuel = true; debutModeManuel = millis();
    int a = 0;
    if (xSemaphoreTake(mutexServo, pdMS_TO_TICKS(200)) == pdTRUE) { a = max(0, angleServo - PAS_SERVO_WEB); angleManuel = a; xSemaphoreGive(mutexServo); }
    char buf[8]; snprintf(buf, sizeof(buf), "%d", a);
    httpd_resp_send(req, buf, strlen(buf)); return ESP_OK;
}

static esp_err_t gestionServoDroite(httpd_req_t *req) {
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    modeManuel = true; debutModeManuel = millis();
    int a = 0;
    if (xSemaphoreTake(mutexServo, pdMS_TO_TICKS(200)) == pdTRUE) { a = min(180, angleServo + PAS_SERVO_WEB); angleManuel = a; xSemaphoreGive(mutexServo); }
    char buf[8]; snprintf(buf, sizeof(buf), "%d", a);
    httpd_resp_send(req, buf, strlen(buf)); return ESP_OK;
}

static esp_err_t gestionStatut(httpd_req_t *req) {
    char buf[320];
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache, no-store, must-revalidate");

    char label[64] = "En attente...";
    if (xSemaphoreTake(mutexDetection, pdMS_TO_TICKS(200)) == pdTRUE) { strncpy(label, labelDetection, 63); xSemaphoreGive(mutexDetection); }

    bool lumPhys = false, chrono = false; unsigned long dernier = 0;
    if (xSemaphoreTake(mutexLumiere, pdMS_TO_TICKS(200)) == pdTRUE) { lumPhys = lumiereAllumee; chrono = chronoActif; dernier = derniereDetection; xSemaphoreGive(mutexLumiere); }

    unsigned long secondesRestantes = 0;
    if (chrono && lumPhys) {
        unsigned long e = millis() - dernier;
        if (e < DUREE_EXTINCTION_MS) secondesRestantes = (DUREE_EXTINCTION_MS - e) / 1000;
    }

    int angle = 0;
    if (xSemaphoreTake(mutexServo, pdMS_TO_TICKS(100)) == pdTRUE) { angle = angleServo; xSemaphoreGive(mutexServo); }

    char heureStr[12] = "??:??";
    if (heureDisponible) {
        unsigned long e = (millis() - millisDemarrage) / 1000;
        time_t n = epochDemarrage + e;
        struct tm* t = localtime(&n);
        snprintf(heureStr, 12, "%02d:%02d", t->tm_hour, t->tm_min);
    }

    snprintf(buf, sizeof(buf), "%s|%s|%lu|%s|%d|%s|%s|%s|%s",
             label, lumPhys ? "ON" : "OFF", secondesRestantes,
             modeManuel ? "MANUEL" : "AUTO", angle,
             estModeNuit() ? "NUIT" : "JOUR", heureStr,
             estJourFerie() ? "FERIE" : "NORMAL",
             holidayList.length() > 0 ? "true" : "false");
    httpd_resp_send(req, buf, strlen(buf));
    return ESP_OK;
}

static esp_err_t gestionSante(httpd_req_t *req) {
    char buf[900];
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    unsigned long uptime = (millis() - sante.heureDemarrage) / 1000;
    uint32_t tasLibre = esp_get_free_heap_size();
    int angle = 0;
    if (xSemaphoreTake(mutexServo, pdMS_TO_TICKS(100)) == pdTRUE) { angle = angleServo; xSemaphoreGive(mutexServo); }

    char heureStr[12] = "Non dispo";
    if (heureDisponible) {
        unsigned long e = (millis() - millisDemarrage) / 1000;
        time_t n = epochDemarrage + e;
        struct tm* t = localtime(&n);
        snprintf(heureStr, 12, "%02d:%02d:%02d", t->tm_hour, t->tm_min, t->tm_sec);
    }

    snprintf(buf, sizeof(buf),
        "Uptime          : %luh %lum %lus\n"
        "Heure actuelle  : %s\n"
        "Heap libre      : %lu octets\n"
        "Heap minimum    : %lu octets\n"
        "Redémarrages    : %lu\n"
        "Erreurs caméra  : %lu\n"
        "Réinit caméra   : %lu\n"
        "Détections      : %lu\n"
        "Allumages       : %lu\n"
        "Basculements    : %lu\n"
        "Timeouts mutex  : %lu\n"
        "Notif. Telegram : %lu\n"
        "Mode            : %s\n"
        "Jour férié      : %s\n"
        "Mode nuit       : %s\n"
        "Servo angle     : %d deg\n"
        "Lumière physique: %s\n"
        "--- Réglages ---\n"
        "Délai IA        : %d ms\n"
        "Délai absence   : %lu min\n"
        "Heure NTP       : %s\n",
        uptime / 3600, (uptime % 3600) / 60, uptime % 60, heureStr,
        (unsigned long)tasLibre, (unsigned long)((sante.heapMin == UINT32_MAX) ? tasLibre : sante.heapMin),
        (unsigned long)sante.redemarrages, (unsigned long)sante.erreursCamera, (unsigned long)sante.reinitialisationsCamera,
        (unsigned long)sante.compteurDetections, (unsigned long)sante.compteurAllumages, (unsigned long)sante.basculementsRelais,
        (unsigned long)sante.timeoutsMutex, (unsigned long)sante.notificationsTelegram,
        modeManuel ? "MANUEL" : "AUTO", estJourFerie() ? "OUI" : "NON",
        estModeNuit() ? "OUI" : "NON", angle,
        estLumiereAllumeePhysique() ? "ALLUMEE" : "ETEINTE",
        DELAI_IA_MS, DELAI_ABSENCE_MS / 60000UL,
        heureDisponible ? "OK" : "NON");
    httpd_resp_send(req, buf, strlen(buf));
    return ESP_OK;
}

static esp_err_t gestionApiTime(httpd_req_t *req) {
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    char buf[80];
    if (heureDisponible) {
        unsigned long e = (millis() - millisDemarrage) / 1000;
        time_t n = epochDemarrage + e;
        struct tm* t = localtime(&n);
        snprintf(buf, sizeof(buf),
            "{\"time\":\"%02d/%02d/%04d %02d:%02d:%02d\",\"ferie\":%s}",
            t->tm_mday, t->tm_mon + 1, t->tm_year + 1900,
            t->tm_hour, t->tm_min, t->tm_sec,
            estJourFerie() ? "true" : "false");
    } else {
        snprintf(buf, sizeof(buf), "{\"time\":\"--/--/---- --:--:--\",\"ferie\":false}");
    }
    httpd_resp_send(req, buf, strlen(buf));
    return ESP_OK;
}

static esp_err_t gestionApiHolidaysGet(httpd_req_t *req) {
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    String json = getHolidayArrayJson();
    httpd_resp_send(req, json.c_str(), json.length());
    return ESP_OK;
}

static esp_err_t gestionApiHolidaysAdd(httpd_req_t *req) {
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    char buf[128] = {0};
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0) {
        httpd_resp_send(req, "{\"success\":false,\"error\":\"Corps vide\"}", -1);
        return ESP_OK;
    }
    String body = String(buf);
    String date = "", recurringStr = "true";
    int di = body.indexOf("date=");
    if (di != -1) {
        int end = body.indexOf('&', di + 5);
        date = (end == -1) ? body.substring(di + 5) : body.substring(di + 5, end);
        date.replace("%2F", "/");
        date.trim();
    }
    int ri = body.indexOf("recurring=");
    if (ri != -1) recurringStr = body.substring(ri + 10, body.indexOf('&', ri + 10));

    if (date.length() != 5 || date.charAt(2) != '/') {
        httpd_resp_send(req, "{\"success\":false,\"error\":\"Format invalide (JJ/MM)\"}", -1);
        return ESP_OK;
    }
    if (holidayExists(date)) {
        httpd_resp_send(req, "{\"success\":false,\"error\":\"Ce jour ferie existe deja\"}", -1);
        return ESP_OK;
    }
    bool isRecurring = (recurringStr == "true");
    String storageDate = date;
    if (!isRecurring && holidayCurrentYear > 0) {
        storageDate = date + "*" + date + "/" + String(holidayCurrentYear);
    }
    ajouterJourFerie(storageDate);
    sauvegarderJoursFeries();
    String json = "{\"success\":true,\"message\":\"Jour ferie ajoute\",\"holidays\":" + getHolidayArrayJson() + "}";
    httpd_resp_send(req, json.c_str(), json.length());
    return ESP_OK;
}

static esp_err_t gestionApiHolidaysDelete(httpd_req_t *req) {
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    char buf[64] = {0};
    httpd_req_recv(req, buf, sizeof(buf) - 1);
    String body = String(buf);
    String date = "";
    int di = body.indexOf("date=");
    if (di != -1) { date = body.substring(di + 5); date.replace("%2F", "/"); date.trim(); }
    if (!holidayExists(date)) {
        httpd_resp_send(req, "{\"success\":false,\"error\":\"Jour ferie non trouve\"}", -1);
        return ESP_OK;
    }
    supprimerJourFerie(date);
    sauvegarderJoursFeries();
    String json = "{\"success\":true,\"message\":\"Jour ferie supprime\",\"holidays\":" + getHolidayArrayJson() + "}";
    httpd_resp_send(req, json.c_str(), json.length());
    return ESP_OK;
}

// ============================================================
//  PAGE WEB PRINCIPALE (HTML/CSS/JS) – interface utilisateur
// ============================================================
static const char index_html[] =
"<!DOCTYPE html>\n"
"<html>\n"
"<head>\n"
"<meta charset=\"UTF-8\">\n"
"<title>ESP32-CAM Détection</title>\n"
"<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">\n"
"<style>\n"
"*{box-sizing:border-box;margin:0;padding:0;}\n"
"body{font-family:Arial;background:#1c1c1c;color:#fff;padding:16px;text-align:center;}\n"
"h1{color:#0f0;font-size:18px;margin-bottom:14px;}\n"
"img{max-width:100%;border:2px solid #0f0;border-radius:6px;display:block;margin:0 auto 12px;}\n"
"#result-box{display:block;padding:10px 20px;border-radius:6px;font-size:1.1em;font-weight:bold;margin:0 auto 10px;max-width:480px;}\n"
"#result-box.oui{background:#1a5c1a;border:2px solid #0f0;color:#0f0;}\n"
"#result-box.non{background:#5c1a1a;border:2px solid #f44;color:#f44;}\n"
"#result-box.attente{background:#333;border:2px solid #888;color:#aaa;}\n"
".badge-row{display:flex;justify-content:space-between;max-width:480px;margin:0 auto 10px;flex-wrap:wrap;gap:6px;}\n"
".badge{display:inline-flex;align-items:center;gap:6px;padding:5px 12px;border-radius:20px;font-size:12px;font-weight:bold;}\n"
".badge-light-on{background:#3d3d00;border:1.5px solid #ff0;color:#ff0;}\n"
".badge-light-off{background:#222;border:1.5px solid #555;color:#555;}\n"
".badge-auto{background:#002244;border:1.5px solid #08f;color:#08f;}\n"
".badge-manual{background:#3d1a00;border:1.5px solid #f80;color:#f80;}\n"
".badge-nuit{background:#1a1a3d;border:1.5px solid #88f;color:#88f;}\n"
".badge-heure{background:#003300;border:1.5px solid #0f0;color:#0f0;}\n"
".badge-ferie{background:#3d0000;border:1.5px solid #f44;color:#f44;}\n"
".dot{width:8px;height:8px;border-radius:50%;display:inline-block;}\n"
".dot-on{background:#ff0;} .dot-off{background:#444;}\n"
".dot-auto{background:#08f;} .dot-manual{background:#f80;animation:blink 1s step-start infinite;}\n"
".dot-nuit{background:#88f;}\n"
"@keyframes blink{50%{opacity:0;}}\n"
".section{background:#252525;border-radius:8px;border:1px solid #444;padding:12px 14px;margin:8px auto;max-width:480px;}\n"
".sec-title{font-size:11px;color:#777;text-transform:uppercase;letter-spacing:1px;margin-bottom:10px;}\n"
".btn-row{display:flex;gap:8px;}\n"
".btn{flex:1;padding:10px 0;border-radius:6px;font-size:13px;font-weight:bold;border:none;cursor:pointer;}\n"
".btn-on{background:#1a5c1a;color:#0f0;border:2px solid #0f0;}\n"
".btn-off{background:#5c1a1a;color:#f44;border:2px solid #f44;}\n"
".btn-auto{background:#222;color:#08f;border:2px solid #08f;font-size:12px;width:100%;padding:8px 0;margin-top:8px;cursor:pointer;border-radius:6px;}\n"
".btn-holidays{background:#1a0033;color:#c8a0ff;border:2px solid #c8a0ff;font-size:12px;width:100%;padding:8px 0;margin-top:6px;cursor:pointer;border-radius:6px;text-decoration:none;display:block;}\n"
".btn-holidays:hover{background:#2a0044;}\n"
".btn-health{background:#222;color:#8f8;border:2px solid #8f8;font-size:11px;width:100%;padding:6px 0;margin-top:6px;cursor:pointer;border-radius:6px;}\n"
".info-row{display:flex;justify-content:space-between;font-size:11px;color:#666;margin-top:8px;}\n"
".servo-ctrl{display:flex;align-items:center;justify-content:center;gap:20px;margin:8px 0;}\n"
".arrow-btn{width:56px;height:56px;border-radius:50%;border:2px solid #0af;background:#002233;color:#0af;font-size:26px;cursor:pointer;display:flex;align-items:center;justify-content:center;}\n"
".arrow-btn:active{background:#0af;color:#000;}\n"
".angle-display{text-align:center;min-width:70px;}\n"
".angle-val{font-size:30px;font-weight:bold;color:#0af;display:block;}\n"
".angle-lbl{font-size:11px;color:#555;}\n"
".bar-bg{background:#333;border-radius:4px;height:6px;width:100%;margin-top:8px;}\n"
".bar-fill{background:#0af;border-radius:4px;height:6px;width:0%;transition:width 0.2s;}\n"
".bar-labels{display:flex;justify-content:space-between;font-size:10px;color:#555;margin-top:3px;}\n"
"#health-box{background:#111;border:1px solid #3a3a3a;border-radius:6px;padding:10px;text-align:left;font-family:monospace;font-size:11px;color:#8f8;white-space:pre;margin-top:8px;display:none;}\n"
"#update-info{color:#555;font-size:11px;margin-top:10px;}\n"
".night-banner{display:none;background:#1a1a3d;border:1px solid #88f;border-radius:6px;padding:8px;color:#88f;font-size:12px;margin:0 auto 8px;max-width:480px;}\n"
".ferie-banner{display:none;background:#3d0000;border:1px solid #f44;border-radius:6px;padding:8px;color:#f44;font-size:12px;margin:0 auto 8px;max-width:480px;}\n"
"</style>\n"
"</head>\n"
"<body>\n"
"<h1>ESP32-CAM — Détection de personne</h1>\n"
"<div class=\"night-banner\" id=\"night-banner\">🌙 Mode nuit actif — Servo en pause</div>\n"
"<div class=\"ferie-banner\" id=\"ferie-banner\">🚨 Aujourd'hui est un jour férié — Alertes Telegram actives</div>\n"
"<img id=\"streamImg\" src=\"\" />\n"
"<div id=\"result-box\" class=\"attente\">En attente...</div>\n"
"<div class=\"badge-row\">\n"
"<span class=\"badge badge-light-off\" id=\"light-badge\"><span class=\"dot dot-off\" id=\"light-dot\"></span><span id=\"light-text\">Lumière : ÉTEINTE</span></span>\n"
"<span class=\"badge badge-auto\" id=\"mode-badge\"><span class=\"dot dot-auto\" id=\"mode-dot\"></span><span id=\"mode-text\">Mode : AUTO</span></span>\n"
"<span class=\"badge badge-nuit\" id=\"night-badge\" style=\"display:none;\"><span class=\"dot dot-nuit\"></span><span>Nuit</span></span>\n"
"<span class=\"badge badge-ferie\" id=\"ferie-badge-inline\" style=\"display:none;\">🚨 Jour Férié</span>\n"
"<span class=\"badge badge-heure\" id=\"heure-badge\" style=\"display:none;\"><span id=\"heure-text\">--:--</span></span>\n"
"</div>\n"
"<div class=\"section\">\n"
"<div class=\"sec-title\">Contrôle lumière</div>\n"
"<div class=\"btn-row\">\n"
"<button class=\"btn btn-on\" onclick=\"setLight('on')\">Allumer</button>\n"
"<button class=\"btn btn-off\" onclick=\"setLight('off')\">Éteindre</button>\n"
"</div>\n"
"<button class=\"btn-auto\" onclick=\"setLight('auto')\">Repasser en mode automatique (IA)</button>\n"
"<a class=\"btn-holidays\" href=\"/holidays\" target=\"_blank\">📅 Gérer les Jours Fériés</a>\n"
"<div class=\"info-row\"><span id=\"timer-txt\"></span><span id=\"mode-info-txt\">Mode : AUTO</span></div>\n"
"</div>\n"
"<div class=\"section\">\n"
"<div class=\"sec-title\">Contrôle servo</div>\n"
"<div class=\"servo-ctrl\">\n"
"<button class=\"arrow-btn\" onclick=\"moveServo('left')\">⬅</button>\n"
"<div class=\"angle-display\"><span class=\"angle-val\" id=\"angle-val\">0°</span><span class=\"angle-lbl\">position</span></div>\n"
"<button class=\"arrow-btn\" onclick=\"moveServo('right')\">➡</button>\n"
"</div>\n"
"<div class=\"bar-bg\"><div class=\"bar-fill\" id=\"servo-bar\"></div></div>\n"
"<div class=\"bar-labels\"><span>0°</span><span>90°</span><span>180°</span></div>\n"
"</div>\n"
"<div class=\"section\">\n"
"<div class=\"sec-title\">Santé du système</div>\n"
"<button class=\"btn-health\" onclick=\"toggleHealth()\">Afficher / Masquer les diagnostics</button>\n"
"<div id=\"health-box\">Chargement...</div>\n"
"</div>\n"
"<p id=\"update-info\">Connexion en cours...</p>\n"
"<script>\n"
"var errorCount=0,healthVisible=false;\n"
"document.getElementById('streamImg').src='//'+window.location.hostname+':81/stream';\n"
"function updateBar(a){document.getElementById('angle-val').textContent=a+'°';document.getElementById('servo-bar').style.width=(a/180*100).toFixed(1)+'%';}\n"
"function setLight(a){fetch('/light/'+a,{cache:'no-store'}).catch(function(){});}\n"
"function moveServo(d){fetch('/servo/'+d,{cache:'no-store'}).then(function(r){return r.text();}).then(function(t){var a=parseInt(t);if(!isNaN(a))updateBar(a);}).catch(function(){});}\n"
"function toggleHealth(){healthVisible=!healthVisible;document.getElementById('health-box').style.display=healthVisible?'block':'none';if(healthVisible)fetchHealth();}\n"
"function fetchHealth(){if(!healthVisible)return;fetch('/health?'+Date.now(),{cache:'no-store'}).then(function(r){return r.text();}).then(function(t){document.getElementById('health-box').textContent=t;}).catch(function(){});}\n"
"function updateStatus(){\n"
"fetch('/status?'+Date.now(),{cache:'no-store'})\n"
".then(function(r){if(!r.ok)throw new Error('HTTP '+r.status);return r.text();})\n"
".then(function(data){\n"
"errorCount=0;\n"
"var p=data.trim().split('|');\n"
"var label=p[0]||'En attente...', light=p[1]||'OFF', seconds=parseInt(p[2])||0;\n"
"var mode=p[3]||'AUTO', angle=parseInt(p[4])||0, phase=p[5]||'JOUR';\n"
"var heure=p[6]||'??:??', jourFerie=p[7]||'NORMAL';\n"
"var box=document.getElementById('result-box');\n"
"box.textContent=label;\n"
"var ll=label.toLowerCase();\n"
"if(ll.indexOf('person')!==-1||ll.indexOf('personne')!==-1) box.className='oui';\n"
"else if(ll.indexOf('rien')!==-1||ll.indexOf('attente')!==-1) box.className='attente';\n"
"else box.className='non';\n"
"var lb=document.getElementById('light-badge'),ld=document.getElementById('light-dot'),lt=document.getElementById('light-text');\n"
"if(light==='ON'){lb.className='badge badge-light-on';ld.className='dot dot-on';lt.textContent='Lumière : ALLUMÉE';}\n"
"else{lb.className='badge badge-light-off';ld.className='dot dot-off';lt.textContent='Lumière : ÉTEINTE';}\n"
"var mb=document.getElementById('mode-badge'),md=document.getElementById('mode-dot'),mt=document.getElementById('mode-text');\n"
"if(mode==='MANUEL'){mb.className='badge badge-manual';md.className='dot dot-manual';mt.textContent='Mode : MANUEL';document.getElementById('mode-info-txt').textContent='Mode : Manuel';}\n"
"else{mb.className='badge badge-auto';md.className='dot dot-auto';mt.textContent='Mode : AUTO';document.getElementById('mode-info-txt').textContent='Mode : Auto (IA)';}\n"
"var nb=document.getElementById('night-badge'),nb2=document.getElementById('night-banner');\n"
"if(phase==='NUIT'){nb.style.display='inline-flex';nb2.style.display='block';}\n"
"else{nb.style.display='none';nb2.style.display='none';}\n"
"var fb=document.getElementById('ferie-badge-inline'),fb2=document.getElementById('ferie-banner');\n"
"if(jourFerie==='FERIE'){fb.style.display='inline-flex';fb2.style.display='block';}\n"
"else{fb.style.display='none';fb2.style.display='none';}\n"
"var hb=document.getElementById('heure-badge'),ht=document.getElementById('heure-text');\n"
"if(heure!=='??:??'){hb.style.display='inline-flex';ht.textContent=heure;}else{hb.style.display='none';}\n"
"var tb=document.getElementById('timer-txt');\n"
"if(light==='ON'&&seconds>0){var min=Math.floor(seconds/60),sec=seconds%60;tb.textContent='Extinction dans : '+min+'min '+sec+'s';}else{tb.textContent='';}\n"
"updateBar(angle);\n"
"var now=new Date();\n"
"document.getElementById('update-info').textContent='Dernière mise à jour : '+now.getHours().toString().padStart(2,'0')+':'+now.getMinutes().toString().padStart(2,'0')+':'+now.getSeconds().toString().padStart(2,'0');\n"
"if(healthVisible)fetchHealth();\n"
"})\n"
".catch(function(e){errorCount++;document.getElementById('update-info').textContent='Reconnexion... (tentative '+errorCount+')';});\n"
"}\n"
"updateStatus(); setInterval(updateStatus,1000);\n"
"</script>\n"
"</body>\n"
"</html>";

static esp_err_t gestionPageIndex(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_send(req, index_html, strlen(index_html));
    return ESP_OK;
}

// ============================================================
//  PAGE DE GESTION DES JOURS FÉRIÉS (HTML/JS)
// ============================================================
static const char holidays_html[] =
"<!DOCTYPE html>\n"
"<html lang=\"fr\">\n"
"<head>\n"
"<meta charset=\"UTF-8\">\n"
"<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">\n"
"<title>ESP32 - Gestion des Jours Feries</title>\n"
"<style>\n"
"*{margin:0;padding:0;box-sizing:border-box;}\n"
"body{font-family:'Segoe UI',Tahoma,Geneva,Verdana,sans-serif;background:linear-gradient(135deg,#1a1a2e 0%,#16213e 50%,#0f3460 100%);min-height:100vh;color:#e0e0e0;padding:20px;}\n"
".container{max-width:600px;margin:0 auto;}\n"
"h1{text-align:center;color:#00d4ff;font-size:1.8rem;margin-bottom:10px;text-shadow:0 0 10px rgba(0,212,255,0.3);}\n"
".subtitle{text-align:center;color:#8892b0;font-size:0.9rem;margin-bottom:25px;}\n"
".clock-container{background:rgba(255,255,255,0.05);border:1px solid rgba(0,212,255,0.2);border-radius:15px;padding:20px;text-align:center;margin-bottom:25px;backdrop-filter:blur(10px);}\n"
".clock-label{font-size:0.8rem;color:#8892b0;text-transform:uppercase;letter-spacing:2px;margin-bottom:8px;}\n"
".clock-time{font-size:2rem;font-weight:700;color:#00d4ff;font-family:'Courier New',monospace;text-shadow:0 0 15px rgba(0,212,255,0.4);}\n"
".mode-buttons{display:flex;gap:15px;margin-bottom:25px;}\n"
".mode-btn{flex:1;padding:15px 20px;border:none;border-radius:12px;font-size:1rem;font-weight:600;cursor:pointer;transition:all 0.3s ease;text-transform:uppercase;letter-spacing:1px;}\n"
".mode-btn.view{background:linear-gradient(135deg,#00d4ff,#0099cc);color:white;}\n"
".mode-btn.view:hover{transform:translateY(-2px);box-shadow:0 8px 25px rgba(0,212,255,0.4);}\n"
".mode-btn.edit{background:linear-gradient(135deg,#ff6b6b,#ee5a5a);color:white;}\n"
".mode-btn.edit:hover{transform:translateY(-2px);box-shadow:0 8px 25px rgba(255,107,107,0.4);}\n"
".mode-btn.active{box-shadow:0 0 20px rgba(255,255,255,0.2);transform:scale(1.02);}\n"
".message-area{min-height:50px;margin-bottom:20px;}\n"
".message{padding:12px 20px;border-radius:10px;font-size:0.95rem;font-weight:500;text-align:center;animation:slideIn 0.3s ease;display:none;}\n"
".message.show{display:block;}\n"
".message.success{background:rgba(46,204,113,0.2);border:1px solid rgba(46,204,113,0.5);color:#2ecc71;}\n"
".message.error{background:rgba(231,76,60,0.2);border:1px solid rgba(231,76,60,0.5);color:#e74c3c;}\n"
"@keyframes slideIn{from{opacity:0;transform:translateY(-10px);}to{opacity:1;transform:translateY(0);}}\n"
".holidays-section{background:rgba(255,255,255,0.03);border:1px solid rgba(255,255,255,0.1);border-radius:15px;padding:20px;margin-bottom:25px;min-height:200px;display:none;}\n"
".holidays-section.visible{display:block;animation:fadeIn 0.4s ease;}\n"
"@keyframes fadeIn{from{opacity:0;transform:translateY(-15px);}to{opacity:1;transform:translateY(0);}}\n"
".section-title{font-size:1.1rem;color:#ccd6f6;margin-bottom:15px;display:flex;align-items:center;gap:10px;}\n"
".section-title::before{content:'\\01F4C5';}\n"
".holidays-list{list-style:none;}\n"
".holiday-item{display:flex;justify-content:space-between;align-items:center;padding:12px 15px;margin-bottom:8px;background:rgba(255,255,255,0.05);border-radius:10px;border-left:3px solid #00d4ff;transition:all 0.2s ease;}\n"
".holiday-item:hover{background:rgba(255,255,255,0.08);transform:translateX(5px);}\n"
".holiday-date{font-size:1.1rem;font-weight:600;color:#e0e0e0;font-family:'Courier New',monospace;}\n"
".holiday-name{font-size:0.85rem;color:#8892b0;margin-top:2px;}\n"
".delete-btn{background:linear-gradient(135deg,#e74c3c,#c0392b);color:white;border:none;padding:8px 16px;border-radius:8px;cursor:pointer;font-size:0.85rem;font-weight:600;transition:all 0.2s ease;}\n"
".delete-btn:hover{transform:scale(1.05);box-shadow:0 4px 15px rgba(231,76,60,0.4);}\n"
".empty-state{text-align:center;padding:40px 20px;color:#8892b0;}\n"
".empty-state-icon{font-size:3rem;margin-bottom:10px;opacity:0.5;}\n"
".add-section{background:rgba(255,255,255,0.03);border:1px solid rgba(255,255,255,0.1);border-radius:15px;padding:20px;display:none;}\n"
".add-section.visible{display:block;animation:fadeIn 0.4s ease;}\n"
".add-title{font-size:1.1rem;color:#ccd6f6;margin-bottom:15px;display:flex;align-items:center;gap:10px;}\n"
".add-title::before{content:'\\2795';}\n"
".add-form{display:flex;gap:10px;align-items:stretch;margin-bottom:15px;}\n"
".date-input{flex:1;padding:14px 18px;border:2px solid rgba(255,255,255,0.1);border-radius:10px;background:rgba(255,255,255,0.05);color:#e0e0e0;font-size:1rem;font-family:'Courier New',monospace;transition:all 0.3s ease;}\n"
".date-input:focus{outline:none;border-color:#00d4ff;box-shadow:0 0 15px rgba(0,212,255,0.2);}\n"
".date-input::placeholder{color:#8892b0;}\n"
".add-btn{padding:14px 25px;background:linear-gradient(135deg,#2ecc71,#27ae60);color:white;border:none;border-radius:10px;font-size:1rem;font-weight:600;cursor:pointer;transition:all 0.3s ease;white-space:nowrap;}\n"
".add-btn:hover{transform:translateY(-2px);box-shadow:0 8px 25px rgba(46,204,113,0.4);}\n"
".add-btn:active{transform:translateY(0);}\n"
".recurring-options{display:flex;gap:20px;align-items:center;padding:10px 0;}\n"
".recurring-options label{display:flex;align-items:center;gap:8px;cursor:pointer;font-size:0.95rem;color:#ccd6f6;}\n"
".recurring-options input[type=\"radio\"]{width:18px;height:18px;accent-color:#00d4ff;cursor:pointer;}\n"
"@media(max-width:480px){body{padding:10px;}h1{font-size:1.4rem;}.clock-time{font-size:1.5rem;}.mode-buttons{flex-direction:column;}.add-form{flex-direction:column;}.add-btn{width:100%;}.recurring-options{flex-direction:column;align-items:flex-start;gap:10px;}}\n"
"</style>\n"
"</head>\n"
"<body>\n"
"<div class=\"container\">\n"
"<h1>🎉 Jours Feries</h1>\n"
"<p class=\"subtitle\">Gestionnaire de jours feries</p>\n"
"<div class=\"clock-container\">\n"
"<div class=\"clock-label\">Date et Heure Actuelles</div>\n"
"<div class=\"clock-time\" id=\"clock\">--/--/---- --:--:--</div>\n"
"</div>\n"
"<div class=\"mode-buttons\">\n"
"<button class=\"mode-btn view\" id=\"btnView\" onclick=\"toggleMode('view')\">👁️ Visualiser</button>\n"
"<button class=\"mode-btn edit\" id=\"btnEdit\" onclick=\"toggleMode('edit')\">✏️ Modifier</button>\n"
"</div>\n"
"<div class=\"message-area\">\n"
"<div class=\"message\" id=\"message\"></div>\n"
"</div>\n"
"<div class=\"holidays-section\" id=\"holidaysSection\">\n"
"<div class=\"section-title\" id=\"listTitle\">Liste des jours feries</div>\n"
"<ul class=\"holidays-list\" id=\"holidaysList\">\n"
"<li class=\"empty-state\">\n"
"<div class=\"empty-state-icon\">📭</div>\n"
"<div>Chargement...</div>\n"
"</li>\n"
"</ul>\n"
"</div>\n"
"<div class=\"add-section\" id=\"addSection\">\n"
"<div class=\"add-title\">Ajouter un jour ferie</div>\n"
"<div class=\"add-form\">\n"
"<input type=\"text\" class=\"date-input\" id=\"dateInput\" placeholder=\"JJ/MM (ex: 25/12)\" maxlength=\"5\" autocomplete=\"off\">\n"
"<button class=\"add-btn\" onclick=\"addHoliday()\">Ajouter</button>\n"
"</div>\n"
"<div class=\"recurring-options\">\n"
"<label><input type=\"radio\" name=\"recurring\" value=\"true\" checked><span>🔄 Tous les ans</span></label>\n"
"<label><input type=\"radio\" name=\"recurring\" value=\"false\"><span>📅 Annee en cours uniquement</span></label>\n"
"</div>\n"
"</div>\n"
"</div>\n"
"<script>\n"
"let currentMode=null;\n"
"let messageTimeout=null;\n"
"let holidaysData=[];\n"
"const holidayNames={\n"
"'01/01':'Jour de l\\'an',\n"
"'03/01':'Anniversaire du martyre de Thomas Sankara',\n"
"'08/03':'Journee internationale des droits des femmes',\n"
"'01/05':'Fete du Travail',\n"
"'15/08':'Assomption',\n"
"'01/11':'Toussaint',\n"
"'25/12':'Noel'\n"
"};\n"
"document.addEventListener('DOMContentLoaded',function(){\n"
"updateClock();\n"
"setInterval(updateClock,1000);\n"
"loadHolidays();\n"
"document.getElementById('dateInput').addEventListener('keypress',function(e){if(e.key==='Enter')addHoliday();});\n"
"document.getElementById('dateInput').addEventListener('input',function(e){\n"
"let value=e.target.value.replace(/[^0-9]/g,'');\n"
"if(value.length>=2){value=value.substring(0,2)+'/'+value.substring(2,4);}\n"
"e.target.value=value;\n"
"});\n"
"});\n"
"function updateClock(){\n"
"fetch('/time')\n"
".then(response=>response.json())\n"
".then(data=>{\n"
"document.getElementById('clock').textContent=data.time;\n"
"})\n"
".catch(err=>{console.error('Erreur horloge:',err);});\n"
"}\n"
"function toggleMode(mode){\n"
"const section=document.getElementById('holidaysSection');\n"
"const addSection=document.getElementById('addSection');\n"
"const btnView=document.getElementById('btnView');\n"
"const btnEdit=document.getElementById('btnEdit');\n"
"if(currentMode===mode){\n"
"currentMode=null;\n"
"section.classList.remove('visible');\n"
"addSection.classList.remove('visible');\n"
"btnView.classList.remove('active');\n"
"btnEdit.classList.remove('active');\n"
"return;\n"
"}\n"
"currentMode=mode;\n"
"btnView.classList.toggle('active',mode==='view');\n"
"btnEdit.classList.toggle('active',mode==='edit');\n"
"document.getElementById('listTitle').textContent=mode==='view'?'Liste des jours feries (Visualisation)':'Liste des jours feries (Modification)';\n"
"section.classList.add('visible');\n"
"if(mode==='edit'){\n"
"addSection.classList.add('visible');\n"
"}else{\n"
"addSection.classList.remove('visible');\n"
"}\n"
"renderHolidays();\n"
"}\n"
"function loadHolidays(){\n"
"fetch('/api/holidays')\n"
".then(response=>response.json())\n"
".then(data=>{\n"
"holidaysData=data;\n"
"})\n"
".catch(err=>{console.error('Erreur chargement:',err);});\n"
"}\n"
"function renderHolidays(){\n"
"const list=document.getElementById('holidaysList');\n"
"if(holidaysData.length===0){\n"
"list.innerHTML='<li class=\"empty-state\"><div class=\"empty-state-icon\">📭</div><div>Aucun jour ferie enregistre</div></li>';\n"
"return;\n"
"}\n"
"list.innerHTML=holidaysData.map(date=>{\n"
"const name=holidayNames[date]||'Jour ferie';\n"
"const deleteBtn=currentMode==='edit'?'<button class=\"delete-btn\" onclick=\"deleteHoliday(\\''+date+'\\')\">🗑️ Supprimer</button>':'';\n"
"return '<li class=\"holiday-item\"><div><div class=\"holiday-date\">'+date+'</div><div class=\"holiday-name\">'+name+'</div></div>'+deleteBtn+'</li>';\n"
"}).join('');\n"
"}\n"
"function addHoliday(){\n"
"const input=document.getElementById('dateInput');\n"
"const date=input.value.trim();\n"
"const recurring=document.querySelector('input[name=\"recurring\"]:checked').value;\n"
"if(!date){showMessage('Veuillez entrer une date','error');return;}\n"
"if(!/^\\d{2}\\/\\d{2}$/.test(date)){showMessage('Format invalide. Utilisez JJ/MM (ex: 25/12)','error');return;}\n"
"const day=parseInt(date.substring(0,2));\n"
"const month=parseInt(date.substring(3,5));\n"
"if(month<1||month>12){showMessage('Mois invalide (doit etre entre 01 et 12)','error');return;}\n"
"const daysInMonth=[31,29,31,30,31,30,31,31,30,31,30,31];\n"
"if(day<1||day>daysInMonth[month-1]){showMessage('Jour invalide pour ce mois','error');return;}\n"
"fetch('/api/holidays/add',{\n"
"method:'POST',\n"
"headers:{'Content-Type':'application/x-www-form-urlencoded'},\n"
"body:'date='+encodeURIComponent(date)+'&recurring='+encodeURIComponent(recurring)\n"
"})\n"
".then(response=>response.json())\n"
".then(data=>{\n"
"if(data.success){\n"
"holidaysData=data.holidays;\n"
"input.value='';\n"
"showMessage(data.message,'success');\n"
"if(currentMode!==null){renderHolidays();}\n"
"}else{\n"
"showMessage(data.error,'error');\n"
"}\n"
"})\n"
".catch(err=>{\n"
"showMessage('Erreur de communication avec le serveur','error');\n"
"console.error('Erreur ajout:',err);\n"
"});\n"
"}\n"
"function deleteHoliday(date){\n"
"if(!confirm('Etes-vous sur de vouloir supprimer le jour ferie '+date+' ?')){return;}\n"
"fetch('/api/holidays/delete',{\n"
"method:'POST',\n"
"headers:{'Content-Type':'application/x-www-form-urlencoded'},\n"
"body:'date='+encodeURIComponent(date)\n"
"})\n"
".then(response=>response.json())\n"
".then(data=>{\n"
"if(data.success){\n"
"holidaysData=data.holidays;\n"
"renderHolidays();\n"
"showMessage(data.message,'success');\n"
"}else{\n"
"showMessage(data.error,'error');\n"
"}\n"
"})\n"
".catch(err=>{\n"
"showMessage('Erreur de communication avec le serveur','error');\n"
"console.error('Erreur suppression:',err);\n"
"});\n"
"}\n"
"function showMessage(text,type){\n"
"const msgEl=document.getElementById('message');\n"
"if(messageTimeout){clearTimeout(messageTimeout);}\n"
"msgEl.textContent=text;\n"
"msgEl.className='message show '+type;\n"
"messageTimeout=setTimeout(()=>{msgEl.classList.remove('show');},3000);\n"
"}\n"
"</script>\n"
"</body>\n"
"</html>";

static esp_err_t gestionPageHolidays(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_send(req, holidays_html, strlen(holidays_html));
    return ESP_OK;
}

// ============================================================
//  STREAMING MJPEG (port 81)
// ============================================================
static esp_err_t gestionFluxVideo(httpd_req_t *req) {
    esp_err_t res = ESP_OK;
    char part_buf[64];
    if (xSemaphoreTake(mutexClient, pdMS_TO_TICKS(200)) == pdTRUE) { clientConnecte = true; xSemaphoreGive(mutexClient); }
    res = httpd_resp_set_type(req, "multipart/x-mixed-replace;boundary=123456789000000000000987654321");
    if (res != ESP_OK) { if (xSemaphoreTake(mutexClient, pdMS_TO_TICKS(200)) == pdTRUE) { clientConnecte = false; xSemaphoreGive(mutexClient); } return res; }

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
                if (res == ESP_OK) {
                    res = httpd_resp_send_chunk(req, (const char*)tamponFlux, longueurFlux);
                }
                if (res == ESP_OK) imageEnvoyee = true;
            }
            xSemaphoreGive(mutexFlux);
        }
        if (res != ESP_OK) break;
        if (!imageEnvoyee) {
            compteurTimeout += 50;
            if (compteurTimeout >= 5000) break;
        } else {
            compteurTimeout = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    if (xSemaphoreTake(mutexClient, pdMS_TO_TICKS(200)) == pdTRUE) { clientConnecte = false; xSemaphoreGive(mutexClient); }
    return res;
}

// ============================================================
//  DÉMARRAGE DES SERVEURS WEB (port 80 & 81)
// ============================================================
bool demarrerServeurs() {
    httpd_config_t config80 = HTTPD_DEFAULT_CONFIG();
    config80.server_port     = 80;
    config80.max_open_sockets = 7;
    config80.lru_purge_enable = true;
    config80.max_uri_handlers = 16;

    httpd_uri_t uri_index         = { "/",                   HTTP_GET,  gestionPageIndex,         NULL };
    httpd_uri_t uri_statut        = { "/status",             HTTP_GET,  gestionStatut,            NULL };
    httpd_uri_t uri_sante         = { "/health",             HTTP_GET,  gestionSante,             NULL };
    httpd_uri_t uri_allumer       = { "/light/on",           HTTP_GET,  gestionAllumerLumiere,    NULL };
    httpd_uri_t uri_eteindre      = { "/light/off",          HTTP_GET,  gestionEteindreLumiere,   NULL };
    httpd_uri_t uri_auto          = { "/light/auto",         HTTP_GET,  gestionModeAuto,          NULL };
    httpd_uri_t uri_servo_g       = { "/servo/left",         HTTP_GET,  gestionServoGauche,       NULL };
    httpd_uri_t uri_servo_d       = { "/servo/right",        HTTP_GET,  gestionServoDroite,       NULL };
    httpd_uri_t uri_holidays_page = { "/holidays",           HTTP_GET,  gestionPageHolidays,      NULL };
    httpd_uri_t uri_time          = { "/time",               HTTP_GET,  gestionApiTime,           NULL };
    httpd_uri_t uri_hol_get       = { "/api/holidays",       HTTP_GET,  gestionApiHolidaysGet,    NULL };
    httpd_uri_t uri_hol_add       = { "/api/holidays/add",   HTTP_POST, gestionApiHolidaysAdd,    NULL };
    httpd_uri_t uri_hol_del       = { "/api/holidays/delete",HTTP_POST, gestionApiHolidaysDelete, NULL };

    if (httpd_start(&serveur80, &config80) != ESP_OK) { LOG_ERROR("SERVER", "Échec port 80"); return false; }
    httpd_register_uri_handler(serveur80, &uri_index);
    httpd_register_uri_handler(serveur80, &uri_statut);
    httpd_register_uri_handler(serveur80, &uri_sante);
    httpd_register_uri_handler(serveur80, &uri_allumer);
    httpd_register_uri_handler(serveur80, &uri_eteindre);
    httpd_register_uri_handler(serveur80, &uri_auto);
    httpd_register_uri_handler(serveur80, &uri_servo_g);
    httpd_register_uri_handler(serveur80, &uri_servo_d);
    httpd_register_uri_handler(serveur80, &uri_holidays_page);
    httpd_register_uri_handler(serveur80, &uri_time);
    httpd_register_uri_handler(serveur80, &uri_hol_get);
    httpd_register_uri_handler(serveur80, &uri_hol_add);
    httpd_register_uri_handler(serveur80, &uri_hol_del);
    LOG_INFO("SERVER", "Port 80 OK (13 routes)");

    httpd_config_t config81 = HTTPD_DEFAULT_CONFIG();
    config81.server_port     = 81;
    config81.max_open_sockets = 3;
    config81.lru_purge_enable = true;
    config81.ctrl_port        = 32769;
    httpd_uri_t uri_flux = { "/stream", HTTP_GET, gestionFluxVideo, NULL };
    if (httpd_start(&serveur81, &config81) != ESP_OK) { LOG_ERROR("SERVER", "Échec port 81"); return false; }
    httpd_register_uri_handler(serveur81, &uri_flux);
    LOG_INFO("SERVER", "Port 81 OK (stream)");
    return true;
}

// ============================================================
//  INTERFACE EDGE IMPULSE : fournit les données image à l'IA
// ============================================================
static int ei_camera_obtenir_donnees(size_t offset, size_t longueur, float *out) {
    size_t pxIdx = offset * 3;
    for (size_t i = 0; i < longueur; i++, pxIdx += 3)
        out[i] = (tamponRgbIa[pxIdx+2] << 16) + (tamponRgbIa[pxIdx+1] << 8) + tamponRgbIa[pxIdx];
    return 0;
}

// ============================================================
//  TÂCHE DE CAPTURE RAPIDE (cœur 1) – alimente le flux et l'IA
// ============================================================
void tacheCaptureRapide(void* p) {
    LOG_INFO("TACHE", "Capture rapide démarrée (Cœur 1)");
    esp_task_wdt_add(NULL);
    while (true) {
        esp_task_wdt_reset();
        camera_fb_t* fb = esp_camera_fb_get();
        if (!fb) {
            sante.erreursCamera++;
            compteurErreursCam++;
            if (compteurErreursCam >= MAX_ERREURS_CAMERA) {
                if (reinitialiserCamera()) compteurErreursCam = 0;
                else if (compteurReinitCam >= MAX_REINITIALISATIONS_CAMERA) esp_restart();
            }
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }
        compteurErreursCam = 0;

        // Mettre à jour le tampon du flux MJPEG
        if (xSemaphoreTake(mutexFlux, pdMS_TO_TICKS(100)) == pdTRUE) {
            if (fb->len <= LARGEUR_IMAGE_IA * HAUTEUR_IMAGE_IA * 2) {
                memcpy(tamponFlux, fb->buf, fb->len);
                longueurFlux = fb->len;
            } else longueurFlux = 0;
            xSemaphoreGive(mutexFlux);
        }

        // Mettre à jour le snapshot (pour les alertes)
        if (xSemaphoreTake(mutexSnapshot, pdMS_TO_TICKS(50)) == pdTRUE) {
            if (fb->len <= LARGEUR_IMAGE_IA * HAUTEUR_IMAGE_IA * 2) {
                memcpy(tamponSnapshot, fb->buf, fb->len);
                longueurSnapshot = fb->len;
            }
            xSemaphoreGive(mutexSnapshot);
        }

        // Vérifier si la tâche IA a besoin d'une image RGB
        if (xSemaphoreTake(semRequeteIa, 0) == pdTRUE) {
            bool ok = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, tamponRgbIa);
            if (ok) {
                // Redimensionner si nécessaire (le modèle peut avoir une taille différente)
                if (EI_CLASSIFIER_INPUT_WIDTH != LARGEUR_IMAGE_IA || EI_CLASSIFIER_INPUT_HEIGHT != HAUTEUR_IMAGE_IA)
                    ei::image::processing::crop_and_interpolate_rgb888(tamponRgbIa, LARGEUR_IMAGE_IA, HAUTEUR_IMAGE_IA,
                                                                        tamponRgbIa, EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT);
                xSemaphoreGive(semPretIa);
            } else {
                xSemaphoreGive(semRequeteIa);
            }
        }
        esp_camera_fb_return(fb);
    }
}

// ============================================================
//  TÂCHE IA (cœur 0) – exécute le classifieur Edge Impulse
// ============================================================
void tacheIa(void* p) {
    LOG_INFO("TACHE", "Tâche IA démarrée (Cœur 0)");
    esp_task_wdt_add(NULL);
    while (true) {
        esp_task_wdt_reset();
        xSemaphoreGive(semRequeteIa);          // Demande une nouvelle image
        if (xSemaphoreTake(semPretIa, pdMS_TO_TICKS(2000)) != pdTRUE) {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        ei::signal_t signal;
        signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
        signal.get_data = &ei_camera_obtenir_donnees;

        ei_impulse_result_t resultat;
        memset(&resultat, 0, sizeof(resultat));

        esp_task_wdt_reset();
        EI_IMPULSE_ERROR erreur = run_classifier(&signal, &resultat, false);
        esp_task_wdt_reset();

        if (erreur != EI_IMPULSE_OK) {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        float meilleurScore = 0.0f;
        char nouveauLabel[64] = "Rien détecté";
        bool personneDetectee = false;

        // Selon que le modèle est en object detection ou classification simple
#if EI_CLASSIFIER_OBJECT_DETECTION == 1
        for (uint32_t i = 0; i < resultat.bounding_boxes_count; i++) {
            ei_impulse_result_bounding_box_t bb = resultat.bounding_boxes[i];
            if (bb.value == 0) continue;
            if (bb.value > meilleurScore) {
                meilleurScore = bb.value;
                snprintf(nouveauLabel, sizeof(nouveauLabel), "%s : %.1f%%", bb.label, bb.value * 100);
                personneDetectee = ((strstr(bb.label, "person") || strstr(bb.label, "personne")) && meilleurScore > 0.90f);
            }
        }
#else
        for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
            if (resultat.classification[i].value > meilleurScore) {
                meilleurScore = resultat.classification[i].value;
                snprintf(nouveauLabel, sizeof(nouveauLabel), "%s : %.1f%%", ei_classifier_inferencing_categories[i], meilleurScore * 100);
                personneDetectee = ((strstr(ei_classifier_inferencing_categories[i], "person") || strstr(ei_classifier_inferencing_categories[i], "personne")) && meilleurScore > 0.90f);
            }
        }
#endif

        // Mettre à jour l'affichage du dernier label
        if (xSemaphoreTake(mutexDetection, pdMS_TO_TICKS(200)) == pdTRUE) {
            strncpy(labelDetection, nouveauLabel, 63);
            labelDetection[63] = 0;
            scoreDetection = meilleurScore;
            xSemaphoreGive(mutexDetection);
        } else sante.timeoutsMutex++;

        // Enregistrer cette détection dans l'historique (pour validation temporelle)
        unsigned long maintenant = millis();
        historiqueDetections[indexHistorique] = { maintenant, personneDetectee };
        indexHistorique = (indexHistorique + 1) % HISTORIQUE_MAX;
        if (tailleHistorique < HISTORIQUE_MAX) tailleHistorique++;

        int nbDetections = compterDetectionsRecentes(FENETRE_VALIDATION_MS);
        bool resetAutorise = (nbDetections >= SEUIL_DETECTIONS);

        // Gérer la lumière (sauf si mode manuel)
        if (!modeManuel) gererLumiere(personneDetectee, resetAutorise);

        // Capturer une copie de l'image si une alerte Telegram est possible (nuit ou férié)
        uint8_t* snapCopy = NULL;
        size_t   snapLen  = 0;
        if (personneDetectee && (estModeNuit() || estJourFerie())) {
            snapCopy = (uint8_t*)ps_malloc(LARGEUR_IMAGE_IA * HAUTEUR_IMAGE_IA * 2);
            if (snapCopy && xSemaphoreTake(mutexSnapshot, pdMS_TO_TICKS(100)) == pdTRUE) {
                memcpy(snapCopy, tamponSnapshot, longueurSnapshot);
                snapLen = longueurSnapshot;
                xSemaphoreGive(mutexSnapshot);
            }
        }

        // Envoyer les notifications Telegram
        if (xSemaphoreTake(mutexTelegram, pdMS_TO_TICKS(100)) == pdTRUE) {
            gererNotificationsTelegram(personneDetectee, snapCopy, snapLen);
            xSemaphoreGive(mutexTelegram);
        }
        if (snapCopy) { free(snapCopy); snapCopy = NULL; }

        // Appliquer éventuellement une commande manuelle du servo
        bool angleManuelApplique = false;
        if (xSemaphoreTake(mutexServo, pdMS_TO_TICKS(50)) == pdTRUE) {
            if (angleManuel >= 0) {
                angleServo = angleManuel;
                angleManuel = -1;
                angleManuelApplique = true;
                int at = angleServo;
                xSemaphoreGive(mutexServo);
                monServo.write(at);
            } else xSemaphoreGive(mutexServo);
        } else sante.timeoutsMutex++;

        if (!angleManuelApplique && !modeManuel) gererServoAuto();

        vTaskDelay(pdMS_TO_TICKS(DELAI_IA_MS));
    }
}

// ============================================================
//  TÂCHE WIFI + MAINTENANCE (cœur 0)
// ============================================================
void tacheWifi(void* p) {
    LOG_INFO("TACHE", "WiFi démarrée (Cœur 0)");
    esp_task_wdt_add(NULL);
    if (WiFi.status() != WL_CONNECTED) {
        WiFi.begin(wifi_ssid, wifi_password);
        int essais = 0;
        while (WiFi.status() != WL_CONNECTED && essais < 20) {
            delay(500);
            essais++;
        }
    }
    if (WiFi.status() == WL_CONNECTED) {
        LOG_INFO("WIFI", "Connecté IP : %s", WiFi.localIP().toString().c_str());
    } else if (sante.redemarrages <= MAX_REDEMARRAGES_WIFI) {
        esp_restart();
    }

    if (!demarrerServeurs()) {
        vTaskDelay(pdMS_TO_TICKS(3000));
        esp_restart();
    }

    unsigned long dernierNtp = 0;
    const unsigned long RETRY_NTP = 30000;
    while (true) {
        esp_task_wdt_reset();
        gererChronoLumiere();              // Gestion du timer d'extinction
        if (WiFi.status() != WL_CONNECTED) WiFi.reconnect();

        // Mise à jour annuelle des jours fériés si l'heure est disponible
        if (heureDisponible) {
            unsigned long e = (millis() - millisDemarrage) / 1000;
            time_t n = epochDemarrage + e;
            struct tm* t = localtime(&n);
            verifierChangementAnnee(t->tm_year + 1900);
        }

        // Tentative de synchronisation NTP si encore non dispo
        if (!heureDisponible && WiFi.status() == WL_CONNECTED) {
            if (millis() - dernierNtp >= RETRY_NTP) {
                dernierNtp = millis();
                synchroniserHeureNTP();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

// ============================================================
//  SETUP – Initialisation générale
// ============================================================
void setup() {
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);   // Désactive la détection de brownout (évite les reboots intempestifs)
    Serial.begin(115200);
    LOG_INFO("INIT", "=== ESP32-CAM FUSION COMPLET ===");

    // Configuration LED témoin et relais
    ledcAttach(BROCHE_LED_CAM, FREQ_PWM_LED, RESOLUTION_PWM_LED);
    ledcWrite(BROCHE_LED_CAM, 0);
    pinMode(BROCHE_RELAIS, OUTPUT);
    pinMode(BROCHE_LDR, INPUT);

    // Vérification initiale de l'état de la lampe
    digitalWrite(BROCHE_RELAIS, LOW);
    delay(500);
    if (lireLDR() > SEUIL_LDR) {
        lumiereAllumee = true;
        LOG_INFO("INIT", "Lumière ON");
    } else {
        digitalWrite(BROCHE_RELAIS, HIGH);
        delay(500);
        if (lireLDR() > SEUIL_LDR) {
            lumiereAllumee = true;
            LOG_INFO("INIT", "Lumière ON après basculement");
        } else {
            digitalWrite(BROCHE_RELAIS, LOW);
            lumiereAllumee = false;
            LOG_WARNING("INIT", "Lumière non détectée");
        }
    }

    // Watchdog
    esp_task_wdt_deinit();
    esp_task_wdt_config_t wdt_config = { .timeout_ms = TIMEOUT_WATCHDOG_SEC * 1000, .idle_core_mask = 0, .trigger_panic = true };
    esp_task_wdt_init(&wdt_config);

    // Connexion WiFi
    LOG_INFO("WIFI", "Connexion à %s...", wifi_ssid);
    WiFi.mode(WIFI_STA);
    WiFi.begin(wifi_ssid, wifi_password);
    unsigned long debutWifi = millis();
    bool wifiOk = false;
    while (millis() - debutWifi < 20000) {
        if (WiFi.status() == WL_CONNECTED) { wifiOk = true; break; }
        delay(500);
        ledcWrite(BROCHE_LED_CAM, DUTY_DIM);
        delay(100);
        ledcWrite(BROCHE_LED_CAM, 0);
        delay(400);
    }

    // Chargement du mode depuis NVS
    preferences.begin("esp32cam", true);
    modeManuel = preferences.getBool("modeManuel", false);
    sante.redemarrages = preferences.getUInt("redemarrages", 0) + 1;
    preferences.end();
    dernierModeSauvegarde = modeManuel;
    preferences.begin("esp32cam", false);
    preferences.putUInt("redemarrages", sante.redemarrages);
    preferences.end();

    if (!wifiOk) {
        if (sante.redemarrages > MAX_REDEMARRAGES_WIFI) {
            LOG_ERROR("WIFI", "Trop de redémarrages — continue sans WiFi");
        } else {
            esp_restart();
        }
    } else {
        LOG_INFO("WIFI", "Connecté IP : %s", WiFi.localIP().toString().c_str());
    }

    // Synchronisation NTP et chargement des jours fériés
    if (synchroniserHeureNTP()) {
        unsigned long e = (millis() - millisDemarrage) / 1000;
        time_t n = epochDemarrage + e;
        struct tm* t = localtime(&n);
        int annee = t->tm_year + 1900;

        chargerJoursFeries();
        if (holidayList.length() == 0) {
            initJoursFeriesDefaut(annee);
        } else {
            verifierChangementAnnee(annee);
        }
        LOG_INFO("HOLIDAYS", "Jours fériés chargés : %s", holidayList.c_str());
    } else {
        chargerJoursFeries();
        LOG_WARNING("NTP", "Heure non dispo — jours fériés chargés sans vérif d'année");
    }

    // Servo
    ESP32PWM::allocateTimer(3);
    monServo.setPeriodHertz(50);
    monServo.attach(BROCHE_SERVO, 500, 2400);
    monServo.write(0);

    // Caméra
    esp_err_t err = esp_camera_init(&configCamera);
    if (err != ESP_OK) {
        LOG_FATAL("CAM", "Erreur init : 0x%x", err);
        while (1) delay(1000);
    }
    sensor_t* capteur = esp_camera_sensor_get();
    capteur->set_vflip(capteur, 1);     // Retournement vertical
    capteur->set_hmirror(capteur, 1);   // Miroir horizontal
    LOG_INFO("CAM", "Initialisée");

    // Création des mutex et sémaphores
    mutexFlux      = xSemaphoreCreateMutex();
    mutexDetection = xSemaphoreCreateMutex();
    mutexLumiere   = xSemaphoreCreateMutex();
    mutexServo     = xSemaphoreCreateMutex();
    mutexRelais    = xSemaphoreCreateMutex();
    mutexClient    = xSemaphoreCreateMutex();
    mutexTelegram  = xSemaphoreCreateMutex();
    mutexSnapshot  = xSemaphoreCreateMutex();
    semRequeteIa   = xSemaphoreCreateBinary();
    semPretIa      = xSemaphoreCreateBinary();

    if (!mutexFlux || !mutexDetection || !mutexLumiere || !mutexServo || !mutexRelais ||
        !mutexClient || !mutexTelegram || !mutexSnapshot || !semRequeteIa || !semPretIa) {
        LOG_FATAL("INIT", "Échec mutex — reboot");
        esp_restart();
    }

    // Allocation des buffers PSRAM (image JPEG, RGB, snapshot)
    tamponFlux     = (uint8_t*)ps_malloc(LARGEUR_IMAGE_IA * HAUTEUR_IMAGE_IA * 2);
    tamponRgbIa    = (uint8_t*)ps_malloc(LARGEUR_IMAGE_IA * HAUTEUR_IMAGE_IA * TAILLE_PIXEL_IA);
    tamponSnapshot = (uint8_t*)ps_malloc(LARGEUR_IMAGE_IA * HAUTEUR_IMAGE_IA * 2);
    if (!tamponFlux || !tamponRgbIa || !tamponSnapshot) {
        LOG_FATAL("MEM", "PSRAM échouée — reboot");
        esp_restart();
    }
    LOG_INFO("MEM", "Buffers PSRAM OK");

    sante.heureDemarrage = millis();

    // Lancement des tâches
    xTaskCreatePinnedToCore(tacheCaptureRapide, "CaptureRapide", 8192,  NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(tacheIa,            "IATask",        20480, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(tacheWifi,          "WiFi",          8192,  NULL, 1, NULL, 0);

    LOG_INFO("INIT", "Système prêt → http://%s", WiFi.localIP().toString().c_str());
    LOG_INFO("INIT", "Jours fériés → http://%s/holidays", WiFi.localIP().toString().c_str());
}

// ============================================================
//  LOOP – Vide car tout est géré par les tâches FreeRTOS
// ============================================================
void loop() {
    vTaskDelay(pdMS_TO_TICKS(10000));
}
