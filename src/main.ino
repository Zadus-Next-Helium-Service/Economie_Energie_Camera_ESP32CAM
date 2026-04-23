// ============================================================
// ESP32-CAM AI THINKER — MODE CLIENT WIFI (Centre d'incubation)
// ============================================================
// Connexion au réseau WiFi "Centre d'incubation 2.4G" (INcubator@@2025)
// Récupération heure par NTP
// Streaming fluide, IA toutes les 2s, servo, lumière, etc.
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

// Vérification du modèle IA
#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_CAMERA
#error "Modele IA invalide — verifier l'export EdgeImpulse"
#endif

// ============================================================
// CONFIGURATION CAMÉRA AI THINKER
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
#define RELAY_PIN   14
#define LDR_PIN     15
#define SERVO_PIN   13
#define CAM_LED_PIN  4

// ============================================================
// CONFIGURATION WIFI (CLIENT UNIQUEMENT)
// ============================================================


const char* wifi_ssid     = "Nom_wifi";
const char* wifi_password = "Mot_de_passe_wifi";
//const char* wifi_ssid     = "Centre dincubation 2.4G_plus";
//const char* wifi_password = "INcubator@@2025";

//const char* wifi_ssid     = "Redmi 12";
//const char* wifi_password = "12345678";

// ============================================================
// CONSTANTES IMAGE
// ============================================================
#define EI_CAMERA_RAW_FRAME_BUFFER_COLS   320
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS   240
#define EI_CAMERA_FRAME_BYTE_SIZE           3

// ============================================================
// CONFIGURATION LUMIÈRE
// ============================================================
#define LDR_THRESHOLD        3000
#define TIMEOUT_MS          180000   // 4 minutes (modifiable)
#define MANUAL_TIMEOUT_MS   240000   // 5 min sans activité web → retour auto
#define RELAY_DEBOUNCE_MS     2000

// ============================================================
// CONFIGURATION SERVO
// ============================================================
#define SERVO_STEP          10
#define SERVO_MAX          100
#define SERVO_RETURN_STEP    7
#define SERVO_WEB_STEP      10
#define SERVO_SCAN_EVERY     3

// ============================================================
// WATCHDOG
// ============================================================
#define WDT_TIMEOUT_SEC     30

// ============================================================
// RÉCUPÉRATION CAMÉRA
// ============================================================
#define CAM_MAX_ERRORS      10
#define CAM_MAX_REINIT       3

// ============================================================
// SURVEILLANCE MÉMOIRE
// ============================================================
#define HEAP_WARNING_BYTES  30000

// ============================================================
// PAUSE IA
// ============================================================
#define AI_ANALYSIS_DELAY_MS  2000

// ============================================================
// CONFIGURATION NTP
// ============================================================
#define NTP_SERVER            "pool.ntp.org"
#define GMT_OFFSET_SEC        0          // Ajustez selon votre fuseau (ex: 3600 pour UTC+1)
#define DAYLIGHT_OFFSET_SEC   0

// ============================================================
// PWM LED
// ============================================================
#define LEDC_CHANNEL    0
#define LEDC_FREQ       5000
#define LEDC_RESOLUTION 8
#define LEDC_DUTY_MAX   255
#define LEDC_DUTY_DIM   128    // luminosité réduite

// ============================================================
// LOGS
// ============================================================
#define LOG_INFO(tag, fmt, ...)    Serial.printf("[INFO][%s] " fmt "\n",    tag, ##__VA_ARGS__)
#define LOG_WARNING(tag, fmt, ...) Serial.printf("[WARNING][%s] " fmt "\n", tag, ##__VA_ARGS__)
#define LOG_ERROR(tag, fmt, ...)   Serial.printf("[ERROR][%s] " fmt "\n",   tag, ##__VA_ARGS__)
#define LOG_FATAL(tag, fmt, ...)   Serial.printf("[FATAL][%s] " fmt "\n",   tag, ##__VA_ARGS__)

// ============================================================
// STRUCTURE SANTÉ
// ============================================================
struct SystemHealth {
    uint32_t reboots, cameraErrors, cameraReinits, detectionCount, lightOnCount, relayToggleCount, mutexTimeouts, heapMin;
    unsigned long bootTime;
};
SystemHealth health = { 0, 0, 0, 0, 0, 0, 0, UINT32_MAX, 0 };

// ============================================================
// NVS
// ============================================================
Preferences prefs;
volatile bool manualMode = false;
volatile unsigned long manualModeStart = 0;
static bool lastSavedMode = false;
void saveStateNVS() {
    if (manualMode == lastSavedMode) return;
    lastSavedMode = manualMode;
    prefs.begin("esp32cam", false);
    prefs.putBool("manualMode", manualMode);
    prefs.putUInt("reboots", health.reboots);
    prefs.end();
    LOG_INFO("NVS", "Etat sauvegarde (manualMode=%s)", manualMode ? "MANUEL" : "AUTO");
}

// ============================================================
// HEURE (NTP)
// ============================================================
static bool realTimeAvailable = false;
static time_t bootEpoch = 0;
static unsigned long bootMillis = 0;

int getCurrentHour() {
    if (realTimeAvailable) {
        unsigned long elapsed = (millis() - bootMillis) / 1000;
        time_t now = bootEpoch + (time_t)elapsed;
        struct tm* t = localtime(&now);
        return t->tm_hour;
    }
    return -1;
}
bool isNightMode() {
    if (realTimeAvailable) { int h = getCurrentHour(); return (h >= 22 || h < 6); }
    // fallback si heure indisponible
    unsigned long uptime = millis() - health.bootTime;
    unsigned long phase = uptime % (16*3600*1000UL + 8*3600*1000UL);
    return (phase >= 16*3600*1000UL);
}

// ============================================================
// STREAMING & IA : BUFFERS DÉDIÉS
// ============================================================
uint8_t* stream_buffer = NULL;
size_t   stream_len = 0;
SemaphoreHandle_t stream_mutex = NULL;
uint8_t* ia_rgb_buffer = NULL;
SemaphoreHandle_t ia_request_sem = NULL;
SemaphoreHandle_t ia_ready_sem = NULL;

// ============================================================
// AUTRES SÉMAPHORES
// ============================================================
SemaphoreHandle_t detection_mutex = NULL;
SemaphoreHandle_t light_mutex = NULL;
SemaphoreHandle_t servo_mutex = NULL;
SemaphoreHandle_t relay_mutex = NULL;
SemaphoreHandle_t client_mutex = NULL;
volatile bool client_connected = false;

char  detection_label[64] = "En attente...";
float detection_score = 0.0f;

bool lightOn = false;
unsigned long lastDetected = 0;
bool timerActive = false;

Servo monServo;
int servoAngle = 0;
int manualAngle = -1;
static bool servoReturning = false;
static int servoCycleCount = 0;
static bool servoNightMode = false;

int cameraErrorCount = 0, cameraReinitCount = 0;

httpd_handle_t httpd_80 = NULL, httpd_81 = NULL;

// ============================================================
// CONFIGURATION CAMÉRA
// ============================================================
static camera_config_t camera_config = {
    .pin_pwdn = PWDN_GPIO_NUM, .pin_reset = RESET_GPIO_NUM, .pin_xclk = XCLK_GPIO_NUM,
    .pin_sscb_sda = SIOD_GPIO_NUM, .pin_sscb_scl = SIOC_GPIO_NUM,
    .pin_d7 = Y9_GPIO_NUM, .pin_d6 = Y8_GPIO_NUM, .pin_d5 = Y7_GPIO_NUM, .pin_d4 = Y6_GPIO_NUM,
    .pin_d3 = Y5_GPIO_NUM, .pin_d2 = Y4_GPIO_NUM, .pin_d1 = Y3_GPIO_NUM, .pin_d0 = Y2_GPIO_NUM,
    .pin_vsync = VSYNC_GPIO_NUM, .pin_href = HREF_GPIO_NUM, .pin_pclk = PCLK_GPIO_NUM,
    .xclk_freq_hz = 20000000, .ledc_timer = LEDC_TIMER_0, .ledc_channel = LEDC_CHANNEL_0,
    .pixel_format = PIXFORMAT_JPEG, .frame_size = FRAMESIZE_QVGA, .jpeg_quality = 12,
    .fb_count = 1, .fb_location = CAMERA_FB_IN_PSRAM, .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

// ============================================================
// FONCTIONS LDR, RELAIS, LUMIÈRE (inchangées)
// ============================================================
void checkHeap() {
    uint32_t freeHeap = esp_get_free_heap_size();
    if (freeHeap < health.heapMin) health.heapMin = freeHeap;
    if (freeHeap < HEAP_WARNING_BYTES) LOG_WARNING("MEM", "Heap faible : %u octets !", freeHeap);
}
int readLDR() { return (digitalRead(LDR_PIN) == HIGH) ? 4095 : 0; }
bool isLightPhysicallyOn() { return (readLDR() > LDR_THRESHOLD); }

static unsigned long lastRelayToggle = 0;
bool basculerRelais() {
    if (xSemaphoreTake(relay_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) { health.mutexTimeouts++; return false; }
    unsigned long maintenant = millis();
    if (maintenant - lastRelayToggle < RELAY_DEBOUNCE_MS) { xSemaphoreGive(relay_mutex); return false; }
    int etatActuel = digitalRead(RELAY_PIN);
    digitalWrite(RELAY_PIN, !etatActuel);
    lastRelayToggle = maintenant;
    health.relayToggleCount++;
    xSemaphoreGive(relay_mutex);
    vTaskDelay(pdMS_TO_TICKS(500));
    return true;
}
void allumerLumiere() {
    if (isLightPhysicallyOn()) {
        if (xSemaphoreTake(light_mutex, pdMS_TO_TICKS(200)) == pdTRUE) { lightOn = true; xSemaphoreGive(light_mutex); }
        else health.mutexTimeouts++;
        LOG_INFO("ALLUMER", "Deja allumee");
        return;
    }
    if (!basculerRelais()) { LOG_ERROR("ALLUMER", "Basculement refuse"); return; }
    if (isLightPhysicallyOn()) {
        if (xSemaphoreTake(light_mutex, pdMS_TO_TICKS(200)) == pdTRUE) { lightOn = true; xSemaphoreGive(light_mutex); }
        else health.mutexTimeouts++;
        health.lightOnCount++;
        LOG_INFO("ALLUMER", "Succes");
    } else {
        basculerRelais();
        if (xSemaphoreTake(light_mutex, pdMS_TO_TICKS(200)) == pdTRUE) { lightOn = false; xSemaphoreGive(light_mutex); }
        else health.mutexTimeouts++;
        LOG_ERROR("ALLUMER", "Echec LDR");
    }
}
void eteindreLumiere() {
    if (!isLightPhysicallyOn()) {
        if (xSemaphoreTake(light_mutex, pdMS_TO_TICKS(200)) == pdTRUE) { lightOn = false; timerActive = false; xSemaphoreGive(light_mutex); }
        else health.mutexTimeouts++;
        LOG_INFO("ETEINDRE", "Deja eteinte");
        return;
    }
    if (!basculerRelais()) { LOG_ERROR("ETEINDRE", "Basculement refuse"); return; }
    if (!isLightPhysicallyOn()) {
        if (xSemaphoreTake(light_mutex, pdMS_TO_TICKS(200)) == pdTRUE) { lightOn = false; timerActive = false; xSemaphoreGive(light_mutex); }
        else health.mutexTimeouts++;
        LOG_INFO("ETEINDRE", "Succes");
    } else {
        basculerRelais();
        if (xSemaphoreTake(light_mutex, pdMS_TO_TICKS(200)) == pdTRUE) { lightOn = true; xSemaphoreGive(light_mutex); }
        else health.mutexTimeouts++;
        LOG_ERROR("ETEINDRE", "Echec LDR");
    }
}
void initLumiere() {
    int ldrVal = readLDR();
    LOG_INFO("INIT", "LDR = %d", ldrVal);
    if (ldrVal > LDR_THRESHOLD) { lightOn = true; lastDetected = millis(); timerActive = true; }
    else lightOn = false;
}
void gererLumiere(bool personneDetectee) {
    bool lumPhysique = isLightPhysicallyOn();
    if (xSemaphoreTake(light_mutex, pdMS_TO_TICKS(200)) == pdTRUE) { lightOn = lumPhysique; xSemaphoreGive(light_mutex); }
    else health.mutexTimeouts++;
    if (personneDetectee) {
        health.detectionCount++;
        if (xSemaphoreTake(light_mutex, pdMS_TO_TICKS(200)) == pdTRUE) { lastDetected = millis(); timerActive = true; xSemaphoreGive(light_mutex); }
        else health.mutexTimeouts++;
        if (!lumPhysique) allumerLumiere();
        else LOG_INFO("LUMIERE", "Personne detectee — chrono reset");
    } else {
        LOG_INFO("LUMIERE", "Aucune presence");
    }
}
void handleLightTimer() {
    bool lumPhysique = isLightPhysicallyOn();
    bool curTimer; unsigned long curLast;
    if (xSemaphoreTake(light_mutex, pdMS_TO_TICKS(200)) == pdTRUE) {
        curTimer = timerActive; curLast = lastDetected;
        xSemaphoreGive(light_mutex);
    } else { health.mutexTimeouts++; return; }
    // Détection d'allumage externe
    if (lumPhysique && !curTimer) {
        if (xSemaphoreTake(light_mutex, pdMS_TO_TICKS(200)) == pdTRUE) {
            lastDetected = millis();
            timerActive = true;
            xSemaphoreGive(light_mutex);
        }
        LOG_INFO("LUMIERE", "Allumage externe detecte — chrono demarre");
        curTimer = true;
        curLast = millis();
    }
    if (!lumPhysique && curTimer) {
        if (xSemaphoreTake(light_mutex, pdMS_TO_TICKS(200)) == pdTRUE) {
            timerActive = false;
            xSemaphoreGive(light_mutex);
        }
        LOG_INFO("LUMIERE", "Extinction externe — chrono arrete");
        curTimer = false;
    }
    if (manualMode) return;  // pas d'extinction auto en manuel
    if (curTimer && lumPhysique) {
        unsigned long elapsed = millis() - curLast;
        if (elapsed >= TIMEOUT_MS) {
            LOG_INFO("TIMER", "Timeout %d ms — extinction", TIMEOUT_MS);
            eteindreLumiere();
        }
    }
    if (curTimer && !lumPhysique) {
        if (xSemaphoreTake(light_mutex, pdMS_TO_TICKS(200)) == pdTRUE) {
            timerActive = false;
            xSemaphoreGive(light_mutex);
        }
    }
    // Timeout mode manuel
    if (manualMode) {
        unsigned long elapsed = millis() - manualModeStart;
        bool clientConn = false;
        if (xSemaphoreTake(client_mutex, pdMS_TO_TICKS(200)) == pdTRUE) { clientConn = client_connected; xSemaphoreGive(client_mutex); }
        if (!clientConn && elapsed >= MANUAL_TIMEOUT_MS) {
            manualMode = false;
            saveStateNVS();
            LOG_INFO("MODE", "Timeout web — retour auto IA");
        }
    }
    checkHeap();
}

bool reinitCamera() {
    LOG_WARNING("CAM", "Reinitialisation (%d/%d)", cameraReinitCount+1, CAM_MAX_REINIT);
    esp_camera_deinit();
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) { LOG_ERROR("CAM", "Reinit echouee : 0x%x", err); health.cameraReinits++; cameraReinitCount++; return false; }
    cameraErrorCount = 0; cameraReinitCount = 0; health.cameraReinits++;
    LOG_INFO("CAM", "Reinit reussie");
    return true;
}

void handleServoAuto() {
    if (isNightMode()) {
        if (!servoNightMode) { servoNightMode = true; if (xSemaphoreTake(servo_mutex, pdMS_TO_TICKS(100)) == pdTRUE) { servoAngle = 0; xSemaphoreGive(servo_mutex); } monServo.write(0); LOG_INFO("SERVO", "Mode nuit — servo a 0"); }
        return;
    }
    if (servoNightMode) { servoNightMode = false; servoReturning = false; LOG_INFO("SERVO", "Mode jour — reprise auto"); }
    servoCycleCount++;
    if (servoCycleCount < SERVO_SCAN_EVERY) return;
    servoCycleCount = 0;
    int currentAngle = 0;
    if (xSemaphoreTake(servo_mutex, pdMS_TO_TICKS(100)) == pdTRUE) { currentAngle = servoAngle; xSemaphoreGive(servo_mutex); } else { health.mutexTimeouts++; return; }
    int newAngle = currentAngle;
    if (currentAngle >= SERVO_MAX) servoReturning = true;
    if (servoReturning) { newAngle = currentAngle - SERVO_RETURN_STEP; if (newAngle <= 0) { newAngle = 0; servoReturning = false; } }
    else { newAngle = currentAngle + SERVO_STEP; if (newAngle > SERVO_MAX) newAngle = SERVO_MAX; }
    if (xSemaphoreTake(servo_mutex, pdMS_TO_TICKS(100)) == pdTRUE) { servoAngle = newAngle; xSemaphoreGive(servo_mutex); } else { health.mutexTimeouts++; return; }
    monServo.write(newAngle);
}

// ============================================================
// SYNCHRONISATION HEURE VIA NTP UNIQUEMENT (sans saisie manuelle)
// ============================================================
bool syncTimeNTP() {
    LOG_INFO("NTP", "Synchronisation heure via NTP...");
    configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER);
    struct tm timeinfo;
    unsigned long start = millis();
    while (millis() - start < 15000) {
        if (getLocalTime(&timeinfo)) {
            bootEpoch = mktime(&timeinfo);
            bootMillis = millis();
            realTimeAvailable = true;
            LOG_INFO("NTP", "Heure NTP : %02d:%02d:%02d", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
            return true;
        }
        delay(500);
    }
    LOG_WARNING("NTP", "Impossible d'obtenir l'heure NTP");
    return false;
}

// ============================================================
// HANDLERS HTTP (tous avec URLs relatives)
// ============================================================
static esp_err_t light_on_handler(httpd_req_t *req) {
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    manualMode = true;
    manualModeStart = millis();
    allumerLumiere();
    saveStateNVS();
    LOG_INFO("WEB", "Commande manuelle : ALLUMER");
    httpd_resp_send(req, "OK", 2);
    return ESP_OK;
}
static esp_err_t light_off_handler(httpd_req_t *req) {
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    manualMode = true;
    manualModeStart = millis();
    eteindreLumiere();
    saveStateNVS();
    LOG_INFO("WEB", "Commande manuelle : ETEINDRE");
    httpd_resp_send(req, "OK", 2);
    return ESP_OK;
}
static esp_err_t light_auto_handler(httpd_req_t *req) {
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    manualMode = false;
    saveStateNVS();
    // Relancer chrono si lumière allumée
    if (isLightPhysicallyOn()) {
        if (xSemaphoreTake(light_mutex, pdMS_TO_TICKS(200)) == pdTRUE) {
            lastDetected = millis();
            timerActive = true;
            xSemaphoreGive(light_mutex);
        }
    }
    LOG_INFO("WEB", "Retour mode automatique IA");
    httpd_resp_send(req, "OK", 2);
    return ESP_OK;
}
static esp_err_t servo_left_handler(httpd_req_t *req) {
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    manualMode = true; manualModeStart = millis();
    int currentAngle = 0;
    if (xSemaphoreTake(servo_mutex, pdMS_TO_TICKS(200)) == pdTRUE) { currentAngle = servoAngle; xSemaphoreGive(servo_mutex); } else health.mutexTimeouts++;
    int newAngle = currentAngle - SERVO_WEB_STEP; if (newAngle < 0) newAngle = 0;
    if (xSemaphoreTake(servo_mutex, pdMS_TO_TICKS(200)) == pdTRUE) { manualAngle = newAngle; xSemaphoreGive(servo_mutex); } else health.mutexTimeouts++;
    char buf[8]; snprintf(buf, sizeof(buf), "%d", newAngle);
    LOG_INFO("WEB", "Servo gauche -> %d deg", newAngle);
    httpd_resp_send(req, buf, strlen(buf));
    return ESP_OK;
}
static esp_err_t servo_right_handler(httpd_req_t *req) {
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    manualMode = true; manualModeStart = millis();
    int currentAngle = 0;
    if (xSemaphoreTake(servo_mutex, pdMS_TO_TICKS(200)) == pdTRUE) { currentAngle = servoAngle; xSemaphoreGive(servo_mutex); } else health.mutexTimeouts++;
    int newAngle = currentAngle + SERVO_WEB_STEP; if (newAngle > 180) newAngle = 180;
    if (xSemaphoreTake(servo_mutex, pdMS_TO_TICKS(200)) == pdTRUE) { manualAngle = newAngle; xSemaphoreGive(servo_mutex); } else health.mutexTimeouts++;
    char buf[8]; snprintf(buf, sizeof(buf), "%d", newAngle);
    LOG_INFO("WEB", "Servo droite -> %d deg", newAngle);
    httpd_resp_send(req, buf, strlen(buf));
    return ESP_OK;
}
static esp_err_t status_handler(httpd_req_t *req) {
    char buf[320];
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache, no-store, must-revalidate");
    char label[64] = "En attente...";
    if (xSemaphoreTake(detection_mutex, pdMS_TO_TICKS(200)) == pdTRUE) { strncpy(label, detection_label, 63); xSemaphoreGive(detection_mutex); } else health.mutexTimeouts++;
    bool curLight = false, curTimer = false; unsigned long curLast = 0;
    if (xSemaphoreTake(light_mutex, pdMS_TO_TICKS(200)) == pdTRUE) { curLight = lightOn; curTimer = timerActive; curLast = lastDetected; xSemaphoreGive(light_mutex); } else health.mutexTimeouts++;
    unsigned long secondsLeft = 0;
    if (curTimer && curLight) { unsigned long elapsed = millis() - curLast; if (elapsed < TIMEOUT_MS) secondsLeft = (TIMEOUT_MS - elapsed)/1000; }
    int curAngle = 0;
    if (xSemaphoreTake(servo_mutex, pdMS_TO_TICKS(100)) == pdTRUE) { curAngle = servoAngle; xSemaphoreGive(servo_mutex); } else health.mutexTimeouts++;
    char heureStr[12] = "??:??";
    if (realTimeAvailable) { int h = getCurrentHour(); unsigned long elapsed = (millis()-bootMillis)/1000; time_t now = bootEpoch+elapsed; struct tm* t = localtime(&now); snprintf(heureStr,12,"%02d:%02d",t->tm_hour,t->tm_min); }
    snprintf(buf, sizeof(buf), "%s|%s|%lu|%s|%d|%s|%s", label, curLight?"ON":"OFF", secondsLeft, manualMode?"MANUEL":"AUTO", curAngle, isNightMode()?"NUIT":"JOUR", heureStr);
    httpd_resp_send(req, buf, strlen(buf));
    return ESP_OK;
}
static esp_err_t health_handler(httpd_req_t *req) {
    char buf[768];
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    unsigned long uptime = (millis()-health.bootTime)/1000;
    unsigned long hours = uptime/3600, minutes = (uptime%3600)/60, seconds = uptime%60;
    uint32_t freeHeap = esp_get_free_heap_size();
    int curAngle = 0;
    if (xSemaphoreTake(servo_mutex, pdMS_TO_TICKS(100)) == pdTRUE) { curAngle = servoAngle; xSemaphoreGive(servo_mutex); }
    char heureStr[12] = "Non dispo";
    if (realTimeAvailable) { unsigned long el = (millis()-bootMillis)/1000; time_t now = bootEpoch+el; struct tm* t = localtime(&now); snprintf(heureStr,12,"%02d:%02d:%02d", t->tm_hour, t->tm_min, t->tm_sec); }
    snprintf(buf, sizeof(buf),
        "Uptime        : %luh %lum %lus\nHeure actuelle: %s\nHeap libre    : %u octets\nHeap minimum  : %u octets\nReboots       : %u\nErreurs cam   : %u\nReinit cam    : %u\nDetections    : %u\nAllumages     : %u\nBascul relais : %u\nMutex timeout : %u\nMode          : %s\nMode nuit     : %s\nServo         : %d deg\nServo retour  : %s\nLumiere       : %s\n--- Ameliorations ---\nPause IA: %d ms\nScan servo: 1/%d cycles\nHeure: %s\n",
        hours, minutes, seconds, heureStr, freeHeap, health.heapMin==UINT32_MAX?freeHeap:health.heapMin,
        health.reboots, health.cameraErrors, health.cameraReinits, health.detectionCount, health.lightOnCount,
        health.relayToggleCount, health.mutexTimeouts, manualMode?"MANUEL":"AUTO", isNightMode()?"OUI":"NON",
        curAngle, servoReturning?"OUI":"NON", isLightPhysicallyOn()?"ON":"OFF", AI_ANALYSIS_DELAY_MS, SERVO_SCAN_EVERY,
        realTimeAvailable?"OK":"NON");
    httpd_resp_send(req, buf, strlen(buf));
    return ESP_OK;
}

static esp_err_t index_handler(httpd_req_t *req) {
    const char* html = R"rawliteral(
<!DOCTYPE html><html><head><title>ESP32-CAM Detection</title><meta name="viewport" content="width=device-width, initial-scale=1"><style>
*{box-sizing:border-box;margin:0;padding:0;}body{font-family:Arial;background:#1c1c1c;color:#fff;padding:16px;text-align:center;}h1{color:#0f0;font-size:18px;margin-bottom:14px;}img{max-width:100%;border:2px solid #0f0;border-radius:6px;display:block;margin:0 auto 12px;}#result-box{display:block;padding:10px 20px;border-radius:6px;font-size:1.1em;font-weight:bold;margin:0 auto 10px;max-width:480px;}#result-box.oui{background:#1a5c1a;border:2px solid #0f0;color:#0f0;}#result-box.non{background:#5c1a1a;border:2px solid #f44;color:#f44;}#result-box.attente{background:#333;border:2px solid #888;color:#aaa;}.badge-row{display:flex;justify-content:space-between;max-width:480px;margin:0 auto 10px;flex-wrap:wrap;gap:6px;}.badge{display:inline-flex;align-items:center;gap:6px;padding:5px 12px;border-radius:20px;font-size:12px;font-weight:bold;}.badge-light-on{background:#3d3d00;border:1.5px solid #ff0;color:#ff0;}.badge-light-off{background:#222;border:1.5px solid #555;color:#555;}.badge-auto{background:#002244;border:1.5px solid #08f;color:#08f;}.badge-manual{background:#3d1a00;border:1.5px solid #f80;color:#f80;}.badge-nuit{background:#1a1a3d;border:1.5px solid #88f;color:#88f;}.badge-heure{background:#003300;border:1.5px solid #0f0;color:#0f0;}.dot{width:8px;height:8px;border-radius:50%;display:inline-block;}.dot-on{background:#ff0;}.dot-off{background:#444;}.dot-auto{background:#08f;}.dot-manual{background:#f80;animation:blink 1s step-start infinite;}.dot-nuit{background:#88f;}@keyframes blink{50%{opacity:0;}}.section{background:#252525;border-radius:8px;border:1px solid #444;padding:12px 14px;margin:8px auto;max-width:480px;}.sec-title{font-size:11px;color:#777;text-transform:uppercase;letter-spacing:1px;margin-bottom:10px;}.btn-row{display:flex;gap:8px;}.btn{flex:1;padding:10px 0;border-radius:6px;font-size:13px;font-weight:bold;border:none;cursor:pointer;}.btn:active{opacity:0.7;}.btn-on{background:#1a5c1a;color:#0f0;border:2px solid #0f0;}.btn-off{background:#5c1a1a;color:#f44;border:2px solid #f44;}.btn-auto{background:#222;color:#08f;border:2px solid #08f;font-size:12px;width:100%;padding:8px 0;margin-top:8px;cursor:pointer;border-radius:6px;}.btn-health{background:#222;color:#8f8;border:2px solid #8f8;font-size:11px;width:100%;padding:6px 0;margin-top:6px;cursor:pointer;border-radius:6px;}.info-row{display:flex;justify-content:space-between;font-size:11px;color:#666;margin-top:8px;}.servo-ctrl{display:flex;align-items:center;justify-content:center;gap:20px;margin:8px 0;}.arrow-btn{width:56px;height:56px;border-radius:50%;border:2px solid #0af;background:#002233;color:#0af;font-size:26px;cursor:pointer;display:flex;align-items:center;justify-content:center;}.arrow-btn:hover{background:#003344;}.arrow-btn:active{background:#0af;color:#000;}.angle-display{text-align:center;min-width:70px;}.angle-val{font-size:30px;font-weight:bold;color:#0af;display:block;}.angle-lbl{font-size:11px;color:#555;}.bar-bg{background:#333;border-radius:4px;height:6px;width:100%;margin-top:8px;}.bar-fill{background:#0af;border-radius:4px;height:6px;width:0%;transition:width 0.2s;}.bar-labels{display:flex;justify-content:space-between;font-size:10px;color:#555;margin-top:3px;}#health-box{background:#111;border:1px solid #3a3a3a;border-radius:6px;padding:10px;text-align:left;font-family:monospace;font-size:11px;color:#8f8;white-space:pre;margin-top:8px;display:none;}#update-info{color:#555;font-size:11px;margin-top:10px;}.night-banner{display:none;background:#1a1a3d;border:1px solid #88f;border-radius:6px;padding:8px;color:#88f;font-size:12px;margin:0 auto 8px;max-width:480px;}</style>
</head><body><h1>ESP32-CAM — Detection de personne</h1><div class="night-banner" id="night-banner">&#128315; Mode nuit actif — Servo en pause</div>
<img id="streamImg" src="" /><div id="result-box" class="attente">En attente...</div>
<div class="badge-row"><span class="badge badge-light-off" id="light-badge"><span class="dot dot-off" id="light-dot"></span><span id="light-text">Lumiere : ETEINTE</span></span><span class="badge badge-auto" id="mode-badge"><span class="dot dot-auto" id="mode-dot"></span><span id="mode-text">Mode : AUTO</span></span><span class="badge badge-nuit" id="night-badge" style="display:none;"><span class="dot dot-nuit"></span><span>Nuit</span></span><span class="badge badge-heure" id="heure-badge" style="display:none;"><span id="heure-text">--:--</span></span></div>
<div class="section"><div class="sec-title">Controle lumiere</div><div class="btn-row"><button class="btn btn-on" onclick="setLight('on')">Allumer</button><button class="btn btn-off" onclick="setLight('off')">Eteindre</button></div><button class="btn-auto" onclick="setLight('auto')">Rendre au mode automatique (IA)</button><div class="info-row"><span id="timer-txt"></span><span id="mode-info-txt">Mode : AUTO</span></div></div>
<div class="section"><div class="sec-title">Controle servo</div><div class="servo-ctrl"><button class="arrow-btn" onclick="moveServo('left')">&#8592;</button><div class="angle-display"><span class="angle-val" id="angle-val">0&deg;</span><span class="angle-lbl">position</span></div><button class="arrow-btn" onclick="moveServo('right')">&#8594;</button></div><div class="bar-bg"><div class="bar-fill" id="servo-bar"></div></div><div class="bar-labels"><span>0&deg;</span><span>90&deg;</span><span>180&deg;</span></div><div class="info-row"><span>Pas : 10&deg; par clic</span><span>Max : 180&deg;</span></div></div>
<div class="section"><div class="sec-title">Sante du systeme</div><button class="btn-health" onclick="toggleHealth()">Afficher / Masquer les diagnostics</button><div id="health-box">Chargement...</div></div>
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
var lb=document.getElementById('light-badge'),ld=document.getElementById('light-dot'),lt=document.getElementById('light-text');if(light==='ON'){lb.className='badge badge-light-on';ld.className='dot dot-on';lt.textContent='Lumiere : ALLUMEE';}else{lb.className='badge badge-light-off';ld.className='dot dot-off';lt.textContent='Lumiere : ETEINTE';}
var mb=document.getElementById('mode-badge'),md=document.getElementById('mode-dot'),mt=document.getElementById('mode-text');if(mode==='MANUEL'){mb.className='badge badge-manual';md.className='dot dot-manual';mt.textContent='Mode : MANUEL';document.getElementById('mode-info-txt').textContent='Mode : Manuel';}else{mb.className='badge badge-auto';md.className='dot dot-auto';mt.textContent='Mode : AUTO';document.getElementById('mode-info-txt').textContent='Mode : Auto (IA)';}
var nb=document.getElementById('night-badge'),nb2=document.getElementById('night-banner');if(phase==='NUIT'){nb.style.display='inline-flex';nb2.style.display='block';}else{nb.style.display='none';nb2.style.display='none';}
var hb=document.getElementById('heure-badge'),ht=document.getElementById('heure-text');if(heure!=='??:??'){hb.style.display='inline-flex';ht.textContent=heure;}else hb.style.display='none';
var tb=document.getElementById('timer-txt');if(light==='ON'&&seconds>0){var min=Math.floor(seconds/60),sec=seconds%60;tb.textContent='Extinction dans : '+min+'min '+sec+'s';}else tb.textContent='';
updateBar(angle);var now=new Date();var h=now.getHours().toString().padStart(2,'0'),m=now.getMinutes().toString().padStart(2,'0'),s=now.getSeconds().toString().padStart(2,'0');document.getElementById('update-info').textContent='Derniere mise a jour : '+h+':'+m+':'+s;if(healthVisible)fetchHealth();}).catch(function(e){errorCount++;document.getElementById('update-info').textContent='Reconnexion... (tentative '+errorCount+')';});}
updateStatus();setInterval(updateStatus,1000);
</script></body></html>
)rawliteral";
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_send(req, html, strlen(html));
    return ESP_OK;
}

// ============================================================
// HANDLER STREAM (port 81)
// ============================================================
static esp_err_t stream_handler(httpd_req_t *req) {
    esp_err_t res = ESP_OK;
    char part_buf[64];
    if (xSemaphoreTake(client_mutex, pdMS_TO_TICKS(200)) == pdTRUE) { client_connected = true; xSemaphoreGive(client_mutex); }
    LOG_INFO("STREAM", "Client connecte");
    res = httpd_resp_set_type(req, "multipart/x-mixed-replace;boundary=123456789000000000000987654321");
    if (res != ESP_OK) { if (xSemaphoreTake(client_mutex, pdMS_TO_TICKS(200)) == pdTRUE) { client_connected = false; xSemaphoreGive(client_mutex); } return res; }
    uint32_t timeout_counter = 0;
    while (true) {
        bool frame_sent = false;
        if (xSemaphoreTake(stream_mutex, pdMS_TO_TICKS(150)) == pdTRUE) {
            if (stream_buffer != NULL && stream_len > 0) {
                res = httpd_resp_send_chunk(req, "\r\n--123456789000000000000987654321\r\n", strlen("\r\n--123456789000000000000987654321\r\n"));
                if (res == ESP_OK) { size_t hlen = snprintf(part_buf, sizeof(part_buf), "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n", stream_len); res = httpd_resp_send_chunk(req, part_buf, hlen); }
                if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char*)stream_buffer, stream_len);
                if (res == ESP_OK) frame_sent = true;
            }
            xSemaphoreGive(stream_mutex);
        }
        if (res != ESP_OK) break;
        if (!frame_sent) { timeout_counter += 50; if (timeout_counter >= 5000) break; } else timeout_counter = 0;
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (xSemaphoreTake(client_mutex, pdMS_TO_TICKS(200)) == pdTRUE) { client_connected = false; xSemaphoreGive(client_mutex); }
    LOG_INFO("STREAM", "Client deconnecte");
    return res;
}

// ============================================================
// DÉMARRAGE DES SERVEURS
// ============================================================
bool startStreamServer() {
    httpd_config_t config80 = HTTPD_DEFAULT_CONFIG();
    config80.server_port = 80;
    config80.max_open_sockets = 7;
    config80.lru_purge_enable = true;
    httpd_uri_t index_uri = { "/", HTTP_GET, index_handler, NULL };
    httpd_uri_t status_uri = { "/status", HTTP_GET, status_handler, NULL };
    httpd_uri_t health_uri = { "/health", HTTP_GET, health_handler, NULL };
    httpd_uri_t light_on_uri = { "/light/on", HTTP_GET, light_on_handler, NULL };
    httpd_uri_t light_off_uri = { "/light/off", HTTP_GET, light_off_handler, NULL };
    httpd_uri_t light_auto_uri = { "/light/auto", HTTP_GET, light_auto_handler, NULL };
    httpd_uri_t servo_left_uri = { "/servo/left", HTTP_GET, servo_left_handler, NULL };
    httpd_uri_t servo_right_uri = { "/servo/right", HTTP_GET, servo_right_handler, NULL };
    if (httpd_start(&httpd_80, &config80) != ESP_OK) { LOG_ERROR("SERVER", "Echec port 80"); return false; }
    httpd_register_uri_handler(httpd_80, &index_uri);
    httpd_register_uri_handler(httpd_80, &status_uri);
    httpd_register_uri_handler(httpd_80, &health_uri);
    httpd_register_uri_handler(httpd_80, &light_on_uri);
    httpd_register_uri_handler(httpd_80, &light_off_uri);
    httpd_register_uri_handler(httpd_80, &light_auto_uri);
    httpd_register_uri_handler(httpd_80, &servo_left_uri);
    httpd_register_uri_handler(httpd_80, &servo_right_uri);
    LOG_INFO("SERVER", "Port 80 OK");
    httpd_config_t config81 = HTTPD_DEFAULT_CONFIG();
    config81.server_port = 81;
    config81.max_open_sockets = 3;
    config81.lru_purge_enable = true;
    config81.ctrl_port = 32769;
    httpd_uri_t stream_uri = { "/stream", HTTP_GET, stream_handler, NULL };
    if (httpd_start(&httpd_81, &config81) != ESP_OK) { LOG_ERROR("SERVER", "Echec port 81"); return false; }
    httpd_register_uri_handler(httpd_81, &stream_uri);
    LOG_INFO("SERVER", "Port 81 OK (stream)");
    return true;
}

// ============================================================
// FONCTION DE LECTURE POUR EDGE IMPULSE
// ============================================================
static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr) {
    size_t pixel_ix = offset * 3;
    size_t pixels_left = length;
    size_t out_ptr_ix = 0;
    while (pixels_left != 0) {
        out_ptr[out_ptr_ix] = (ia_rgb_buffer[pixel_ix + 2] << 16) + (ia_rgb_buffer[pixel_ix + 1] << 8) + ia_rgb_buffer[pixel_ix];
        out_ptr_ix++; pixel_ix += 3; pixels_left--;
    }
    return 0;
}

// ============================================================
// TÂCHE DE CAPTURE RAPIDE (CORE 1)
// ============================================================
void fastCaptureTask(void* pvParameters) {
    LOG_INFO("TASK", "Capture rapide demarree (Core 1)");
    esp_task_wdt_add(NULL);
    while (true) {
        esp_task_wdt_reset();
        camera_fb_t* fb = esp_camera_fb_get();
        if (!fb) {
            health.cameraErrors++; cameraErrorCount++;
            LOG_ERROR("CAM", "Erreur capture (%d/%d)", cameraErrorCount, CAM_MAX_ERRORS);
            if (cameraErrorCount >= CAM_MAX_ERRORS) {
                if (reinitCamera()) cameraErrorCount = 0;
                else if (cameraReinitCount >= CAM_MAX_REINIT) esp_restart();
            }
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }
        cameraErrorCount = 0;
        if (xSemaphoreTake(stream_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            if (fb->len <= EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * 2) {
                memcpy(stream_buffer, fb->buf, fb->len);
                stream_len = fb->len;
            } else { stream_len = 0; }
            xSemaphoreGive(stream_mutex);
        }
        if (xSemaphoreTake(ia_request_sem, 0) == pdTRUE) {
            bool conv = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, ia_rgb_buffer);
            if (conv) {
                if ((EI_CLASSIFIER_INPUT_WIDTH != EI_CAMERA_RAW_FRAME_BUFFER_COLS) ||
                    (EI_CLASSIFIER_INPUT_HEIGHT != EI_CAMERA_RAW_FRAME_BUFFER_ROWS)) {
                    ei::image::processing::crop_and_interpolate_rgb888(
                        ia_rgb_buffer,
                        EI_CAMERA_RAW_FRAME_BUFFER_COLS,
                        EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
                        ia_rgb_buffer,
                        EI_CLASSIFIER_INPUT_WIDTH,
                        EI_CLASSIFIER_INPUT_HEIGHT
                    );
                }
                xSemaphoreGive(ia_ready_sem);
            } else {
                xSemaphoreGive(ia_request_sem);
            }
        }
        esp_camera_fb_return(fb);
    }
}

// ============================================================
// TÂCHE IA (CORE 0)
// ============================================================
void iaTask(void* pvParameters) {
    LOG_INFO("TASK", "Tâche IA demarree (Core 0)");
    esp_task_wdt_add(NULL);
    while (true) {
        esp_task_wdt_reset();
        xSemaphoreGive(ia_request_sem);
        if (xSemaphoreTake(ia_ready_sem, pdMS_TO_TICKS(2000)) != pdTRUE) {
            LOG_WARNING("IA", "Timeout attente image");
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }
        ei::signal_t signal;
        signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
        signal.get_data = &ei_camera_get_data;
        ei_impulse_result_t result = {0};
        EI_IMPULSE_ERROR err = run_classifier(&signal, &result, false);
        if (err != EI_IMPULSE_OK) { LOG_ERROR("IA", "Erreur classifier : %d", err); vTaskDelay(pdMS_TO_TICKS(200)); continue; }
        float best_score = 0.0f;
        char new_label[64] = "Rien detecte";
        bool new_person = false;
#if EI_CLASSIFIER_OBJECT_DETECTION == 1
        for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
            ei_impulse_result_bounding_box_t bb = result.bounding_boxes[i];
            if (bb.value == 0) continue;
            if (bb.value > best_score) {
                best_score = bb.value;
                snprintf(new_label, sizeof(new_label), "%s : %.1f%%", bb.label, bb.value*100);
                new_person = ((strstr(bb.label, "person") != NULL || strstr(bb.label, "personne") != NULL) && best_score > 0.80f);
            }
        }
#else
        for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
            if (result.classification[i].value > best_score) {
                best_score = result.classification[i].value;
                snprintf(new_label, sizeof(new_label), "%s : %.1f%%", ei_classifier_inferencing_categories[i], best_score*100);
                new_person = ((strstr(ei_classifier_inferencing_categories[i], "person") != NULL ||
                              strstr(ei_classifier_inferencing_categories[i], "personne") != NULL)
                              && best_score > 0.80f);
            }
        }
#endif
        if (xSemaphoreTake(detection_mutex, pdMS_TO_TICKS(200)) == pdTRUE) {
            strncpy(detection_label, new_label, 63);
            detection_label[63] = 0;
            detection_score = best_score;
            xSemaphoreGive(detection_mutex);
        } else health.mutexTimeouts++;
        if (!manualMode) gererLumiere(new_person);
        else LOG_INFO("LUMIERE", "Mode manuel — IA suspendue");
        int curAngleLog = 0;
        if (xSemaphoreTake(servo_mutex, pdMS_TO_TICKS(50)) == pdTRUE) { curAngleLog = servoAngle; xSemaphoreGive(servo_mutex); }
        LOG_INFO("IA", "%s | Personne:%s | Lum:%s | Mode:%s | Nuit:%s | Servo:%ddeg | Heap:%u",
                 new_label, new_person?"OUI":"NON", isLightPhysicallyOn()?"ON":"OFF",
                 manualMode?"MAN":"AUTO", isNightMode()?"OUI":"NON", curAngleLog, esp_get_free_heap_size());
        bool servoManuelApplique = false;
        if (xSemaphoreTake(servo_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            if (manualAngle >= 0) {
                servoAngle = manualAngle; manualAngle = -1; servoManuelApplique = true;
                int ang = servoAngle; xSemaphoreGive(servo_mutex); monServo.write(ang);
                LOG_INFO("SERVO", "Commande manuelle : %d deg", ang);
            } else xSemaphoreGive(servo_mutex);
        } else health.mutexTimeouts++;
        if (!servoManuelApplique && !manualMode) handleServoAuto();
        vTaskDelay(pdMS_TO_TICKS(AI_ANALYSIS_DELAY_MS));
    }
}

// ============================================================
// TÂCHE WIFI + SERVEUR (CORE 0)
// ============================================================
void wifiTask(void* pvParameters) {
    LOG_INFO("TASK", "WiFi client demarree (Core 0)");
    esp_task_wdt_add(NULL);
    // Vérifier que nous sommes connectés (la connexion a été établie dans setup)
    if (WiFi.status() != WL_CONNECTED) {
        LOG_WARNING("WIFI", "Non connecte, tentative de connexion...");
        WiFi.begin(wifi_ssid, wifi_password);
        int attempts = 0;
        while (WiFi.status() != WL_CONNECTED && attempts < 20) {
            delay(500);
            attempts++;
        }
    }
    if (WiFi.status() == WL_CONNECTED) {
        LOG_INFO("WIFI", "Connecte, IP : %s", WiFi.localIP().toString().c_str());
    } else {
        LOG_ERROR("WIFI", "Impossible de se connecter au WiFi, redemarrage...");
        esp_restart();
    }
    if (!startStreamServer()) {
        LOG_FATAL("SERVER", "Serveurs non demarres — reboot");
        vTaskDelay(pdMS_TO_TICKS(3000));
        esp_restart();
    }
    while (true) {
        esp_task_wdt_reset();
        handleLightTimer();
        // Surveillance de la connexion WiFi
        if (WiFi.status() != WL_CONNECTED) {
            LOG_WARNING("WIFI", "Connexion perdue, reconnexion...");
            WiFi.reconnect();
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

// ============================================================
// SETUP
// ============================================================


void setup() {
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
    Serial.begin(115200);
    LOG_INFO("INIT", "=== ESP32-CAM CLIENT WIFI (Centre d'incubation) ===");
    esp_task_wdt_init(WDT_TIMEOUT_SEC, true);
    // LED PWM
    ledcSetup(LEDC_CHANNEL, LEDC_FREQ, LEDC_RESOLUTION);
    ledcAttachPin(CAM_LED_PIN, LEDC_CHANNEL);
    ledcWrite(LEDC_CHANNEL, 0);
    pinMode(RELAY_PIN, OUTPUT);
    pinMode(LDR_PIN, INPUT);
    digitalWrite(RELAY_PIN, LOW);
    // Connexion WiFi
    LOG_INFO("WIFI", "Connexion a %s...", wifi_ssid);
    WiFi.mode(WIFI_STA);
    WiFi.begin(wifi_ssid, wifi_password);
    unsigned long startWifi = millis();
    bool wifiConnected = false;
    while (millis() - startWifi < 20000) { // 20 secondes max
        if (WiFi.status() == WL_CONNECTED) { wifiConnected = true; break; }
        delay(500);
        ledcWrite(LEDC_CHANNEL, LEDC_DUTY_DIM); delay(100);
        ledcWrite(LEDC_CHANNEL, 0); delay(400);
    }
    if (!wifiConnected) {
        LOG_ERROR("WIFI", "Echec connexion WiFi, redemarrage...");
        esp_restart();
    }
    LOG_INFO("WIFI", "Connecte, IP : %s", WiFi.localIP().toString().c_str());
    // Synchronisation NTP
    if (!syncTimeNTP()) {
        LOG_WARNING("NTP", "Heure non disponible, mode nuit base sur uptime");
    }

    initLumiere();
    // Servo
    ESP32PWM::allocateTimer(3);
    monServo.setPeriodHertz(50);
    monServo.attach(SERVO_PIN, 500, 2400);
    monServo.write(0);
    LOG_INFO("SERVO", "Initialise a 0 deg");
    // Caméra
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        LOG_FATAL("CAM", "Erreur init : 0x%x", err);
        while(1) delay(1000);
    }
    sensor_t* s = esp_camera_sensor_get();
    if (s->id.PID == OV3660_PID) { s->set_vflip(s,1); s->set_brightness(s,1); s->set_saturation(s,0); }
    else { s->set_vflip(s,1); s->set_hmirror(s,1); }
    LOG_INFO("CAM", "Initialisee");
    // Mutex et sémaphores
    stream_mutex = xSemaphoreCreateMutex();
    detection_mutex = xSemaphoreCreateMutex();
    light_mutex = xSemaphoreCreateMutex();
    servo_mutex = xSemaphoreCreateMutex();
    relay_mutex = xSemaphoreCreateMutex();
    client_mutex = xSemaphoreCreateMutex();
    ia_request_sem = xSemaphoreCreateBinary();
    ia_ready_sem = xSemaphoreCreateBinary();
    if (!stream_mutex || !detection_mutex || !light_mutex || !servo_mutex || !relay_mutex || !client_mutex || !ia_request_sem || !ia_ready_sem) {
        LOG_FATAL("INIT", "Echec creation mutex — reboot");
        esp_restart();
    }
    // Buffers
    stream_buffer = (uint8_t*)malloc(EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * 2);
    ia_rgb_buffer = (uint8_t*)malloc(EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * EI_CAMERA_FRAME_BYTE_SIZE);
    if (!stream_buffer || !ia_rgb_buffer) {
        LOG_FATAL("MEM", "Allocation buffers echouee");
        esp_restart();
    }
    LOG_INFO("MEM", "Buffers alloues");
    // NVS
    prefs.begin("esp32cam", true);
    manualMode = prefs.getBool("manualMode", false);
    health.reboots = prefs.getUInt("reboots", 0) + 1;
    prefs.end();
    lastSavedMode = manualMode;
    prefs.begin("esp32cam", false);
    prefs.putUInt("reboots", health.reboots);
    prefs.end();
    LOG_INFO("NVS", "Reboots: %u | Mode: %s", health.reboots, manualMode ? "MANUEL" : "AUTO");
    health.bootTime = millis();
    initLumiere();
    // Création des tâches
    xTaskCreatePinnedToCore(fastCaptureTask, "FastCapture", 8192, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(iaTask, "IATask", 16384, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(wifiTask, "WiFi", 8192, NULL, 1, NULL, 0);
    LOG_INFO("INIT", "Systeme pret → http://%s", WiFi.localIP().toString().c_str());
    LOG_INFO("INIT", "Diagnostics → http://%s/health", WiFi.localIP().toString().c_str());
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(10000));
}
