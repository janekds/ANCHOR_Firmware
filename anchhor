#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#define ENABLE_LEGACY_TOKEN
#define ENABLE_DATABASE
#define ENABLE_ESP_SSLCLIENT
#include <FirebaseClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ArduinoJson.h>
#include "time.h"
#include <WiFiUdp.h>
#include <Preferences.h>
#include <SPIFFS.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <HTTPClient.h>
#include <Update.h>
#include <Wire.h>
#include <DIYables_TFT_Round.h>
#include <SPI.h>

// Pin definitions
#define FLOAT_PIN 35
#define ESTOP_PIN 25
#define ONE_WIRE_BUS 18
#define MTR_ATO 19
#define MTR_DOSE1 27
#define MTR_DOSE2 32
#define MTR_DOSE3 33
#define MTR_WATER_CHANGE_OUT 26
#define MTR_WATER_CHANGE_IN 22

// TFT Display Configuration
#define RED DIYables_TFT::colorRGB(255, 0, 0)
#define BLUE DIYables_TFT::colorRGB(0, 0, 255)
#define WHITE DIYables_TFT::colorRGB(255, 255, 255)
#define BLACK DIYables_TFT::colorRGB(0, 0, 0)
#define GREEN DIYables_TFT::colorRGB(0, 255, 0)
#define YELLOW DIYables_TFT::colorRGB(255, 255, 0)
#define ORANGE DIYables_TFT::colorRGB(255, 165, 0)

// Display rotation: 0..3. Use 1 for 90 degrees
#define DISPLAY_ROTATION 0

#define PIN_RST 5 // The ESP32 pin GPIO15 connected to the RST pin of the circular TFT display
#define PIN_DC 2   // The ESP32 pin GPIO2 connected to the DC pin of the circular TFT display
#define PIN_CS 4   // The ESP32 pin GPIO4 connected to the CS pin of the circular TFT display

DIYables_TFT_GC9A01_Round TFT_display(PIN_RST, PIN_DC, PIN_CS);

// Global variables (moved to top to avoid forward reference issues)
float temperatureF ;
char uuidStr[33];  // Increased to avoid truncation (supports long numeric UUIDs)
bool dosedAmountsLoaded = false; // Flag to track if dosed amounts have been loaded from Firebase

// TFT refresh timer
unsigned long lastTFTRefresh = 0;
const unsigned long TFT_REFRESH_INTERVAL = 60000; // 1 minute in milliseconds

// WiFi connection attempt tracking
int wifiConnectionAttempts = 0;
const int MAX_WIFI_ATTEMPTS = 5;
bool wifiMaxAttemptsReached = false;

// Global display name for this Anchor (sanitized email or saved name)
char anchorName[64] = "";
// Global sanitized email for pathing and display fallback
char sanitizedEmailStr[64] = "";

// E-stop and system status variables
char estopStatus[8] = "OK";
char pumpState[8] = "STOP";
char wavemakerState[8] = "OFF";

// TFT Display state tracking for partial updates
struct DisplayState {
    char lastAnchorName[64] = "";
    float lastTemperatureF = 0.0;
    char lastTimeStr[6] = "";
    bool lastWifiStatus = false;
    char lastMotorName[20] = "";
    bool lastMotorRunning = false;
    unsigned long lastRemainingMs = 0;
    char lastRemainingText[16] = ""; // Track last time remaining text for proper clearing
    bool displayInitialized = false;
    // Animation state for rotating pulsating circle
    unsigned long lastAnimationUpdate = 0;
    float animationAngle = 0.0;
    float pulsePhase = 0.0;
    // Track last full screen reload during water change
    unsigned long lastFullReloadTime = 0;
} displayState;

// Forward declarations and struct definitions
void otaTask(void *parameter);
void checkOTAUpdateFromFirebase();
bool shouldUpdateFirmware();
bool downloadFirmwareChunk();
const char* getEmptyContainerName();
void processEStop();
void handleSystemErrors();

// Forward declaration of DoseSettings struct
struct DoseSettings;
bool shouldRunScheduledDose(int doseNum, DoseSettings &dose);

struct DoseSettings {
  int amount;
  char days[8];      // Use char array instead of String
  char time[6];      // Use char array instead of String
  
  bool dosedToday;
  int lastDoseDay;
  int splitCount = 4;
  int currentSplit = 0;
  unsigned long lastSplitTime = 0;
  unsigned long nextSplit;
};

// Forward declarations for TFT display functions
void renderWatchFaceIdle();
void renderWatchFaceMotor(const char* motorName);
void displayEmptyContainerWarning(const char* containerName);
void updateContainerStatus();
void drawWatchRings();
void drawRotatingPulsatingCircle();

// Forward declarations for WiFi functions
void startCaptivePortal();
void resetWiFiAttemptCounter();
void buildPath(char* path, const char* base, const char* suffix);
void batchWriteToFirebase();
void printCurrentSettings(const char* settingsJson = nullptr);
void setMotorState(int pin, bool state);
void validateWaterChangeState();


void resetWatchdog();
void resetManualWaterChangeFlags();
void resetContainerDosedAmounts(int containerType);
void endWaterChange(bool stoppedByEstop = false);
void loadDosedAmountsFromFirebase();
void requestAtoDosedFromFirebase();
void requestAllDosedAmountsFromFirebase();
void queueAtoVolumeIncrement(float addGal);

// TFT Display Functions
// Helper: compute pixel width for default font (approx 6 px per char at size 1)
int textWidthPixels(const char* text, uint8_t textSize) {
    if (text == nullptr) return 0;
    size_t len = strlen(text);
    return static_cast<int>(len) * 6 * textSize;
}

// Helper: compute pixel height for default font (approx 8 px at size 1)
int textHeightPixels(uint8_t textSize) {
    return 8 * textSize;
}

// Helper: print a centered line at y with given size and color
void printCenteredLine(int y, const char* text, uint8_t textSize, uint16_t color) {
    TFT_display.setTextSize(textSize);
    TFT_display.setTextColor(color);
    int x = (TFT_display.width() - textWidthPixels(text, textSize)) / 2;
    if (x < 0) x = 0;
    TFT_display.setCursor(x, y);
    TFT_display.println(text);
}

// Helper: clear a rectangular area for partial updates
void clearRect(int x, int y, int width, int height) {
    TFT_display.fillRect(x, y, width, height, BLACK);
}

// Helper: update text with partial screen clearing
void updateTextLine(int y, const char* newText, const char* oldText, uint8_t textSize, uint16_t color) {
    if (strcmp(newText, oldText) != 0) {
        // Calculate width of both old and new text to ensure we clear enough area
        int newTextWidth = textWidthPixels(newText, textSize);
        int oldTextWidth = textWidthPixels(oldText, textSize);
        int textWidth = (newTextWidth > oldTextWidth) ? newTextWidth : oldTextWidth;
        int textHeight = textHeightPixels(textSize);
        int x = (TFT_display.width() - newTextWidth) / 2; // Center based on new text
        if (x < 0) x = 0;
        
        // Clear a bit more area to ensure old text is completely erased
        clearRect(x - 2, y - 2, textWidth + 4, textHeight + 4);
        
        // Draw the new text
        printCenteredLine(y, newText, textSize, color);
    }
}

// Function to reset display state (useful for major changes or errors)
void resetDisplayState() {
    memset(&displayState, 0, sizeof(displayState));
    displayState.displayInitialized = false;
    TFT_display.fillScreen(BLACK);
    drawWatchRings();
    displayState.displayInitialized = true;
}

// Function to perform complete TFT screen refresh to clear any legacy text
void refreshTFTDisplay() {
    Serial.println("Performing complete TFT screen refresh");
    
    // Clear the entire screen
    TFT_display.fillScreen(BLACK);
    
    // Reset display state to force complete redraw
    memset(&displayState, 0, sizeof(displayState));
    displayState.displayInitialized = false;
    
    // Redraw the watch rings
    drawWatchRings();
    
    // Mark display as initialized
    displayState.displayInitialized = true;
    
    // Update the refresh timer
    lastTFTRefresh = millis();
    
    Serial.println("TFT screen refresh completed");
}

void initTFT() {
    SPI.end();
    SPI.begin(/*sck=*/16, /*miso=*/-1, /*mosi=*/15, /*ss=*/-1);
    TFT_display.begin();
    TFT_display.setRotation(DISPLAY_ROTATION);  // Rotate screen 90 degrees
    TFT_display.fillScreen(BLACK);
    TFT_display.setTextSize(2);
    TFT_display.setTextColor(WHITE);
    
    // Initialize display state
    memset(&displayState, 0, sizeof(displayState));
    displayState.displayInitialized = false;
    
    Serial.println(F("TFT Display initialized"));
}

void displaySetupInfo(const char* ssid, const char* ip) {
    drawAnchorLogo(); // Show anchor logo during setup
    
    // Add setup information below logo
    int baseY = (TFT_display.height() * 65) / 100;
    printCenteredLine(baseY - 40, "Anchor Setup", 1, WHITE);
    
    printCenteredLine(baseY - 25, "Connect to WiFi:", 1, WHITE);
    
    printCenteredLine(baseY - 10, ssid, 1, YELLOW);
    
    char visitBuf[40];
    snprintf(visitBuf, sizeof(visitBuf), "Visit: %s", ip);
    printCenteredLine(baseY + 5, visitBuf, 1, GREEN);
}

void displayWiFiFailureSetup(const char* ssid, const char* ip) {
    // Clear screen and show only "Configure WiFi"
    TFT_display.fillScreen(BLACK);
    
    // Center the text on screen
    int centerY = TFT_display.height() / 2;
    printCenteredLine(centerY, "Configure WiFi", 2, WHITE);
}

void displayStatus(const char* status) {
    drawAnchorLogo(); // Show anchor logo with status
    
    // Add status message below logo
    char statusBuf[50];
    snprintf(statusBuf, sizeof(statusBuf), "Status: %s", status);
    printCenteredLine((TFT_display.height() * 70) / 100, statusBuf, 1, YELLOW);
}

void displayConnecting(const char* ssid) {
    drawAnchorLogo(); // Show anchor logo during connection
    
    // Add simple connection status below logo
    int y = (TFT_display.height() * 68) / 100;
    printCenteredLine(y, "connecting to wifi", 1, YELLOW);
}

void displayConnected(const char* ip) {
    drawAnchorLogo(); // Show anchor logo with success message
    
    // Add success message below logo
    int y = (TFT_display.height() * 68) / 100;
    printCenteredLine(y, "Connected!", 1, GREEN);
    
    char ipBuf[40];
    snprintf(ipBuf, sizeof(ipBuf), "IP: %s", ip);
    printCenteredLine(y + 16, ipBuf, 1, WHITE);
    
    delay(2000);
}

// New TFT Display Functions for Anchor Logo and State-based Display

// Draw anchor logo in center of screen
void drawAnchorLogo() {
    TFT_display.fillScreen(BLACK);
    
    // Calculate center position
    int centerX = TFT_display.width() / 2;
    // Center slightly lower (around 60% of height)
    int centerY = (TFT_display.height() * 60) / 100;
    
    // Draw "anchor" text in center
    printCenteredLine(centerY, "anchor", 4, WHITE);
}

// Display loading screen with anchor logo
void displayLoading() {
    TFT_display.fillScreen(BLACK);
    // Center the word 'anchor' prominently
    int textSize = 4;
    int y = (TFT_display.height() - textHeightPixels(textSize)) / 2;
    printCenteredLine(y, "anchor", textSize, WHITE);
}

// Display idle state information (temperature, tank name, time)
void displayIdleState() {
    renderWatchFaceIdle();
}

// Display motor running state
void displayMotorRunning(const char* motorName) {
    renderWatchFaceMotor(motorName);
}

void updateMainDisplay() {
    static unsigned long lastDisplayUpdate = 0;
    if (millis() - lastDisplayUpdate < 1000) return; // Update every second
    lastDisplayUpdate = millis();
    
    // Check if any motors are running
    bool motorRunning = false;
    char motorName[20] = "";
    
    // Check water change motors
    if (digitalRead(MTR_WATER_CHANGE_IN) == HIGH) {
        motorRunning = true;
        strcpy(motorName, "Water In");
    } else if (digitalRead(MTR_WATER_CHANGE_OUT) == HIGH) {
        motorRunning = true;
        strcpy(motorName, "Water Out");
    }
    
    // Check dosing motors
    if (digitalRead(MTR_DOSE1) == HIGH) {
        motorRunning = true;
        strcpy(motorName, "Dose 1");
    } else if (digitalRead(MTR_DOSE2) == HIGH) {
        motorRunning = true;
        strcpy(motorName, "Dose 2");
    } else if (digitalRead(MTR_DOSE3) == HIGH) {
        motorRunning = true;
        strcpy(motorName, "Dose 3");
    }
    
    // Check ATO pump
    if (digitalRead(MTR_ATO) == HIGH) {
        motorRunning = true;
        strcpy(motorName, "ATO Pump");
    }
    
    // Handle transition from motor to idle state
    if (!motorRunning && displayState.lastMotorRunning) {
        // Switching from motor to idle - clear and redraw everything
        TFT_display.fillScreen(BLACK);
        drawWatchRings();
        displayState.displayInitialized = true;
        displayState.lastMotorRunning = false;
        // Reset all display state to force redraw
        displayState.lastAnchorName[0] = '\0';
        displayState.lastTemperatureF = 0.0;
        displayState.lastTimeStr[0] = '\0';
        displayState.lastWifiStatus = false;
        displayState.lastRemainingMs = 0;
        // Reset animation state
        displayState.lastAnimationUpdate = 0;
        displayState.animationAngle = 0.0;
        displayState.pulsePhase = 0.0;
        // Reset water change full reload timer
        displayState.lastFullReloadTime = 0;
    }
    
    // Handle transition from E-stop to normal display
    if (strcmp(estopStatus, "OK") == 0 && strcmp(displayState.lastAnchorName, "ESTOP") == 0) {
        // E-stop was cleared - reset display state to force redraw
        displayState.lastAnchorName[0] = '\0';
        displayState.lastTemperatureF = 0.0;
        displayState.lastTimeStr[0] = '\0';
        displayState.lastWifiStatus = false;
        displayState.lastRemainingMs = 0;
        // Reset animation state
        displayState.lastAnimationUpdate = 0;
        displayState.animationAngle = 0.0;
        displayState.pulsePhase = 0.0;
        displayState.displayInitialized = false;
        // Reset water change full reload timer
        displayState.lastFullReloadTime = 0;
        Serial.println("E-stop cleared - resetting display state");
    }
    
    // Handle transition from empty container warning to normal display
    const char* emptyContainer = getEmptyContainerName();
    if (emptyContainer == nullptr && (strstr(displayState.lastAnchorName, "Empty") != nullptr || strstr(displayState.lastAnchorName, "Full") != nullptr)) {
        // Empty container warning was cleared - reset display state to force redraw
        displayState.lastAnchorName[0] = '\0';
        displayState.lastTemperatureF = 0.0;
        displayState.lastTimeStr[0] = '\0';
        displayState.lastWifiStatus = false;
        displayState.lastRemainingMs = 0;
        // Reset animation state
        displayState.lastAnimationUpdate = 0;
        displayState.animationAngle = 0.0;
        displayState.pulsePhase = 0.0;
        displayState.displayInitialized = false;
        // Reset water change full reload timer
        displayState.lastFullReloadTime = 0;
        Serial.println("Empty container warning cleared - resetting display state");
    }
    
    // Check for empty containers first (highest priority after E-stop)
    if (emptyContainer != nullptr) {
        displayEmptyContainerWarning(emptyContainer);
        return;
    }
    
    // Display appropriate screen based on state
    if (motorRunning) {
        displayMotorRunning(motorName);
    } else {
        displayIdleState();
    }
}

// // // Wi-Fi credentials
// #define WIFI_SSID "GoBears"
// #define WIFI_PASSWORD "GoBears123"
// #define WIFI_SSID "Wisk-Guest"
// #define WIFI_PASSWORD "safeairmobility"

// Access Point credentials and wifi connection
#define AP_SSID "Anchor"
// No password - open access point

// Web server for laptop configuration
WebServer webServer(80);
DNSServer dnsServer;
const byte DNS_PORT = 53;

// Firebase configuration
#define DATABASE_SECRET "TnVH0RXPIcmJaXEssviceieY8WUTjR3lqH8NTcfL"
#define DATABASE_URL "https://esp32tankcontroller-default-rtdb.firebaseio.com"


#define FIRMWARE_BUFFER_SIZE 512      // Reduced buffer size

// OTA Configuration
#define FIRMWARE_VERSION "1.0.1"
#define FIRMWARE_NAME "CompleteControllerAppintegration"
#define OTA_CHECK_INTERVAL 60000    // Check every 1 minute
#define OTA_DOWNLOAD_CHUNK_SIZE 512   // Download chunk size for non-blocking operation
#define OTA_TASK_STACK_SIZE 8192      // Stack size for OTA task

// OTA Update States
enum OTAState {
    OTA_IDLE,
    OTA_CHECKING,
    OTA_DOWNLOADING,
    OTA_INSTALLING,
    OTA_COMPLETED,
    OTA_FAILED
};

// OTA Update Structure
struct OTAUpdateInfo {
    char firmwareUrl[256];
    char version[16];
    bool forceUpdate;
    OTAState state;
    unsigned long lastCheck;
    unsigned long downloadStart;
    unsigned long downloadSize;
    unsigned long downloadedBytes;
    bool updateAvailable;
};

OTAUpdateInfo otaInfo = {0};
TaskHandle_t otaTaskHandle = NULL;
SemaphoreHandle_t otaMutex = NULL;
HTTPClient httpClient;

// Database paths - increased to 128 bytes to prevent overflow
char BASE_PATH[128];
char DOSE1_PATH[128];
char DOSE2_PATH[128];
char DOSE3_PATH[128];
char WATER_CHANGE_PATH[128];
char SYSTEM_STATUS_PATH[128];
char SETTINGS_PATH[128];
char MANUAL_CONTROL_PATH[128];
char OTA_BASE_PATH[128];
char OTA_VERSION_PATH[128];
char OTA_URL_PATH[128];
char OTA_FORCE_PATH[128];
char OTA_STATUS_PATH[128];



// Delays and intervals
#define DEBOUNCE_DELAY 50
#define FIREBASE_INTERVAL 10000          // Increased to reduce Firebase calls
#define DOSE_DEBOUNCE_INTERVAL 5000
#define FLOW_RATE_ML_PER_MIN 85.0f
#define ML_PER_GALLON 3785.411784f        // Exact ml in one US gallon
#define MS_PER_MINUTE 60000.0f            // Milliseconds in one minute

// Forward declarations for web server functions
void handleRoot();
void handleNotFound();
void handleSetupAnchor();
void handleScanWiFi();
void handleCaptivePortal();
void handleCaptivePortalRedirect();

// WiFi reconnection function declarations  
void checkWiFiReconnection();
void startCaptivePortal();
void stopCaptivePortal();

// Firebase setup
void asyncCB(AsyncResult &aResult);
void checkWatchdog();
void setMotorState(bool waterIn, bool waterOut);
WiFiClientSecure ssl_client;
using AsyncClient = AsyncClientClass;
AsyncClient aClient(ssl_client);
LegacyToken dbSecret(DATABASE_SECRET);
FirebaseApp app;
RealtimeDatabase Database;

WiFiUDP udp;
Preferences core;
Preferences pref;
unsigned int localUdpPort = 4210;  // Port for receiving and sending data
char incomingPacket[64];  // Reduced from 100
char ssid[32];
char password[50];
char username[30];
bool uuidExists = false;
bool connectedToWifi = false;
IPAddress remoteIP;
unsigned int remotePort;

// Access Point credentials and wifi connection
#define AP_SSID "Anchor"


// Mutex for resource protection
SemaphoreHandle_t floatMutex;

// OneWire setup
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// Global variables
volatile bool estopChanged = false;
volatile bool floatChanged = false;
volatile int floatState = LOW;
volatile int estopState = HIGH;
unsigned long lastFirebaseUpdate = 0;
char lastDoseTime[20] = "Never";
char lastWaterChangeTime[20] = "Never";
bool isWaterChangeActive = false;
unsigned long lastDebounceTime = 0;
volatile bool ignoreFloatSensor = false;
// Serial test overrides: when true, raw pin reads use the simulated level instead of digitalRead.
volatile bool serialFloatOverride = false;
volatile int serialFloatSimLevel = HIGH;
volatile bool serialEstopOverride = false;
volatile int serialEstopSimLevel = LOW;
volatile bool atoRefillRecheckPending = false;
bool atoEmptyNotified = false;
float pendingAtoIncrementGal = 0.0f;
bool atoSyncRequestInFlight = false;
const unsigned long MAX_MANUAL_WATER_CHANGE_TIME = 30000;

// WiFi reconnection variables
unsigned long wifiDisconnectTime = 0;
bool wifiDisconnected = false;
bool captivePortalActive = false;
char backupSSID[32] = "";
char backupPassword[50] = "";
bool hasBackupCredentials = false;

// User email for OTA updates
char userEmail[50] = "";  // Use char array instead of String

// Pump calibration factors (1.0 = no calibration, >1.0 = faster, <1.0 = slower)
float pump1Calibration = 1.0f;
float pump2Calibration = 1.0f;
float pump3Calibration = 1.0f;
float atoCalibration = 1.0f;  // ATO pump calibration factor
// Water change motor calibration factors
float waterChangeInCalibration = 1.0f;
float waterChangeOutCalibration = 1.0f;
// Add with other water change variables
bool operationStarted = false; // Tracks if scheduled water change has started
// Add this with other function declarations
bool shouldStartWaterChange();



DoseSettings dose1 = {0, "0000000", "08:00", false, -1};
DoseSettings dose2 = {0, "0000000", "09:00", false, -1};
DoseSettings dose3 = {0, "0000000", "10:00", false, -1};

char lastDose1Time[20] = "Never";
char lastDose2Time[20] = "Never";
char lastDose3Time[20] = "Never";

// Add with other global variables
struct ContainerVolumes {
  int dose1;
  int dose2;
  int dose3;
  float ato;  // ATO container volume in gallons
  float water_in;  // Fresh water reservoir
  float water_out; // Waste water reservoir
};


ContainerVolumes containerVolumes;

struct DosedAmounts {
  int dose1;
  int dose2;
  int dose3;
  float ato;  // Track ATO water added in gallons
  float water_in;  // Track water added to tank
  float water_out; // Track water removed from tank
};
DosedAmounts dosedAmounts;



struct ContainerStatus {
    bool dose1Empty = false;
    bool dose2Empty = false;
    bool dose3Empty = false;
    bool atoEmpty = false;
    bool waterEmpty = false;
    bool waterFull = false;
};
ContainerStatus containerStatus;



// Add these to your global variables section
unsigned long waterChangeDurationMs = 0; // Will be calculated based on volume
bool isManualWaterChange = false;


// Water change variables
bool isFirstMotorRunning = false;
unsigned long waterChangeStartMillis = 0;
unsigned long waterChangeDuration = 0;
char waterChangeDays[8] = "1000000";
unsigned long waterChangeRemainingMs = 0;

float waterChangeVolume = 0.0f;
char waterChangeTime[6] = "12:00";


struct PreviousValues {
    char pumpState[8];      // Use char arrays instead of Strings
    char estopStatus[8];
    char lastDose1Time[20];
    char lastDose2Time[20];
    char lastDose3Time[20];
    char lastWaterChangeTime[20];
    char wavemakerState[8];
    long waterChangeRemaining = -1;
    bool manualDose1 = false;
    bool manualDose2 = false;
    bool manualDose3 = false;
    bool dose1Empty = false;
    bool dose2Empty = false;
    bool dose3Empty = false;
    bool atoEmpty = false;
    bool waterEmpty = false;
    bool waterFull = false;
    bool primePump1 = false;
    bool primePump2 = false;
    bool primePump3 = false;
    int dose1Volume = 1000;
    int dose2Volume = 1000;
    int dose3Volume = 1000;
    float atoVolume = 5.0f;
    float waterVolume = 5.0;
    float waterInVolume = 5.0;
    float waterOutVolume = 5.0;
    float waterInDosed = 0.0;
    float waterOutDosed = 0.0;
    int dose1 = 0;
    int dose2 = 0;
    int dose3 = 0;
    float ato = 0.0f;
    char waterChangeStatus[16];
    float pump1Calibration = 1.0f;  // Previous value for comparison
    float pump2Calibration = 1.0f;  // Previous value for comparison
    float pump3Calibration = 1.0f;  // Previous value for comparison
    float atoCalibration = 1.0f;  // Previous value for comparison
    float waterChangeInCalibration = 1.0f;  // Previous value for comparison
    float waterChangeOutCalibration = 1.0f;  // Previous value for comparison
};


#define TEMP_UPDATE_INTERVAL 3000 // 3 seconds in milliseconds
#define TEMP_CHANGE_THRESHOLD 0.2    // 2°F change required for update
unsigned long lastTempUpdate = 0;
float lastReportedTemp = 0.0;

PreviousValues previousValues;
bool firebaseNeedsUpdate = false;
// Add with other water change variables
unsigned long lastPhaseChange = 0;
const unsigned long PHASE_COOLDOWN = 2000; // 2 seconds


// Water change state machine for better state management
enum WaterChangeStateEnum {
    WC_IDLE = 0,
    WC_MANUAL_OUT,
    WC_MANUAL_IN,
    WC_SCHEDULED_OUT,
    WC_SCHEDULED_IN,
    WC_COMPLETING,
    WC_ERROR,
    WC_TIMEOUT
};

// Add state machine to WaterChangeState struct
struct WaterChangeState {
    bool active = false;
    bool manual = false;
    bool outPhaseCompleted = false;
    unsigned long startMillis = 0;
    unsigned long phaseStartMillis = 0;
    float targetVolume = 0;
    float processedVolume = 0;
    SemaphoreHandle_t mutex;
    unsigned long durationMs;
    unsigned long remainingMs;
    bool needsRemainingUpdate;
    char status[16];  // Use char array instead of String
    bool needsProgressUpdate = false;
    unsigned long estimatedCompletionTime = 0;
    float actualVolumeProcessed = 0;
    WaterChangeStateEnum state = WC_IDLE;  // Add state machine
    unsigned long lastStateChange = 0;  // Track state changes
    unsigned long lastProgressUpdate = 0;  // Track progress updates
    // Separate variables for processed volumes during each phase
    float processedVolumeOutPhase = 0.0;
    float processedVolumeInPhase = 0.0;
    // Store initial direction for manual water changes (true = waterIn, false = waterOut)
    bool manualWaterInMode = false;
};

WaterChangeState waterChange;

// Motor state tracking
struct MotorState {
    bool waterInRunning = false;
    bool waterOutRunning = false;
    unsigned long lastWaterInToggle = 0;
    unsigned long lastWaterOutToggle = 0;
    bool waterInRequested = false;
    bool waterOutRequested = false;
};
MotorState motorState;

// Activity tracking
unsigned long lastWaterChangeActivity = 0;

// Motor control constants
const unsigned long MOTOR_DEBOUNCE_TIME = 1000; // 1 second debounce

// Task handles
TaskHandle_t floatSensorTaskHandle = NULL;

// Manual dose control
struct {
  volatile bool requested[3] = {false, false, false};  // Dose triggers
  volatile bool completed[3] = {false, false, false}; // Completion flags
} manualDose;
struct {
  volatile bool waterIn = false;
  volatile bool waterOut = false;
  volatile bool completed = false;
  volatile bool manuallyReset = false; // New flag to track if manually reset
  volatile bool stopWaterChange = false; // New flag to stop water change manually
} manualWater;

SemaphoreHandle_t doseMutex;

// Add with other globals
volatile bool manualDoseFlags[3] = {false, false, false}; // For dose1-3


// Time configuration
#define GMT_OFFSET -28800
#define DAYLIGHT_OFFSET 3600
#define NTP_SERVER "pool.ntp.org"

// Firebase connection state
enum FirebaseState {
  FIREBASE_DISCONNECTED,
  FIREBASE_CONNECTING,
  FIREBASE_READY
};
FirebaseState firebaseState = FIREBASE_DISCONNECTED;
unsigned long lastConnectionAttempt = 0;
#define CONNECTION_RETRY_INTERVAL 30000



// Simplified E-Stop ISR using atomic operations
volatile bool estopPressed = false;
volatile unsigned long estopPressTime = 0;

void IRAM_ATTR estopISR() {
    // Just flag a change; main loop reads pin state
    estopChanged = true;
}

void IRAM_ATTR floatSensorISR() {
    if (!ignoreFloatSensor) {  // Only trigger if we're not in water change (ATO disabled during water changes)
        floatChanged = true;
    }
}

// Debugging Helpers

// Check if any containers are empty and return the name of the first empty container
const char* getEmptyContainerName() {
    if (containerStatus.dose1Empty) {
        Serial.println("Container check: Dose 1 is empty");
        return "Dose 1 Empty";
    }
    if (containerStatus.dose2Empty) {
        Serial.println("Container check: Dose 2 is empty");
        return "Dose 2 Empty";
    }
    if (containerStatus.dose3Empty) {
        Serial.println("Container check: Dose 3 is empty");
        return "Dose 3 Empty";
    }
    if (containerStatus.atoEmpty) {
        return "ATO Empty";
    }
    if (containerStatus.waterEmpty) {
        Serial.println("Container check: Water is empty");
        return "Water Empty";
    }
    if (containerStatus.waterFull) {
        Serial.println("Container check: Water is full");
        return "Water Full";
    }
    return nullptr; // No empty containers
}

void displayEmptyContainerWarning(const char* containerName) {
    const int cx = TFT_display.width() / 2;
    const int cy = TFT_display.height() / 2;
    
    // Check if we need to switch to empty container warning display
    if (!displayState.displayInitialized || strcmp(displayState.lastAnchorName, containerName) != 0) {
        // Clear screen and show empty container warning
        TFT_display.fillScreen(ORANGE); // Orange background for warning
        drawWatchRings();
        
        // Show empty container message
        printCenteredLine(cy - 30, "CONTAINER", 2, BLACK);
        printCenteredLine(cy, containerName, 2, BLACK);
        printCenteredLine(cy + 30, "REFILL NEEDED", 1, BLACK);
        
        // Update display state to remember we're showing empty container warning
        strcpy(displayState.lastAnchorName, containerName);
        displayState.displayInitialized = true;
    }
}

// Update container status based on current volumes and dosed amounts
void updateContainerStatus() {
    static uint8_t lastLoggedMask = 0xFF;
    const float ATO_EMPTY_EPSILON_GAL = 0.0005f;
    bool prevAtoEmpty = containerStatus.atoEmpty;
    // Calculate remaining volumes for dose containers
    int remaining1 = containerVolumes.dose1 - dosedAmounts.dose1;
    int remaining2 = containerVolumes.dose2 - dosedAmounts.dose2;
    int remaining3 = containerVolumes.dose3 - dosedAmounts.dose3;
    float remainingAto = containerVolumes.ato - dosedAmounts.ato;
    
    // Update dose container status
    containerStatus.dose1Empty = (remaining1 <= 0);
    containerStatus.dose2Empty = (remaining2 <= 0);
    containerStatus.dose3Empty = (remaining3 <= 0);
    // No capacity (0 gal), depleted, or overdosed: treat as empty so ATO stays off and warning shows
    bool atoOverdrawn = (containerVolumes.ato > 0.0f) &&
                        (dosedAmounts.ato > (containerVolumes.ato + ATO_EMPTY_EPSILON_GAL));
    containerStatus.atoEmpty = (containerVolumes.ato <= 0) ||
                               (remainingAto <= ATO_EMPTY_EPSILON_GAL) ||
                               atoOverdrawn;
    if (containerStatus.atoEmpty != prevAtoEmpty) {
        firebaseNeedsUpdate = true;
        if (firebaseState == FIREBASE_READY && app.ready() && strlen(SYSTEM_STATUS_PATH) > 0) {
            char atoEmptyPath[160];
            buildPath(atoEmptyPath, SYSTEM_STATUS_PATH, "/atoempty");
            Database.set(aClient, atoEmptyPath, containerStatus.atoEmpty, asyncCB);
        }
        if (containerStatus.atoEmpty) {
            if (!atoEmptyNotified) {
                Serial.printf("ATO empty (remaining: %.3f gal)\n", remainingAto);
                atoEmptyNotified = true;
            }
        } else {
            atoEmptyNotified = false;
        }
    }
    
    // Water container status is updated elsewhere (in water change logic)
    // containerStatus.waterEmpty and containerStatus.waterFull are set in water change functions
    
    // Log container status only when the combined status changes.
    uint8_t statusMask = 0;
    if (containerStatus.dose1Empty) statusMask |= 1 << 0;
    if (containerStatus.dose2Empty) statusMask |= 1 << 1;
    if (containerStatus.dose3Empty) statusMask |= 1 << 2;
    if (containerStatus.atoEmpty)   statusMask |= 1 << 3;
    if (containerStatus.waterEmpty) statusMask |= 1 << 4;
    if (containerStatus.waterFull)  statusMask |= 1 << 5;

    if (statusMask != 0 && statusMask != lastLoggedMask) {
        Serial.println("Container status update:");
        if (containerStatus.dose1Empty) Serial.printf("  Dose 1: Empty (remaining: %d ml)\n", remaining1);
        if (containerStatus.dose2Empty) Serial.printf("  Dose 2: Empty (remaining: %d ml)\n", remaining2);
        if (containerStatus.dose3Empty) Serial.printf("  Dose 3: Empty (remaining: %d ml)\n", remaining3);
        if (containerStatus.waterEmpty) Serial.println("  Water: Empty");
        if (containerStatus.waterFull) Serial.println("  Water: Full");
    }
    lastLoggedMask = statusMask;
}

int readEffectiveFloatPin() {
    return serialFloatOverride ? serialFloatSimLevel : digitalRead(FLOAT_PIN);
}

int readEffectiveEstopPin() {
    return serialEstopOverride ? serialEstopSimLevel : digitalRead(ESTOP_PIN);
}

// Line-based serial commands (case-insensitive): FLOAT ON/OFF, ESTOP ON/OFF
void processSerialSensorCommands() {
    static char lineBuf[48];
    static size_t lineLen = 0;
    while (Serial.available()) {
        char c = static_cast<char>(Serial.read());
        if (c == '\r') {
            continue;
        }
        if (c == '\n') {
            lineBuf[lineLen] = '\0';
            while (lineLen > 0 && lineBuf[lineLen - 1] == ' ') {
                lineBuf[--lineLen] = '\0';
            }
            if (lineLen > 0) {
                for (size_t i = 0; i < lineLen; i++) {
                    if (lineBuf[i] >= 'a' && lineBuf[i] <= 'z') {
                        lineBuf[i] = static_cast<char>(lineBuf[i] - 32);
                    }
                }
                if (strcmp(lineBuf, "FLOAT ON") == 0) {
                    serialFloatOverride = true;
                    serialFloatSimLevel = HIGH;
                    Serial.println(F("Serial: FLOAT ON (override HIGH, use FLOAT OFF for hardware)"));
                } else if (strcmp(lineBuf, "FLOAT OFF") == 0) {
                    serialFloatOverride = false;
                    Serial.println(F("Serial: FLOAT OFF (hardware float pin)"));
                } else if (strcmp(lineBuf, "ESTOP ON") == 0) {
                    serialEstopOverride = true;
                    serialEstopSimLevel = LOW;
                    Serial.println(F("Serial: ESTOP ON (override STOP / active-low)"));
                } else if (strcmp(lineBuf, "ESTOP OFF") == 0) {
                    serialEstopOverride = false;
                    Serial.println(F("Serial: ESTOP OFF (hardware E-stop pin)"));
                }
            }
            lineLen = 0;
        } else if (lineLen < sizeof(lineBuf) - 1) {
            lineBuf[lineLen++] = c;
        } else {
            lineLen = 0;
        }
    }
}

void floatSensorTask(void *parameter) {
    Serial.println("Float sensor task started on Core " + String(xPortGetCoreID()));
    
    int lastPrintedEstopState = HIGH;
    int lastPrintedFloatState = LOW;
    
    // Hysteresis variables
    unsigned long floatHighTime = 0;
    unsigned long floatLowTime = 0;
    const unsigned long HYSTERESIS_DELAY = 2000; // 2 second delay for state change
    bool currentStableState = LOW; // The stable state we consider after hysteresis
    
    // ATO state tracking variables
    bool wasWaterChangeActive = false;
    unsigned long lastAtoDisabledLog = 0;
    
    // ATO volume tracking variables
    unsigned long atoPumpStartTime = 0;
    bool atoPumpWasRunning = false;
    static unsigned long lastAtoVolumeUpdate = 0;
    const unsigned long ATO_VOLUME_UPDATE_INTERVAL = 1000; // Update every second
    
    while (true) {
        if (xSemaphoreTake(floatMutex, pdMS_TO_TICKS(100))) {
            unsigned long currentTime = millis();
            
            // Skip float sensor processing during water changes
            if (!isWaterChangeActive) {
                // Log when ATO is re-enabled after water change
                if (wasWaterChangeActive) {
                    Serial.println("ATO ENABLED: Water change completed - float sensor monitoring resumed");
                    wasWaterChangeActive = false;
                }
                ignoreFloatSensor = false;
                
                // Read current states (honor serial test overrides)
                int newEstopState = readEffectiveEstopPin();
                int newFloatState = readEffectiveFloatPin();
                if (atoRefillRecheckPending) {
                    // On ATO refill/reset, trust the current raw float state immediately (skip hysteresis delay once).
                    atoRefillRecheckPending = false;
                    floatState = newFloatState;
                    currentStableState = (newFloatState == HIGH) ? HIGH : LOW;
                    floatHighTime = (currentStableState == HIGH) ? currentTime : 0;
                    floatLowTime = (currentStableState == LOW) ? currentTime : 0;
                    Serial.println("ATO refill sync: float re-read, resuming ATO control");
                }
                
                // Update E-stop state
                if (newEstopState != estopState) {
                    estopState = newEstopState;
                    strcpy(estopStatus, estopState ? "OK" : "STOP");
                    Serial.print("E-Stop raw state: ");
                    Serial.println(estopState == HIGH ? "HIGH" : "LOW");
                    Serial.println("E-Stop state: " + String(estopStatus));
                }
                
                // Handle float sensor with hysteresis
                if (newFloatState != floatState) {
                    floatState = newFloatState;
                    Serial.println("Float raw state: " + String(floatState ? "HIGH" : "LOW"));
                    
                    // Reset timers when state changes
                    if (floatState == HIGH) {
                        floatHighTime = currentTime;
                        floatLowTime = 0;
                    } else {
                        floatLowTime = currentTime;
                        floatHighTime = 0;
                    }
                }
                
                // Check if we've had a stable state long enough to consider it valid
                if (serialFloatOverride) {
                    bool wantHigh = (floatState == HIGH);
                    if (currentStableState != wantHigh) {
                        currentStableState = wantHigh;
                        Serial.println(wantHigh ? "Float (serial): stable HIGH" : "Float (serial): stable LOW");
                    }
                } else if (floatState == HIGH && floatHighTime > 0 && 
                    (currentTime - floatHighTime) >= HYSTERESIS_DELAY &&
                    currentStableState != HIGH) {
                    
                    currentStableState = HIGH;
                    Serial.println("Float state stabilized: HIGH");
                    
                } else if (floatState == LOW && floatLowTime > 0 && 
                          (currentTime - floatLowTime) >= HYSTERESIS_DELAY &&
                          currentStableState != LOW) {
                    
                    currentStableState = LOW;
                    Serial.println("Float state stabilized: LOW");
                }
                
                // Check if ATO container is empty or capacity is zero before running pump
                bool atoEmpty = false;
                if (xSemaphoreTake(doseMutex, pdMS_TO_TICKS(10))) {
                    // Backup protection: if dosed exceeds configured volume, hard-stop ATO as empty.
                    if (containerVolumes.ato > 0.0f &&
                        dosedAmounts.ato > (containerVolumes.ato + 0.0005f)) {
                        dosedAmounts.ato = containerVolumes.ato;
                        containerStatus.atoEmpty = true;
                        firebaseNeedsUpdate = true;
                        if (!atoEmptyNotified) {
                            Serial.println("ATO backup empty detection: dosed exceeded container volume");
                            atoEmptyNotified = true;
                        }
                    }
                    atoEmpty = containerStatus.atoEmpty || (containerVolumes.ato <= 0.0f);
                    xSemaphoreGive(doseMutex);
                }
                
                // Control ATO pump based on stable state, E-Stop, and container status
                bool shouldPumpRun = (currentStableState == HIGH) && (estopState == HIGH) && !atoEmpty;
                
                // Track pump start/stop for volume calculation
                if (shouldPumpRun && !atoPumpWasRunning) {
                    atoPumpStartTime = currentTime;
                    atoPumpWasRunning = true;
                    Serial.printf("ATO pump started (dosed=%.3f gal)\n", dosedAmounts.ato);
                } else if (!shouldPumpRun && atoPumpWasRunning) {
                    // Pump just stopped - calculate and record volume
                    unsigned long runTime = currentTime - atoPumpStartTime;
                    if (runTime > 0) {
                        // Calculate volume based on runtime and calibration
                        float calibratedFlowRate = FLOW_RATE_ML_PER_MIN * atoCalibration;
                        float volumeAdded = (runTime * calibratedFlowRate) / (MS_PER_MINUTE * ML_PER_GALLON); // gallons
                        
                        if (volumeAdded > 0.0f) {
                            queueAtoVolumeIncrement(volumeAdded);
                            Serial.printf("ATO added %.3f gal (runtime: %lu ms, queued for Firebase sync)\n",
                                        volumeAdded, runTime);
                        }
                    }
                    atoPumpWasRunning = false;
                } else if (shouldPumpRun && atoPumpWasRunning) {
                    // Pump is running - check if container would be empty periodically
                    if (currentTime - lastAtoVolumeUpdate >= ATO_VOLUME_UPDATE_INTERVAL) {
                        const float ATO_EMPTY_EPSILON_GAL = 0.0005f;
                        unsigned long runTime = currentTime - atoPumpStartTime;
                        if (runTime > 0) {
                            float calibratedFlowRate = FLOW_RATE_ML_PER_MIN * atoCalibration;
                            float volumeAdded = (runTime * calibratedFlowRate) / (MS_PER_MINUTE * ML_PER_GALLON);
                            float queuedIncrement = 0.0f;
                            
                            if (xSemaphoreTake(doseMutex, pdMS_TO_TICKS(10))) {
                                // Check if container would be empty with current volume.
                                // Use projected total to avoid drift from async timing.
                                float projectedTotal = dosedAmounts.ato + volumeAdded;
                                float remainingProjected = containerVolumes.ato - projectedTotal;
                                Serial.printf("ATO running: projected=%.3f/%.3f gal, remaining=%.3f gal\n",
                                            projectedTotal, containerVolumes.ato, remainingProjected);
                                if ((containerVolumes.ato > 0.0f && projectedTotal >= (containerVolumes.ato - ATO_EMPTY_EPSILON_GAL)) ||
                                    (containerVolumes.ato <= 0.0f)) {
                                    // Container would be empty - stop pump and record volume
                                    shouldPumpRun = false;
                                    atoPumpWasRunning = false; // Mark as stopped so volume gets recorded
                                    containerStatus.atoEmpty = true;
                                    if (!atoEmptyNotified) {
                                        Serial.println("ATO empty");
                                        atoEmptyNotified = true;
                                    }
                                    // On empty threshold, hard-latch to full container volume.
                                    // Do not queue partial increments; this avoids async drift around final threshold.
                                    dosedAmounts.ato = containerVolumes.ato;
                                    queuedIncrement = 0.0f;
                                    containerStatus.atoEmpty = true;
                                    firebaseNeedsUpdate = true;
                                }
                                xSemaphoreGive(doseMutex);
                            }
                            (void)queuedIncrement;
                        }
                        lastAtoVolumeUpdate = currentTime;
                    }
                }
                
                digitalWrite(MTR_ATO, shouldPumpRun ? HIGH : LOW);
                strcpy(pumpState, shouldPumpRun ? "RUN" : "STOP");
                
                // Print state changes after debounce
                if ((currentTime - lastDebounceTime) > DEBOUNCE_DELAY) {
                    if (estopState != lastPrintedEstopState) {
                        Serial.println("E-Stop state: " + String(estopState ? "OK" : "STOP"));
                        lastPrintedEstopState = estopState;
                    }
                    if (currentStableState != lastPrintedFloatState) {
                        Serial.println("Float stable state: " + String(currentStableState ? "HIGH" : "LOW"));
                        lastPrintedFloatState = currentStableState;
                    }
                    lastDebounceTime = currentTime;
                }
                
                estopChanged = false;
                floatChanged = false;
            } else {
                // Log when ATO is disabled for water change
                if (!wasWaterChangeActive) {
                    Serial.println("ATO DISABLED: Water change started - float sensor monitoring suspended");
                    wasWaterChangeActive = true;
                }
                
                ignoreFloatSensor = true;  // Ignore float sensor during water changes
                digitalWrite(MTR_ATO, LOW);  // Ensure ATO is off during water change
                
                // Log ATO disabled status periodically (every 5 seconds)
                if (currentTime - lastAtoDisabledLog > 5000) {
                    Serial.println("ATO DISABLED: Water change in progress - float sensor ignored");
                    lastAtoDisabledLog = currentTime;
                }
            }
            xSemaphoreGive(floatMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}





String getFormattedTime() {
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
        Serial.println("Failed to obtain time");
        return "Time Error";
    }
    char buffer[30];
    strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &timeinfo);
    return String(buffer);
}

void printLocalTime() {
    struct tm timeinfo;
    if(!getLocalTime(&timeinfo)) {
        Serial.println("Failed to obtain time");
        return;
    }
    Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
}

bool isWaterChangeDay() {
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
        Serial.println("Failed to obtain time");
        return false;
    }
    int dayOfWeek = timeinfo.tm_wday;
    
    // Convert from Sunday (0) - Saturday (6) to Monday (0) - Sunday (6)
    int webUIDayIndex = (dayOfWeek + 6) % 7;
    
    return webUIDayIndex >= 0 && webUIDayIndex < 8 && waterChangeDays[webUIDayIndex] == '1';
}


void resetContainerStatus(int container) {
    switch(container) {
        case 1: 
            containerStatus.dose1Empty = false;
            containerVolumes.dose1 = 1000;
            break;
        case 2:
            containerStatus.dose2Empty = false;
            containerVolumes.dose2 = 1000;
            break;
        case 3:
            containerStatus.dose3Empty = false;
            containerVolumes.dose3 = 1000;
            break;
        case 4: // Water container
            containerStatus.waterEmpty = false;
            containerStatus.waterFull = false;
            containerVolumes.water_in = 5.0; // Default 5 gallons
            containerVolumes.water_out = 5.0;
            break;
    }
    firebaseNeedsUpdate = true;
}

bool isDoseDay(int doseNum, int dayOfWeek) {
    char* days;
    switch(doseNum) {
        case 1: days = dose1.days; break;
        case 2: days = dose2.days; break;
        case 3: days = dose3.days; break;
        default: return false;
    }
    
    // Convert from Sunday (0) - Saturday (6) to Monday (0) - Sunday (6)
    int webUIDayIndex = (dayOfWeek + 6) % 7; // Rotate the index to match web UI
    
    return webUIDayIndex >= 0 && webUIDayIndex < 7 && days[webUIDayIndex] == '1';
}


// Calculate calibrated dose time based on pump calibration factors
unsigned long calculateCalibratedDoseTime(int doseNum, int amount) {
    float calibrationFactor = 1.0f;
    switch(doseNum) {
        case 1: calibrationFactor = pump1Calibration; break;
        case 2: calibrationFactor = pump2Calibration; break;
        case 3: calibrationFactor = pump3Calibration; break;
        default: calibrationFactor = 1.0f; break;
    }
    float adjustedFlowRate = FLOW_RATE_ML_PER_MIN * calibrationFactor;
    unsigned long doseTime = (amount * 60 * 1000) / adjustedFlowRate;
    Serial.printf("Dose %d calibration: factor=%.3f, flow=%.1f ml/min, time=%lu ms\n", 
                doseNum, calibrationFactor, adjustedFlowRate, doseTime);
    return doseTime;
}

// Test pump calibration by running for a calculated time
void testPumpCalibration(int doseNum, int testAmount) {
    int pin = 0;
    switch(doseNum) {
        case 1: pin = MTR_DOSE1; break;
        case 2: pin = MTR_DOSE2; break;
        case 3: pin = MTR_DOSE3; break;
        default: return;
    }
    
    unsigned long testTime = calculateCalibratedDoseTime(doseNum, testAmount);
    Serial.printf("Testing pump %d calibration: %dml for %lu ms\n", doseNum, testAmount, testTime);
    
    digitalWrite(pin, HIGH);
    delay(testTime);
    digitalWrite(pin, LOW);
    
    Serial.printf("Pump %d calibration test completed\n", doseNum);
}

// Calculate calibrated water change time based on motor calibration factors
unsigned long calculateCalibratedWaterChangeTime(bool isWaterIn, float volumeGallons) {
    float calibrationFactor = 1.0f;
    if (isWaterIn) {
        calibrationFactor = waterChangeInCalibration;
    } else {
        calibrationFactor = waterChangeOutCalibration;
    }
    
    float adjustedFlowRate = FLOW_RATE_ML_PER_MIN * calibrationFactor;
    unsigned long waterChangeTime = (volumeGallons * ML_PER_GALLON * MS_PER_MINUTE) / adjustedFlowRate;
    Serial.printf("Water change %s calibration: factor=%.3f, flow=%.1f ml/min, time=%lu ms\n", 
                isWaterIn ? "IN" : "OUT", calibrationFactor, adjustedFlowRate, waterChangeTime);
    return waterChangeTime;
}

// Test water change motor calibration by running for a calculated time
void testWaterChangeCalibration(bool isWaterIn, float testVolumeGallons) {
    int pin = isWaterIn ? MTR_WATER_CHANGE_IN : MTR_WATER_CHANGE_OUT;
    
    unsigned long testTime = calculateCalibratedWaterChangeTime(isWaterIn, testVolumeGallons);
    Serial.printf("Testing water change %s calibration: %.1f gallons for %lu ms\n", 
                isWaterIn ? "IN" : "OUT", testVolumeGallons, testTime);
    
    digitalWrite(pin, HIGH);
    delay(testTime);
    digitalWrite(pin, LOW);
    
    Serial.printf("Water change %s calibration test completed\n", isWaterIn ? "IN" : "OUT");
}

void performDosing(int doseNum, DoseSettings &dose, int pin) {
    // Get current time once at start
    struct tm timeinfo;
    bool timeValid = getLocalTime(&timeinfo);
    int currentDay = timeValid ? timeinfo.tm_wday : -1;
    
    // 1. Check for PRIMING request (highest priority)
    if (xSemaphoreTake(doseMutex, pdMS_TO_TICKS(100))) {
        if (manualDoseFlags[doseNum-1]) {
            Serial.printf("Starting PRIMING for pump %d (30 seconds)\n", doseNum);
            
            // Clear the priming flag immediately
            manualDoseFlags[doseNum-1] = false;
            firebaseNeedsUpdate = true;
            xSemaphoreGive(doseMutex);
            
            // Run priming sequence (30 seconds fixed)
            digitalWrite(pin, HIGH);
            unsigned long startTime = millis();
            
            while (millis() - startTime < 30000) {
                delay(100);
                if (readEffectiveEstopPin() == LOW) {
                    Serial.println("Priming interrupted by E-Stop");
                    break;
                }
            }
            
            digitalWrite(pin, LOW);
            Serial.printf("Completed PRIMING for pump %d at %s\n", 
                        doseNum, getFormattedTime().c_str());
            return;
        }
        xSemaphoreGive(doseMutex);
    }

    // 2. Check for MANUAL dose request
    if (xSemaphoreTake(doseMutex, pdMS_TO_TICKS(100))) {
        if (manualDose.requested[doseNum-1] && !manualDose.completed[doseNum-1]) {
            int containerVolume = 0;
            int dosedAmount = 0;
            bool containerEmpty = false;
            
            switch(doseNum) {
                case 1:
                    containerVolume = containerVolumes.dose1;
                    dosedAmount = dosedAmounts.dose1;
                    containerEmpty = containerStatus.dose1Empty;
                    break;
                case 2:
                    containerVolume = containerVolumes.dose2;
                    dosedAmount = dosedAmounts.dose2;
                    containerEmpty = containerStatus.dose2Empty;
                    break;
                case 3:
                    containerVolume = containerVolumes.dose3;
                    dosedAmount = dosedAmounts.dose3;
                    containerEmpty = containerStatus.dose3Empty;
                    break;
            }
            xSemaphoreGive(doseMutex);
            
            // Skip if container is empty
            if (containerEmpty) {
                Serial.printf("Manual dose %d skipped - container empty\n", doseNum);
                if (xSemaphoreTake(doseMutex, pdMS_TO_TICKS(100))) {
                    manualDose.completed[doseNum-1] = true;
                    manualDose.requested[doseNum-1] = false;
                    firebaseNeedsUpdate = true;
                    xSemaphoreGive(doseMutex);
                }
                return;
            }

            // Calculate remaining volume
            int remaining = containerVolume - dosedAmount;
            
            if (remaining <= dose.amount) {
                Serial.printf("Manual dose %d skipped - would leave %dml (container: %dml, dosed: %dml)\n", 
                            doseNum, remaining - dose.amount, containerVolume, dosedAmount);
                
                // Mark as completed anyway since it was requested
                if (xSemaphoreTake(doseMutex, pdMS_TO_TICKS(100))) {
                    manualDose.completed[doseNum-1] = true;
                    manualDose.requested[doseNum-1] = false;
                    firebaseNeedsUpdate = true;
                    xSemaphoreGive(doseMutex);
                }
                return;
            }

            // Calculate run time based on dose amount with calibration
            unsigned long doseTime = calculateCalibratedDoseTime(doseNum, dose.amount);
            
            Serial.printf("Starting MANUAL dose %d: %dml (%lums)\n", 
                        doseNum, dose.amount, doseTime);
            digitalWrite(pin, HIGH);
            
            unsigned long startTime = millis();
            bool estopPressed = false;
            while (millis() - startTime < doseTime) {
                delay(100);
                if (readEffectiveEstopPin() == LOW) {
                    Serial.println("Dose interrupted by E-Stop");
                    estopPressed = true;
                    break;
                }
            }
            
            digitalWrite(pin, LOW);
            
            // Calculate actual dose delivered based on time run
            unsigned long actualRunTime = millis() - startTime;
            int actualDoseAmount = 0;
            
            if (!estopPressed) {
                // Full dose delivered - use requested amount
                actualDoseAmount = dose.amount;
                Serial.printf("Full dose delivered: %d ml\n", actualDoseAmount);
            } else {
                // Partial dose delivered - calculate based on time run
                actualDoseAmount = (actualRunTime * dose.amount) / doseTime;
                Serial.printf("Partial dose delivered: %d ml (%.1f%% of requested)\n", 
                            actualDoseAmount, (float)actualDoseAmount / dose.amount * 100.0);
            }
            
            // Update remaining volume and completion flags
            if (xSemaphoreTake(doseMutex, pdMS_TO_TICKS(100))) {
                // Always update with actual dose amount delivered
                switch(doseNum) {
                    case 1: 
                        dosedAmounts.dose1 += actualDoseAmount;
                        containerStatus.dose1Empty = (containerVolumes.dose1 - dosedAmounts.dose1) <= 0;
                        break;
                    case 2:
                        dosedAmounts.dose2 += actualDoseAmount;
                        containerStatus.dose2Empty = (containerVolumes.dose2 - dosedAmounts.dose2) <= 0;
                        break;
                    case 3:
                        dosedAmounts.dose3 += actualDoseAmount;
                        containerStatus.dose3Empty = (containerVolumes.dose3 - dosedAmounts.dose3) <= 0;
                        break;
                }
                
                if (!estopPressed) {
                    Serial.printf("Manual dose %d completed successfully - added %d ml\n", doseNum, actualDoseAmount);
                } else {
                    Serial.printf("Manual dose %d interrupted by E-Stop - added %d ml\n", doseNum, actualDoseAmount);
                }
                
                manualDose.completed[doseNum-1] = true;
                manualDose.requested[doseNum-1] = false;
                firebaseNeedsUpdate = true;
                xSemaphoreGive(doseMutex);
            }
            
            String currentTime = getFormattedTime();
            switch(doseNum) {
                case 1: strcpy(lastDose1Time, currentTime.c_str()); break;
                case 2: strcpy(lastDose2Time, currentTime.c_str()); break;
                case 3: strcpy(lastDose3Time, currentTime.c_str()); break;
            }
            
            Serial.printf("Completed MANUAL dose %d at %s\n", 
                        doseNum, currentTime.c_str());
            return;
        }
        xSemaphoreGive(doseMutex);
    }

    // 3. Handle SCHEDULED DOSE (only if we have valid time)
    if (timeValid && shouldRunScheduledDose(doseNum, dose)) {
        // Get container status in a mutex-protected block
        int containerVolume = 0;
        int dosedAmount = 0;
        bool containerEmpty = false;
        
        if (xSemaphoreTake(doseMutex, pdMS_TO_TICKS(100))) {
            switch(doseNum) {
                case 1:
                    containerVolume = containerVolumes.dose1;
                    dosedAmount = dosedAmounts.dose1;
                    containerEmpty = containerStatus.dose1Empty;
                    break;
                case 2:
                    containerVolume = containerVolumes.dose2;
                    dosedAmount = dosedAmounts.dose2;
                    containerEmpty = containerStatus.dose2Empty;
                    break;
                case 3:
                    containerVolume = containerVolumes.dose3;
                    dosedAmount = dosedAmounts.dose3;
                    containerEmpty = containerStatus.dose3Empty;
                    break;
            }
            xSemaphoreGive(doseMutex);
        } else {
            Serial.printf("Dose %d: Couldn't get mutex for volume check\n", doseNum);
            return;
        }

        // Skip if container is empty
        if (containerEmpty) {
            Serial.printf("Scheduled dose %d skipped - container empty\n", doseNum);
            dose.currentSplit++; // Still count as attempted
            if (dose.currentSplit >= dose.splitCount) {
                dose.dosedToday = true;
            }
            return;
        }

        // Validate split count - ensure it's at least 1
        if (dose.splitCount <= 0) {
            Serial.printf("Dose %d splitCount is %d - setting to 1 (single dose)\n", doseNum, dose.splitCount);
            dose.splitCount = 1;
        }
        
        // Calculate split amount
        int splitAmount = dose.amount / dose.splitCount;
        if (dose.currentSplit == dose.splitCount - 1) {
            // Last split gets any remainder
            splitAmount = dose.amount - (dose.splitCount - 1) * (dose.amount / dose.splitCount);
        }

        // Calculate remaining volume
        int remaining = containerVolume - dosedAmount;
        
        if (remaining <= splitAmount) {
            Serial.printf("Scheduled dose %d split %d skipped - would leave %dml\n",
                        doseNum, dose.currentSplit, remaining - splitAmount);
            dose.currentSplit++; // Still count as attempted
            if (dose.currentSplit >= dose.splitCount) {
                dose.dosedToday = true;
            }
            return;
        }

        // Calculate run time based on split amount with calibration
        unsigned long doseTime = calculateCalibratedDoseTime(doseNum, splitAmount);
        
        if (dose.splitCount == 1) {
            Serial.printf("Starting SCHEDULED dose %d (single dose): %dml (%lums)\n", 
                        doseNum, splitAmount, doseTime);
        } else {
            Serial.printf("Starting SCHEDULED dose %d split %d/%d: %dml (%lums)\n", 
                        doseNum, dose.currentSplit + 1, dose.splitCount, splitAmount, doseTime);
        }
        digitalWrite(pin, HIGH);
        
        unsigned long startTime = millis();
        bool estopPressed = false;
        while (millis() - startTime < doseTime) {
            delay(100);
            if (readEffectiveEstopPin() == LOW) {
                Serial.println("Dose interrupted by E-Stop");
                estopPressed = true;
                break;
            }
        }
        
        digitalWrite(pin, LOW);
        
        // Calculate actual dose delivered based on time run
        unsigned long actualRunTime = millis() - startTime;
        int actualDoseAmount = 0;
        
        if (!estopPressed) {
            // Full dose delivered - use requested amount
            actualDoseAmount = splitAmount;
            Serial.printf("Full scheduled dose delivered: %d ml\n", actualDoseAmount);
        } else {
            // Partial dose delivered - calculate based on time run
            actualDoseAmount = (actualRunTime * splitAmount) / doseTime;
            Serial.printf("Partial scheduled dose delivered: %d ml (%.1f%% of requested)\n", 
                        actualDoseAmount, (float)actualDoseAmount / splitAmount * 100.0);
        }
        
        // Update remaining volume
        if (xSemaphoreTake(doseMutex, pdMS_TO_TICKS(100))) {
            // Always update with actual dose amount delivered
            switch(doseNum) {
                case 1: 
                    dosedAmounts.dose1 += actualDoseAmount;
                    containerStatus.dose1Empty = (containerVolumes.dose1 - dosedAmounts.dose1) <= 0;
                    break;
                case 2:
                    dosedAmounts.dose2 += actualDoseAmount;
                    containerStatus.dose2Empty = (containerVolumes.dose2 - dosedAmounts.dose2) <= 0;
                    break;
                case 3:
                    dosedAmounts.dose3 += actualDoseAmount;
                    containerStatus.dose3Empty = (containerVolumes.dose3 - dosedAmounts.dose3) <= 0;
                    break;
            }
            
            if (!estopPressed) {
                Serial.printf("Scheduled dose %d split %d completed successfully - added %d ml\n", 
                            doseNum, dose.currentSplit, actualDoseAmount);
            } else {
                Serial.printf("Scheduled dose %d split %d interrupted by E-Stop - added %d ml\n", 
                            doseNum, dose.currentSplit, actualDoseAmount);
            }
            firebaseNeedsUpdate = true;
            xSemaphoreGive(doseMutex);
        }
        
        // Update split counter
        dose.currentSplit++;
        if (dose.currentSplit >= dose.splitCount) {
            dose.dosedToday = true;
        }
        
        String currentTime = getFormattedTime();
        switch(doseNum) {
            case 1: strcpy(lastDose1Time, currentTime.c_str()); break;
            case 2: strcpy(lastDose2Time, currentTime.c_str()); break;
            case 3: strcpy(lastDose3Time, currentTime.c_str()); break;
        }
        
        if (dose.splitCount == 1) {
            Serial.printf("Completed SCHEDULED dose %d (single dose) at %s\n", 
                        doseNum, currentTime.c_str());
        } else {
            Serial.printf("Completed SCHEDULED dose %d split %d/%d at %s\n", 
                        doseNum, dose.currentSplit + 1, dose.splitCount, currentTime.c_str());
        }
    }
}

bool shouldRunScheduledDose(int doseNum, DoseSettings &dose) {
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) return false;

    // Reset dosing flags at midnight
    if (timeinfo.tm_yday != dose.lastDoseDay) {
        dose.dosedToday = false;
        dose.currentSplit = 0;
        dose.lastDoseDay = timeinfo.tm_yday;
        dose.lastSplitTime = 0; // Reset last split time
    }

    // Only proceed if it's a dose day and we haven't completed all splits today
    if (!isDoseDay(doseNum, timeinfo.tm_wday) || dose.dosedToday) {
        return false;
    }

    // Calculate current time in minutes since midnight
    unsigned long currentMinutes = timeinfo.tm_hour * 60 + timeinfo.tm_min;
    
    // Parse scheduled time
    char hourStr[3] = {dose.time[0], dose.time[1], '\0'};
    char minuteStr[3] = {dose.time[3], dose.time[4], '\0'};
    int scheduledHour = atoi(hourStr);
    int scheduledMinute = atoi(minuteStr);
    unsigned long scheduledMinutes = scheduledHour * 60 + scheduledMinute;
    
    // For the first split, do exact time comparison
    if (dose.currentSplit == 0) {
        if (currentMinutes >= scheduledMinutes && 
            currentMinutes <= scheduledMinutes + 2) { // 2-minute window
            dose.lastSplitTime = millis();
            return true;
        }
    }
    
    // For subsequent splits, calculate based on intervals
    // Validate split count first
    if (dose.splitCount <= 0) {
        dose.splitCount = 1; // Treat as single dose
    }
    
    // Spread splits over 12 hours (720 minutes) instead of 24 hours
    unsigned long splitInterval = 720 / dose.splitCount; // Minutes between splits
    unsigned long nextSplit = scheduledMinutes + (dose.currentSplit * splitInterval);
    
    // Add some tolerance (+/- 2 minutes) to account for clock drift
    if (currentMinutes >= nextSplit - 2 && currentMinutes <= nextSplit + 2) {
        // Make sure we haven't already run this split
        if (millis() - dose.lastSplitTime > (splitInterval * 60000) || dose.lastSplitTime == 0) {
            dose.lastSplitTime = millis();
            dose.nextSplit = nextSplit;
            return true;
        }
    }
    
    return false;
}



void resetDosedAmounts() {
    if (xSemaphoreTake(doseMutex, pdMS_TO_TICKS(10))) {
        dosedAmounts.dose1 = 0;
        dosedAmounts.dose2 = 0;
        dosedAmounts.dose3 = 0;
        firebaseNeedsUpdate = true;
        xSemaphoreGive(doseMutex);
        Serial.println("Dosed amounts reset to zero");
    }
}

// Reset specific dosed amounts when containers are refilled
void resetContainerDosedAmounts(int containerType) {
    // Refill signal source is dosed_amounts reset in Firebase, not container volume changes.
    switch(containerType) {
        case 1:
            Serial.println("Dose 1 refill check requested - waiting for dosed_amounts/dose1 async sync");
            break;
        case 2:
            Serial.println("Dose 2 refill check requested - waiting for dosed_amounts/dose2 async sync");
            break;
        case 3:
            Serial.println("Dose 3 refill check requested - waiting for dosed_amounts/dose3 async sync");
            break;
        case 4:
            Serial.println("Water IN refill check requested - waiting for dosed_amounts/water_in async sync");
            break;
        case 5:
            Serial.println("Water OUT reset check requested - waiting for dosed_amounts/water_out async sync");
            break;
        case 6:
            Serial.println("ATO refill check requested - waiting for dosed_amounts/ato async sync");
            break;
        default:
            Serial.printf("Unknown container type: %d\n", containerType);
            break;
    }
    requestAllDosedAmountsFromFirebase();
}

// Re-read settings/dosed_amounts/ato (e.g. after refill reset)
void requestAtoDosedFromFirebase() {
    if (firebaseState != FIREBASE_READY || !app.ready()) {
        Serial.println("requestAtoDosedFromFirebase: Firebase not ready, skipping");
        return;
    }
    if (strlen(SETTINGS_PATH) == 0) {
        Serial.println("requestAtoDosedFromFirebase: SETTINGS_PATH not set, skipping");
        return;
    }
    char tempPath[256];
    buildPath(tempPath, SETTINGS_PATH, "/dosed_amounts/ato");
    Database.get(aClient, tempPath, asyncCB, false, "getAtoDosed");
    Serial.println("requestAtoDosedFromFirebase: requested dosed_amounts/ato (async)");
}

void queueAtoVolumeIncrement(float addGal) {
    if (addGal <= 0.0f) {
        return;
    }
    if (!xSemaphoreTake(doseMutex, pdMS_TO_TICKS(100))) {
        Serial.println("queueAtoVolumeIncrement: could not acquire doseMutex");
        return;
    }

    bool shouldRequestAtoSync = false;
    pendingAtoIncrementGal += addGal;
    if (!atoSyncRequestInFlight) {
        if (firebaseState == FIREBASE_READY && app.ready() && strlen(SETTINGS_PATH) > 0) {
            atoSyncRequestInFlight = true;
            shouldRequestAtoSync = true;
        } else {
            Serial.println("queueAtoVolumeIncrement: waiting for Firebase before syncing dosed_amounts/ato");
        }
    }
    xSemaphoreGive(doseMutex);
    if (shouldRequestAtoSync) {
        requestAtoDosedFromFirebase();
    }
}

void requestAllDosedAmountsFromFirebase() {
    if (firebaseState != FIREBASE_READY || !app.ready()) {
        Serial.println("requestAllDosedAmountsFromFirebase: Firebase not ready, skipping");
        return;
    }
    if (strlen(SETTINGS_PATH) == 0) {
        Serial.println("requestAllDosedAmountsFromFirebase: SETTINGS_PATH not set, skipping");
        return;
    }
    char tempPath[256];
    buildPath(tempPath, DOSE1_PATH, "/volume");
    Database.get(aClient, tempPath, asyncCB, false, "getDose1Dosed");
    buildPath(tempPath, DOSE2_PATH, "/volume");
    Database.get(aClient, tempPath, asyncCB, false, "getDose2Dosed");
    buildPath(tempPath, DOSE3_PATH, "/volume");
    Database.get(aClient, tempPath, asyncCB, false, "getDose3Dosed");
    buildPath(tempPath, SETTINGS_PATH, "/dosed_amounts/water_in");
    Database.get(aClient, tempPath, asyncCB, false, "getWaterInDosed");
    buildPath(tempPath, SETTINGS_PATH, "/dosed_amounts/water_out");
    Database.get(aClient, tempPath, asyncCB, false, "getWaterOutDosed");
    buildPath(tempPath, SETTINGS_PATH, "/dosed_amounts/ato");
    Database.get(aClient, tempPath, asyncCB, false, "getAtoDosed");
    Serial.println("requestAllDosedAmountsFromFirebase: requested all dosed amounts (async)");
}

// Load dosed amounts from Firebase on restart to avoid overwriting existing data
void loadDosedAmountsFromFirebase() {
    Serial.println("Loading dosed amounts from Firebase...");
    requestAllDosedAmountsFromFirebase();
}






void checkScheduledWaterChange() {
    static unsigned long lastCheck = 0;
    const unsigned long CHECK_INTERVAL = 60000; // Check once per minute
    
    if (millis() - lastCheck < CHECK_INTERVAL) return;
    lastCheck = millis();

    if (xSemaphoreTake(waterChange.mutex, pdMS_TO_TICKS(100))) {
        if (waterChange.active) {
            xSemaphoreGive(waterChange.mutex);
            return;
        }
        xSemaphoreGive(waterChange.mutex);
    }

    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) return;

    unsigned long currentMillis = millis();
    char currentTime[6];
    strftime(currentTime, sizeof(currentTime), "%H:%M", &timeinfo);
    
    if (isWaterChangeDay() && strcmp(currentTime, waterChangeTime) == 0) {
        // Check if manual control is active - if so, skip scheduled water change
        if (manualWater.waterIn || manualWater.waterOut) {
            Serial.println("Scheduled water change skipped - manual control is active");
            return;
        }
        
        // Load current dosed amounts from Firebase before starting scheduled water change
        loadDosedAmountsFromFirebase();
        
        if (xSemaphoreTake(waterChange.mutex, pdMS_TO_TICKS(100))) {
            waterChange.active = true;
            waterChange.manual = false;
            waterChange.outPhaseCompleted = false;
            waterChange.startMillis = millis();
            waterChange.phaseStartMillis = millis();
            waterChange.targetVolume = waterChangeVolume;
            waterChange.processedVolume = 0;
            waterChange.actualVolumeProcessed = 0;
            // Reset volume tracking variables
            waterChange.processedVolumeOutPhase = 0.0;
            waterChange.processedVolumeInPhase = 0.0;
            // Use calibrated flow rate for duration calculation
            float avgCalibration = (waterChangeInCalibration + waterChangeOutCalibration) / 2.0f;
            float calibratedFlowRate = FLOW_RATE_ML_PER_MIN * avgCalibration;
            waterChange.durationMs = (waterChangeVolume * ML_PER_GALLON * MS_PER_MINUTE) / calibratedFlowRate;
            waterChange.remainingMs = waterChange.durationMs;
            // Overflow protection for estimated completion time
            if (millis() < 0xFFFFFFFF - waterChange.durationMs) {
                waterChange.estimatedCompletionTime = millis() + waterChange.durationMs;
            } else {
                waterChange.estimatedCompletionTime = 0xFFFFFFFF; // Max value if overflow
            }
            waterChange.needsRemainingUpdate = true;
            waterChange.needsProgressUpdate = true;
            strcpy(waterChange.status, "running");
            
            strcpy(lastWaterChangeTime, getFormattedTime().c_str());
            firebaseNeedsUpdate = true;
            
            // Reset watchdog activity timestamp for new water change
            lastWaterChangeActivity = millis();
            
            Serial.printf("Starting scheduled water change: %.2f gallons (%.0f ms)\n", 
                         waterChangeVolume, waterChange.durationMs);
            
            xSemaphoreGive(waterChange.mutex);
            
            // Reset static flags in handleWaterChanges for new scheduled water change
            // Note: These will be reset when handleWaterChanges() is called next
        }
    }
}

void handleWaterChanges() {
    static unsigned long lastProgressUpdate = 0;
    static bool inPhaseStarted = false; // Track IN phase start for scheduled water changes
    static bool scheduledWaterChangeStarted = false; // Track scheduled water change start
    const unsigned long PROGRESS_UPDATE_INTERVAL = 1000; // Update every second
    const unsigned long TIMEOUT_BUFFER = 300000; // 5 minute buffer beyond estimated completion

    // Get current state with mutex protection
    bool isActive = false;
    bool isManual = false;
    bool outCompleted = false;
    float processed = 0;
    float target = 0;
    String currentStatus = "ready";
    unsigned long currentMillis = millis();
    unsigned long remainingMs = 0;
    unsigned long startMillis = 0;
    unsigned long phaseStartMillis = 0;
    unsigned long estimatedCompletionTime = 0;

    if (xSemaphoreTake(waterChange.mutex, pdMS_TO_TICKS(100))) {
        isActive = waterChange.active;
        isManual = waterChange.manual;
        outCompleted = waterChange.outPhaseCompleted;
        processed = waterChange.processedVolume;
        target = waterChange.targetVolume;
        currentStatus = waterChange.status;
        remainingMs = waterChange.remainingMs;
        startMillis = waterChange.startMillis;
        phaseStartMillis = waterChange.phaseStartMillis;
        estimatedCompletionTime = waterChange.estimatedCompletionTime;
        xSemaphoreGive(waterChange.mutex);
    } else {
        Serial.println("Warning: Couldn't get water change mutex");
        return;
    }
    
    // Debug logging for state tracking
    static unsigned long lastStateDebug = 0;

    
    // Reset static flags when a new water change starts
    static bool wasActive = false;
    if (isActive && !wasActive) {
        // New water change just started
        scheduledWaterChangeStarted = false;
        inPhaseStarted = false;

    }
    wasActive = isActive;

    // Safety timeout check - force stop if water change runs beyond estimated completion + buffer
    if (isActive && estimatedCompletionTime > 0 && 
        estimatedCompletionTime < 0xFFFFFFFF - TIMEOUT_BUFFER && // Overflow protection
        currentMillis > (estimatedCompletionTime + TIMEOUT_BUFFER)) {
        Serial.printf("WARNING: Water change timeout reached - estimated completion: %lu, current: %lu, forcing stop\n", 
                     estimatedCompletionTime, currentMillis);
        if (xSemaphoreTake(waterChange.mutex, pdMS_TO_TICKS(100))) {
            strcpy(waterChange.status, "timeout");
            waterChange.needsProgressUpdate = true;
            firebaseNeedsUpdate = true;
            xSemaphoreGive(waterChange.mutex);
        }
        endWaterChange();
        return;
    }

    // Handle stop command (highest priority)
    if (currentStatus == "stopping") {
        endWaterChange();
        return; // Exit immediately after stopping
    }

    // Check for manual water change requests
    if ((manualWater.waterIn || manualWater.waterOut) && !isActive) {
        if (xSemaphoreTake(waterChange.mutex, pdMS_TO_TICKS(100))) {
            waterChange.active = true;
            waterChange.manual = true;
            waterChange.outPhaseCompleted = false;
            waterChange.startMillis = currentMillis;
            waterChange.phaseStartMillis = currentMillis;
            // Store the initial direction for this manual water change
            waterChange.manualWaterInMode = manualWater.waterIn;
            // Use the water change volume from Firebase settings for manual water changes
            waterChange.targetVolume = waterChangeVolume;
            waterChange.processedVolume = 0;
            waterChange.actualVolumeProcessed = 0;
            // Reset volume tracking variables
            waterChange.processedVolumeOutPhase = 0.0;
            waterChange.processedVolumeInPhase = 0.0;
            // Use calibrated flow rate for duration calculation
            float avgCalibration = (waterChangeInCalibration + waterChangeOutCalibration) / 2.0f;
            float calibratedFlowRate = FLOW_RATE_ML_PER_MIN * avgCalibration;
            waterChange.durationMs = (waterChange.targetVolume * ML_PER_GALLON * MS_PER_MINUTE) / calibratedFlowRate;
            waterChange.remainingMs = waterChange.durationMs;
            // Overflow protection for estimated completion time
            if (millis() < 0xFFFFFFFF - waterChange.durationMs) {
                waterChange.estimatedCompletionTime = millis() + waterChange.durationMs;
            } else {
                waterChange.estimatedCompletionTime = 0xFFFFFFFF; // Max value if overflow
            }
            waterChange.needsRemainingUpdate = true;
            strcpy(waterChange.status, "running");
            waterChange.needsProgressUpdate = true;
            firebaseNeedsUpdate = true;
            
            strcpy(lastWaterChangeTime, getFormattedTime().c_str());
            firebaseNeedsUpdate = true;
            xSemaphoreGive(waterChange.mutex);
            
            // Clear triggering flags in Firebase to avoid repeat triggers
            char tempPath[128];
            if (manualWater.waterIn) {
                buildPath(tempPath, MANUAL_CONTROL_PATH, "/water_in");
                Database.set(aClient, tempPath, false, asyncCB);
            }
            if (manualWater.waterOut) {
                buildPath(tempPath, MANUAL_CONTROL_PATH, "/water_out");
                Database.set(aClient, tempPath, false, asyncCB);
            }

            // Reset watchdog activity timestamp for new water change
            lastWaterChangeActivity = millis();
            
            Serial.printf("=== MANUAL WATER CHANGE TRIGGERED ===\n");
            Serial.printf("Direction: %s\n", waterChange.manualWaterInMode ? "WATER IN" : "WATER OUT");
            Serial.printf("Target volume: %.1f gallons\n", waterChange.targetVolume);
            Serial.printf("Status: %s\n", waterChange.status);
            Serial.printf("Direction stored for full cycle completion\n");
            Serial.printf("=====================================\n");
        }
    }

    // Calculate motor states
    bool shouldRunOut = false;
    bool shouldRunIn = false;

    if (isActive && (currentStatus == "running" || currentStatus == "stopping")) {
        // E-STOP CHECK: Check if E-stop is pressed (highest priority)
        if (readEffectiveEstopPin() == LOW) {
            Serial.println("E-STOP: E-stop is pressed - stopping water change immediately");
            if (xSemaphoreTake(waterChange.mutex, pdMS_TO_TICKS(100))) {
                strcpy(waterChange.status, "estop_interrupted");
                waterChange.needsProgressUpdate = true;
                firebaseNeedsUpdate = true;
                xSemaphoreGive(waterChange.mutex);
            }
            // Set E-stop flags for processing
            estopPressed = true;
            estopPressTime = millis();
            endWaterChange(true); // Stop with E-stop flag
            return; // Exit immediately
        }
        
        if (isManual) {
            // MANUAL STOP: Check if stop water change flag is set
            if (manualWater.stopWaterChange) {
                Serial.println("MANUAL STOP: Stop water change flag is set - stopping water change immediately");
                if (xSemaphoreTake(waterChange.mutex, pdMS_TO_TICKS(100))) {
                    strcpy(waterChange.status, "manually_stopped");
                    waterChange.needsProgressUpdate = true;
                    firebaseNeedsUpdate = true;
                    xSemaphoreGive(waterChange.mutex);
                }
                // Clear the stop flag and manual water flags
                manualWater.stopWaterChange = false;
                manualWater.waterIn = false;
                manualWater.waterOut = false;
                manualWater.manuallyReset = true;
                endWaterChange(false); // Stop normally
                return; // Exit immediately
            }
            
            // Use the stored direction to determine which pump to run
            // This prevents early stopping if Firebase flags are cleared during the water change
            if (!waterChange.manualWaterInMode) {  // Water OUT mode
                shouldRunOut = true;
                // Use calibrated flow rate for progress calculation
                float calibrationFactor = waterChangeOutCalibration;
                float calibratedFlowRate = FLOW_RATE_ML_PER_MIN * calibrationFactor;
                processed = ((currentMillis - startMillis) * calibratedFlowRate) / 
                           (MS_PER_MINUTE * ML_PER_GALLON);
                
                // Track progress for display/telemetry without altering cumulative dosed amounts
                if (xSemaphoreTake(doseMutex, pdMS_TO_TICKS(100))) {
                    previousValues.waterOutDosed = processed;
    
                    firebaseNeedsUpdate = true; // Trigger Firebase update for progress
                    xSemaphoreGive(doseMutex);
                } else {
                    Serial.println("ERROR: Failed to acquire doseMutex for manual OUT phase update");
                }
                
                // Update water change state separately
                if (xSemaphoreTake(waterChange.mutex, pdMS_TO_TICKS(100))) {
                    waterChange.actualVolumeProcessed = processed;
                    waterChange.processedVolumeOutPhase = processed;  // Track OUT phase volume
                    xSemaphoreGive(waterChange.mutex);
                }
                
                // Update estimated completion time based on remaining volume
                float remainingVolume = waterChange.targetVolume - processed;
                if (remainingVolume > 0) {
                    // Use calibrated flow rate for remaining time calculation
                    float calibrationFactor = waterChangeOutCalibration;
                    float calibratedFlowRate = FLOW_RATE_ML_PER_MIN * calibrationFactor;
                    unsigned long remainingTime = (remainingVolume * ML_PER_GALLON * MS_PER_MINUTE) / calibratedFlowRate;
                    if (xSemaphoreTake(waterChange.mutex, pdMS_TO_TICKS(100))) {
                        // Overflow protection for estimated completion time
                        if (currentMillis < 0xFFFFFFFF - remainingTime) {
                            waterChange.estimatedCompletionTime = currentMillis + remainingTime;
                        } else {
                            waterChange.estimatedCompletionTime = 0xFFFFFFFF; // Max value if overflow
                        }
                        waterChange.remainingMs = remainingTime;
                        waterChange.needsRemainingUpdate = true;
                        firebaseNeedsUpdate = true;
                        xSemaphoreGive(waterChange.mutex);
                    }
                }
            } 
            else if (waterChange.manualWaterInMode) {  // Water IN mode
                shouldRunIn = true;
                // Use calibrated flow rate for progress calculation
                float calibrationFactor = waterChangeInCalibration;
                float calibratedFlowRate = FLOW_RATE_ML_PER_MIN * calibrationFactor;
                processed = ((currentMillis - startMillis) * calibratedFlowRate) / 
                           (MS_PER_MINUTE * ML_PER_GALLON);
                
                // Track progress for display/telemetry without altering cumulative dosed amounts
                if (xSemaphoreTake(doseMutex, pdMS_TO_TICKS(100))) {
                    previousValues.waterInDosed = processed;
    
                    firebaseNeedsUpdate = true; // Trigger Firebase update for progress
                    xSemaphoreGive(doseMutex);
                } else {
                    Serial.println("ERROR: Failed to acquire doseMutex for manual IN phase update");
                }
                
                // Update water change state separately
                if (xSemaphoreTake(waterChange.mutex, pdMS_TO_TICKS(100))) {
                    waterChange.actualVolumeProcessed = processed;
                    waterChange.processedVolumeInPhase = processed;  // Track IN phase volume
                    xSemaphoreGive(waterChange.mutex);
                }
                
                // Update estimated completion time based on remaining volume
                float remainingVolume = waterChange.targetVolume - processed;
                if (remainingVolume > 0) {
                    // Use calibrated flow rate for remaining time calculation
                    float calibrationFactor = waterChangeInCalibration;
                    float calibratedFlowRate = FLOW_RATE_ML_PER_MIN * calibrationFactor;
                    unsigned long remainingTime = (remainingVolume * ML_PER_GALLON * MS_PER_MINUTE) / calibratedFlowRate;
                    if (xSemaphoreTake(waterChange.mutex, pdMS_TO_TICKS(100))) {
                        // Overflow protection for estimated completion time
                        if (currentMillis < 0xFFFFFFFF - remainingTime) {
                            waterChange.estimatedCompletionTime = currentMillis + remainingTime;
                        } else {
                            waterChange.estimatedCompletionTime = 0xFFFFFFFF; // Max value if overflow
                        }
                        waterChange.remainingMs = remainingTime;
                        waterChange.needsRemainingUpdate = true;
                        firebaseNeedsUpdate = true;
                        xSemaphoreGive(waterChange.mutex);
                    }
                }
            }

            // Check if manual water change has reached target volume
    Serial.printf("DEBUG: Manual water change completion check - processed=%.3f, target=%.3f, threshold=%.3f\n", 
                processed, waterChange.targetVolume, waterChange.targetVolume - 0.01f);
    if (processed >= waterChange.targetVolume - 0.01f) { // Add tolerance for floating point

        if (xSemaphoreTake(waterChange.mutex, pdMS_TO_TICKS(100))) {
            strcpy(waterChange.status, "completed");
            waterChange.needsProgressUpdate = true;
            strcpy(previousValues.waterChangeStatus, "completed");
            // Only reset flags when water change is completed successfully
            manualWater.waterOut = false;
            manualWater.waterIn = false;
            manualWater.completed = true;
            manualWater.manuallyReset = false; // Reset the manual reset flag
            manualWater.stopWaterChange = false; // Reset stop water change flag
            firebaseNeedsUpdate = true;
            xSemaphoreGive(waterChange.mutex);
        }
                // Persist cleared triggers in Firebase to prevent re-runs
                char tempPath[128];
                buildPath(tempPath, MANUAL_CONTROL_PATH, "/water_in");
                Database.set(aClient, tempPath, false, asyncCB);
                buildPath(tempPath, MANUAL_CONTROL_PATH, "/water_out");
                Database.set(aClient, tempPath, false, asyncCB);
                endWaterChange();
                Serial.printf("Manual water change completed: %.3f gallons processed\n", processed);
                return; // Exit the function immediately after completion
            }
        } else {
            // E-STOP CHECK: Check if E-stop is pressed (highest priority for scheduled water changes too)
            if (readEffectiveEstopPin() == LOW) {
                Serial.println("E-STOP: E-stop is pressed during scheduled water change - stopping immediately");
                if (xSemaphoreTake(waterChange.mutex, pdMS_TO_TICKS(100))) {
                    strcpy(waterChange.status, "estop_interrupted");
                    waterChange.needsProgressUpdate = true;
                    firebaseNeedsUpdate = true;
                    xSemaphoreGive(waterChange.mutex);
                }
                // Set E-stop flags for processing
                estopPressed = true;
                estopPressTime = millis();
                endWaterChange(true); // Stop with E-stop flag
                return; // Exit immediately
            }
            
            // Scheduled water change logic - check if manual control has become active or stop is requested
            if (manualWater.waterIn || manualWater.waterOut) {
                // Manual control has been activated - stop scheduled water change
                Serial.println("Scheduled water change stopped - manual control activated");
                if (xSemaphoreTake(waterChange.mutex, pdMS_TO_TICKS(100))) {
                    strcpy(waterChange.status, "stopped");
                    waterChange.needsProgressUpdate = true;
                    firebaseNeedsUpdate = true;
                    xSemaphoreGive(waterChange.mutex);
                }
                endWaterChange();
                return;
            }
            
            // Check if manual stop is requested for scheduled water change
            if (manualWater.stopWaterChange) {
                Serial.println("Scheduled water change stopped - manual stop requested");
                if (xSemaphoreTake(waterChange.mutex, pdMS_TO_TICKS(100))) {
                    strcpy(waterChange.status, "manually_stopped");
                    waterChange.needsProgressUpdate = true;
                    firebaseNeedsUpdate = true;
                    xSemaphoreGive(waterChange.mutex);
                }
                // Clear the stop flag
                manualWater.stopWaterChange = false;
                endWaterChange(false);
                return;
            }
            
            if (!scheduledWaterChangeStarted) {
                scheduledWaterChangeStarted = true;
            }
            if (!outCompleted) {
                    // Water OUT phase
                    shouldRunOut = true;
                    // Use calibrated flow rate for progress calculation
                    float calibrationFactor = waterChangeOutCalibration;
                    float calibratedFlowRate = FLOW_RATE_ML_PER_MIN * calibrationFactor;
                    processed = ((currentMillis - startMillis) * calibratedFlowRate) / 
                               (MS_PER_MINUTE * ML_PER_GALLON);
                    


                // Update processed volume for OUT phase
                if (xSemaphoreTake(waterChange.mutex, pdMS_TO_TICKS(100))) {
                    waterChange.processedVolumeOutPhase = processed;
                    firebaseNeedsUpdate = true;
                    xSemaphoreGive(waterChange.mutex);
                }

                // Update estimated completion time for OUT phase
                float remainingVolume = target - processed;
                if (remainingVolume > 0) {
                    // Use calibrated flow rates for remaining time calculation
                    float outCalibrationFactor = waterChangeOutCalibration;
                    float inCalibrationFactor = waterChangeInCalibration;
                    float outCalibratedFlowRate = FLOW_RATE_ML_PER_MIN * outCalibrationFactor;
                    float inCalibratedFlowRate = FLOW_RATE_ML_PER_MIN * inCalibrationFactor;
                    unsigned long remainingTime = (remainingVolume * ML_PER_GALLON * MS_PER_MINUTE) / outCalibratedFlowRate;
                    // For scheduled water changes, add time for both OUT and IN phases
                    unsigned long totalRemainingTime = remainingTime + (target * ML_PER_GALLON * MS_PER_MINUTE) / inCalibratedFlowRate;
                    waterChange.estimatedCompletionTime = currentMillis + totalRemainingTime;
                    waterChange.remainingMs = totalRemainingTime;
                    waterChange.needsRemainingUpdate = true;
                    firebaseNeedsUpdate = true;
                }

                // Check if OUT phase completed
                if (processed >= target - 0.01f) { // Add tolerance for floating point
                    if (xSemaphoreTake(waterChange.mutex, pdMS_TO_TICKS(100))) {
                        waterChange.outPhaseCompleted = true;
                        waterChange.processedVolumeOutPhase = processed;  // Final OUT phase volume
                        waterChange.phaseStartMillis = currentMillis;
                        waterChange.needsProgressUpdate = true;
                        xSemaphoreGive(waterChange.mutex);
                    }
                
                }
            } else {
                // Water IN phase
                if (!inPhaseStarted) {
                    inPhaseStarted = true;
                }
                shouldRunIn = true;
                // Calculate IN phase processed volume (since OUT phase completed)
                // Use calibrated flow rate for progress calculation
                float calibrationFactor = waterChangeInCalibration;
                float calibratedFlowRate = FLOW_RATE_ML_PER_MIN * calibrationFactor;
                float processed = ((currentMillis - phaseStartMillis) * calibratedFlowRate) / 
                           (MS_PER_MINUTE * ML_PER_GALLON);
                
                // Debug calibration values for IN phase
                static unsigned long lastInCalibrationDebug = 0;
                if (currentMillis - lastInCalibrationDebug > 5000) { // Every 5 seconds
                    Serial.printf("DEBUG: IN phase calibration (scheduled) - factor: %.3f, flowRate: %.1f ml/min, processed: %.3f gallons\n",
                                calibrationFactor, calibratedFlowRate, processed);
                    lastInCalibrationDebug = currentMillis;
                }
                
        
                // Update processed volume for IN phase
                if (xSemaphoreTake(waterChange.mutex, pdMS_TO_TICKS(100))) {
                    waterChange.processedVolumeInPhase = processed;
                    xSemaphoreGive(waterChange.mutex);
                }
              

                // Update estimated completion time for IN phase
                float remainingVolume = target - processed; // Use IN phase remaining
                if (remainingVolume > 0) {
                    // Use calibrated flow rate for remaining time calculation
                    float calibrationFactor = waterChangeInCalibration;
                    float calibratedFlowRate = FLOW_RATE_ML_PER_MIN * calibrationFactor;
                    unsigned long remainingTime = (remainingVolume * ML_PER_GALLON * MS_PER_MINUTE) / calibratedFlowRate;
                    // For IN phase, only account for remaining IN time
                    if (xSemaphoreTake(waterChange.mutex, pdMS_TO_TICKS(100))) {
                        // Overflow protection for estimated completion time
                        if (currentMillis < 0xFFFFFFFF - remainingTime) {
                            waterChange.estimatedCompletionTime = currentMillis + remainingTime;
                        } else {
                            waterChange.estimatedCompletionTime = 0xFFFFFFFF; // Max value if overflow
                        }
                        waterChange.remainingMs = remainingTime;
                        waterChange.needsRemainingUpdate = true;
                        firebaseNeedsUpdate = true;
                        xSemaphoreGive(waterChange.mutex);
                    }
                }

                // Check if IN phase completed
                if (processed >= target - 0.01f) { // Add tolerance for floating point
                    if (xSemaphoreTake(waterChange.mutex, pdMS_TO_TICKS(100))) {
                        strcpy(waterChange.status, "completed");
                        waterChange.processedVolumeInPhase = processed;  // Final IN phase volume
                        waterChange.needsProgressUpdate = true;
                        xSemaphoreGive(waterChange.mutex);
                    }
                    // Reset stop water change flag before ending
                    manualWater.stopWaterChange = false;
                    endWaterChange();
                    Serial.printf("Scheduled water change completed: %.3f gallons processed\n", processed);
                }
            }
        }

        // Update processed volume in waterChange struct
        if (xSemaphoreTake(waterChange.mutex, pdMS_TO_TICKS(100))) {
            waterChange.processedVolume = processed;
            waterChange.needsProgressUpdate = true;
            xSemaphoreGive(waterChange.mutex);
        }
    }

    // Only continue with motor updates and progress if water change is active
    if (!isActive) {
        return; // Exit if no water change is active
    }

    // Update motor states - use robust motor control function
    setMotorState(shouldRunIn, shouldRunOut);
    
    // Validate water change state
    validateWaterChangeState();
    
    // If status is stopping, call endWaterChange to properly clean up
    if (isActive && currentStatus == "stopping") {
        endWaterChange(false);
        return; // Exit immediately after stopping
    }

    // Update progress if needed - only if water change is active and running
    if (currentMillis - lastProgressUpdate >= PROGRESS_UPDATE_INTERVAL) {
        if (xSemaphoreTake(waterChange.mutex, pdMS_TO_TICKS(100))) {
            if (waterChange.needsProgressUpdate && waterChange.active && waterChange.status == "running") {
                Serial.printf("Water change progress: %.2f/%.2f gallons\n", 
                            waterChange.processedVolume, waterChange.targetVolume);
                waterChange.needsProgressUpdate = false;
                firebaseNeedsUpdate = true;
            }
            xSemaphoreGive(waterChange.mutex);
            lastProgressUpdate = currentMillis;
        }
    }
}



void endWaterChange(bool stoppedByEstop) {
    Serial.println("Ending water change with enhanced cleanup");
    
    // Declare variables at function scope so they can be used throughout the function
    float actualVolumeOut = 0.0;
    float actualVolumeIn = 0.0;
    bool wasManualWaterChange = false;
    
    // Stop motors immediately
    digitalWrite(MTR_WATER_CHANGE_IN, LOW);
    digitalWrite(MTR_WATER_CHANGE_OUT, LOW);
    
    // Reset motor state tracking
    motorState.waterInRunning = false;
    motorState.waterOutRunning = false;
    
    if (xSemaphoreTake(waterChange.mutex, pdMS_TO_TICKS(100))) {
        // Capture processed volumes from each phase
        actualVolumeOut = waterChange.processedVolumeOutPhase;
        actualVolumeIn = waterChange.processedVolumeInPhase;
        wasManualWaterChange = waterChange.manual;
        

        
        if (waterChange.manual) {
            Serial.printf("Manual water change completed - OUT: %.3f, IN: %.3f gallons\n", 
                        actualVolumeOut, actualVolumeIn);
        } else {
            Serial.printf("Scheduled water change completed - OUT: %.3f, IN: %.3f gallons\n", 
                        actualVolumeOut, actualVolumeIn);
        }
        
        // Clear any global water change flags for consistency
        isWaterChangeActive = false;
        isManualWaterChange = false;
        
        // Set completion status to indicate water change is complete
        firebaseNeedsUpdate = true;
        xSemaphoreGive(waterChange.mutex);
    }
    
    // Update dosed amounts with processed volumes
    if (actualVolumeOut > 0 || actualVolumeIn > 0) {
        if (xSemaphoreTake(doseMutex, pdMS_TO_TICKS(100))) {
            Serial.printf("Before update: water_in=%.3f, water_out=%.3f gallons\n", 
                        dosedAmounts.water_in, dosedAmounts.water_out);
            
            // Simple approach: Add the actual processed volumes to dosed amounts
            if (actualVolumeOut > 0) {
                dosedAmounts.water_out += actualVolumeOut;
                Serial.printf("Water change - Added %.3f gallons to water_out dosed amount (total: %.3f)\n", 
                            actualVolumeOut, dosedAmounts.water_out);
            }
            
            if (actualVolumeIn > 0) {
                dosedAmounts.water_in += actualVolumeIn;
                Serial.printf("Water change - Added %.3f gallons to water_in dosed amount (total: %.3f)\n", 
                            actualVolumeIn, dosedAmounts.water_in);
            }
            
            Serial.printf("After update: water_in=%.3f, water_out=%.3f gallons\n", 
                        dosedAmounts.water_in, dosedAmounts.water_out);
            firebaseNeedsUpdate = true;
            xSemaphoreGive(doseMutex);
        } else {
            Serial.println("ERROR: Failed to acquire doseMutex for dosedAmounts update in endWaterChange");
        }
    }

    // Reset all water change state

    previousValues.waterInDosed = 0;
    previousValues.waterOutDosed = 0;
    waterChange.active = false;
    waterChange.manual = false;
    waterChange.manualWaterInMode = false; // Reset stored direction
    waterChange.outPhaseCompleted = false;
    waterChange.actualVolumeProcessed = 0;
    // Reset all volume tracking variables after dosed amounts are updated
    waterChange.processedVolumeOutPhase = 0.0;
    waterChange.processedVolumeInPhase = 0.0;
    waterChange.estimatedCompletionTime = 0;
    waterChange.needsProgressUpdate = false; // Stop progress updates
    waterChange.remainingMs = 0; // Reset remaining time
    waterChange.needsRemainingUpdate = true; // Force update to reset time remaining
    waterChange.state = WC_IDLE; // Reset state machine
    // Reset stop water change flag regardless of how water change ended
    manualWater.stopWaterChange = false;
    
    // Only reset manual water change flags if manually reset or water change completed successfully
    if (manualWater.manuallyReset || manualWater.completed) {
        manualWater.waterIn = false;
        manualWater.waterOut = false;
        manualWater.completed = true;
        manualWater.manuallyReset = false; // Reset the manual reset flag
    }
    strcpy(waterChange.status, "ready");        
    strcpy(lastWaterChangeTime, getFormattedTime().c_str());
    strcpy(previousValues.waterChangeStatus, "ready"); // Reset to ready instead of completed
    
    // Reset stop_water_change flag in Firebase
    if (firebaseState == FIREBASE_READY) {
        char tempPath[128];
        buildPath(tempPath, MANUAL_CONTROL_PATH, "/stop_water_change");
        Database.set(aClient, tempPath, false, asyncCB);
        Serial.println("Reset stop_water_change flag in Firebase");
    }
    
    // Reset watchdog after successful cleanup
    resetWatchdog();
    lastWaterChangeActivity = millis();
    
    Serial.println("Water change cleanup completed");
}





void resetReservoir(const char* reservoirType) {
  if (xSemaphoreTake(doseMutex, pdMS_TO_TICKS(100))) {
    if (strcmp(reservoirType, "water_in") == 0) {
      dosedAmounts.water_in = 0;
      containerStatus.waterEmpty = false;
    } 
    else if (strcmp(reservoirType, "water_out") == 0) {
      dosedAmounts.water_out = 0;
      containerStatus.waterFull = false;
    }
    firebaseNeedsUpdate = true;
    xSemaphoreGive(doseMutex);
    Serial.printf("%s reservoir reset\n", reservoirType);
  }
}

void checkReservoirs() {
  // Check fresh water reservoir (water_in)
  float inVolume = containerVolumes.water_in;
  float inDosed = dosedAmounts.water_in;
  bool inEmpty = (inVolume - inDosed) <= 0.1; // 0.1 gallon threshold
  containerStatus.waterEmpty = inEmpty;
  
  // Check waste water reservoir (water_out)
  float outVolume = containerVolumes.water_out;
  float outDosed = dosedAmounts.water_out;
  bool outFull = (outVolume - outDosed) <= 0.1; // Considered full when 0.1gal left
  containerStatus.waterFull = outFull;
  
  // Debug reservoir status periodically
  static unsigned long lastReservoirDebug = 0;
  if (millis() - lastReservoirDebug > 60000) { // Every minute
    Serial.printf("Reservoir Status - IN: %.1f/%.1f gal (%s), OUT: %.1f/%.1f gal (%s)\n",
                inDosed, inVolume, inEmpty ? "EMPTY" : "OK",
                outDosed, outVolume, outFull ? "FULL" : "OK");
    lastReservoirDebug = millis();
  }
  
  // Emergency stop checks
  if ((inEmpty && digitalRead(MTR_WATER_CHANGE_IN)) || 
      (outFull && digitalRead(MTR_WATER_CHANGE_OUT))) {
    Serial.println("EMERGENCY STOP: Reservoir limit reached");
    Serial.printf("IN reservoir: %s, OUT reservoir: %s\n", 
                inEmpty ? "EMPTY" : "OK", outFull ? "FULL" : "OK");
    Serial.printf("Motor IN: %s, Motor OUT: %s\n",
                digitalRead(MTR_WATER_CHANGE_IN) == HIGH ? "ON" : "OFF",
                digitalRead(MTR_WATER_CHANGE_OUT) == HIGH ? "ON" : "OFF");
    endWaterChange();
  }
}

// Removed printWaterChangeDebugInfo() to save memory

void checkAndConnectFirebase() {
    if (!app.ready()) {
        Serial.println("Initializing Firebase connection...");
        Serial.printf("Database URL: %s\n", DATABASE_URL);
        ssl_client.setInsecure();
        initializeApp(aClient, app, getAuth(dbSecret), asyncCB, "authtask");
        app.getApp<RealtimeDatabase>(Database);
        Database.url(DATABASE_URL);
        Serial.println("Firebase initialized successfully");
    } else {
        Serial.println("Firebase already ready");
        
        // Load dosed amounts from Firebase on first successful connection
        if (!dosedAmountsLoaded && firebaseState == FIREBASE_READY) {
            Serial.println("Loading dosed amounts from Firebase on restart...");
            loadDosedAmountsFromFirebase();
            dosedAmountsLoaded = true;
        }
    }
}

// Improved Firebase connection management
void manageFirebaseConnection() {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("Firebase: WiFi not connected, skipping connection management");
        return;
    }

    unsigned long currentMillis = millis();
    
    switch(firebaseState) {
        case FIREBASE_DISCONNECTED:
            if (currentMillis - lastConnectionAttempt >= CONNECTION_RETRY_INTERVAL) {
                Serial.println("Attempting Firebase connection...");
                Serial.printf("Last attempt was %lu ms ago\n", currentMillis - lastConnectionAttempt);
                ssl_client.setInsecure();
                initializeApp(aClient, app, getAuth(dbSecret), asyncCB, "authtask");
                app.getApp<RealtimeDatabase>(Database);
                Database.url(DATABASE_URL);
                firebaseState = FIREBASE_CONNECTING;
                lastConnectionAttempt = currentMillis;
            }
            break;
            
        case FIREBASE_CONNECTING:
            if (app.ready()) {
                firebaseState = FIREBASE_READY;
                Serial.println("Firebase connected successfully");
                
                // Load dosed amounts from Firebase on first successful connection
                if (!dosedAmountsLoaded) {
                    Serial.println("Loading dosed amounts from Firebase on restart...");
                    loadDosedAmountsFromFirebase();
                    dosedAmountsLoaded = true;
                }
            } else if (currentMillis - lastConnectionAttempt > 10000) {
                firebaseState = FIREBASE_DISCONNECTED;
                Serial.println("Firebase connection timed out");
                Serial.printf("Connection attempt lasted %lu ms\n", currentMillis - lastConnectionAttempt);
            }
            break;
            
        case FIREBASE_READY:
            if (!app.ready()) {
                firebaseState = FIREBASE_DISCONNECTED;
                Serial.println("Firebase connection lost");
            }
            break;
    }
}

void fetchAllSettings() {
    if (firebaseState != FIREBASE_READY) {
        Serial.println("Firebase not ready, skipping settings fetch");
        return;
    }
    
    Serial.printf("Fetching settings from path: %s\n", SETTINGS_PATH);
    Database.get(aClient, SETTINGS_PATH, asyncCB, false, "getSettings");
}



// Modified updateFirebase function
void updateFirebase() {
    if (xSemaphoreTake(floatMutex, pdMS_TO_TICKS(5000))) {
        manageFirebaseConnection();

        if (firebaseState == FIREBASE_READY) {
            // Read temperature (but only check for updates every 5 minutes)
            unsigned long currentMillis = millis();
            if (currentMillis - lastTempUpdate >= TEMP_UPDATE_INTERVAL) {
                Serial.println("Reading temperature sensors...");
                sensors.requestTemperatures();
                float tempF = sensors.getTempFByIndex(0);
                
                // Only update if temperature changed by threshold or more
                if (tempF != DEVICE_DISCONNECTED_F && 
                    (fabs(tempF - lastReportedTemp) >= TEMP_CHANGE_THRESHOLD)) {
                    
                    temperatureF = tempF;
                    firebaseNeedsUpdate = true;
                    Serial.printf("Temperature change detected: %.1f°F (threshold: %.1f°F)\n", 
                                tempF - lastReportedTemp, TEMP_CHANGE_THRESHOLD);

                } else {
                    Serial.printf("Temperature: %.1f°F (no significant change)\n", tempF);

                }
                lastTempUpdate = currentMillis;
            }

            // Always update when it's time (to catch manual dose completions, etc.)
            if (millis() - lastFirebaseUpdate >= FIREBASE_INTERVAL) {
                firebaseNeedsUpdate = true;
                Serial.println("Firebase update interval reached - triggering update");
            }

            if (firebaseNeedsUpdate) {
                Serial.println("Writing batch update to Firebase...");
                batchWriteToFirebase();
                firebaseNeedsUpdate = false;
            }

            bool shouldRequestAtoSync = false;
            if (xSemaphoreTake(doseMutex, pdMS_TO_TICKS(10))) {
                if (pendingAtoIncrementGal > 0.0f && !atoSyncRequestInFlight) {
                    atoSyncRequestInFlight = true;
                    shouldRequestAtoSync = true;
                }
                xSemaphoreGive(doseMutex);
            }
            if (shouldRequestAtoSync) {
                requestAtoDosedFromFirebase();
            }
            
            // Fetch settings (less frequently than status updates)
            static unsigned long lastSettingsFetch = 0;
            if (millis() - lastSettingsFetch >= FIREBASE_INTERVAL * 2) {
                Serial.println("Fetching settings from Firebase...");
                lastSettingsFetch = millis();
                fetchAllSettings();
            }
        } else {
            Serial.printf("Firebase not ready (state: %d), skipping update\n", firebaseState);
        }
        xSemaphoreGive(floatMutex);
    } else {
        Serial.println("Could not acquire float mutex for Firebase update");
    }
}



void reportDeviceStatus() {
    if (WiFi.status() != WL_CONNECTED) {
        return; // Don't report if not connected
    }
    
    StaticJsonDocument<256> doc; // Further reduced to save memory
    doc["wifi_connected"] = true;
    doc["ip_address"] = WiFi.localIP().toString();
    doc["rssi"] = WiFi.RSSI();
    doc["uptime"] = millis();
    
    // Use char array instead of String for timestamp
    char lastSeen[20];
    struct tm timeinfo;
    if (getLocalTime(&timeinfo)) {
        strftime(lastSeen, sizeof(lastSeen), "%Y-%m-%d %H:%M:%S", &timeinfo);
    } else {
        strcpy(lastSeen, "Time Error");
    }
    doc["last_seen"] = lastSeen;
    
    String jsonString;
    serializeJson(doc, jsonString);
    
    // Report to device-specific status path
    {
    char deviceStatusPath[128];
    strcpy(deviceStatusPath, BASE_PATH);
    strcat(deviceStatusPath, "/status");
    Database.set(aClient, deviceStatusPath, object_t(jsonString), asyncCB);
    }
    

}

void asyncCB(AsyncResult &aResult) {
    // 1. Error handling
    if (aResult.isError()) {
        // Check if this is an OTA-related error
        String uid = aResult.uid();
        if (uid == "getAtoDosed") {
            atoSyncRequestInFlight = false;
            if (xSemaphoreTake(doseMutex, pdMS_TO_TICKS(100))) {
                atoSyncRequestInFlight = false;
                xSemaphoreGive(doseMutex);
            }
            Serial.printf("ATO Firebase sync error [%s]: %s (code %d)\n",
                        uid.c_str(),
                        aResult.error().message().c_str(),
                        aResult.error().code());
            return;
        }
        if (uid == "getOTAVersion" || uid == "getOTAURL" || uid == "getOTAForce") {
            Serial.printf("OTA Firebase error [%s]: %s (code %d)\n",
                        uid.c_str(),
                        aResult.error().message().c_str(),
                        aResult.error().code());
            // Reset OTA state to IDLE on error
            if (xSemaphoreTake(otaMutex, pdMS_TO_TICKS(100))) {
                if (otaInfo.state == OTA_CHECKING) {
                    otaInfo.state = OTA_IDLE;
                    Serial.println("OTA check failed - resetting to IDLE");
                }
                xSemaphoreGive(otaMutex);
            }
        } else {
            Serial.printf("Firebase Error [%s]: %s (code %d)\n",
                        aResult.uid().c_str(),
                        aResult.error().message().c_str(),
                        aResult.error().code());
        }
        return;
    }

    // 2. Only process successful responses with data
    if (aResult.available()) {
        // Handle dosed amounts loading responses
        if (aResult.uid() == "getDose1Dosed") {
            int dosedAmount = aResult.to<RealtimeDatabaseResult>().to<int>();
            if (xSemaphoreTake(doseMutex, pdMS_TO_TICKS(100))) {
                dosedAmounts.dose1 = dosedAmount;
                if (dosedAmount <= 0) {
                    containerStatus.dose1Empty = false;
                    Serial.println("Dose1 refill detected from dosed_amounts reset to 0");
                }
                Serial.printf("Loaded dose1 dosed amount from Firebase: %d ml\n", dosedAmount);
                xSemaphoreGive(doseMutex);
            }
            return;
        }
        
        if (aResult.uid() == "getDose2Dosed") {
            int dosedAmount = aResult.to<RealtimeDatabaseResult>().to<int>();
            if (xSemaphoreTake(doseMutex, pdMS_TO_TICKS(100))) {
                dosedAmounts.dose2 = dosedAmount;
                if (dosedAmount <= 0) {
                    containerStatus.dose2Empty = false;
                    Serial.println("Dose2 refill detected from dosed_amounts reset to 0");
                }
                Serial.printf("Loaded dose2 dosed amount from Firebase: %d ml\n", dosedAmount);
                xSemaphoreGive(doseMutex);
            }
            return;
        }
        
        if (aResult.uid() == "getDose3Dosed") {
            int dosedAmount = aResult.to<RealtimeDatabaseResult>().to<int>();
            if (xSemaphoreTake(doseMutex, pdMS_TO_TICKS(100))) {
                dosedAmounts.dose3 = dosedAmount;
                if (dosedAmount <= 0) {
                    containerStatus.dose3Empty = false;
                    Serial.println("Dose3 refill detected from dosed_amounts reset to 0");
                }
                Serial.printf("Loaded dose3 dosed amount from Firebase: %d ml\n", dosedAmount);
                xSemaphoreGive(doseMutex);
            }
            return;
        }
        
        if (aResult.uid() == "getWaterInDosed") {
            float dosedAmount = aResult.to<RealtimeDatabaseResult>().to<float>();
            if (xSemaphoreTake(doseMutex, pdMS_TO_TICKS(100))) {
                dosedAmounts.water_in = dosedAmount;
                if (dosedAmount <= 0.0001f) {
                    containerStatus.waterEmpty = false;
                    Serial.println("Water IN refill detected from dosed_amounts reset to 0");
                }
                Serial.printf("Loaded water_in dosed amount from Firebase: %.2f gallons\n", dosedAmount);
                xSemaphoreGive(doseMutex);
            }
            return;
        }
        
        if (aResult.uid() == "getWaterOutDosed") {
            float dosedAmount = aResult.to<RealtimeDatabaseResult>().to<float>();
            if (xSemaphoreTake(doseMutex, pdMS_TO_TICKS(100))) {
                dosedAmounts.water_out = dosedAmount;
                if (dosedAmount <= 0.0001f) {
                    containerStatus.waterFull = false;
                    Serial.println("Water OUT reset detected from dosed_amounts reset to 0");
                }
                Serial.printf("Loaded water_out dosed amount from Firebase: %.2f gallons\n", dosedAmount);
                xSemaphoreGive(doseMutex);
            }
            return;
        }
        
        if (aResult.uid() == "getAtoDosed") {
            RealtimeDatabaseResult dbResult = aResult.to<RealtimeDatabaseResult>();
            float dosedAmount = dbResult.to<float>();
            String dosedRaw = dbResult.to<String>();
            if (xSemaphoreTake(doseMutex, pdMS_TO_TICKS(100))) {
                bool wasAtoEmpty = containerStatus.atoEmpty;
                // Treat integer/string zero from Firebase as a hard reset trigger.
                bool resetToZero = (dosedAmount <= 0.001f) ||
                                   (dosedRaw == "0") ||
                                   (dosedRaw == "0.0") ||
                                   (dosedRaw == "0.00") ||
                                   (dosedRaw == "0.000");
                float localBeforeSync = dosedAmounts.ato;
                float queuedIncrement = pendingAtoIncrementGal;
                pendingAtoIncrementGal = 0.0f;
                Serial.printf("getAtoDosed callback: firebase=%.6f (raw='%s'), resetToZero=%s, wasAtoEmpty=%s\n",
                            dosedAmount,
                            dosedRaw.c_str(),
                            resetToZero ? "true" : "false",
                            wasAtoEmpty ? "true" : "false");

                // Refill reset path must be immediate and unconditional.
                if (resetToZero) {
                    dosedAmounts.ato = 0.0f;
                    if (queuedIncrement > 0.0f) {
                        dosedAmounts.ato = queuedIncrement;
                        Serial.printf("ATO reset: preserving queued increment of %.3f gal from pump run\n", queuedIncrement);
                    }
                    containerStatus.atoEmpty = false;
                    atoEmptyNotified = false;
                    floatState = readEffectiveFloatPin();
                    atoRefillRecheckPending = true;
                    floatChanged = true;
                    int estopNow = readEffectiveEstopPin();
                    estopState = estopNow;
                    bool canResumeAto = (floatState == HIGH) &&
                                        !isWaterChangeActive &&
                                        (containerVolumes.ato > 0.0f);
                    atoSyncRequestInFlight = false;
                    Serial.println("ATO refill detected from dosed_amounts reset to 0 - container status reset to full, resuming ATO control");
                    Serial.printf("ATO reset status: dosed=%.3f gal, volume=%.3f gal, remaining=%.3f gal, atoEmpty=%s, float=%s, estop=%s, resumed=%s\n",
                                dosedAmounts.ato,
                                containerVolumes.ato,
                                containerVolumes.ato - dosedAmounts.ato,
                                containerStatus.atoEmpty ? "true" : "false",
                                floatState == HIGH ? "HIGH" : "LOW",
                                estopNow == HIGH ? "OK" : "STOP",
                                canResumeAto ? "true" : "false");
                    Serial.printf("Loaded ATO dosed amount from Firebase: %.3f gal (effective local: %.3f gal)\n",
                                dosedAmount, dosedAmounts.ato);
                    xSemaphoreGive(doseMutex);
                    updateContainerStatus();
                    firebaseNeedsUpdate = true;
                    return;
                }

                // Prevent non-reset async reads from rolling local dosed amount backward.
                dosedAmounts.ato = max(dosedAmount, localBeforeSync);
                // Refill/reset to zero must win over any queued local increment from stale in-flight sync.
                if (!wasAtoEmpty && queuedIncrement > 0.0f) {
                    float cap = containerVolumes.ato;
                    if (cap > 0.0f) {
                        dosedAmounts.ato = min(dosedAmounts.ato + queuedIncrement, cap);
                    } else {
                        dosedAmounts.ato += queuedIncrement;
                    }
                    Serial.printf("ATO sync applied %.3f gal after Firebase read (new total: %.3f gal)\n",
                                queuedIncrement, dosedAmounts.ato);
                }
                if (containerVolumes.ato > 0.0f && dosedAmounts.ato >= (containerVolumes.ato - 0.0005f)) {
                    containerStatus.atoEmpty = true;
                    dosedAmounts.ato = containerVolumes.ato;
                }
                
                if (wasAtoEmpty) {
                    // Once empty is latched, keep ATO dosed pinned to container volume until explicit reset/refill.
                    dosedAmounts.ato = containerVolumes.ato;
                    containerStatus.atoEmpty = true;
                } else if (dosedAmounts.ato <= 0.0001f) {
                    containerStatus.atoEmpty = false;
                    atoEmptyNotified = false;
                } else {
                    containerStatus.atoEmpty = (containerVolumes.ato <= 0.0f) ||
                        ((containerVolumes.ato - dosedAmounts.ato) <= 0.0f);
                    if (!containerStatus.atoEmpty) {
                        atoEmptyNotified = false;
                    }
                }
                atoSyncRequestInFlight = false;
                Serial.printf("Loaded ATO dosed amount from Firebase: %.3f gal (effective local: %.3f gal)\n",
                            dosedAmount, dosedAmounts.ato);
                xSemaphoreGive(doseMutex);
            } else {
                atoSyncRequestInFlight = false;
                Serial.println("getAtoDosed: doseMutex busy, will retry on next cycle");
            }
            updateContainerStatus();
            firebaseNeedsUpdate = true;
            return;
        }
        
        // OTA Update callbacks (same pattern as AnchorPower)
        if (aResult.uid() == "getOTAVersion") {
            String version = aResult.to<RealtimeDatabaseResult>().to<String>();
            if (version.length() > 0 && version != "null") {
                if (xSemaphoreTake(otaMutex, pdMS_TO_TICKS(100))) {
                    strncpy(otaInfo.version, version.c_str(), sizeof(otaInfo.version) - 1);
                    otaInfo.version[sizeof(otaInfo.version) - 1] = '\0';
                    Serial.printf("OTA Version from Firebase: %s\n", otaInfo.version);
                    xSemaphoreGive(otaMutex);
                }
            } else {
                Serial.println("OTA Version: empty or null (no update available)");
            }
            // Reset state to IDLE after receiving response
            if (xSemaphoreTake(otaMutex, pdMS_TO_TICKS(100))) {
                if (otaInfo.state == OTA_CHECKING) {
                    otaInfo.state = OTA_IDLE;
                    Serial.println("OTA check complete - resetting to IDLE");
                }
                xSemaphoreGive(otaMutex);
            }
            return;
        }
        
        if (aResult.uid() == "getOTAURL") {
            String url = aResult.to<RealtimeDatabaseResult>().to<String>();
            if (url.length() > 0 && url != "null") {
                if (xSemaphoreTake(otaMutex, pdMS_TO_TICKS(100))) {
                    strncpy(otaInfo.firmwareUrl, url.c_str(), sizeof(otaInfo.firmwareUrl) - 1);
                    otaInfo.firmwareUrl[sizeof(otaInfo.firmwareUrl) - 1] = '\0';
                    Serial.printf("OTA Firmware URL from Firebase: %s\n", otaInfo.firmwareUrl);
                    xSemaphoreGive(otaMutex);
                }
            } else {
                Serial.println("OTA URL: empty or null (no update available)");
            }
            return;
        }
        
        if (aResult.uid() == "getOTAForce") {
            bool force = aResult.to<RealtimeDatabaseResult>().to<bool>();
            if (xSemaphoreTake(otaMutex, pdMS_TO_TICKS(100))) {
                otaInfo.forceUpdate = force;
                Serial.printf("OTA Force Update from Firebase: %s\n", force ? "Yes" : "No");
                xSemaphoreGive(otaMutex);
            }
            return;
        }
        
    if (aResult.uid() == "getSettings") {
        // Use optimized buffer size
        StaticJsonDocument<512> doc; // Further reduced to save memory
        
        // Get and debug raw data
        String jsonStr = aResult.to<RealtimeDatabaseResult>().to<String>();
        
        Serial.printf("Firebase data: %d bytes\n", jsonStr.length());

        // Parse JSON
        DeserializationError error = deserializeJson(doc, jsonStr);
        if (error) {
            Serial.print("JSON parse failed: ");
            Serial.println(error.c_str());
            return;
        }

        // Handle dose settings with split count
        for (int doseNum = 1; doseNum <= 3; doseNum++) {
            char doseKey[8];
            sprintf(doseKey, "dose%d", doseNum);
            if (doc.containsKey(doseKey)) {
                Serial.printf("Found dose%d settings\n", doseNum);
                JsonObject doseObj = doc[doseKey];
                DoseSettings* dose = doseNum == 1 ? &dose1 : (doseNum == 2 ? &dose2 : &dose3);
                
                dose->amount = doseObj["Status"] | 0;
                strcpy(dose->days, doseObj["days"] | "0000000");
                strcpy(dose->time, doseObj["time"] | (doseNum == 1 ? "08:00" : (doseNum == 2 ? "09:00" : "10:00")));
                
                // Only set split_count if explicitly provided in Firebase
                if (doseObj.containsKey("split_count")) {
                    dose->splitCount = doseObj["split_count"];
                    // Validate split count - ensure it's at least 1 if explicitly set
                    if (dose->splitCount <= 0) {
                        Serial.printf("Dose %d split_count is %d - setting to 1 (single dose)\n", doseNum, dose->splitCount);
                        dose->splitCount = 1;
                    }
                    Serial.printf("Dose %d split_count explicitly set to %d\n", doseNum, dose->splitCount);
                } else {
                    // No split_count in Firebase - default to single dose
                    dose->splitCount = 1;
                    Serial.printf("Dose %d split_count not set - using single dose (1)\n", doseNum);
                }
                
                // Add volume parsing
                if (doseObj.containsKey("volume")) {
                    int* volume = doseNum == 1 ? &containerVolumes.dose1 : 
                                  (doseNum == 2 ? &containerVolumes.dose2 : &containerVolumes.dose3);
                    int newVolume = doseObj["volume"] | 1000;
                    *volume = newVolume;
                    
                    // Update container status based on remaining volume
                    if (doseNum == 1) {
                        containerStatus.dose1Empty = (containerVolumes.dose1 - dosedAmounts.dose1) <= 0;
                    } else if (doseNum == 2) {
                        containerStatus.dose2Empty = (containerVolumes.dose2 - dosedAmounts.dose2) <= 0;
                    } else if (doseNum == 3) {
                        containerStatus.dose3Empty = (containerVolumes.dose3 - dosedAmounts.dose3) <= 0;
                    }
                    
                    firebaseNeedsUpdate = true;
                }
                
                // Add calibration parsing - read from dose object (legacy support)
                if (doseObj.containsKey("calibration")) {
                    float* calibration = doseNum == 1 ? &pump1Calibration : 
                                       (doseNum == 2 ? &pump2Calibration : &pump3Calibration);
                    float newCalibration = doseObj["calibration"] | 1.0f;
                    if (newCalibration != *calibration) {
                        *calibration = newCalibration;
                        Serial.printf("Dose %d calibration updated from legacy path: %.3f\n", doseNum, *calibration);
                        firebaseNeedsUpdate = true;
                    }
                }
            }
        }

        // Handle motor calibration settings (new UI path)
        if (doc.containsKey("motor_calibration")) {
            Serial.println("Found motor_calibration settings");
            JsonObject motorCalObj = doc["motor_calibration"];
            
            // Read calibration factors from motor_calibration paths
            if (motorCalObj.containsKey("motor1")) {
                float newCalibration = motorCalObj["motor1"] | 1.0f;
                if (newCalibration != pump1Calibration) {
                    pump1Calibration = newCalibration;
                    Serial.printf("Pump 1 calibration updated from Firebase: %.3f\n", pump1Calibration);
                    firebaseNeedsUpdate = true;
                }
            }
            if (motorCalObj.containsKey("motor2")) {
                float newCalibration = motorCalObj["motor2"] | 1.0f;
                if (newCalibration != pump2Calibration) {
                    pump2Calibration = newCalibration;
                    Serial.printf("Pump 2 calibration updated from Firebase: %.3f\n", pump2Calibration);
                    firebaseNeedsUpdate = true;
                }
            }
            if (motorCalObj.containsKey("motor3")) {
                float newCalibration = motorCalObj["motor3"] | 1.0f;
                if (newCalibration != pump3Calibration) {
                    pump3Calibration = newCalibration;
                    Serial.printf("Pump 3 calibration updated from Firebase: %.3f\n", pump3Calibration);
                    firebaseNeedsUpdate = true;
                }
            }
            if (motorCalObj.containsKey("ato")) {
                float newCalibration = motorCalObj["ato"] | 1.0f;
                if (newCalibration != atoCalibration) {
                    atoCalibration = newCalibration;
                    Serial.printf("ATO pump calibration updated from Firebase: %.3f\n", atoCalibration);
                    firebaseNeedsUpdate = true;
                }
            }
        }

       if (doc.containsKey("container_volumes")) {
            JsonObject volumes = doc["container_volumes"];

            if (xSemaphoreTake(doseMutex, pdMS_TO_TICKS(100))) {
                // Update only if values exist
                if (volumes.containsKey("dose1")) {
                    int newVolume = volumes["dose1"];
                    containerVolumes.dose1 = newVolume;
                    containerStatus.dose1Empty = (containerVolumes.dose1 - dosedAmounts.dose1) <= 0;
                }
                if (volumes.containsKey("dose2")) {
                    int newVolume = volumes["dose2"];
                    containerVolumes.dose2 = newVolume;
                    containerStatus.dose2Empty = (containerVolumes.dose2 - dosedAmounts.dose2) <= 0;
                }
                if (volumes.containsKey("dose3")) {
                    int newVolume = volumes["dose3"];
                    containerVolumes.dose3 = newVolume;
                    containerStatus.dose3Empty = (containerVolumes.dose3 - dosedAmounts.dose3) <= 0;
                }
                if (volumes.containsKey("ato")) {
                    float newVolume = volumes["ato"];
                    containerVolumes.ato = newVolume;
                    containerStatus.atoEmpty = (containerVolumes.ato <= 0.0f) ||
                        ((containerVolumes.ato - dosedAmounts.ato) <= 0.0f);
                }
                
                xSemaphoreGive(doseMutex);
            }
        }

        if (doc.containsKey("dosed_amounts")) {
            JsonObject da = doc["dosed_amounts"];
            if (da.containsKey("ato")) {
                float atoVal = da["ato"] | 0.0f;
                Serial.printf("getSettings: dosed_amounts/ato = %.6f\n", atoVal);
                if (atoVal <= 0.001f) {
                    bool skipReset = false;
                    if (xSemaphoreTake(doseMutex, pdMS_TO_TICKS(100))) {
                        if (pendingAtoIncrementGal > 0.0f || atoSyncRequestInFlight) {
                            Serial.println("getSettings: ignoring ato=0, pending sync in progress");
                            skipReset = true;
                        }
                        if (!skipReset && dosedAmounts.ato <= 0.001f) {
                            Serial.println("getSettings: ato already 0, no reset needed");
                            skipReset = true;
                        }
                        if (!skipReset) {
                            dosedAmounts.ato = 0.0f;
                            containerStatus.atoEmpty = false;
                            atoEmptyNotified = false;
                            floatState = readEffectiveFloatPin();
                            atoRefillRecheckPending = true;
                            floatChanged = true;
                            int estopNow = readEffectiveEstopPin();
                            estopState = estopNow;
                            atoSyncRequestInFlight = false;
                            pendingAtoIncrementGal = 0.0f;
                            Serial.println("ATO refill detected from getSettings dosed_amounts/ato=0 - resuming ATO control");
                            Serial.printf("ATO reset status: float=%s, estop=%s, atoEmpty=%s\n",
                                        floatState == HIGH ? "HIGH" : "LOW",
                                        estopNow == HIGH ? "OK" : "STOP",
                                        containerStatus.atoEmpty ? "true" : "false");
                        }
                        xSemaphoreGive(doseMutex);
                    }
                    if (!skipReset) {
                        updateContainerStatus();
                        firebaseNeedsUpdate = true;
                    }
                }
            }
        }

        if (doc.containsKey("name")) {
            const char* fetchedName = doc["name"] | "";
            if (fetchedName[0] != '\0') {
                strncpy(anchorName, fetchedName, sizeof(anchorName)-1);
                anchorName[sizeof(anchorName)-1] = '\0';
            }
        }
        
        // Debug: Print all loaded calibration values
        Serial.printf("=== CALIBRATION VALUES LOADED FROM FIREBASE ===\n");
        Serial.printf("Pump 1: %.3f, Pump 2: %.3f, Pump 3: %.3f\n", 
                    pump1Calibration, pump2Calibration, pump3Calibration);
        Serial.printf("Water Change IN: %.3f, Water Change OUT: %.3f\n", 
                    waterChangeInCalibration, waterChangeOutCalibration);
        Serial.printf("===============================================\n");
        // Handle water change settings
        if (doc.containsKey("water_change")) {
            Serial.println("Found water_change settings");
            JsonObject wcObj = doc["water_change"];
            
            // Try both "Status" and "status" to handle case sensitivity
            if (wcObj.containsKey("Status")) {
                waterChangeVolume = max(0.0f, wcObj["Status"] | 0.0f);
            } else if (wcObj.containsKey("status")) {
                waterChangeVolume = max(0.0f, wcObj["status"] | 0.0f);
            } else {
                waterChangeVolume = 0.0f; // Default value
            }
            
            // Calculate water change duration using calibrated time (average of IN and OUT calibration)
            float avgCalibration = (waterChangeInCalibration + waterChangeOutCalibration) / 2.0f;
            waterChangeDurationMs = (waterChangeVolume * ML_PER_GALLON * MS_PER_MINUTE) / (FLOW_RATE_ML_PER_MIN * avgCalibration);
            strcpy(waterChangeDays, wcObj["days"] | "1000000");
            strcpy(waterChangeTime, wcObj["time"] | "12:00");
            
            // Add water volume parsing
            if (wcObj.containsKey("volume")) {
                float newVolume = wcObj["volume"] | 5.0;
                containerVolumes.water_in = newVolume;
                containerVolumes.water_out = newVolume;
                firebaseNeedsUpdate = true;
            }
            
            // Add water change calibration parsing
            if (wcObj.containsKey("calibration_in")) {
                float newCalibrationIn = wcObj["calibration_in"] | 1.0f;
                if (newCalibrationIn != waterChangeInCalibration) {
                    waterChangeInCalibration = newCalibrationIn;
                    firebaseNeedsUpdate = true;
                }
            }
            if (wcObj.containsKey("calibration_out")) {
                float newCalibrationOut = wcObj["calibration_out"] | 1.0f;
                if (newCalibrationOut != waterChangeOutCalibration) {
                    waterChangeOutCalibration = newCalibrationOut;
                    firebaseNeedsUpdate = true;
                }
            }
        }

        if (doc.containsKey("manual_control")) {
        Serial.println("Found manual_control settings");
        JsonObject manual = doc["manual_control"];
        
        // Water controls - always update local flags to match Firebase values
        if (manual.containsKey("water_in")) {
            bool waterInRequested = manual["water_in"];
            if (waterInRequested != manualWater.waterIn) {
                manualWater.waterIn = waterInRequested;
                manualWater.completed = false;
                // If setting to false, mark as manually reset
                if (!waterInRequested) {
                    manualWater.manuallyReset = true;
                }
                Serial.printf("Manual water_in flag updated: %s\n", waterInRequested ? "true" : "false");
            }
        }
        
        if (manual.containsKey("water_out")) {
            bool waterOutRequested = manual["water_out"];
            if (waterOutRequested != manualWater.waterOut) {
                manualWater.waterOut = waterOutRequested;
                manualWater.completed = false;
                // If setting to false, mark as manually reset
                if (!waterOutRequested) {
                    manualWater.manuallyReset = true;
                }
                Serial.printf("Manual water_out flag updated: %s\n", waterOutRequested ? "true" : "false");
            }
        }
        
        // Stop water change control
        if (manual.containsKey("stop_water_change")) {
            bool stopRequested = manual["stop_water_change"];
            if (stopRequested != manualWater.stopWaterChange) {
                manualWater.stopWaterChange = stopRequested;
                Serial.printf("Manual stop_water_change flag updated: %s\n", stopRequested ? "true" : "false");
            }
        }
        
        // Manual dose controls - thread-safe
        if (xSemaphoreTake(doseMutex, pdMS_TO_TICKS(10))) {
            for (int i = 1; i <= 3; i++) {
                char doseKey[8];
                sprintf(doseKey, "dose%d", i);
                if (manual.containsKey(doseKey)) {
                    bool newState = manual[doseKey];
                    if (newState != manualDose.requested[i-1]) {
                        manualDose.requested[i-1] = newState;
                    }
                }
            }
            xSemaphoreGive(doseMutex);
        }
        
        
        if (manual.containsKey("status")) {
            const char* status = manual["status"];
            if (strcmp(status, "stopped") == 0) {
                Serial.println("Stop command received - stopping water change");
                if (xSemaphoreTake(waterChange.mutex, pdMS_TO_TICKS(100))) {
                    if (waterChange.active) {
                        strcpy(waterChange.status, "stopping");
                        firebaseNeedsUpdate = true;
                        Serial.println("Water change status set to 'stopping'");
                    }
                    xSemaphoreGive(waterChange.mutex);
                }
                // Only clear manual water flags when stop command is received AND manually reset
                manualWater.waterIn = false;
                manualWater.waterOut = false;
                manualWater.manuallyReset = true; // Mark as manually reset
                manualWater.stopWaterChange = false; // Reset stop water change flag
                Serial.println("Manual water flags cleared due to stop command (manually reset)");
            }
        }
        
          

        // Pump priming controls - only set if true (Firebase should clear it)
         for (int i = 1; i <= 3; i++) {
            char primeKey[16];
            sprintf(primeKey, "prime_pump%d", i);
            if (manual.containsKey(primeKey)) {
                bool primeRequested = manual[primeKey];
                if (xSemaphoreTake(doseMutex, pdMS_TO_TICKS(10))) {
                    if (primeRequested && !manualDoseFlags[i-1]) {
                        manualDoseFlags[i-1] = true;
                        // Update the correct individual field
                        switch(i) {
                            case 1: previousValues.primePump1 = true; break;
                            case 2: previousValues.primePump2 = true; break;
                            case 3: previousValues.primePump3 = true; break;
                        }
                    }
                    xSemaphoreGive(doseMutex);
                }
            }
        }
    

        
        // Calibration test commands
        for (int i = 1; i <= 3; i++) {
            char testKey[32];
            sprintf(testKey, "calibration_test%d", i);
            if (manual.containsKey(testKey)) {
                bool testRequested = manual[testKey];
                if (testRequested) {
                    // Run a 10ml calibration test
                    testPumpCalibration(i, 10);
                    // Clear the test flag
                    char tempPath[128];
                    strcpy(tempPath, MANUAL_CONTROL_PATH);
                    strcat(tempPath, "/");
                    strcat(tempPath, testKey);
                    Database.set(aClient, tempPath, false, asyncCB);
                }
            }
        }
        
        // Container refill commands
        for (int i = 1; i <= 3; i++) {
            char refillKey[32];
            sprintf(refillKey, "refill_container%d", i);
            if (manual.containsKey(refillKey)) {
                bool refillRequested = manual[refillKey];
                if (refillRequested) {
                    // Reset dosed amount for this container
                    resetContainerDosedAmounts(i);
                    // Clear the refill flag
                    char tempPath[128];
                    strcpy(tempPath, MANUAL_CONTROL_PATH);
                    strcat(tempPath, "/");
                    strcat(tempPath, refillKey);
                    Database.set(aClient, tempPath, false, asyncCB);
                }
            }
        }
        
        // Water container refill commands
        if (manual.containsKey("refill_water_in")) {
            bool refillRequested = manual["refill_water_in"];
            if (refillRequested) {
                resetContainerDosedAmounts(4); // Water IN container
                char tempPath[128];
                strcpy(tempPath, MANUAL_CONTROL_PATH);
                strcat(tempPath, "/refill_water_in");
                Database.set(aClient, tempPath, false, asyncCB);
            }
        }
        
        // Manual water change flag reset command
        if (manual.containsKey("reset_manual_water_flags")) {
            bool resetRequested = manual["reset_manual_water_flags"];
            if (resetRequested) {
                resetManualWaterChangeFlags();
                char tempPath[128];
                strcpy(tempPath, MANUAL_CONTROL_PATH);
                strcat(tempPath, "/reset_manual_water_flags");
                Database.set(aClient, tempPath, false, asyncCB);
            }
        }
        
        if (manual.containsKey("refill_water_out")) {
            bool refillRequested = manual["refill_water_out"];
            if (refillRequested) {
                resetContainerDosedAmounts(5); // Water OUT container
                char tempPath[128];
                strcpy(tempPath, MANUAL_CONTROL_PATH);
                strcat(tempPath, "/refill_water_out");
                Database.set(aClient, tempPath, false, asyncCB);
            }
        }
        
        // Water change calibration test commands
        if (manual.containsKey("calibration_test_water_in")) {
            bool testRequested = manual["calibration_test_water_in"];
            if (testRequested) {
                // Run a 0.5 gallon calibration test for water IN
                testWaterChangeCalibration(true, 0.5);
                // Clear the test flag
                char tempPath[128];
                strcpy(tempPath, MANUAL_CONTROL_PATH);
                strcat(tempPath, "/calibration_test_water_in");
                Database.set(aClient, tempPath, false, asyncCB);
            }
        }
        if (manual.containsKey("calibration_test_water_out")) {
            bool testRequested = manual["calibration_test_water_out"];
            if (testRequested) {
                // Run a 0.5 gallon calibration test for water OUT
                testWaterChangeCalibration(false, 0.5);
                // Clear the test flag
                char tempPath[128];
                strcpy(tempPath, MANUAL_CONTROL_PATH);
                strcat(tempPath, "/calibration_test_water_out");
                Database.set(aClient, tempPath, false, asyncCB);
            }
        }
    
        

        // Handle split dose configuration
        if (doc.containsKey("split_dosing")) {
            JsonObject splitObj = doc["split_dosing"];
            int splitCount = splitObj["count"] | 4;
            // This is global setting - if you want per-dose splits, use the dose-specific settings above
        }



        // Print current settings after processing Firebase data
        Serial.println("Firebase settings processed - printing current state:");
        printCurrentSettings(jsonStr.c_str());
    }}
    

    

    
    // Handle dose time retrievals
    if (aResult.available() && aResult.uid() == "getDose1Time") {
        String doseTime = aResult.to<RealtimeDatabaseResult>().to<String>();
        if (doseTime.length() > 0 && doseTime != "Never") {
            strcpy(lastDose1Time, doseTime.c_str());
        }
    }
    
    if (aResult.available() && aResult.uid() == "getDose2Time") {
        String doseTime = aResult.to<RealtimeDatabaseResult>().to<String>();
        if (doseTime.length() > 0 && doseTime != "Never") {
            strcpy(lastDose2Time, doseTime.c_str());
        }
    }
    
    if (aResult.available() && aResult.uid() == "getDose3Time") {
        String doseTime = aResult.to<RealtimeDatabaseResult>().to<String>();
        if (doseTime.length() > 0 && doseTime != "Never") {
            strcpy(lastDose3Time, doseTime.c_str());
        }
    }
    // Deprecated: name is now read from settings (BASE_PATH/settings.name)
}}

// Helper function to build Firebase paths
void buildPath(char* dest, const char* base, const char* suffix) {
    strcpy(dest, base);
    strcat(dest, suffix);
}

void batchWriteToFirebase() {
    StaticJsonDocument<384> jsonDoc; // Further reduced to save memory
    bool hasChanges = false;

    // Always add last seen timestamp when updating Firebase
    char currentTime[20];
    struct tm timeinfo;
    if (getLocalTime(&timeinfo)) {
        strftime(currentTime, sizeof(currentTime), "%Y-%m-%d %H:%M:%S", &timeinfo);
    } else {
        strcpy(currentTime, "Time Error");
    }
    
    char tempPath[128]; // Increased to 128 bytes to prevent overflow

    // Always publish firmware version on every batch (same path pattern as AnchorPower)
    if (strlen(BASE_PATH) > 0) {
        buildPath(tempPath, BASE_PATH, "/firmware_version");
        jsonDoc[tempPath] = String(FIRMWARE_VERSION);
        hasChanges = true;
    }

    // System status updates - only add if changed
    if (temperatureF != lastReportedTemp) {
        buildPath(tempPath, SYSTEM_STATUS_PATH, "/temperature");
        jsonDoc[tempPath] = temperatureF;
        Serial.printf("Firebase: Writing temperature %.1f°F to Firebase (previous: %.1f°F)\n", 
                    temperatureF, lastReportedTemp);
        lastReportedTemp = temperatureF;
        hasChanges = true;
    }
    
    if (strcmp(pumpState, previousValues.pumpState) != 0) {
        buildPath(tempPath, SYSTEM_STATUS_PATH, "/pump_state");
        jsonDoc[tempPath] = pumpState;
        strcpy(previousValues.pumpState, pumpState);
        hasChanges = true;
    }
    
    if (strcmp(estopStatus, previousValues.estopStatus) != 0) {
        buildPath(tempPath, SYSTEM_STATUS_PATH, "/estop_status");
        jsonDoc[tempPath] = estopStatus;
        strcpy(previousValues.estopStatus, estopStatus);
        hasChanges = true;
    }
    
    if (containerStatus.atoEmpty != previousValues.atoEmpty) {
        buildPath(tempPath, SYSTEM_STATUS_PATH, "/atoempty");
        jsonDoc[tempPath] = containerStatus.atoEmpty;
        previousValues.atoEmpty = containerStatus.atoEmpty;
        hasChanges = true;
    }
    
    if (strcmp(lastDose1Time, previousValues.lastDose1Time) != 0) {
        buildPath(tempPath, SYSTEM_STATUS_PATH, "/last_dose1_time");
        jsonDoc[tempPath] = lastDose1Time;
        strcpy(previousValues.lastDose1Time, lastDose1Time);
        hasChanges = true;
    }
    
    if (strcmp(lastDose2Time, previousValues.lastDose2Time) != 0) {
        buildPath(tempPath, SYSTEM_STATUS_PATH, "/last_dose2_time");
        jsonDoc[tempPath] = lastDose2Time;
        strcpy(previousValues.lastDose2Time, lastDose2Time);
        hasChanges = true;
    }
    
    if (strcmp(lastDose3Time, previousValues.lastDose3Time) != 0) {
        buildPath(tempPath, SYSTEM_STATUS_PATH, "/last_dose3_time");
        jsonDoc[tempPath] = lastDose3Time;
        strcpy(previousValues.lastDose3Time, lastDose3Time);
        hasChanges = true;
    }
    
    if (strcmp(lastWaterChangeTime, previousValues.lastWaterChangeTime) != 0) {
        buildPath(tempPath, SYSTEM_STATUS_PATH, "/last_water_change_time");
        jsonDoc[tempPath] = lastWaterChangeTime;
        strcpy(previousValues.lastWaterChangeTime, lastWaterChangeTime);
        hasChanges = true;
    }

    if (containerVolumes.water_in != previousValues.waterInVolume) {
        buildPath(tempPath, SETTINGS_PATH, "/container_volumes/water_in");
        jsonDoc[tempPath] = containerVolumes.water_in;
        previousValues.waterInVolume = containerVolumes.water_in;
    }
  
    if (containerVolumes.water_out != previousValues.waterOutVolume) {
        buildPath(tempPath, SETTINGS_PATH, "/container_volumes/water_out");
        jsonDoc[tempPath] = containerVolumes.water_out;
        previousValues.waterOutVolume = containerVolumes.water_out;
    }
    
    if (containerVolumes.ato != previousValues.atoVolume) {
        buildPath(tempPath, SETTINGS_PATH, "/container_volumes/ato");
        jsonDoc[tempPath] = containerVolumes.ato;
        previousValues.atoVolume = containerVolumes.ato;
        hasChanges = true;
    }
  
    // Update water dosed amounts (both during and after water changes)
    if (dosedAmounts.water_in != previousValues.waterInDosed) {
        float oldValue = previousValues.waterInDosed;
        buildPath(tempPath, SETTINGS_PATH, "/dosed_amounts/water_in");
        jsonDoc[tempPath] = dosedAmounts.water_in;
        previousValues.waterInDosed = dosedAmounts.water_in;
        hasChanges = true;
        Serial.printf("Firebase: Updating water_in dosed amount to %.2f gallons (previous: %.2f)\n", 
                    dosedAmounts.water_in, oldValue);
    }
  
    if (dosedAmounts.water_out != previousValues.waterOutDosed) {
        float oldValue = previousValues.waterOutDosed;
        buildPath(tempPath, SETTINGS_PATH, "/dosed_amounts/water_out");
        jsonDoc[tempPath] = dosedAmounts.water_out;
        previousValues.waterOutDosed = dosedAmounts.water_out;
        hasChanges = true;
        Serial.printf("Firebase: Updating water_out dosed amount to %.2f gallons (previous: %.2f)\n", 
                    dosedAmounts.water_out, oldValue);
    }
    

    
    if (xSemaphoreTake(doseMutex, pdMS_TO_TICKS(10))) {
    if (dosedAmounts.dose1 != previousValues.dose1) {
        buildPath(tempPath, DOSE1_PATH, "/volume");
        jsonDoc[tempPath] = dosedAmounts.dose1;
        previousValues.dose1 = dosedAmounts.dose1;
        hasChanges = true;
    }
    if (dosedAmounts.dose2 != previousValues.dose2) {
        buildPath(tempPath, DOSE2_PATH, "/volume");
        jsonDoc[tempPath] = dosedAmounts.dose2;
        previousValues.dose2 = dosedAmounts.dose2;
        hasChanges = true;
    }
    if (dosedAmounts.dose3 != previousValues.dose3) {
        buildPath(tempPath, DOSE3_PATH, "/volume");
        jsonDoc[tempPath] = dosedAmounts.dose3;
        previousValues.dose3 = dosedAmounts.dose3;
        hasChanges = true;
    }
    if (dosedAmounts.ato != previousValues.ato) {
        buildPath(tempPath, SETTINGS_PATH, "/dosed_amounts/ato");
        jsonDoc[tempPath] = dosedAmounts.ato;
        float oldAto = previousValues.ato;
        previousValues.ato = dosedAmounts.ato;
        hasChanges = true;
        Serial.printf("Firebase: Updating ATO dosed amount to %.3f gal (previous: %.3f)\n", 
                    dosedAmounts.ato, oldAto);
    }
    
    // Update pump calibration factors (only if they've changed from previous values)
    static bool calibrationInitialized = false;
    
    // Only write calibration values if they've actually changed from previous values
    // This prevents overwriting Firebase values with defaults on startup
    if (pump1Calibration != previousValues.pump1Calibration) {
        buildPath(tempPath, DOSE1_PATH, "/calibration");
        jsonDoc[tempPath] = pump1Calibration;
        previousValues.pump1Calibration = pump1Calibration;
        hasChanges = true;
    }
    if (pump2Calibration != previousValues.pump2Calibration) {
        buildPath(tempPath, DOSE2_PATH, "/calibration");
        jsonDoc[tempPath] = pump2Calibration;
        previousValues.pump2Calibration = pump2Calibration;
        hasChanges = true;
    }
    if (pump3Calibration != previousValues.pump3Calibration) {
        buildPath(tempPath, DOSE3_PATH, "/calibration");
        jsonDoc[tempPath] = pump3Calibration;
        previousValues.pump3Calibration = pump3Calibration;
        hasChanges = true;
    }
    if (atoCalibration != previousValues.atoCalibration) {
        buildPath(tempPath, SETTINGS_PATH, "/motor_calibration/ato");
        jsonDoc[tempPath] = atoCalibration;
        previousValues.atoCalibration = atoCalibration;
        hasChanges = true;
    }
    
    // Write to new motor_calibration paths for UI compatibility (only if changed)
    if (pump1Calibration != previousValues.pump1Calibration) {
        buildPath(tempPath, SETTINGS_PATH, "/motor_calibration/motor1");
        jsonDoc[tempPath] = pump1Calibration;
        hasChanges = true;
    }
    if (pump2Calibration != previousValues.pump2Calibration) {
        buildPath(tempPath, SETTINGS_PATH, "/motor_calibration/motor2");
        jsonDoc[tempPath] = pump2Calibration;
        hasChanges = true;
    }
    if (pump3Calibration != previousValues.pump3Calibration) {
        buildPath(tempPath, SETTINGS_PATH, "/motor_calibration/motor3");
        jsonDoc[tempPath] = pump3Calibration;
        hasChanges = true;
    }
    if (atoCalibration != previousValues.atoCalibration) {
        buildPath(tempPath, SETTINGS_PATH, "/motor_calibration/ato");
        jsonDoc[tempPath] = atoCalibration;
        hasChanges = true;
    }
    
    if (!calibrationInitialized) {
        calibrationInitialized = true;
        Serial.println("Calibration factors initialized - only reading from Firebase, not writing defaults");
        Serial.printf("Current values - Pump 1: %.3f, Pump 2: %.3f, Pump 3: %.3f, ATO: %.3f\n", 
                    pump1Calibration, pump2Calibration, pump3Calibration, atoCalibration);
    }
    
    // Update water change calibration factors (only if changed)
    if (waterChangeInCalibration != previousValues.waterChangeInCalibration) {
        buildPath(tempPath, WATER_CHANGE_PATH, "/calibration_in");
        jsonDoc[tempPath] = waterChangeInCalibration;
        previousValues.waterChangeInCalibration = waterChangeInCalibration;
        hasChanges = true;
    }
    if (waterChangeOutCalibration != previousValues.waterChangeOutCalibration) {
        buildPath(tempPath, WATER_CHANGE_PATH, "/calibration_out");
        jsonDoc[tempPath] = waterChangeOutCalibration;
        previousValues.waterChangeOutCalibration = waterChangeOutCalibration;
        hasChanges = true;
    }
    if (!calibrationInitialized) {
        Serial.printf("Current values - Water Change IN: %.3f, Water Change OUT: %.3f\n", 
                    waterChangeInCalibration, waterChangeOutCalibration);
    }
    xSemaphoreGive(doseMutex);}

 if (!isWaterChangeActive && !isManualWaterChange && 
        (strcmp(previousValues.waterChangeStatus, "running") == 0 || strcmp(previousValues.waterChangeStatus, "starting") == 0)) {
        buildPath(tempPath, MANUAL_CONTROL_PATH, "/status");
        jsonDoc[tempPath] = "stopped";
        strcpy(previousValues.waterChangeStatus, "stopped");
        hasChanges = true;
    }




    if (containerVolumes.water_in != previousValues.waterInVolume) {
        buildPath(tempPath, SYSTEM_STATUS_PATH, "/container_volumes/water_in");
        jsonDoc[tempPath] = containerVolumes.water_in;
        previousValues.waterInVolume = containerVolumes.water_in;
        hasChanges = true;
    }
    if (containerVolumes.water_out != previousValues.waterOutVolume) {
        buildPath(tempPath, SYSTEM_STATUS_PATH, "/container_volumes/water_out");
        jsonDoc[tempPath] = containerVolumes.water_out;
        previousValues.waterOutVolume = containerVolumes.water_out;
        hasChanges = true;
    }

    // Water change remaining time - use the actual remaining time from waterChange struct
    long currentRemaining = 0;
    if (waterChange.active) {
        currentRemaining = (waterChange.remainingMs + 59999) / 60000; // Round up to nearest minute
    }
    // Always update if water change is not active and previous value was not 0
    if (currentRemaining != previousValues.waterChangeRemaining || 
        (!waterChange.active && previousValues.waterChangeRemaining != 0)) {
        buildPath(tempPath, SYSTEM_STATUS_PATH, "/water_change_remaining");
        jsonDoc[tempPath] = currentRemaining;
        previousValues.waterChangeRemaining = currentRemaining;
        hasChanges = true;
        if (!waterChange.active && currentRemaining == 0) {
            Serial.println("Resetting water change remaining time to 0");
        }
    }
    


        if (xSemaphoreTake(doseMutex, pdMS_TO_TICKS(10))) {
        // Check each pump individually
        if (!manualDoseFlags[0] && previousValues.primePump1) {
            buildPath(tempPath, MANUAL_CONTROL_PATH, "/prime_pump1");
            jsonDoc[tempPath] = false;
            previousValues.primePump1 = false;
            hasChanges = true;
        }
        if (!manualDoseFlags[1] && previousValues.primePump2) {
            buildPath(tempPath, MANUAL_CONTROL_PATH, "/prime_pump2");
            jsonDoc[tempPath] = false;
            previousValues.primePump2 = false;
            hasChanges = true;
        }
        if (!manualDoseFlags[2] && previousValues.primePump3) {
            buildPath(tempPath, MANUAL_CONTROL_PATH, "/prime_pump3");
            jsonDoc[tempPath] = false;
            previousValues.primePump3 = false;
            hasChanges = true;
        }
        xSemaphoreGive(doseMutex);
    }


    // Manual operation completions
    if (xSemaphoreTake(doseMutex, pdMS_TO_TICKS(100))) {
        // Water change status update
        if (strcmp(previousValues.waterChangeStatus, "completed") == 0) {
            buildPath(tempPath, MANUAL_CONTROL_PATH, "/status");
            jsonDoc[tempPath] = "ready";
            buildPath(tempPath, MANUAL_CONTROL_PATH, "/water_in");
            jsonDoc[tempPath] = false;
            buildPath(tempPath, MANUAL_CONTROL_PATH, "/water_out");
            jsonDoc[tempPath] = false;
            // Also reset time remaining when water change completes
            buildPath(tempPath, SYSTEM_STATUS_PATH, "/water_change_remaining");
            jsonDoc[tempPath] = 0;
            buildPath(tempPath, SYSTEM_STATUS_PATH, "/water_change_estimated_completion");
            jsonDoc[tempPath] = 0;
            buildPath(tempPath, SYSTEM_STATUS_PATH, "/water_change_completion");
            jsonDoc[tempPath] = "Not active";
            previousValues.waterChangeRemaining = 0;
            strcpy(previousValues.waterChangeStatus, "ready");  // Reset to ready state
            hasChanges = true;
        }

            

        // Handle manual dose completions
        if (manualDose.completed[0]) {
            buildPath(tempPath, MANUAL_CONTROL_PATH, "/dose1");
            jsonDoc[tempPath] = false;
            manualDose.completed[0] = false;
            hasChanges = true;
        }
        if (manualDose.completed[1]) {
            buildPath(tempPath, MANUAL_CONTROL_PATH, "/dose2");
            jsonDoc[tempPath] = false;
            manualDose.completed[1] = false;
            hasChanges = true;
        }
        if (manualDose.completed[2]) {
            buildPath(tempPath, MANUAL_CONTROL_PATH, "/dose3");
            jsonDoc[tempPath] = false;
            manualDose.completed[2] = false;
            hasChanges = true;
        }
        
        xSemaphoreGive(doseMutex);
    }
     if (xSemaphoreTake(waterChange.mutex, pdMS_TO_TICKS(100))) {
        if (waterChange.needsRemainingUpdate && waterChange.active) {
            long remainingMinutes = (waterChange.remainingMs + 59999) / 60000; // Round up to nearest minute
            buildPath(tempPath, SYSTEM_STATUS_PATH, "/water_change_remaining");
            jsonDoc[tempPath] = remainingMinutes;
            waterChange.needsRemainingUpdate = false;
            hasChanges = true;
            
        } else if (waterChange.needsRemainingUpdate && !waterChange.active) {
            // Reset remaining time when water change is not active
            buildPath(tempPath, SYSTEM_STATUS_PATH, "/water_change_remaining");
            jsonDoc[tempPath] = 0;
            waterChange.needsRemainingUpdate = false;
            hasChanges = true;
        }
        
        // Update estimated completion time
        if (waterChange.active && waterChange.estimatedCompletionTime > 0) {
            unsigned long currentMillis = millis();
            if (waterChange.estimatedCompletionTime > currentMillis) {
                unsigned long remainingMs = waterChange.estimatedCompletionTime - currentMillis;
                long remainingMinutes = (remainingMs + 59999) / 60000; // Round up to nearest minute
                buildPath(tempPath, SYSTEM_STATUS_PATH, "/water_change_estimated_completion");
                jsonDoc[tempPath] = remainingMinutes;
                
                // Also update completion status for UI compatibility
                if (waterChange.manual) {
                    buildPath(tempPath, SYSTEM_STATUS_PATH, "/water_change_completion");
                    jsonDoc[tempPath] = "Manual water change in progress";
                } else {
                                          buildPath(tempPath, SYSTEM_STATUS_PATH, "/water_change_completion");
                      jsonDoc[tempPath] = "Scheduled water change in progress";
                }
                hasChanges = true;
            }
        } else if (!waterChange.active && previousValues.waterChangeRemaining != 0) {
            // Reset estimated completion time when water change is not active
            buildPath(tempPath, SYSTEM_STATUS_PATH, "/water_change_estimated_completion");
            jsonDoc[tempPath] = 0;
            buildPath(tempPath, SYSTEM_STATUS_PATH, "/water_change_completion");
            jsonDoc[tempPath] = "Not active";
            hasChanges = true;
            Serial.println("Resetting estimated completion time to 0");
        }
        xSemaphoreGive(waterChange.mutex);
    }

    // Only send if there are changes
    if (hasChanges) {
        buildPath(tempPath, SYSTEM_STATUS_PATH, "/last_seen");
        jsonDoc[tempPath] = currentTime;

        String jsonString;
        serializeJson(jsonDoc, jsonString);

        Serial.println("Sending to Firebase");

        // Single Firebase update with all changes using the root path
        Serial.printf("Updating Firebase at path: %s\n", BASE_PATH);
        Database.update<object_t>(aClient, "/", object_t(jsonString), asyncCB);
    } else {
        Serial.println("No Firebase updates");
    }
}






void printCurrentSettings(const char* settingsJson) {
    // Only print if serial is available to prevent blocking
    if (!Serial) return;
    
    // Print header with separation
    Serial.println("\n=== Current Settings (Firebase) ===");

    if (settingsJson != nullptr && strlen(settingsJson) > 0) {
        Serial.println("Settings JSON from database:");
        int indent = 0;
        bool inString = false;
        bool escapeNext = false;
        const size_t len = strlen(settingsJson);
        for (size_t i = 0; i < len; i++) {
            char c = settingsJson[i];

            if (escapeNext) {
                Serial.print(c);
                escapeNext = false;
                continue;
            }
            if (inString && c == '\\') {
                Serial.print(c);
                escapeNext = true;
                continue;
            }
            if (c == '"') {
                inString = !inString;
                Serial.print(c);
                continue;
            }

            if (!inString) {
                if (c == '{' || c == '[') {
                    Serial.print(c);
                    Serial.println();
                    indent++;
                    for (int j = 0; j < indent; j++) Serial.print("  ");
                    continue;
                }
                if (c == '}' || c == ']') {
                    Serial.println();
                    if (indent > 0) indent--;
                    for (int j = 0; j < indent; j++) Serial.print("  ");
                    Serial.print(c);
                    continue;
                }
                if (c == ',') {
                    Serial.print(c);
                    Serial.println();
                    for (int j = 0; j < indent; j++) Serial.print("  ");
                    continue;
                }
                if (c == ':') {
                    Serial.print(": ");
                    continue;
                }
                if (c == ' ' || c == '\n' || c == '\r' || c == '\t') {
                    continue;
                }
            }

            Serial.print(c);
        }
        Serial.println();
        Serial.println("==================================\n");
        return;
    }

    // Handle time information with proper error checking
    struct tm timeinfo;
    bool timeValid = getLocalTime(&timeinfo);
    
    if (timeValid) {
        char timeStr[20];
        strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S", &timeinfo);
        Serial.printf("Time: %s\n", timeStr);
    } else {
        Serial.println("Time: [Unable to get time]");
    }

    // Print dose settings
    for (int i = 1; i <= 3; i++) {
        DoseSettings* dose = (i == 1) ? &dose1 : (i == 2) ? &dose2 : &dose3;
        Serial.printf("Dose %d: %d ml, %s, %s\n", 
                    i, dose->amount, dose->days, dose->time);
    }

    // Print water change settings
    Serial.printf("Water Change: %.1f gal, %s, %s\n", 
                waterChangeVolume, waterChangeDays, waterChangeTime);

    // Print calibration factors
    Serial.printf("Calibration - Pump1: %.3f, Pump2: %.3f, Pump3: %.3f\n", 
                pump1Calibration, pump2Calibration, pump3Calibration);
    Serial.printf("Calibration - Water IN: %.3f, Water OUT: %.3f\n", 
                waterChangeInCalibration, waterChangeOutCalibration);

    // Print system status
    Serial.printf("Status: Temp=%.1f°F, Pump=%s, E-Stop=%s\n", 
                temperatureF, pumpState, estopStatus);

    // Print dosed amounts
    Serial.printf("Dosed: D1=%d, D2=%d, D3=%d ml\n", 
                dosedAmounts.dose1, dosedAmounts.dose2, dosedAmounts.dose3);

    Serial.println("=======================\n");
}


void sendUdpMessage(const char* message) {
  udp.beginPacket(remoteIP, remotePort);
  udp.write((const uint8_t*)message, strlen(message));
  udp.endPacket();
}  



// Function to connect ESP32 to the specified Wi-Fi network
void connectToWiFi(const char* ssid, const char* password, bool fromPortal = false) {
  // If already connected to the requested SSID, do nothing
  if (WiFi.status() == WL_CONNECTED) {
    String current = WiFi.SSID();
    if (current == String(ssid)) {
      Serial.println("Already connected to target SSID; skipping reconnect");
      return;
    } else {
      Serial.println("Connected to different SSID; disconnecting to reconnect...");
      WiFi.disconnect();
      delay(250);
    }
  }

  // Ensure STA mode and stop any ongoing connection attempt
  // If called from portal, we need to switch from AP mode to STA mode
  if (fromPortal) {
    Serial.println("Switching from AP mode to STA mode for connection");
    WiFi.mode(WIFI_OFF);
    delay(500);
    WiFi.mode(WIFI_STA);
    delay(500);
  } else {
    WiFi.mode(WIFI_STA);
  }
  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);
  WiFi.disconnect(true);
  delay(500);

  // Attempt to connect to the new Wi-Fi network
  WiFi.begin(ssid, password);

  Serial.printf("Connecting to %s...\n", ssid);
  Serial.printf("Connection attempt from portal: %s\n", fromPortal ? "Yes" : "No");
  displayConnecting(ssid);

  unsigned long startAttemptTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(F("\nConnected to WiFi!"));
    Serial.print(F("ESP32 IP Address: "));
    Serial.println(WiFi.localIP());
    connectedToWifi = true;
    
    // Reset WiFi attempt counter on successful connection
    resetWiFiAttemptCounter();
    
    // Display connection success on OLED
    char ipStr[16];
    IPAddress localIP = WiFi.localIP();
    sprintf(ipStr, "%d.%d.%d.%d", localIP[0], localIP[1], localIP[2], localIP[3]);
    displayConnected(ipStr);
  } else {
    Serial.println(F("\nFailed to connect to WiFi"));
    displayStatus("WiFi Failed");
    
    // Only increment connection attempt counter if not called from portal
    if (!fromPortal) {
      wifiConnectionAttempts++;
      Serial.printf("WiFi connection attempt %d/%d failed\n", wifiConnectionAttempts, MAX_WIFI_ATTEMPTS);
      
      // Check if we've reached max attempts
      if (wifiConnectionAttempts >= MAX_WIFI_ATTEMPTS) {
        wifiMaxAttemptsReached = true;
        Serial.println("Maximum WiFi connection attempts reached. Stopping reconnection attempts.");
        Serial.println("Captive portal will remain active until WiFi is configured.");
      }
      
      // Launch access portal for new WiFi configuration
      if (!captivePortalActive) {
        Serial.println("WiFi connection failed - launching access portal for new WiFi configuration");
        startCaptivePortal();
      }
    } else {
      Serial.println("WiFi connection failed from portal - will allow retry");
    }
  }
}

// Forward declarations for web server functions
void handleRoot();
void handleNotFound();
void handleSetupAnchor();
void handleScanWiFi();

// WiFi reconnection function declarations
void checkWiFiReconnection();
void startCaptivePortal();
void stopCaptivePortal();
void saveBackupCredentials();
void tryBackupCredentials();


void reportDeviceStatus();

// Function to sanitize email for Firebase key usage
String sanitizeEmailForFirebase(String email) {
  String sanitized = email;
  sanitized.replace("@", "_at_");
  sanitized.replace(".", "_dot_");
  return sanitized;
}





// WiFi reconnection functions
void checkWiFiReconnection() {
    static unsigned long lastWiFiCheck = 0;
    const unsigned long WIFI_CHECK_INTERVAL = 1000; // check every 1s
    
    if (millis() - lastWiFiCheck < WIFI_CHECK_INTERVAL) return;
    lastWiFiCheck = millis();
    
    wl_status_t st = WiFi.status();
    if (st != WL_CONNECTED) {
        if (!wifiDisconnected) {
            wifiDisconnected = true;
            wifiDisconnectTime = millis();
            Serial.println("WiFi disconnected - starting disconnect timer");
        }
        
        // Keep trying to reconnect with stored credentials every 5 seconds
        // BUT NOT when captive portal is active OR max attempts reached - let user configure WiFi first
        static unsigned long lastRetry = 0;
        if (millis() - lastRetry > 5000 && !captivePortalActive && !wifiMaxAttemptsReached) { // retry every 5s (more frequent)
            lastRetry = millis();
            if (pref.isKey("wifiSSID") && pref.isKey("wifiPass")) {
                String s = pref.getString("wifiSSID");
                String p = pref.getString("wifiPass");
                Serial.printf("Re-attempting WiFi with stored SSID: %s\n", s.c_str());
                
                // Only attempt connection if not already trying to connect
                if (WiFi.getMode() == WIFI_STA || WiFi.getMode() == WIFI_AP_STA) {
                    WiFi.begin(s.c_str(), p.c_str());
                }
            }
        }
        
        // Launch captive portal after 5s if not already active
        if ((millis() - wifiDisconnectTime > 5000) && !captivePortalActive) {
            Serial.println("WiFi disconnected for 5 seconds - launching captive portal");
            startCaptivePortal();
        }
        
        // Also launch captive portal immediately if no stored credentials exist
        if (!pref.isKey("wifiSSID") || !pref.isKey("wifiPass")) {
            if (!captivePortalActive) {
                Serial.println("No WiFi credentials found - launching captive portal immediately");
                startCaptivePortal();
            }
        }
    } else {
        if (wifiDisconnected) {
            wifiDisconnected = false;
            if (captivePortalActive) {
                captivePortalActive = false;
                stopCaptivePortal();
                Serial.println("WiFi reconnected - stopping captive portal");
            }
        }
    }
}

void startCaptivePortal() {
    if (captivePortalActive) return; // Already active
    
    Serial.println("Starting captive portal for WiFi reconfiguration");
    captivePortalActive = true;
    
    // Save current credentials as backup
    saveBackupCredentials();
    
    // Stop any ongoing WiFi connections and clear any pending operations
    WiFi.disconnect(true);  // Force disconnect and clear stored credentials temporarily
    delay(500);  // Give more time for disconnect to complete
    
    // Switch to AP mode only for captive portal
    WiFi.mode(WIFI_AP);
    delay(200);  // Give more time for mode switch
    
    // Create access point
    bool apStarted = WiFi.softAP(AP_SSID);
    if (!apStarted) {
        Serial.println("Failed to start access point!");
        captivePortalActive = false;
        return;
    }
    
    delay(100); // Give AP time to start
    
    IPAddress myIP = WiFi.softAPIP();
    Serial.printf("Access point started successfully. IP: %s\n", myIP.toString().c_str());
    
    // Display setup information on TFT
    char ipStr[16];
    sprintf(ipStr, "%d.%d.%d.%d", myIP[0], myIP[1], myIP[2], myIP[3]);
    displayWiFiFailureSetup(AP_SSID, ipStr);
    
    // Start DNS server for captive portal
    dnsServer.start(DNS_PORT, "*", myIP);
    
    // Setup web server routes
    webServer.on("/", handleRoot);
    webServer.on("/setup-anchor", handleSetupAnchor);
    webServer.on("/scan-wifi", handleScanWiFi);
    webServer.onNotFound(handleRoot);
    webServer.begin();
    
    Serial.printf("Captive portal active at %s\n", ipStr);
    Serial.println("Connect to 'Anchor' network and visit http://192.168.4.1");
    Serial.println("Access point is now available for WiFi configuration");
    Serial.println("Portal will remain active until WiFi is configured successfully");
    
    // Additional debugging
    Serial.printf("WiFi mode: %d\n", WiFi.getMode());
    Serial.printf("AP SSID: %s\n", WiFi.softAPSSID().c_str());
    Serial.printf("AP IP: %s\n", WiFi.softAPIP().toString().c_str());
    Serial.printf("Connected stations: %d\n", WiFi.softAPgetStationNum());
}

void stopCaptivePortal() {
    if (!captivePortalActive) return; // Not active
    
    Serial.println("Stopping captive portal");
    captivePortalActive = false;
    
    // Stop web server and DNS server
    webServer.close();
    dnsServer.stop();
    
    // Disconnect AP and switch back to STA mode
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_STA);
    
    // Clear display
    TFT_display.fillScreen(BLACK);
    
    Serial.println("Captive portal stopped - switched back to STA mode");
}

void saveBackupCredentials() {
    if (strlen(ssid) > 0 && strlen(password) > 0) {
        strcpy(backupSSID, ssid);
        strcpy(backupPassword, password);
        hasBackupCredentials = true;
        Serial.printf("Saved backup credentials for SSID: %s\n", backupSSID);
    }
}

void tryBackupCredentials() {
    if (!hasBackupCredentials || wifiMaxAttemptsReached) return;
    
    Serial.printf("Trying backup credentials for SSID: %s\n", backupSSID);
    WiFi.begin(backupSSID, backupPassword);
    
    unsigned long startAttemptTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) {
        delay(500);
        Serial.print(".");
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nConnected with backup credentials!");
        connectedToWifi = true;
        wifiDisconnected = false;
        captivePortalActive = false;
        stopCaptivePortal();
        
        // Reset WiFi attempt counter on successful connection
        resetWiFiAttemptCounter();
        
        // Update current credentials to use backup as primary
        strcpy(ssid, backupSSID);
        strcpy(password, backupPassword);
        pref.putString("wifiSSID", backupSSID);
        pref.putString("wifiPass", backupPassword);
        Serial.println("Updated primary credentials to backup credentials");
    } else {
        Serial.println("\nBackup credentials failed");
    }
}

void resetWiFiAttemptCounter() {
    wifiConnectionAttempts = 0;
    wifiMaxAttemptsReached = false;
    Serial.println("WiFi attempt counter reset");
}

// OTA Update Functions (same pattern as AnchorPower)
void checkOTAUpdateFromFirebase() {
    if (!app.ready() || WiFi.status() != WL_CONNECTED) {
        Serial.println("OTA check skipped: Firebase not ready or WiFi disconnected");
        return;
    }
    
    if (xSemaphoreTake(otaMutex, pdMS_TO_TICKS(100))) {
        otaInfo.state = OTA_CHECKING;
        otaInfo.lastCheck = millis();
        xSemaphoreGive(otaMutex);
    }
    
    Serial.println("========================================");
    Serial.println("OTA: Checking for updates from Firebase");
    Serial.println("========================================");
    Serial.printf("OTA: Checking version at path: %s\n", OTA_VERSION_PATH);
    Serial.printf("OTA: Checking firmware URL at path: %s\n", OTA_URL_PATH);
    Serial.printf("OTA: Checking force update flag at path: %s\n", OTA_FORCE_PATH);
    Serial.printf("OTA: Current firmware version: %s\n", FIRMWARE_VERSION);
    Serial.println("OTA: Sending Firebase read requests...");
    
    // Read OTA configuration from Firebase
    Database.get(aClient, OTA_VERSION_PATH, asyncCB, false, "getOTAVersion");
    Serial.printf("OTA: Requested version from: %s\n", OTA_VERSION_PATH);
    Database.get(aClient, OTA_URL_PATH, asyncCB, false, "getOTAURL");
    Serial.printf("OTA: Requested URL from: %s\n", OTA_URL_PATH);
    Database.get(aClient, OTA_FORCE_PATH, asyncCB, false, "getOTAForce");
    Serial.printf("OTA: Requested force flag from: %s\n", OTA_FORCE_PATH);
    Serial.println("OTA: Waiting for Firebase responses...");
}

bool shouldUpdateFirmware() {
    if (xSemaphoreTake(otaMutex, pdMS_TO_TICKS(100))) {
        bool result = false;
        
        Serial.printf("shouldUpdateFirmware check - version: '%s' (len=%d), url: '%s' (len=%d), current: '%s'\n",
                     otaInfo.version, strlen(otaInfo.version),
                     otaInfo.firmwareUrl, strlen(otaInfo.firmwareUrl),
                     FIRMWARE_VERSION);
        
        if (strlen(otaInfo.version) > 0 && strlen(otaInfo.firmwareUrl) > 0) {
            bool versionDifferent = (strcmp(otaInfo.version, FIRMWARE_VERSION) != 0);
            result = versionDifferent || otaInfo.forceUpdate;
            Serial.printf("shouldUpdateFirmware: versionDifferent=%s, forceUpdate=%s, result=%s\n",
                         versionDifferent ? "YES" : "NO",
                         otaInfo.forceUpdate ? "YES" : "NO",
                         result ? "YES" : "NO");
        } else {
            Serial.println("shouldUpdateFirmware: version or URL not set, skipping update");
        }
        
        xSemaphoreGive(otaMutex);
        return result;
    }
    Serial.println("shouldUpdateFirmware: could not acquire mutex");
    return false;
}

bool downloadFirmwareChunk() {
    static WiFiClientSecure secureClient;
    static bool downloadStarted = false;
    static unsigned long totalSize = 0;
    static unsigned long downloaded = 0;
    static uint8_t buffer[OTA_DOWNLOAD_CHUNK_SIZE];
    
    if (!downloadStarted) {
        Serial.println("=== OTA DOWNLOAD STARTING ===");
        Serial.printf("WiFi status: %d\n", WiFi.status());
        Serial.printf("WiFi RSSI: %d\n", WiFi.RSSI());
        Serial.printf("Downloading from: %s\n", otaInfo.firmwareUrl);
        
        // Block all interrupts during OTA update
        Serial.println("Blocking interrupts for OTA update...");
        detachInterrupt(digitalPinToInterrupt(FLOAT_PIN));
        detachInterrupt(digitalPinToInterrupt(ESTOP_PIN));
        
        // Display OTA status on screen
        displayStatus("OTA Update Starting...");
        
        // Ensure WiFi is connected
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("ERROR: WiFi not connected!");
            // Reattach interrupts on failure
            attachInterrupt(digitalPinToInterrupt(FLOAT_PIN), floatSensorISR, CHANGE);
            attachInterrupt(digitalPinToInterrupt(ESTOP_PIN), estopISR, CHANGE);
            return false;
        }
        
        Serial.println("Configuring HTTP client...");
        secureClient.setInsecure();
        
        // Try to connect with shorter timeout
        if (!httpClient.begin(secureClient, otaInfo.firmwareUrl)) {
            Serial.println("ERROR: httpClient.begin() failed - invalid URL?");
            // Reattach interrupts on failure
            attachInterrupt(digitalPinToInterrupt(FLOAT_PIN), floatSensorISR, CHANGE);
            attachInterrupt(digitalPinToInterrupt(ESTOP_PIN), estopISR, CHANGE);
            displayStatus("OTA Init Failed");
            return false;
        }
        
        httpClient.setTimeout(5000);  // Reduce timeout to 5 seconds
        httpClient.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
        
        Serial.println("Sending HTTP GET request...");
        unsigned long getStart = millis();
        int httpCode = httpClient.GET();
        unsigned long getDuration = millis() - getStart;
        
        Serial.printf("HTTP GET took %lu ms\n", getDuration);
        Serial.printf("HTTP response code: %d\n", httpCode);
        
        if (httpCode <= 0) {
            Serial.printf("HTTP GET failed with error: %s\n", httpClient.errorToString(httpCode).c_str());
            httpClient.end();
            // Reattach interrupts on failure
            attachInterrupt(digitalPinToInterrupt(FLOAT_PIN), floatSensorISR, CHANGE);
            attachInterrupt(digitalPinToInterrupt(ESTOP_PIN), estopISR, CHANGE);
            displayStatus("OTA HTTP Failed");
            return false;
        }
        
        if (httpCode != HTTP_CODE_OK && httpCode != HTTP_CODE_MOVED_PERMANENTLY && httpCode != HTTP_CODE_FOUND) {
            Serial.printf("HTTP GET failed with code: %d\n", httpCode);
            httpClient.end();
            // Reattach interrupts on failure
            attachInterrupt(digitalPinToInterrupt(FLOAT_PIN), floatSensorISR, CHANGE);
            attachInterrupt(digitalPinToInterrupt(ESTOP_PIN), estopISR, CHANGE);
            displayStatus("OTA HTTP Failed");
            return false;
        }
        
        totalSize = httpClient.getSize();
        Serial.printf("Firmware size: %lu bytes\n", totalSize);
        
        if (totalSize <= 0) {
            Serial.println("ERROR: Invalid firmware size!");
            httpClient.end();
            // Reattach interrupts on failure
            attachInterrupt(digitalPinToInterrupt(FLOAT_PIN), floatSensorISR, CHANGE);
            attachInterrupt(digitalPinToInterrupt(ESTOP_PIN), estopISR, CHANGE);
            displayStatus("OTA Size Error");
            return false;
        }
        
        downloaded = 0;
        downloadStarted = true;
        
        if (xSemaphoreTake(otaMutex, pdMS_TO_TICKS(100))) {
            otaInfo.downloadSize = totalSize;
            otaInfo.downloadedBytes = 0;
            otaInfo.downloadStart = millis();
            otaInfo.state = OTA_DOWNLOADING;
            xSemaphoreGive(otaMutex);
        }
        
        Serial.println("Starting OTA update...");
        if (!Update.begin(totalSize)) {
            Serial.printf("ERROR: Update.begin() failed! Error: %s\n", Update.errorString());
            Serial.printf("Free sketch space: %d bytes\n", ESP.getFreeSketchSpace());
            httpClient.end();
            downloadStarted = false;
            // Reattach interrupts on failure
            attachInterrupt(digitalPinToInterrupt(FLOAT_PIN), floatSensorISR, CHANGE);
            attachInterrupt(digitalPinToInterrupt(ESTOP_PIN), estopISR, CHANGE);
            displayStatus("OTA Begin Failed");
            return false;
        }
        
        Serial.println("OTA update initialized successfully!");
    }
    
    WiFiClient* stream = httpClient.getStreamPtr();
    if (stream->available()) {
        size_t bytesRead = stream->readBytes(buffer, OTA_DOWNLOAD_CHUNK_SIZE);
        
        if (bytesRead > 0) {
            if (Update.write(buffer, bytesRead) != bytesRead) {
                Serial.println("ERROR: Update.write() failed!");
                Update.abort();
                httpClient.end();
                downloadStarted = false;
                // Reattach interrupts on failure
                attachInterrupt(digitalPinToInterrupt(FLOAT_PIN), floatSensorISR, CHANGE);
                attachInterrupt(digitalPinToInterrupt(ESTOP_PIN), estopISR, CHANGE);
                displayStatus("OTA Update Failed");
                return false;
            }
            
            downloaded += bytesRead;
            
            if (xSemaphoreTake(otaMutex, pdMS_TO_TICKS(10))) {
                otaInfo.downloadedBytes = downloaded;
                xSemaphoreGive(otaMutex);
            }
            
            // Print and display progress every 50KB
            static unsigned long lastProgressPrint = 0;
            if (downloaded - lastProgressPrint >= 50000 || downloaded == totalSize) {
                unsigned long elapsed = millis() - otaInfo.downloadStart;
                float percentComplete = (float)downloaded / (float)totalSize * 100.0;
                float speedKBps = (float)downloaded / (float)elapsed;  // KB per millisecond
                speedKBps *= 1000.0;  // Convert to KB per second
                
                unsigned long remainingBytes = totalSize - downloaded;
                unsigned long etaSeconds = 0;
                if (speedKBps > 0) {
                    etaSeconds = (unsigned long)(remainingBytes / speedKBps);
                }
                
                Serial.printf("OTA Progress: %.1f%% (%lu/%lu bytes) | Speed: %.1f KB/s | ETA: %lu sec\n",
                             percentComplete, downloaded, totalSize, speedKBps, etaSeconds);
                
                // Update display with progress
                char progressMsg[60];
                snprintf(progressMsg, sizeof(progressMsg), "OTA: %.0f%%", percentComplete);
                displayStatus(progressMsg);
                
                lastProgressPrint = downloaded;
            }
        }
    }
    
    if (downloaded >= totalSize) {
        unsigned long totalTime = millis() - otaInfo.downloadStart;
        float avgSpeedKBps = (float)downloaded / (float)totalTime * 1000.0;
        
        Serial.println("==========================================================");
        Serial.printf("OTA Download Complete! Downloaded %lu bytes in %.1f seconds\n", 
                     downloaded, (float)totalTime / 1000.0);
        Serial.printf("Average speed: %.1f KB/s\n", avgSpeedKBps);
        Serial.println("==========================================================");
        
        httpClient.end();
        downloadStarted = false;
        
        Serial.println("Finalizing OTA update...");
        displayStatus("OTA Installing...");
        if (Update.end()) {
            if (xSemaphoreTake(otaMutex, pdMS_TO_TICKS(100))) {
                otaInfo.state = OTA_INSTALLING;
                xSemaphoreGive(otaMutex);
            }
            
            if (Update.isFinished()) {
                if (xSemaphoreTake(otaMutex, pdMS_TO_TICKS(100))) {
                    otaInfo.state = OTA_COMPLETED;
                    xSemaphoreGive(otaMutex);
                }
                
                Serial.println("==========================================================");
                Serial.println("OTA UPDATE SUCCESSFUL!");
                Serial.printf("New firmware version: %s\n", otaInfo.version);
                Serial.println("Rebooting in 2 seconds...");
                Serial.println("==========================================================");
                
                // Display success message before reboot
                displayStatus("OTA Complete - Rebooting...");
                
                // Note: Interrupts remain detached - device will reboot
                delay(2000);
                ESP.restart();
                return true;
            } else {
                Update.abort();
                if (xSemaphoreTake(otaMutex, pdMS_TO_TICKS(100))) {
                    otaInfo.state = OTA_FAILED;
                    xSemaphoreGive(otaMutex);
                }
                // Reattach interrupts on failure
                attachInterrupt(digitalPinToInterrupt(FLOAT_PIN), floatSensorISR, CHANGE);
                attachInterrupt(digitalPinToInterrupt(ESTOP_PIN), estopISR, CHANGE);
                displayStatus("OTA Install Failed");
                return false;
            }
        } else {
            Update.abort();
            downloadStarted = false;
            if (xSemaphoreTake(otaMutex, pdMS_TO_TICKS(100))) {
                otaInfo.state = OTA_FAILED;
                xSemaphoreGive(otaMutex);
            }
            // Reattach interrupts on failure
            attachInterrupt(digitalPinToInterrupt(FLOAT_PIN), floatSensorISR, CHANGE);
            attachInterrupt(digitalPinToInterrupt(ESTOP_PIN), estopISR, CHANGE);
            displayStatus("OTA Update Failed");
            return false;
        }
    }
    
    return true;
}

// OTA Task running on Core 1
void otaTask(void *parameter) {
    Serial.println("OTA Task started on Core 1");
    
    // Wait for WiFi and Firebase to be ready
    unsigned long startWait = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - startWait < 30000)) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("OTA Task: WiFi not connected, exiting");
        vTaskDelete(NULL);
        return;
    }
    
    // Wait for Firebase to be ready
    startWait = millis();
    while (!app.ready() && (millis() - startWait < 30000)) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    if (!app.ready()) {
        Serial.println("OTA Task: Firebase not ready, exiting");
        vTaskDelete(NULL);
        return;
    }
    
    Serial.println("OTA Task: System ready, starting OTA checks");
    
    // Initialize OTA state
    if (xSemaphoreTake(otaMutex, pdMS_TO_TICKS(1000))) {
        otaInfo.state = OTA_IDLE;
        otaInfo.lastCheck = 0;
        otaInfo.updateAvailable = false;
        xSemaphoreGive(otaMutex);
    }
    
    while (true) {
        unsigned long currentTime = millis();
        bool shouldCheck = false;
        bool isDownloading = false;
        OTAState currentState = OTA_IDLE;
        unsigned long lastCheckTime = 0;
        
        if (xSemaphoreTake(otaMutex, pdMS_TO_TICKS(100))) {
            currentState = otaInfo.state;
            lastCheckTime = otaInfo.lastCheck;
            if (otaInfo.state == OTA_IDLE) {
                shouldCheck = (currentTime - otaInfo.lastCheck >= OTA_CHECK_INTERVAL);
            } else if (otaInfo.state == OTA_DOWNLOADING) {
                isDownloading = true;
            } else if (otaInfo.state == OTA_CHECKING) {
                // Timeout after 10 seconds if stuck in CHECKING state
                if (currentTime - otaInfo.lastCheck > 10000) {
                    Serial.println("OTA check timeout - resetting to IDLE");
                    otaInfo.state = OTA_IDLE;
                    // Set lastCheck to currentTime minus OTA_CHECK_INTERVAL to allow next check after interval
                    otaInfo.lastCheck = currentTime - OTA_CHECK_INTERVAL + 1000; // Add 1 second buffer
                }
            }
            xSemaphoreGive(otaMutex);
        }
        
        if (shouldCheck && WiFi.status() == WL_CONNECTED && app.ready()) {
            Serial.println("========================================");
            Serial.println("OTA: Periodic check triggered (1 minute)");
            Serial.printf("OTA: Checking Firebase paths for updates...\n");
            checkOTAUpdateFromFirebase();
            vTaskDelay(pdMS_TO_TICKS(5000)); // Increased delay to allow all callbacks to complete
        }
        
        // Check if we should download or are already downloading
        bool needsDownload = false;
        OTAState currentOtaState = OTA_IDLE;
        
        if (xSemaphoreTake(otaMutex, pdMS_TO_TICKS(100))) {
            currentOtaState = otaInfo.state;
            
            // Check if we need to start download (version different OR force update)
            if (currentOtaState == OTA_IDLE && 
                strlen(otaInfo.version) > 0 && 
                strlen(otaInfo.firmwareUrl) > 0 &&
                (strcmp(otaInfo.version, FIRMWARE_VERSION) != 0 || otaInfo.forceUpdate)) {
                
                Serial.println("==========================================================");
                Serial.println("OTA: NEW FIRMWARE DETECTED - STARTING DOWNLOAD");
                Serial.printf("Current: %s -> New: %s\n", FIRMWARE_VERSION, otaInfo.version);
                Serial.println("==========================================================");
                otaInfo.state = OTA_DOWNLOADING;
                otaInfo.updateAvailable = true;
                currentOtaState = OTA_DOWNLOADING;
                needsDownload = true;
            } else if (currentOtaState == OTA_DOWNLOADING) {
                needsDownload = true;
            }
            
            xSemaphoreGive(otaMutex);
        }
        
        // Actually perform the download outside of mutex
        if (needsDownload && WiFi.status() == WL_CONNECTED) {
            bool continueDownload = downloadFirmwareChunk();
            
            if (!continueDownload) {
                if (xSemaphoreTake(otaMutex, pdMS_TO_TICKS(100))) {
                    if (otaInfo.state != OTA_COMPLETED && otaInfo.state == OTA_DOWNLOADING) {
                        Serial.println("=== OTA: Download failed, setting state to FAILED ===");
                        otaInfo.state = OTA_FAILED;
                    } else if (otaInfo.state == OTA_COMPLETED) {
                        Serial.println("=== OTA: Download completed successfully ===");
                    }
                    xSemaphoreGive(otaMutex);
                }
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("Starting Fish Tank Controller...");
    Serial.println("Initializing system components...");
    
    // Initialize OLED display
    Wire.begin();
    Serial.println("Initializing TFT display...");
    initTFT();
    displayLoading(); // Show anchor logo during loading
    
    // Initialize watchdog
    resetWatchdog();
    lastWaterChangeActivity = millis();
    
    // Initialize motor state
    motorState.waterInRunning = false;
    motorState.waterOutRunning = false;
    motorState.lastWaterInToggle = 0;
    motorState.lastWaterOutToggle = 0;
    motorState.waterInRequested = false;
    motorState.waterOutRequested = false;
    
    // Initialize E-Stop flags
    estopPressed = false;
    estopPressTime = 0;

    Serial.println("Creating system mutexes...");
    floatMutex = xSemaphoreCreateMutex();
    if (floatMutex == NULL) {
        Serial.println("Error creating float mutex! Restarting...");
        ESP.restart();
    }
    Serial.println("Float mutex created successfully");
    
    // Create OTA mutex
    otaMutex = xSemaphoreCreateMutex();
    if (otaMutex == NULL) {
        Serial.println("Error creating OTA mutex! Restarting...");
        ESP.restart();
    }
    Serial.println("OTA mutex created successfully");

    // Initialize GPIO pins
    Serial.println("Initializing GPIO pins...");
    pinMode(FLOAT_PIN, INPUT_PULLUP);
    // GPIO34-39 have no internal pull-ups; require external pull-up/down
    pinMode(ESTOP_PIN, INPUT_PULLUP);
    pinMode(MTR_ATO, OUTPUT);
    pinMode(MTR_DOSE1, OUTPUT);
    pinMode(MTR_DOSE2, OUTPUT);
    pinMode(MTR_DOSE3, OUTPUT);
    pinMode(MTR_WATER_CHANGE_OUT, OUTPUT);
    pinMode(MTR_WATER_CHANGE_IN, OUTPUT);
    
    // Ensure all motors are off initially
    Serial.println("Setting all motors to OFF state...");
    digitalWrite(MTR_ATO, LOW);
    digitalWrite(MTR_DOSE1, LOW);
    digitalWrite(MTR_DOSE2, LOW);
    digitalWrite(MTR_DOSE3, LOW);
    digitalWrite(MTR_WATER_CHANGE_OUT, LOW);
    digitalWrite(MTR_WATER_CHANGE_IN, LOW);
    
    manualDose.requested[0] = false;
    manualDose.requested[1] = false;
    manualDose.requested[2] = false;
    containerVolumes.dose1 = 1000;
    containerVolumes.dose2 = 1000;
    containerVolumes.dose3 = 1000;
    containerVolumes.ato = 5.0f; // Default 5 gallons
    containerVolumes.water_in = 5.0; // Default 5 gallons
    containerVolumes.water_out = 5.0;
    // Note: dosed amounts will be loaded from Firebase on restart
    // to avoid overwriting existing data
   

    for (int i = 0; i < 3; i++) {
        manualDoseFlags[i] = false;
    }

    // Initialize states
    floatState = readEffectiveFloatPin();
    estopState = readEffectiveEstopPin();
    strcpy(estopStatus, estopState ? "OK" : "STOP");
    strcpy(pumpState, "STOP");

    // Initialize sensors
    Serial.println("Initializing temperature sensors...");
    sensors.begin();
    Serial.println("Temperature sensors initialized");
    sensors.requestTemperatures(); 
    temperatureF = sensors.getTempFByIndex(0);
    Serial.print(temperatureF);

    // Attach interrupts with proper triggering
    Serial.println("Attaching interrupts...");
    attachInterrupt(digitalPinToInterrupt(FLOAT_PIN), floatSensorISR, CHANGE);
    // If ESTOP has external pull-up and is active-low to GND, trigger on FALLING
    attachInterrupt(digitalPinToInterrupt(ESTOP_PIN), estopISR, CHANGE);
    Serial.println("Interrupts attached successfully");

            // Connect to WiFi
        Serial.println("Setting up WiFi connection...");
        // Do not clear NVS or forcefully disconnect here; keep any existing connection

        
        pref.begin("wifiCredentials", false);
        
        core.begin("core", false);
        uuidExists = core.isKey("UUID");

        if (uuidExists == false) {
            // Generate UUID safely without String concatenation
            sprintf(uuidStr, "%lu%lu", random(999999999), random(999999999));
            Serial.print(F("New UUID for this device is: "));
            Serial.println(uuidStr);

            core.putString("UUID", uuidStr);
            Serial.println("UUID saved to preferences");
        } else {
            strcpy(uuidStr, core.getString("UUID").c_str());
            Serial.print("Retrieved existing UUID: ");
            Serial.println(uuidStr);
        }

        Serial.print("DEVICE WITH THE FOLLOWING UUID HAS BOOTED: ");
        Serial.println(uuidStr);
        


        bool wifiSSIDExists = pref.isKey("wifiSSID");
        bool wifiPassExists = pref.isKey("wifiPass");
        bool backupSSIDExists = pref.isKey("backupSSID");
        bool backupPassExists = pref.isKey("backupPassword");
        
        Serial.printf("WiFi credentials check - SSID exists: %s, Password exists: %s\n", 
                    wifiSSIDExists ? "true" : "false", wifiPassExists ? "true" : "false");
        Serial.printf("Backup credentials check - SSID exists: %s, Password exists: %s\n", 
                    backupSSIDExists ? "true" : "false", backupPassExists ? "true" : "false");

        if (wifiSSIDExists == true && wifiPassExists == true) {
            String tempUsername = pref.getString("username"); // Get the String object
            tempUsername.toCharArray(username, sizeof(username));
            while (!connectedToWifi && !wifiMaxAttemptsReached) {
                Serial.println("Trying primary WiFi credentials...");
                connectToWiFi(pref.getString("wifiSSID").c_str(), pref.getString("wifiPass").c_str());
                
                if (!connectedToWifi && backupSSIDExists && backupPassExists && !wifiMaxAttemptsReached) {
                    Serial.println("Primary credentials failed, trying backup credentials...");
                    strcpy(backupSSID, pref.getString("backupSSID").c_str());
                    strcpy(backupPassword, pref.getString("backupPassword").c_str());
                    hasBackupCredentials = true;
                    tryBackupCredentials();
                }
            }
            
            // If max attempts reached, launch captive portal and wait for configuration
            if (wifiMaxAttemptsReached && !connectedToWifi) {
                Serial.println("Maximum WiFi attempts reached in setup - launching captive portal");
                
                // Use exact same method as first-time setup
                Serial.println("No WiFi credentials found - configuring access point...");
                displayStatus("Starting AP Mode...");
                
                // Disconnect from any existing WiFi and switch to AP mode
                WiFi.disconnect(true);
                delay(500);
                WiFi.mode(WIFI_AP);
                delay(500);
                
                // Create open access point (no password) - following working example
                Serial.printf("Creating access point with SSID: %s\n", AP_SSID);
                bool apStarted = WiFi.softAP(AP_SSID);
                if (!apStarted) {
                    Serial.println("ERROR: Failed to start access point!");
                    return;
                }
                delay(1000); // Give AP time to start
                
                IPAddress myIP = WiFi.softAPIP();
                Serial.print("AP IP address: ");
                Serial.println(myIP);
                
                // Additional debugging
                Serial.printf("WiFi mode: %d (should be 2 for AP mode)\n", WiFi.getMode());
                Serial.printf("AP SSID: %s\n", WiFi.softAPSSID().c_str());
                Serial.printf("AP IP: %s\n", WiFi.softAPIP().toString().c_str());
                Serial.printf("Connected stations: %d\n", WiFi.softAPgetStationNum());
                Serial.printf("AP MAC: %s\n", WiFi.softAPmacAddress().c_str());
                Serial.printf("AP is running: %s\n", WiFi.softAPgetStationNum() >= 0 ? "Yes" : "No");
                
                // Display setup information on TFT
                char ipStr[16];
                sprintf(ipStr, "%d.%d.%d.%d", myIP[0], myIP[1], myIP[2], myIP[3]);
                displayWiFiFailureSetup(AP_SSID, ipStr);
                
                // Start DNS server for captive portal
                dnsServer.start(DNS_PORT, "*", myIP);
                
                // Setup web server routes
                Serial.println("Setting up web server routes...");
                webServer.on("/", handleRoot);
                webServer.on("/setup-anchor", handleSetupAnchor);
                webServer.on("/scan-wifi", handleScanWiFi);
                webServer.onNotFound(handleRoot); // Redirect all unknown requests to config page
                webServer.begin();
                
                Serial.println("Web server started successfully");
                Serial.println("Connect to 'Anchor' network (no password) and visit http://192.168.4.1");
                Serial.println("=== CAPTIVE PORTAL ACTIVE ===");
                Serial.println("The ESP32 will now wait for WiFi configuration");
                Serial.println("Do NOT restart or interrupt the device");
                
                // Set captive portal active flag and reset attempt counter for portal attempts
                captivePortalActive = true;
                wifiConnectionAttempts = 0; // Reset counter for portal attempts
                wifiMaxAttemptsReached = false; // Reset max attempts flag
                
                // Wait for web configuration
                while (!connectedToWifi) {
                    // Handle web server requests
                    webServer.handleClient();
                    dnsServer.processNextRequest();
                    
                    // Show connection status every 5 seconds
                    static unsigned long lastStatus = 0;
                    if (millis() - lastStatus > 5000) {
                        Serial.printf("AP Status: %d connected devices, Mode: %d\n", 
                                    WiFi.softAPgetStationNum(), WiFi.getMode());
                        Serial.println("Waiting for WiFi configuration...");
                        Serial.println("If you can't see 'Anchor' network, try restarting your device's WiFi");
                        Serial.println("You can enter WiFi credentials multiple times if needed");
                        lastStatus = millis();
                    }
                    
                    delay(10);
                }
                
                Serial.println("WiFi configuration completed via captive portal");
                captivePortalActive = false;
            }
        } else {
            Serial.println("No WiFi credentials found - configuring access point...");
            displayStatus("Starting AP Mode...");
            
            // Create open access point (no password) - following working example
            Serial.printf("Creating access point with SSID: %s\n", AP_SSID);
            WiFi.softAP(AP_SSID);
            
            IPAddress myIP = WiFi.softAPIP();
            Serial.print("AP IP address: ");
            Serial.println(myIP);
            
            // Display setup information on OLED
            char ipStr[16];
            sprintf(ipStr, "%d.%d.%d.%d", myIP[0], myIP[1], myIP[2], myIP[3]);
            displaySetupInfo(AP_SSID, ipStr);
            
            // Start DNS server for captive portal
            dnsServer.start(DNS_PORT, "*", myIP);
            
            // Setup web server routes
            Serial.println("Setting up web server routes...");
            webServer.on("/", handleRoot);
            webServer.on("/setup-anchor", handleSetupAnchor);
            webServer.on("/scan-wifi", handleScanWiFi);
            webServer.onNotFound(handleRoot); // Redirect all unknown requests to config page
            webServer.begin();
            
            Serial.println("Web server started successfully");
            Serial.println("Connect to 'Anchor' network (no password) and visit http://192.168.4.1");
            
            // Wait for web configuration
            while (!connectedToWifi) {
                // Handle web server requests
                webServer.handleClient();
                dnsServer.processNextRequest();
                
                // Show connection status every 5 seconds
                static unsigned long lastStatus = 0;
                if (millis() - lastStatus > 5000) {
                    Serial.printf("AP Status: %d connected devices\n", WiFi.softAPgetStationNum());
                    lastStatus = millis();
                }
                
                delay(10);
            }
            
                    // Save credentials after successful web configuration
        Serial.println("Saving new WiFi credentials...");
        
        // If we have existing credentials, save them as backup
        if (pref.isKey("wifiSSID") && pref.isKey("wifiPass")) {
            String oldSSID = pref.getString("wifiSSID");
            String oldPass = pref.getString("wifiPass");
            pref.putString("backupSSID", oldSSID);
            pref.putString("backupPassword", oldPass);
            Serial.printf("Saved old credentials as backup: %s\n", oldSSID.c_str());
        }
        
        // Save new credentials as primary
        pref.putString("wifiSSID", ssid);
        pref.putString("wifiPass", password);
        pref.putString("username", username);
        Serial.printf("Saved new primary credentials: %s\n", ssid);
        
        WiFi.softAPdisconnect(true);
        }
                        // Get user email from preferences
        bool userEmailExists = pref.isKey("userEmail");
        if (userEmailExists) {
            String tempEmail = pref.getString("userEmail");
            strcpy(userEmail, tempEmail.c_str());
        } else {
            userEmail[0] = '\0'; // Empty string
        }
        Serial.printf("userEmail key exists: %s\n", userEmailExists ? "true" : "false");
        Serial.printf("Retrieved userEmail from preferences: '%s' (length: %d)\n", userEmail, strlen(userEmail));
        
        // Use email/uuid structure for Firebase paths (sanitize email for Firebase)
        if (strlen(userEmail) == 0) {
          Serial.println("WARNING: userEmail is empty! Using fallback path with UUID only");
          strcpy(BASE_PATH, uuidStr); // Fallback to just UUID if no email
          // Also set display name to UUID
          strncpy(anchorName, uuidStr, sizeof(anchorName)-1);
          anchorName[sizeof(anchorName)-1] = '\0';
        } else {
          // Create sanitized email directly without String objects
          char sanitizedEmail[50];
          strcpy(sanitizedEmail, userEmail);
          // Replace @ with _at_ and . with _dot_
          char *at_pos = strchr(sanitizedEmail, '@');
          while (at_pos) {
            memmove(at_pos + 4, at_pos + 1, strlen(at_pos + 1) + 1);
            memcpy(at_pos, "_at_", 4);
            at_pos = strchr(at_pos + 4, '@');
          }
          char *dot_pos = strchr(sanitizedEmail, '.');
          while (dot_pos) {
            memmove(dot_pos + 5, dot_pos + 1, strlen(dot_pos + 1) + 1);
            memcpy(dot_pos, "_dot_", 5);
            dot_pos = strchr(dot_pos + 5, '.');
          }
          
                  strcpy(BASE_PATH, sanitizedEmail);
        strcat(BASE_PATH, "/");
        strcat(BASE_PATH, uuidStr);
          // Save for later Firebase name fetch
          strncpy(sanitizedEmailStr, sanitizedEmail, sizeof(sanitizedEmailStr)-1);
          sanitizedEmailStr[sizeof(sanitizedEmailStr)-1] = '\0';
          // Clear display name; will be fetched from sanitized_email/uuid/name
          anchorName[0] = '\0';
      }
      // Use shorter path names to save memory
    strcpy(DOSE1_PATH, BASE_PATH);
    strcat(DOSE1_PATH, "/settings/dose1");
    strcpy(DOSE2_PATH, BASE_PATH);
    strcat(DOSE2_PATH, "/settings/dose2");
    strcpy(DOSE3_PATH, BASE_PATH);
    strcat(DOSE3_PATH, "/settings/dose3");
    strcpy(WATER_CHANGE_PATH, BASE_PATH);
    strcat(WATER_CHANGE_PATH, "/settings/water_change");
    strcpy(SYSTEM_STATUS_PATH, BASE_PATH);
    strcat(SYSTEM_STATUS_PATH, "/system_status");
    strcpy(SETTINGS_PATH, BASE_PATH);
    strcat(SETTINGS_PATH, "/settings");
    strcpy(MANUAL_CONTROL_PATH, BASE_PATH);
    strcat(MANUAL_CONTROL_PATH, "/settings/manual_control");
    
    // OTA Database paths - global OTA update paths
    strcpy(OTA_BASE_PATH, "/global_ota_updates");
    strcpy(OTA_VERSION_PATH, OTA_BASE_PATH);
    strcat(OTA_VERSION_PATH, "/anchor_version");
    strcpy(OTA_URL_PATH, OTA_BASE_PATH);
    strcat(OTA_URL_PATH, "/firmware_anchor_url");
    strcpy(OTA_FORCE_PATH, OTA_BASE_PATH);
    strcat(OTA_FORCE_PATH, "/force_update");
    strcpy(OTA_STATUS_PATH, OTA_BASE_PATH);
    strcat(OTA_STATUS_PATH, "/status_info");
        

        
        // Firebase paths configured successfully
        Serial.println("Firebase paths configured successfully");


        // Configure time and Firebase after WiFi connection
        if (connectedToWifi) {
            Serial.println("Configuring time and Firebase...");
            configTime(GMT_OFFSET, DAYLIGHT_OFFSET, NTP_SERVER);
            Serial.println("Time configured");
            checkAndConnectFirebase();
            

            
            // Report initial device status
            Serial.println("Reporting initial device status...");
            reportDeviceStatus();
        }
        
        // Only continue with system setup if WiFi is connected
        if (connectedToWifi) {
            Serial.println("Creating dose mutex...");
            doseMutex = xSemaphoreCreateMutex();
            if (doseMutex == NULL) {
              Serial.println("Error creating dose mutex! Restarting...");
              ESP.restart();
            }
            Serial.println("Dose mutex created successfully");
            
            if (firebaseState == FIREBASE_READY) {
        char tempPath[128];
        strcpy(tempPath, SYSTEM_STATUS_PATH);
        strcat(tempPath, "/last_dose1_time");
        Database.get(aClient, tempPath, asyncCB, false, "getDose1Time");
        
        strcpy(tempPath, SYSTEM_STATUS_PATH);
        strcat(tempPath, "/last_dose2_time");
        Database.get(aClient, tempPath, asyncCB, false, "getDose2Time");
        
        strcpy(tempPath, SYSTEM_STATUS_PATH);
        strcat(tempPath, "/last_dose3_time");
        Database.get(aClient, tempPath, asyncCB, false, "getDose3Time");
        
        // Force initial calibration factor update
        firebaseNeedsUpdate = true;
    }
     manualWater.waterIn = false;
    manualWater.waterOut = false;
    manualWater.completed = false;
    manualWater.stopWaterChange = false; // Initialize stop water change flag
    strcpy(previousValues.waterChangeStatus, "completed");
    Serial.println("Creating water change mutex...");
    waterChange.mutex = xSemaphoreCreateMutex();
    if (waterChange.mutex == NULL) {
        Serial.println("Error creating water change mutex! Restarting...");
        ESP.restart();
    }
    Serial.println("Water change mutex created successfully");
    // Start float sensor task on Core 1
    Serial.println("Creating float sensor task on Core 1...");
    xTaskCreatePinnedToCore(
        floatSensorTask,
        "FloatSensorTask",
        4096,
        NULL,
        1,
        &floatSensorTaskHandle,
        1
    );
    Serial.println("Float sensor task created successfully");
    
    // Create OTA task on Core 1 (second core)
    Serial.println("Creating OTA task on Core 1...");
    xTaskCreatePinnedToCore(
        otaTask,
        "OTATask",
        OTA_TASK_STACK_SIZE,
        NULL,
        1,  // Lower priority than float sensor
        &otaTaskHandle,
        1   // Core 1
    );
    Serial.println("OTA task created successfully on Core 1");
            
            // Display system ready status
            Serial.println("Setup complete - displaying system ready status");
            displayStatus("System Ready");
            delay(1000);
            TFT_display.fillScreen(BLACK); // Clear display after setup
            Serial.println("System initialization complete!");
        } else {
            Serial.println("WiFi not connected - system setup incomplete");
            Serial.println("Please configure WiFi via captive portal");
        }

}





void loop() {
    processSerialSensorCommands();
    app.loop();
    Database.loop();

    // Manual water changes are now controlled by volume completion, not timeouts

    // Check WiFi reconnection status
    checkWiFiReconnection();
    
    // Handle web server and DNS server (if captive portal is active)
    if (captivePortalActive) {
        webServer.handleClient();
        dnsServer.processNextRequest();
    }

    // Rest of your existing loop code...
    if (WiFi.status() != WL_CONNECTED && !captivePortalActive && !wifiMaxAttemptsReached) {
        static unsigned long lastReconnectAttempt = 0;
        const unsigned long RECONNECT_COOLDOWN_MS = 5000;
        if (millis() - lastReconnectAttempt > RECONNECT_COOLDOWN_MS) {
            lastReconnectAttempt = millis();
            connectToWiFi(pref.getString("wifiSSID").c_str(), pref.getString("wifiPass").c_str());
        }
    }
        if (millis() - lastFirebaseUpdate >= FIREBASE_INTERVAL) {
        updateFirebase();
        lastFirebaseUpdate = millis();
    }
    performDosing(1, dose1, MTR_DOSE1);
    performDosing(2, dose2, MTR_DOSE2);
    performDosing(3, dose3, MTR_DOSE3);
    checkScheduledWaterChange();  // Replaces performWaterChange()
    handleWaterChanges();
    checkReservoirs();  // Check water reservoir levels and emergency stop if needed
    
    // New robust functions
    processEStop();  // Handle E-Stop from ISR
    checkWatchdog(); // Check for stuck states
    validateWaterChangeState(); // Validate water change state
    handleSystemErrors(); // Comprehensive error handling
    
    // Continuous E-stop monitoring during water changes
    static unsigned long lastEstopCheck = 0;
    if (millis() - lastEstopCheck > 100) { // Check every 100ms
        int currentEstopState = readEffectiveEstopPin();
        if (currentEstopState != estopState) {
            estopState = currentEstopState;
            strcpy(estopStatus, estopState ? "OK" : "STOP");
            Serial.printf("Main loop: E-Stop state changed to %s\n", estopStatus);
            
            // If E-stop was pressed and water change is active, trigger immediate stop
            if (currentEstopState == LOW && isWaterChangeActive) {
                Serial.println("Main loop: E-Stop pressed during active water change - triggering stop");
                estopPressed = true;
                estopPressTime = millis();
            }
        }
        lastEstopCheck = millis();
    }
    

    
    // Report device status periodically
    static unsigned long lastStatusReport = 0;
    if (millis() - lastStatusReport > 3600000) { // Every 60 minutes (further reduced)
        reportDeviceStatus();
        lastStatusReport = millis();
    }
    
    // Reset watchdog periodically
    static unsigned long lastWatchdogReset = 0;
    if (millis() - lastWatchdogReset > 60000) { // Every minute
        resetWatchdog();
        lastWatchdogReset = millis();
    }
    
    // TFT screen refresh to clear any legacy text
    if (millis() - lastTFTRefresh > TFT_REFRESH_INTERVAL) {
        refreshTFTDisplay();
    }
    
    // Synchronize isWaterChangeActive with waterChange.active state
    bool currentWaterChangeActive = false;
    if (xSemaphoreTake(waterChange.mutex, pdMS_TO_TICKS(10))) {
        currentWaterChangeActive = waterChange.active;
        xSemaphoreGive(waterChange.mutex);
    }
    
    // Update isWaterChangeActive to match waterChange.active
    if (isWaterChangeActive != currentWaterChangeActive) {
        isWaterChangeActive = currentWaterChangeActive;
        Serial.printf("Water change active state updated: %s\n", isWaterChangeActive ? "true" : "false");
    }
    
    // Update container status for empty container warnings
    updateContainerStatus();
    
    // Update TFT display with state-based information
    updateMainDisplay();
    
    // Periodic system status debug (every 30 seconds)
    static unsigned long lastDebugPrint = 0;
    if (millis() - lastDebugPrint > 30000) {
        Serial.printf("System Status - WiFi: %s, Portal: %s, Firebase: %s, Temp: %.1f°F, Pump: %s, E-Stop: %s, WC: %s\n",
                    WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected",
                    captivePortalActive ? "Active" : "Inactive",
                    firebaseState == FIREBASE_READY ? "Ready" : "Not Ready",
                    temperatureF,
                    pumpState,
                    estopStatus,
                    isWaterChangeActive ? "Active" : "Inactive");
        
        if (wifiDisconnected) {
            Serial.printf("WiFi disconnected for %lu seconds\n", (millis() - wifiDisconnectTime) / 1000);
        }
        
        lastDebugPrint = millis();
    }
    
    delay(10);
}

// Web server functions for laptop configuration
void handleRoot() {
  Serial.println("Root page accessed - showing setup form");
  // Show simple setup form with email
  String html = R"(
<!DOCTYPE html>
<html>
<head>
    <title>Anchor Setup</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { 
            font-family: Arial, sans-serif; 
            margin: 0; 
            padding: 20px; 
            background: linear-gradient(135deg, #ffffff 0%, #f0f0f0 100%);
            color: black;
            text-align: center;
            min-height: 100vh;
            display: flex;
            flex-direction: column;
            justify-content: center;
            align-items: center;
        }
        .container { 
            max-width: 500px; 
            background: rgba(0,0,0,0.05); 
            padding: 40px; 
            border-radius: 20px; 
            backdrop-filter: blur(10px);
            box-shadow: 0 8px 32px rgba(0,0,0,0.1);
            border: 1px solid rgba(0,0,0,0.2);
        }
        .logo {
            font-size: 48px;
            font-weight: bold;
            margin: 0 auto 20px;
            color: black;
            text-shadow: 2px 2px 4px rgba(255,255,255,0.5);
            letter-spacing: 2px;
        }
        h1 { 
            color: black; 
            text-align: center; 
            margin-bottom: 20px;
            font-size: 28px;
        }
        .form-group {
            margin-bottom: 20px;
            text-align: left;
        }
        label {
            display: block;
            margin-bottom: 5px;
            font-weight: bold;
        }
        input[type="text"], input[type="password"], input[type="email"] {
            width: 100%;
            padding: 12px;
            border: none;
            border-radius: 8px;
            background: rgba(255,255,255,0.9);
            color: #333;
            font-size: 16px;
            box-sizing: border-box;
        }
        .config-btn {
            width: 100%;
            padding: 15px;
            background: black;
            border: 2px solid black;
            border-radius: 8px;
            color: white;
            font-size: 16px;
            font-weight: bold;
            cursor: pointer;
            transition: all 0.3s ease;
        }
        .config-btn:hover {
            background: white;
            color: black;
            border-color: black;
            transform: translateY(-2px);
            box-shadow: 0 5px 15px rgba(0,0,0,0.3);
        }
        .error {
            background: rgba(255,0,0,0.2);
            padding: 10px;
            border-radius: 5px;
            margin-bottom: 15px;
            display: none;
        }
        .success {
            background: rgba(0,255,0,0.2);
            padding: 10px;
            border-radius: 5px;
            margin-bottom: 15px;
            display: none;
        }
        .loading {
            display: none;
            text-align: center;
            margin-top: 10px;
        }
        .spinner {
            border: 3px solid rgba(255,255,255,0.3);
            border-top: 3px solid white;
            border-radius: 50%;
            width: 20px;
            height: 20px;
            animation: spin 1s linear infinite;
            margin: 0 auto 10px;
        }
        @keyframes spin {
            0% { transform: rotate(0deg); }
            100% { transform: rotate(360deg); }
        }
        .instructions {
            background: rgba(0,0,0,0.05);
            padding: 20px;
            border-radius: 10px;
            margin-bottom: 20px;
            border: 1px solid rgba(0,0,0,0.2);
            color: black;
        }
        .scan-btn {
            background: black;
            color: white;
            border: 2px solid black;
            padding: 10px 15px;
            border-radius: 5px;
            cursor: pointer;
            font-size: 14px;
            white-space: nowrap;
            transition: all 0.3s;
            font-weight: bold;
        }
        .scan-btn:hover {
            background: white;
            color: black;
            border-color: black;
        }
        .scan-btn:disabled {
            background: #999;
            color: #666;
            border-color: #999;
            cursor: not-allowed;
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="logo">ANCHOR</div>
        <h1>Anchor Setup</h1>
        
        <div class="instructions">
            <h3>Setup Instructions:</h3>
            <p>1. Enter your email address (used for your Anchor account)</p>
            <p>2. Enter your home WiFi credentials</p>
            <p>3. Click "Setup Anchor" to configure your device</p>
        </div>
        
        <div class="error" id="error"></div>
        <div class="success" id="success"></div>
        
        <form id="configForm">
            <div class="form-group">
                <label for="email">Your Email Address</label>
                <input type="email" id="email" name="email" placeholder="Enter your email address..." required>
            </div>
            <div class="form-group">
                <label for="ssid">WiFi Network Name (SSID)</label>
                <div style="display: flex; gap: 10px;">
                    <input type="text" id="ssid" name="ssid" required style="flex: 1;">
                    <button type="button" id="scanBtn" class="scan-btn">Scan Networks</button>
                </div>
                <select id="networkList" style="display: none; width: 100%; margin-top: 10px; padding: 10px; border: 1px solid #ddd; border-radius: 5px; background: white; color: #333;">
                    <option value="">Select a network...</option>
                </select>
            </div>
            <div class="form-group">
                <label for="password">WiFi Password</label>
                <input type="password" id="password" name="password" required>
            </div>
            <button type="submit" class="config-btn">Setup Anchor</button>
        </form>
        
        <div class="loading" id="loading">
            <div class="spinner"></div>
            <p>Setting up anchor...</p>
        </div>
    </div>
    
    <script>
        // WiFi scanning functionality
        document.getElementById('scanBtn').addEventListener('click', function() {
            const scanBtn = this;
            const networkList = document.getElementById('networkList');
            const ssidInput = document.getElementById('ssid');
            
            scanBtn.disabled = true;
            scanBtn.textContent = 'Scanning...';
            
            fetch('/scan-wifi')
            .then(response => response.json())
            .then(networks => {
                networkList.innerHTML = '<option value="">Select a network...</option>';
                
                if (networks.length === 0) {
                    networkList.innerHTML += '<option value="" disabled>No networks found</option>';
                } else {
                    networks.forEach(network => {
                        const option = document.createElement('option');
                        option.value = network.ssid;
                        option.textContent = network.ssid + ' (' + network.encryption + ', ' + network.rssi + ' dBm)';
                        networkList.appendChild(option);
                    });
                }
                
                networkList.style.display = 'block';
                scanBtn.textContent = 'Scan Again';
                scanBtn.disabled = false;
            })
            .catch(error => {
                console.error('Scan error:', error);
                document.getElementById('error').textContent = 'Failed to scan for networks: ' + error.message;
                document.getElementById('error').style.display = 'block';
                scanBtn.textContent = 'Scan Networks';
                scanBtn.disabled = false;
            });
        });
        
        document.getElementById('networkList').addEventListener('change', function() {
            const ssidInput = document.getElementById('ssid');
            ssidInput.value = this.value;
        });
        
        document.getElementById('configForm').addEventListener('submit', async function(e) {
            e.preventDefault();
            
            const email = document.getElementById('email').value;
            const ssid = document.getElementById('ssid').value;
            const password = document.getElementById('password').value;
            const errorDiv = document.getElementById('error');
            const successDiv = document.getElementById('success');
            const loadingDiv = document.getElementById('loading');
            
            // Show loading
            loadingDiv.style.display = 'block';
            errorDiv.style.display = 'none';
            successDiv.style.display = 'none';
            
            try {
                const formData = new FormData();
                formData.append('email', email);
                formData.append('ssid', ssid);
                formData.append('password', password);

                const response = await fetch('/setup-anchor', {
                    method: 'POST',
                    body: formData,
                });

                const result = await response.json();

                if (result.success) {
                    successDiv.innerHTML = '<strong>Connection Successful!</strong><br>Anchor setup complete! Your device is now connected and linked to your account.';
                    successDiv.style.display = 'block';
                    
                    // Hide the form and show success message
                    document.getElementById('configForm').style.display = 'none';
                    
                    console.log('Setup successful, redirect URL:', result.redirectUrl);
                    
                    // Redirect to web app after a short delay
                    setTimeout(function() {
                        if (result.redirectUrl) {
                            console.log('Redirecting to:', result.redirectUrl);
                            window.location.href = result.redirectUrl;
                        } else {
                            console.log('No redirect URL provided');
                        }
                    }, 3000);
                } else {
                    errorDiv.textContent = result.message || 'Setup failed';
                    errorDiv.style.display = 'block';
                }
            } catch (error) {
                errorDiv.textContent = 'Network error. Please try again.';
                errorDiv.style.display = 'block';
            } finally {
                loadingDiv.style.display = 'none';
            }
        });
    </script>
</body>
</html>
  )";
  webServer.send(200, "text/html", html);
}



void handleScanWiFi() {
  Serial.println("WiFi scan endpoint accessed");
  
  if (webServer.method() == HTTP_GET) {
    // Perform WiFi scan
    int n = WiFi.scanNetworks();
    
    // Create JSON response
    String json = "[";
    for (int i = 0; i < n; i++) {
      if (i > 0) json += ",";
      json += "{";
      json += "\"ssid\":\"" + WiFi.SSID(i) + "\",";
      json += "\"rssi\":" + String(WiFi.RSSI(i)) + ",";
      json += "\"encryption\":\"" + String((WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? "Open" : "Secured") + "\"";
      json += "}";
    }
    json += "]";
    
    webServer.send(200, "application/json", json);
    Serial.printf("WiFi scan completed: %d networks found\n", n);
  } else {
    webServer.send(405, "text/plain", "Method Not Allowed");
  }
}

void handleSetupAnchor() {
  Serial.println("Setup anchor endpoint accessed");
  if (webServer.method() == HTTP_POST) {
    String email = webServer.arg("email");
    String ssidParam = webServer.arg("ssid");
    String passwordParam = webServer.arg("password");
    
    if (email.length() > 0 && ssidParam.length() > 0 && passwordParam.length() > 0) {
      Serial.printf("Setup request received - Email: %s, SSID: %s\n", email.c_str(), ssidParam.c_str());
      Serial.printf("Attempting to connect to: %s with password length: %d\n", ssidParam.c_str(), passwordParam.length());
      
      // Store credentials
      ssidParam.toCharArray(::ssid, sizeof(::ssid));
      passwordParam.toCharArray(::password, sizeof(::password));
      email.toCharArray(::username, sizeof(::username));
      
      // Attempt to connect to WiFi (from portal)
      connectToWiFi(ssidParam.c_str(), passwordParam.c_str(), true);
      
              if (connectedToWifi) {
          // Reset WiFi attempt counter on successful connection
          resetWiFiAttemptCounter();
          
          // Save credentials and user info following the same structure
          Serial.println("Saving new WiFi credentials...");
          
          // If we have existing credentials, save them as backup
          if (pref.isKey("wifiSSID") && pref.isKey("wifiPass")) {
              String oldSSID = pref.getString("wifiSSID");
              String oldPass = pref.getString("wifiPass");
              pref.putString("backupSSID", oldSSID);
              pref.putString("backupPassword", oldPass);
              Serial.printf("Saved old credentials as backup: %s\n", oldSSID.c_str());
          }
          
          // Save new credentials as primary
          pref.putString("wifiSSID", ssidParam);
          pref.putString("wifiPass", passwordParam);
          pref.putString("username", email);
          pref.putString("userEmail", email);
          Serial.printf("Saved new primary credentials: %s\n", ssidParam.c_str());
          
          Serial.printf("Anchor setup successful - Email: %s, UUID: %s\n", 
                                               email.c_str(), uuidStr);
        
        // Send success response
        String redirectUrl = "https://anchorcontrol.web.app";
        String responseJson = "{\"success\": true, \"message\": \"Anchor setup successful\", \"redirectUrl\": \"" + redirectUrl + "\"}";
        Serial.printf("Sending success response: %s\n", responseJson.c_str());
        webServer.sendHeader("Access-Control-Allow-Origin", "*");
        webServer.sendHeader("Access-Control-Allow-Methods", "POST, OPTIONS");
        webServer.sendHeader("Access-Control-Allow-Headers", "Content-Type");
        webServer.send(200, "application/json", responseJson);
        
        // Disconnect AP after successful setup
        Serial.println("Anchor setup complete. Disconnecting AP...");
        WiFi.softAPdisconnect(true);
        
      } else {
        // Connection failed - send error response but keep portal open
        Serial.println("WiFi connection failed - keeping portal open for retry");
        
        // Switch back to AP mode to keep portal active
        Serial.println("Switching back to AP mode to keep portal active");
        WiFi.mode(WIFI_AP);
        delay(500);
        WiFi.softAP(AP_SSID);
        delay(500);
        
        // Restart DNS server for captive portal
        IPAddress myIP = WiFi.softAPIP();
        dnsServer.stop();
        dnsServer.start(DNS_PORT, "*", myIP);
        
        webServer.sendHeader("Access-Control-Allow-Origin", "*");
        webServer.sendHeader("Access-Control-Allow-Methods", "POST, OPTIONS");
        webServer.sendHeader("Access-Control-Allow-Headers", "Content-Type");
        webServer.send(400, "application/json", "{\"success\": false, \"message\": \"Failed to connect to WiFi network. Please check your credentials and try again.\"}");
      }
      
    } else {
      webServer.sendHeader("Access-Control-Allow-Origin", "*");
      webServer.sendHeader("Access-Control-Allow-Methods", "POST, OPTIONS");
      webServer.sendHeader("Access-Control-Allow-Headers", "Content-Type");
      webServer.send(400, "application/json", "{\"success\": false, \"message\": \"Email, SSID, and password are required\"}");
    }
  } else if (webServer.method() == HTTP_OPTIONS) {
    webServer.sendHeader("Access-Control-Allow-Origin", "*");
    webServer.sendHeader("Access-Control-Allow-Methods", "POST, OPTIONS");
    webServer.sendHeader("Access-Control-Allow-Headers", "Content-Type");
    webServer.send(200);
  } else {
    webServer.sendHeader("Access-Control-Allow-Origin", "*");
    webServer.sendHeader("Access-Control-Allow-Methods", "POST, OPTIONS");
    webServer.sendHeader("Access-Control-Allow-Headers", "Content-Type");
    webServer.send(405, "application/json", "{\"success\": false, \"message\": \"Method not allowed\"}");
  }
}



void handleNotFound() {
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += webServer.uri();
  message += "\nMethod: ";
  message += (webServer.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += webServer.args();
  message += "\n";
  for (uint8_t i = 0; i < webServer.args(); i++) {
    message += " " + webServer.argName(i) + ": " + webServer.arg(i) + "\n";
  }
  webServer.send(404, "text/plain", message);
}

// Motor debouncing constants
const unsigned long MOTOR_STATE_CHECK_INTERVAL = 100; // Check motor state every 100ms

// Watchdog timer for detecting stuck states
const unsigned long WATCHDOG_TIMEOUT = 300000; // 5 minutes
unsigned long lastWatchdogReset = 0;



void resetWatchdog() {
    lastWatchdogReset = millis();
}

// Function to manually reset water change flags
void resetManualWaterChangeFlags() {
    manualWater.waterIn = false;
    manualWater.waterOut = false;
    manualWater.completed = false;
    manualWater.manuallyReset = true;
    manualWater.stopWaterChange = false; // Reset stop water change flag
    Serial.println("Manual water change flags manually reset");
}

void checkWatchdog() {
    unsigned long currentMillis = millis();
    
    // Check if water change has been active too long without progress
    if (waterChange.active && (currentMillis - lastWaterChangeActivity) > WATCHDOG_TIMEOUT) {
        Serial.println("WATCHDOG: Water change stuck - forcing reset");
        Serial.printf("Water change active for %lu ms, timeout is %lu ms\n", 
                    currentMillis - lastWaterChangeActivity, WATCHDOG_TIMEOUT);
        if (xSemaphoreTake(waterChange.mutex, pdMS_TO_TICKS(100))) {
            strcpy(waterChange.status, "watchdog_timeout");
            waterChange.needsProgressUpdate = true;
            firebaseNeedsUpdate = true;
            xSemaphoreGive(waterChange.mutex);
        }
        endWaterChange();
        return;
    }
    
    // Check if system has been running too long without activity
    if ((currentMillis - lastWatchdogReset) > WATCHDOG_TIMEOUT * 2) {
        Serial.println("WATCHDOG: System appears stuck - resetting watchdog");
        Serial.printf("System running for %lu ms, timeout is %lu ms\n", 
                    currentMillis - lastWatchdogReset, WATCHDOG_TIMEOUT * 2);
        resetWatchdog();
    }
}

// Robust motor control with debouncing and safety checks
void setMotorState(bool waterIn, bool waterOut) {
    unsigned long currentMillis = millis();
    
    // Get water change state with mutex protection
    bool motorsShouldRun = false;
    if (xSemaphoreTake(waterChange.mutex, pdMS_TO_TICKS(10))) {
        motorsShouldRun = waterChange.active && strcmp(waterChange.status, "running") == 0;
        xSemaphoreGive(waterChange.mutex);
    }
    
    // Check E-stop status - motors should not run if E-stop is pressed
    int currentEstopState = readEffectiveEstopPin();
    bool estopOk = (currentEstopState == HIGH);
    
    // Update estopStatus if it changed
    if (currentEstopState != estopState) {
        estopState = currentEstopState;
        strcpy(estopStatus, estopState ? "OK" : "STOP");
        Serial.printf("E-Stop state changed: %s\n", estopStatus);
        
        // If E-stop was pressed (transitioned to LOW), set the flag for processing
        if (currentEstopState == LOW) {
            estopPressed = true;
            estopPressTime = currentMillis;
            Serial.println("E-Stop pressed during water change - flag set for processing");
        }
    }
    
    // Check if enough time has passed since last motor toggle
    bool canToggleWaterIn = (currentMillis - motorState.lastWaterInToggle) >= MOTOR_DEBOUNCE_TIME;
    bool canToggleWaterOut = (currentMillis - motorState.lastWaterOutToggle) >= MOTOR_DEBOUNCE_TIME;
    
    // Determine desired motor states - E-stop takes priority
    bool desiredWaterIn = waterIn && motorsShouldRun && estopOk;
    bool desiredWaterOut = waterOut && motorsShouldRun && estopOk;
    
    // Only change motor state if it's different and debounce time has passed
    if (desiredWaterIn != motorState.waterInRunning && canToggleWaterIn) {
        digitalWrite(MTR_WATER_CHANGE_IN, desiredWaterIn ? HIGH : LOW);
        motorState.waterInRunning = desiredWaterIn;
        motorState.lastWaterInToggle = currentMillis;
        Serial.printf("Motor IN: %s\n", desiredWaterIn ? "ON" : "OFF");
    }
    
    if (desiredWaterOut != motorState.waterOutRunning && canToggleWaterOut) {
        digitalWrite(MTR_WATER_CHANGE_OUT, desiredWaterOut ? HIGH : LOW);
        motorState.waterOutRunning = desiredWaterOut;
        motorState.lastWaterOutToggle = currentMillis;
        Serial.printf("Motor OUT: %s\n", desiredWaterOut ? "ON" : "OFF");
    }
    
    // Update activity timestamp if motors are running
    if (motorState.waterInRunning || motorState.waterOutRunning) {
        lastWaterChangeActivity = currentMillis;
    }
}

void processEStop() {
    static unsigned long lastEstopCheck = 0;
    const unsigned long ESTOP_CHECK_INTERVAL = 100; // Check every 100ms
    
    if (millis() - lastEstopCheck < ESTOP_CHECK_INTERVAL) return;
    lastEstopCheck = millis();
    
    if (estopPressed) {
        Serial.println("E-Stop pressed - processing emergency stop");
        // Check if water change is active
        bool waterChangeActive = false;
        bool isManual = false;
        float actualVolumeProcessed = 0.0;
        bool wasWaterOut = false;
        bool wasWaterIn = false;
        
        if (xSemaphoreTake(waterChange.mutex, pdMS_TO_TICKS(10))) {
            waterChangeActive = waterChange.active;
            if (waterChangeActive) {
                isManual = waterChange.manual;
                actualVolumeProcessed = waterChange.actualVolumeProcessed;
                
                // Determine direction for manual water changes
                if (isManual) {
                    wasWaterOut = (previousValues.waterOutDosed > 0 && previousValues.waterInDosed == 0);
                    wasWaterIn = (previousValues.waterInDosed > 0 && previousValues.waterOutDosed == 0);
                    
                    // Fallback: decide based on manual flags snapshot
                    if (!wasWaterOut && !wasWaterIn) {
                        if (manualWater.waterOut) {
                            wasWaterOut = true;
                        } else if (manualWater.waterIn) {
                            wasWaterIn = true;
                        }
                    }
                }
            }
            xSemaphoreGive(waterChange.mutex);
        }
        
        if (waterChangeActive) {
            Serial.println("E-Stop: Water change active - stopping water change immediately");
            
            // Update dosed_amounts with volume processed before stopping using new approach
            float actualVolumeOut = 0.0;
            float actualVolumeIn = 0.0;
            float previousValuesOut = 0.0;
            float previousValuesIn = 0.0;
            
            // Get current processed volumes and previous values
            if (xSemaphoreTake(waterChange.mutex, pdMS_TO_TICKS(100))) {
                actualVolumeOut = waterChange.processedVolumeOutPhase;
                actualVolumeIn = waterChange.processedVolumeInPhase;
                xSemaphoreGive(waterChange.mutex);
            }
            
            if (xSemaphoreTake(doseMutex, pdMS_TO_TICKS(100))) {
                previousValuesOut = previousValues.waterOutDosed;
                previousValuesIn = previousValues.waterInDosed;
                xSemaphoreGive(doseMutex);
            }
            
            if (actualVolumeOut > 0 || actualVolumeIn > 0) {
                Serial.printf("E-Stop: Updating dosed_amounts - actualVolumeOut: %.3f, actualVolumeIn: %.3f gallons\n", 
                            actualVolumeOut, actualVolumeIn);
                Serial.printf("E-Stop: Previous values - previousValuesOut: %.3f, previousValuesIn: %.3f gallons\n", 
                            previousValuesOut, previousValuesIn);
                
                if (xSemaphoreTake(doseMutex, pdMS_TO_TICKS(100))) {
                    // NEW APPROACH: Update dosed amounts with actualVolume + previousValues
                    if (actualVolumeOut > 0) {
                        float totalOutVolume = actualVolumeOut + previousValuesOut;
                        dosedAmounts.water_out += totalOutVolume;
                        Serial.printf("E-Stop: Added %.3f gallons to water_out dosed amount (actualVolume: %.3f + previousValues: %.3f, total: %.3f)\n", 
                                    totalOutVolume, actualVolumeOut, previousValuesOut, dosedAmounts.water_out);
                    }
                    
                    if (actualVolumeIn > 0) {
                        float totalInVolume = actualVolumeIn + previousValuesIn;
                        dosedAmounts.water_in += totalInVolume;
                        Serial.printf("E-Stop: Added %.3f gallons to water_in dosed amount (actualVolume: %.3f + previousValues: %.3f, total: %.3f)\n", 
                                    totalInVolume, actualVolumeIn, previousValuesIn, dosedAmounts.water_in);
                    }
                    
                    firebaseNeedsUpdate = true;
                    xSemaphoreGive(doseMutex);
                }
            }
            
            // Update water change status
            if (xSemaphoreTake(waterChange.mutex, pdMS_TO_TICKS(100))) {
                strcpy(waterChange.status, "estop_interrupted");
                waterChange.needsProgressUpdate = true;
                firebaseNeedsUpdate = true;
                xSemaphoreGive(waterChange.mutex);
            }
            
            // Stop the water change
            endWaterChange(true);
        } else {
            Serial.println("E-Stop: No water change active");
        }
        
        // Clear the flag
        estopPressed = false;
        estopChanged = false;
        Serial.println("E-Stop processed - flags cleared");
    }
}

// Error recovery and state validation
void validateWaterChangeState() {
    if (!waterChange.active) return;
    
    unsigned long currentMillis = millis();
    
        // Check for invalid states
        if (waterChange.manual) {
            // Manual water change should have either waterIn or waterOut active
            if (!manualWater.waterIn && !manualWater.waterOut) {
                Serial.println("ERROR: Manual water change has no direction set");
                if (xSemaphoreTake(waterChange.mutex, pdMS_TO_TICKS(100))) {
                    strcpy(waterChange.status, "error_no_direction");
                    waterChange.needsProgressUpdate = true;
                    firebaseNeedsUpdate = true;
                    xSemaphoreGive(waterChange.mutex);
                }
                endWaterChange();
                return;
            }
        } else {
            // Scheduled water change should have valid phase
            if (waterChange.outPhaseCompleted && (currentMillis - waterChange.phaseStartMillis) > 3600000) {
                Serial.println("ERROR: Scheduled water change phase stuck");
                Serial.printf("Phase running for %lu ms, timeout is 3600000 ms\n", 
                            currentMillis - waterChange.phaseStartMillis);
                if (xSemaphoreTake(waterChange.mutex, pdMS_TO_TICKS(100))) {
                    strcpy(waterChange.status, "error_phase_stuck");
                    waterChange.needsProgressUpdate = true;
                    firebaseNeedsUpdate = true;
                    xSemaphoreGive(waterChange.mutex);
                }
                endWaterChange();
                return;
            }
        }
    
    // Check for motor state inconsistencies
    bool motorInRunning = digitalRead(MTR_WATER_CHANGE_IN) == HIGH;
    bool motorOutRunning = digitalRead(MTR_WATER_CHANGE_OUT) == HIGH;
    
    if (motorInRunning && motorOutRunning) {
        Serial.println("ERROR: Both motors running simultaneously - forcing stop");
        Serial.printf("Motor IN: %s, Motor OUT: %s\n", 
                    motorInRunning ? "ON" : "OFF", motorOutRunning ? "ON" : "OFF");
        if (xSemaphoreTake(waterChange.mutex, pdMS_TO_TICKS(100))) {
            strcpy(waterChange.status, "error_both_motors");
            waterChange.needsProgressUpdate = true;
            firebaseNeedsUpdate = true;
            xSemaphoreGive(waterChange.mutex);
        }
        endWaterChange();
        return;
    }
}

// Simplified recovery function to save memory
void recoverWaterChangeState() {
    digitalWrite(MTR_WATER_CHANGE_IN, LOW);
    digitalWrite(MTR_WATER_CHANGE_OUT, LOW);
    motorState.waterInRunning = false;
    motorState.waterOutRunning = false;
    manualWater.waterIn = false;
    manualWater.waterOut = false;
    manualWater.completed = false;
    manualWater.stopWaterChange = false; // Reset stop water change flag
    isWaterChangeActive = false;
    isManualWaterChange = false;
}

// Simplified error handling to save memory
void handleSystemErrors() {
    static unsigned long lastErrorCheck = 0;
    if (millis() - lastErrorCheck < 10000) return; // Check every 10 seconds
    lastErrorCheck = millis();
    
    // Check if both motors are running (should never happen)
    if (digitalRead(MTR_WATER_CHANGE_IN) == HIGH && digitalRead(MTR_WATER_CHANGE_OUT) == HIGH) {
        Serial.println("ERROR: Both motors running simultaneously");
        Serial.printf("Motor IN: %s, Motor OUT: %s\n", 
                    digitalRead(MTR_WATER_CHANGE_IN) == HIGH ? "ON" : "OFF",
                    digitalRead(MTR_WATER_CHANGE_OUT) == HIGH ? "ON" : "OFF");
        recoverWaterChangeState();
        Serial.println("System error recovered - motors stopped");
    }
}

// --- Watch-face helpers and renderers ---
// Draw outer/inner decorative rings for watch-face look
void drawWatchRings() {
    int cx = TFT_display.width() / 2;
    int cy = TFT_display.height() / 2;
    int rOuter = min(TFT_display.width(), TFT_display.height()) / 2 - 2;
    int rInner = rOuter - 6;
    TFT_display.drawCircle(cx, cy, rOuter, WHITE);
    TFT_display.drawCircle(cx, cy, rInner, WHITE);
}

// Draw rotating pulsating circle animation around the edge
void drawRotatingPulsatingCircle() {
    const int cx = TFT_display.width() / 2;
    const int cy = TFT_display.height() / 2;
    const int radius = min(TFT_display.width(), TFT_display.height()) / 2 - 8;
    
    // Update animation timing (every 50ms for smooth animation)
    unsigned long currentTime = millis();
    if (currentTime - displayState.lastAnimationUpdate >= 50) {
        displayState.lastAnimationUpdate = currentTime;
        
        // Rotate the circle (360 degrees over 3 seconds)
        displayState.animationAngle += 0.06; // 360/6000 * 50ms
        if (displayState.animationAngle >= 2 * PI) {
            displayState.animationAngle -= 2 * PI;
        }
        
        // Pulsate the circle size (1.5 second cycle)
        displayState.pulsePhase += 0.21; // 2*PI/3000 * 50ms
        if (displayState.pulsePhase >= 2 * PI) {
            displayState.pulsePhase -= 2 * PI;
        }
    }
    
    // Calculate pulsating radius (varies between 3 and 8 pixels)
    float pulseFactor = (sin(displayState.pulsePhase) + 1.0) / 2.0; // 0 to 1
    int circleRadius = 3 + (int)(5 * pulseFactor);
    
    // Calculate position of the rotating circle
    int x = cx + (int)(radius * cos(displayState.animationAngle));
    int y = cy + (int)(radius * sin(displayState.animationAngle));
    
    // Draw the pulsating circle in bright green
    TFT_display.fillCircle(x, y, circleRadius, GREEN);
    
    // Add a smaller white center for extra visibility
    TFT_display.fillCircle(x, y, circleRadius / 2, WHITE);
}

// Simple WiFi icon with 3 arcs
void drawWifiIcon(int cx, int cy, uint16_t color) {
    TFT_display.drawCircle(cx, cy, 10, color);
    TFT_display.drawCircle(cx, cy, 7, color);
    TFT_display.drawCircle(cx, cy, 4, color);
    TFT_display.fillCircle(cx, cy, 2, color);
}

void renderWatchFaceIdle() {
    const int cx = TFT_display.width() / 2;
    const int cy = TFT_display.height() / 2;

    // Check for E-stop status first
    if (strcmp(estopStatus, "STOP") == 0) {
        // E-stop is active - show red screen with E-stop message
        if (!displayState.displayInitialized || strcmp(displayState.lastAnchorName, "ESTOP") != 0) {
            // Clear screen and show E-stop display
            TFT_display.fillScreen(RED);
            drawWatchRings();
            
            // Show E-stop message
            printCenteredLine(cy - 20, "E-STOP", 3, WHITE);
            printCenteredLine(cy + 20, "ACTIVE", 2, WHITE);
            
            // Update display state to remember we're showing E-stop
            strcpy(displayState.lastAnchorName, "ESTOP");
            displayState.displayInitialized = true;
        }
        return; // Don't show normal idle display when E-stop is active
    }

    // Initialize display if needed
    if (!displayState.displayInitialized) {
        TFT_display.fillScreen(BLACK);
        drawWatchRings();
        displayState.displayInitialized = true;
    }

    // Top: anchor/name label (increased size by 30%)
    const char* nameToShow = (strlen(anchorName) > 0) ? anchorName : uuidStr;
    updateTextLine(cy - 48, nameToShow, displayState.lastAnchorName, 2, BLUE);
    strncpy(displayState.lastAnchorName, nameToShow, sizeof(displayState.lastAnchorName) - 1);
    displayState.lastAnchorName[sizeof(displayState.lastAnchorName) - 1] = '\0';

    // Center: Temperature beautiful in green, proportional to other text
    char tempBuf[24];
    snprintf(tempBuf, sizeof(tempBuf), "%.1f%cF", temperatureF, char(247));
    char lastTempBuf[24];
    snprintf(lastTempBuf, sizeof(lastTempBuf), "%.1f%cF", displayState.lastTemperatureF, char(247));
    updateTextLine(cy - 5, tempBuf, lastTempBuf, 3, GREEN);
    displayState.lastTemperatureF = temperatureF;

    // Below temperature: Time medium (swapped position)
    struct tm timeinfo;
    char timeStr[6] = "--:--"; // HH:MM
    if (getLocalTime(&timeinfo)) {
        snprintf(timeStr, sizeof(timeStr), "%02d:%02d", timeinfo.tm_hour, timeinfo.tm_min);
    }
    updateTextLine(cy + 24, timeStr, displayState.lastTimeStr, 2, WHITE);
    strncpy(displayState.lastTimeStr, timeStr, sizeof(displayState.lastTimeStr) - 1);
    displayState.lastTimeStr[sizeof(displayState.lastTimeStr) - 1] = '\0';

    // Bottom: WiFi status icon (moved lower)
    bool wifiOk = WiFi.status() == WL_CONNECTED;
    if (wifiOk != displayState.lastWifiStatus) {
        // Clear the WiFi icon area
        clearRect(cx - 12, cy + 52, 24, 24);
        drawWifiIcon(cx, cy + 64, wifiOk ? GREEN : RED);
        displayState.lastWifiStatus = wifiOk;
    }
}

void renderWatchFaceMotor(const char* motorName) {
    const int cx = TFT_display.width() / 2;
    const int cy = TFT_display.height() / 2;

    // Check for E-stop status first
    if (strcmp(estopStatus, "STOP") == 0) {
        // E-stop is active - show red screen with E-stop message
        if (!displayState.displayInitialized || strcmp(displayState.lastAnchorName, "ESTOP") != 0) {
            // Clear screen and show E-stop display
            TFT_display.fillScreen(RED);
            drawWatchRings();
            
            // Show E-stop message
            printCenteredLine(cy - 20, "E-STOP", 3, WHITE);
            printCenteredLine(cy + 20, "ACTIVE", 2, WHITE);
            
            // Update display state to remember we're showing E-stop
            strcpy(displayState.lastAnchorName, "ESTOP");
            displayState.displayInitialized = true;
        }
        return; // Don't show normal motor display when E-stop is active
    }

    // Check if water change is active and if we need to force a full screen reload every 2 minutes
    bool wcActiveLocal = false;
    if (xSemaphoreTake(waterChange.mutex, pdMS_TO_TICKS(10))) {
        wcActiveLocal = waterChange.active && strcmp(waterChange.status, "running") == 0;
        xSemaphoreGive(waterChange.mutex);
    }
    
    unsigned long currentMillis = millis();
    const unsigned long FULL_RELOAD_INTERVAL = 120000; // 2 minutes in milliseconds
    
    // Force full reload every 2 minutes during water change
    bool forceFullReload = false;
    if (wcActiveLocal) {
        // Initialize the reload timer when water change starts
        if (displayState.lastFullReloadTime == 0) {
            displayState.lastFullReloadTime = currentMillis;
        }
        // Check if 2 minutes have passed since last full reload
        if (currentMillis - displayState.lastFullReloadTime >= FULL_RELOAD_INTERVAL) {
            forceFullReload = true;
            displayState.lastFullReloadTime = currentMillis;
            Serial.println("Water change: Forcing full TFT screen reload (2 minute interval)");
        }
    } else {
        // Reset the reload timer when water change is not active
        displayState.lastFullReloadTime = 0;
    }

    // Check if we need to switch from idle to motor display or force full reload
    if (!displayState.lastMotorRunning || forceFullReload) {
        // Switching from idle to motor - clear and redraw everything
        TFT_display.fillScreen(BLACK);
        drawWatchRings();
        displayState.displayInitialized = true;
        
        // Header small
        printCenteredLine(cy - 56, "Motor Active", 1, WHITE);
        
        // Force redraw of all elements by resetting state
        if (forceFullReload) {
            displayState.lastMotorName[0] = '\0';
            displayState.lastRemainingMs = 0;
            displayState.lastRemainingText[0] = '\0';
            displayState.lastTemperatureF = 0.0;
        }
    }

    // Motor name
    char runBuf[40];
    snprintf(runBuf, sizeof(runBuf), "%s", motorName);
    updateTextLine(cy - 28, runBuf, displayState.lastMotorName, 2, YELLOW);
    strncpy(displayState.lastMotorName, motorName, sizeof(displayState.lastMotorName) - 1);
    displayState.lastMotorName[sizeof(displayState.lastMotorName) - 1] = '\0';
    
    // Draw rotating pulsating circle animation
    drawRotatingPulsatingCircle();

    // Time remaining instead of progress dots when water change is running
    unsigned long remainingMsLocal = 0;
    wcActiveLocal = false;
    if (xSemaphoreTake(waterChange.mutex, pdMS_TO_TICKS(10))) {
        wcActiveLocal = waterChange.active && strcmp(waterChange.status, "running") == 0;
        remainingMsLocal = waterChange.remainingMs;
        xSemaphoreGive(waterChange.mutex);
    }
    
    char remainingBuf[16];
    if (wcActiveLocal && remainingMsLocal > 0 && remainingMsLocal < 24UL*60UL*60UL*1000UL) {
        unsigned long seconds = remainingMsLocal / 1000UL;
        unsigned int minutesPart = (seconds / 60UL) % 60UL;
        unsigned int hoursPart = seconds / 3600UL;
        unsigned int secsPart = seconds % 60UL;
        if (hoursPart > 0) {
            snprintf(remainingBuf, sizeof(remainingBuf), "%u:%02u:%02u", hoursPart, minutesPart, secsPart);
        } else {
            snprintf(remainingBuf, sizeof(remainingBuf), "%02u:%02u", minutesPart, secsPart);
        }
    } else {
        // Fallback: show 00:00 if not active or invalid
        strcpy(remainingBuf, "00:00");
    }
    
    // Only update if remaining time changed
    if (remainingMsLocal != displayState.lastRemainingMs) {
        updateTextLine(cy + 4, remainingBuf, displayState.lastRemainingText, 3, GREEN);
        displayState.lastRemainingMs = remainingMsLocal;
        // Update the last remaining text for next comparison
        strncpy(displayState.lastRemainingText, remainingBuf, sizeof(displayState.lastRemainingText) - 1);
        displayState.lastRemainingText[sizeof(displayState.lastRemainingText) - 1] = '\0';
    }

    // Temperature larger and beautiful in green at bottom
    char tempSmall[24];
    snprintf(tempSmall, sizeof(tempSmall), "%.1f%cF", temperatureF, char(247));
    char lastTempSmall[24];
    snprintf(lastTempSmall, sizeof(lastTempSmall), "%.1f%cF", displayState.lastTemperatureF, char(247));
    updateTextLine(cy + 40, tempSmall, lastTempSmall, 2, GREEN);
    displayState.lastTemperatureF = temperatureF;
    
    displayState.lastMotorRunning = true;
}



