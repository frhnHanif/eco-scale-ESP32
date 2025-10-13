#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include <ezButton.h>
#include <HX711_ADC.h>
#include <time.h>
#include <EEPROM.h>
#include <ESP32Ping.h>
#include <esp_task_wdt.h>
#include "credentials.h"

// --- Include library LCDBigNumbers ---
#define USE_SERIAL_2004_LCD
#include "LCDBigNumbers.hpp"

// ==================== KONFIGURASI SISTEM ====================
namespace Config {
  // Timing intervals (ms)
  constexpr unsigned long WEIGHT_READ_INTERVAL    = 50;
  constexpr unsigned long LCD_UPDATE_INTERVAL     = 100;
  constexpr unsigned long WIFI_CHECK_INTERVAL     = 15000;
  constexpr unsigned long SIGNAL_UPDATE_INTERVAL  = 2000;
  constexpr unsigned long INTERNET_CHECK_INTERVAL = 10000;
  constexpr unsigned long STATUS_MSG_DURATION     = 2000; // Durasi pesan "Sukses/Gagal"
  constexpr unsigned long CALIBRATION_VALUE       = 12.01;

  // Weight settings
  constexpr float MIN_WEIGHT_THRESHOLD = 0.1f; // Perubahan minimal untuk update LCD

  // Pin configuration
  constexpr int PIN_TOMBOL_1 = 27;
  constexpr int PIN_TOMBOL_2 = 26;
  constexpr int PIN_TOMBOL_3 = 25;
  constexpr int PIN_TOMBOL_4 = 33;
  constexpr int PIN_BUZZER   = 5;
  constexpr int HX711_DOUT   = 2;
  constexpr int HX711_SCK    = 4;
}

// ==================== STATE MANAGEMENT ====================
enum class AppState {
  IDLE,
  SELECTING_SUBTYPE,
  SENDING_DATA,
  SHOWING_STATUS
};

// ==================== DATA STRUCTURES (NO STRING CLASS) ====================
struct SampahType {
  char jenis[16];
  char subJenis[16];
};

// ==================== GLOBAL OBJECTS ====================
LiquidCrystal_I2C lcd(0x27, LCD_COLUMNS, LCD_ROWS);
LCDBigNumbers bigNumbers(&lcd, BIG_NUMBERS_FONT_2_COLUMN_3_ROWS_VARIANT_2);
HX711_ADC LoadCell(Config::HX711_DOUT, Config::HX711_SCK);
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig firebaseConfig;
ezButton tombol[] = {
    ezButton(Config::PIN_TOMBOL_1), 
    ezButton(Config::PIN_TOMBOL_2), 
    ezButton(Config::PIN_TOMBOL_3), 
    ezButton(Config::PIN_TOMBOL_4)
};

// ==================== GLOBAL VARIABLES ====================
AppState currentState = AppState::IDLE;
SampahType sampah;
char fakultas[8] = "FIB";
bool isOnline = false;

// Weight management
float currentWeight = 0.0;
float lastDisplayedWeight = -1.00;
float weightBuffer[2] = {0.0f, 0.0f};
int bufferIndex = 0;
bool newDataReady = false;

// Timers
unsigned long lastWeightReadTime = 0;
unsigned long lastLCDUpdateTime = 0;
unsigned long lastSignalUpdateTime = 0;
unsigned long lastInternetCheckTime = 0;
unsigned long lastWifiCheckTime = 0;
unsigned long statusMsgTimestamp = 0;

// --- ASET IKON KUSTOM UNTUK STATUS JARINGAN ---
// Diberi nomor indeks agar mudah dipanggil
#define ICON_IDX_SIGNAL_1   0
#define ICON_IDX_SIGNAL_2   1
#define ICON_IDX_SIGNAL_3   2
#define ICON_IDX_SIGNAL_4   3
#define ICON_IDX_NO_INTERNET 4

// Desain karakter kustom (custom characters)
byte wifiSignal_1[] = { B00000, B00000, B00000, B00000, B00000, B00000, B11000, B11000 }; // twentyChar
byte wifiSignal_2[] = { B00000, B00000, B00000, B00000, B00011, B00011, B11011, B11011 }; // fortyChar
byte wifiSignal_3[] = { B00000, B00000, B11000, B11000, B11000, B11000, B11000, B11000 }; // sixtyChar (untuk kolom kanan)
byte wifiSignal_4[] = { B00011, B00011, B11011, B11011, B11011, B11011, B11011, B11011 }; // eightyChar (untuk kolom kanan)
byte noInternetIcon[] = { B10100, B01000, B10100, B00000, B00000, B00000, B11000, B11000 }; // lostInternetChar

// ==================== FUNCTION DECLARATIONS ====================
// --- Core Logic ---
void prosesTombol();
void handleKirimData();
bool sendDataToFirebase();
float readSmoothedWeight();


// --- Setup & Connectivity ---
void initializeSystem();
bool connectWiFi();
bool syncTime();
bool authenticateFirebase();
void manageWifiConnection();

// --- Display ---
void updateWeightDisplay(float weight);
void restoreDefaultDisplay();
void tampilkanSubJenisAnorganik();
void updateStatusIndicators();

// --- Utilities ---
void safeStringCopy(char* dest, const char* src, size_t destSize);
void getTimestampUTC(char* buffer, size_t bufferSize);

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  Serial.println("\nStarting production firmware...");

  esp_task_wdt_init(60, true); 
  esp_task_wdt_add(NULL);
  Serial.println("Watchdog Timer activated.");

  initializeSystem();

  bool setupSuccess = true;
  char failReason[16] = "";

  if (!connectWiFi()) {
    setupSuccess = false;
    safeStringCopy(failReason, "WiFi Gagal!", sizeof(failReason));
  }
  if (setupSuccess && !syncTime()) {
    setupSuccess = false;
    safeStringCopy(failReason, "NTP Gagal!", sizeof(failReason));
  }
  if (setupSuccess && !authenticateFirebase()) {
    setupSuccess = false;
    safeStringCopy(failReason, "Auth Gagal!", sizeof(failReason));
  }

  if (!setupSuccess) {
    lcd.clear();
    lcd.setCursor(0, 1); lcd.print("Setup Gagal:");
    lcd.setCursor(0, 2); lcd.print(failReason);
    tone(Config::PIN_BUZZER, 500, 1000);
    while (true) { 
      esp_task_wdt_reset(); // Tetap reset WDT agar tidak restart di state error
      delay(1000); 
    }
  }

  Serial.println("\n--- Sistem Siap ---");
  lcd.clear();
  restoreDefaultDisplay();
  updateWeightDisplay(0.0);

  lastWeightReadTime = millis();
  lastLCDUpdateTime = millis();
}

// ==================== MAIN LOOP (STATE MACHINE) ====================
void loop() {
  esp_task_wdt_reset();

  // Functions that must always run
  for (int i = 0; i < 4; i++) {
    tombol[i].loop();
  }
  manageWifiConnection();

  // --- STATE MACHINE ---
  switch (currentState) {
    case AppState::IDLE: {
      // Baca berat & update display
      unsigned long currentMillis = millis();
      if (LoadCell.update()) { newDataReady = true; }
      if (newDataReady && currentMillis - lastWeightReadTime >= Config::WEIGHT_READ_INTERVAL) {
        currentWeight = readSmoothedWeight();
        lastWeightReadTime = currentMillis;
        newDataReady = false;
      }
      if (currentMillis - lastLCDUpdateTime >= Config::LCD_UPDATE_INTERVAL) {
        if (abs(currentWeight - lastDisplayedWeight) > Config::MIN_WEIGHT_THRESHOLD || lastDisplayedWeight == -1.00) {
          updateWeightDisplay(currentWeight);
          lastDisplayedWeight = currentWeight;
        }
        lastLCDUpdateTime = currentMillis;
      }

      // Cek input tombol
      prosesTombol();
      handleKirimData(); // This function will change the state if button 4 is pressed

      // Update status indicators
      updateStatusIndicators();
      break;
    }

    case AppState::SELECTING_SUBTYPE: {
      // Hanya cek input tombol saat di state ini
      prosesTombol();
      break;
    }

    case AppState::SENDING_DATA: {
      // State ini sangat singkat, handleKirimData akan langsung mengubah ke SHOWING_STATUS
      // Tidak ada yang perlu dilakukan di sini
      break;
    }

    case AppState::SHOWING_STATUS: {
      // Tahan pesan "Sukses/Gagal" selama durasi yang ditentukan
      if (millis() - statusMsgTimestamp > Config::STATUS_MSG_DURATION) {
        restoreDefaultDisplay();
        currentState = AppState::IDLE;
      }
      break;
    }
  }
}

// ==================== IMPLEMENTATION DETAILS ====================

// --- Setup & Connectivity ---
void initializeSystem() {
  safeStringCopy(sampah.jenis, "--", sizeof(sampah.jenis));
  safeStringCopy(sampah.subJenis, "--", sizeof(sampah.subJenis));
  
  pinMode(Config::PIN_BUZZER, OUTPUT);
  
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.createChar(ICON_IDX_SIGNAL_1, wifiSignal_1);
  lcd.createChar(ICON_IDX_SIGNAL_2, wifiSignal_2);
  lcd.createChar(ICON_IDX_SIGNAL_3, wifiSignal_3);
  lcd.createChar(ICON_IDX_SIGNAL_4, wifiSignal_4);
  lcd.createChar(ICON_IDX_NO_INTERNET, noInternetIcon);
  bigNumbers.begin();
  
  LoadCell.begin();
  float calibrationValue = Config::CALIBRATION_VALUE;
  EEPROM.begin(512);
  LoadCell.start(2000, true);
  if (LoadCell.getTareTimeoutFlag()) {
    lcd.clear(); lcd.print("HX711 Error!"); while (1);
  } else {
    LoadCell.setCalFactor(calibrationValue);
    LoadCell.setSamplesInUse(1);
    Serial.println("Startup is complete");
  }
}

bool connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  lcd.clear();
  lcd.setCursor(0, 1); lcd.print("Connecting to WiFi");
  
  int dotCount = 0;
  while (WiFi.status() != WL_CONNECTED && dotCount < 30) {
    lcd.setCursor(0, 2);
    for(int i=0; i<dotCount%4; i++) lcd.print(".");
    for(int i=dotCount%4; i<4; i++) lcd.print(" ");
    Serial.print(".");
    delay(500);
    dotCount++;
    esp_task_wdt_reset();
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\nWiFi connection failed");
    return false;
  }
  
  Serial.println("\nWiFi connected successfully");
  delay(1000); // Wait for network stack to stabilize
  return true;
}

bool syncTime() {
  Serial.print("Synchronizing time...");
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo, 10000)) {
    Serial.println("\nTime synchronization failed");
    return false;
  }
  Serial.println("\nTime synchronized successfully");
  return true;
}

bool authenticateFirebase() {
  firebaseConfig.api_key = API_KEY;
  // Jika perlu, aktifkan sertifikat untuk menghemat RAM
  // firebaseConfig.cert.data = root_ca_google;
  Firebase.begin(&firebaseConfig, &auth);
  Firebase.reconnectWiFi(true);
  
  Serial.println("Authenticating with Firebase...");
  if (!Firebase.signUp(&firebaseConfig, &auth, "", "")) {
    Serial.println("\nFirebase authentication failed");
    return false;
  }
  
  Serial.println("Firebase authentication successful");
  return true;
}

void manageWifiConnection() {
  if (millis() - lastWifiCheckTime >= Config::WIFI_CHECK_INTERVAL) {
    if (WiFi.status() != WL_CONNECTED && WiFi.getMode() == WIFI_STA) {
      Serial.println("WiFi disconnected! Triggering reconnect...");
      WiFi.reconnect();
    }
    lastWifiCheckTime = millis();
  }
}

// --- Core Logic ---
float readSmoothedWeight() {
  float rawWeight = LoadCell.getData();
  float weightInKg = rawWeight / 1000.0;
  
  if (weightInKg < 0.05) { weightInKg = 0; }
  
  weightBuffer[bufferIndex] = weightInKg;
  bufferIndex = (bufferIndex + 1) % 2;
  
  return (weightBuffer[0] + weightBuffer[1]) / 2.0f;
}

void prosesTombol() {
  if (tombol[0].isPressed()) {
    if (currentState == AppState::IDLE) {
      safeStringCopy(sampah.jenis, "Organik", sizeof(sampah.jenis));
      safeStringCopy(sampah.subJenis, "--", sizeof(sampah.subJenis));
      tone(Config::PIN_BUZZER, 2500, 100);
      restoreDefaultDisplay();
    } else if (currentState == AppState::SELECTING_SUBTYPE) {
      safeStringCopy(sampah.jenis, "Anorganik", sizeof(sampah.jenis));
      safeStringCopy(sampah.subJenis, "Umum", sizeof(sampah.subJenis));
      tone(Config::PIN_BUZZER, 2500, 100);
      restoreDefaultDisplay();
      currentState = AppState::IDLE;
    }
  } else if (tombol[1].isPressed()) {
    if (currentState == AppState::IDLE) {
      currentState = AppState::SELECTING_SUBTYPE;
      tampilkanSubJenisAnorganik();
    } else if (currentState == AppState::SELECTING_SUBTYPE) {
      safeStringCopy(sampah.jenis, "Anorganik", sizeof(sampah.jenis));
      safeStringCopy(sampah.subJenis, "Botol", sizeof(sampah.subJenis));
      tone(Config::PIN_BUZZER, 2500, 100);
      restoreDefaultDisplay();
      currentState = AppState::IDLE;
    }
  } else if (tombol[2].isPressed()) {
    if (currentState == AppState::IDLE) {
      safeStringCopy(sampah.jenis, "Residu", sizeof(sampah.jenis));
      safeStringCopy(sampah.subJenis, "--", sizeof(sampah.subJenis));
      tone(Config::PIN_BUZZER, 2500, 100);
      restoreDefaultDisplay();
    } else if (currentState == AppState::SELECTING_SUBTYPE) {
      safeStringCopy(sampah.jenis, "Anorganik", sizeof(sampah.jenis));
      safeStringCopy(sampah.subJenis, "Kertas", sizeof(sampah.subJenis));
      tone(Config::PIN_BUZZER, 2500, 100);
      restoreDefaultDisplay();
      currentState = AppState::IDLE;
    }
  }
}

void handleKirimData() {
  // Pastikan aksi kirim hanya bisa dimulai dari state IDLE
  if (currentState != AppState::IDLE) {
    return;
  }

  if (tombol[3].isPressed()) {
    tone(Config::PIN_BUZZER, 2000, 300);
    currentState = AppState::SENDING_DATA; // Ubah state untuk menandakan proses dimulai
    
    lcd.setCursor(0, 0);
    
    // --- PERUBAHAN LOGIKA UTAMA DI SINI ---
    
    // KONDISI 1: Jenis sampah BELUM dipilih
    if (strcmp(sampah.jenis, "--") == 0) {
      lcd.print("Error: Pilih Jenis!   ");
    
    // KONDISI 2: Jenis sampah SUDAH dipilih
    } else {
      lcd.print("Status: Mengirim... ");
      bool success = sendDataToFirebase();
      
      // Tampilkan hasilnya
      lcd.setCursor(0, 0);
      lcd.print(success ? "Status: Sukses!      " : "Status: Gagal!        ");
      
      // Jika sukses, reset pilihan sampah
      if (success) {
        safeStringCopy(sampah.jenis, "--", sizeof(sampah.jenis));
        safeStringCopy(sampah.subJenis, "--", sizeof(sampah.subJenis));
      }
    }
    
    // Bagian ini akan berjalan untuk KEDUA kondisi di atas,
    // untuk menahan pesan di layar selama beberapa saat.
    statusMsgTimestamp = millis(); // Atur timer untuk pesan status
    currentState = AppState::SHOWING_STATUS; // Ubah state untuk menampilkan pesan
  }
}

bool sendDataToFirebase() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Send failed: WiFi disconnected!");
    return false;
  }
  
  char timestampBuffer[24];
  getTimestampUTC(timestampBuffer, sizeof(timestampBuffer));
  if (strlen(timestampBuffer) == 0) {
    Serial.println("Send failed: invalid timestamp (NTP sync issue).");
    return false;
  }

  FirebaseJson content;
  content.set("fields/berat/doubleValue", String(currentWeight, 2)); // String is ok for short-lived object

  char jenisLengkap[16];
  if (strcmp(sampah.jenis, "Anorganik") == 0 && strcmp(sampah.subJenis, "--") != 0) {
    if (strcmp(sampah.subJenis, "Umum") == 0){
      safeStringCopy(jenisLengkap, "Anorganik", sizeof(jenisLengkap));
    } else {
      safeStringCopy(jenisLengkap, sampah.subJenis, sizeof(jenisLengkap));
    }
  } else {
    safeStringCopy(jenisLengkap, sampah.jenis, sizeof(jenisLengkap));
  }
  content.set("fields/jenis/stringValue", jenisLengkap);
  content.set("fields/fakultas/stringValue", fakultas);
  content.set("fields/timestamp/timestampValue", timestampBuffer);

  Serial.printf("Sending: %.2f kg, %s\n", currentWeight, jenisLengkap);

  if (Firebase.Firestore.createDocument(&fbdo, FIREBASE_PROJECT_ID, "", "sampah", content.raw())) {
    return true;
    Serial.print("Send success");
  } else {
    Serial.print("Send failed: "); Serial.println(fbdo.errorReason());
    return false;
  }
}

// --- Display ---
void tampilkanSubJenisAnorganik() {
  lcd.clear();
  lcd.setCursor(2, 0);
  lcd.print("Pilih Sub-jenis:");
  lcd.setCursor(0, 1);
  lcd.print(" 1.Umum     2.Botol");
  lcd.setCursor(0, 2);
  lcd.print(" 3.Kertas");
}

void restoreDefaultDisplay() {
  lcd.clear();
  lcd.setCursor(0, 0);

  char displayText[21];
  if (strcmp(sampah.jenis, "Anorganik") == 0 && strcmp(sampah.subJenis, "--") != 0) {
    if (strcmp(sampah.subJenis, "Umum") == 0){
      snprintf(displayText, sizeof(displayText), "Jenis: Anorganik");
    } else {
      snprintf(displayText, sizeof(displayText), "Jenis: %s", sampah.subJenis);
    }
  } else {
    snprintf(displayText, sizeof(displayText), "Jenis: %s", sampah.jenis);
  }
  lcd.print(displayText);

  lcd.setCursor(17, 1);
  lcd.print("kg");
  lastDisplayedWeight = -1.00;
}

void updateWeightDisplay(float weight) {
  char weightString[10];
  snprintf(weightString, sizeof(weightString), "%6.2f", weight);
  bigNumbers.setBigNumberCursor(1, 1);
  bigNumbers.print(weightString);
}

// Ganti nama fungsi agar lebih sesuai
void updateStatusIndicators() {
  // --- Timer untuk pembaruan TAMPILAN (lebih cepat) ---
  const unsigned long DISPLAY_UPDATE_INTERVAL = 2000; // Update 2x per detik untuk kedipan
  static unsigned long lastDisplayUpdateTime = 0;
  static bool blinkerState = false; // Menyimpan status kedip (on/off)

  if (millis() - lastDisplayUpdateTime >= DISPLAY_UPDATE_INTERVAL) {
    
    // --- Bagian Sinyal WiFi (tetap sama, tapi disatukan di sini) ---
    char signalText[5];
    if (WiFi.status() == WL_CONNECTED) {
      long rssi = WiFi.RSSI();
      snprintf(signalText, sizeof(signalText), "%3ld", rssi);
    } else {
      safeStringCopy(signalText, "!!!", sizeof(signalText));
    }
    lcd.setCursor(17, 3); 
    lcd.print(signalText);

    // --- Bagian Status Internet (dengan logika kedip) ---
    char statusSymbol = ' '; // Defaultnya kosong
    if (WiFi.status() == WL_CONNECTED) {
      if (isOnline) {
        blinkerState = !blinkerState;
        statusSymbol = blinkerState? '\0' : ' '; // Ikon awan jika online
      } else {
        // Jika offline, gunakan state kedip untuk menampilkan 'X' atau spasi
        blinkerState = !blinkerState; // Bolak-balik state (true/false)
        statusSymbol = blinkerState ? 'X' : ' ';
      }
    }
    lcd.setCursor(0, 3); // Pindahkan kursor ke posisi ikon internet
    lcd.print(statusSymbol);
    
    lastDisplayUpdateTime = millis(); // Reset timer tampilan
  }

  // --- Bagian Pengecekan INTERNET (tetap lambat) ---
  const unsigned long INTERNET_CHECK_INTERVAL = 10000;
  static unsigned long lastInternetCheckTime = 0;
  
  if (millis() - lastInternetCheckTime >= INTERNET_CHECK_INTERVAL) {
    if (WiFi.status() == WL_CONNECTED) {
      isOnline = Ping.ping("8.8.8.8", 1); // Lakukan ping dan update variabel global
    } else {
      isOnline = false;
    }
    lastInternetCheckTime = millis(); // Reset timer pengecekan
  }
}

// --- Utilities ---
void safeStringCopy(char* dest, const char* src, size_t destSize) {
  strncpy(dest, src, destSize - 1);
  dest[destSize - 1] = '\0';
}

void getTimestampUTC(char* buffer, size_t bufferSize) {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    buffer[0] = '\0';
    return;
  }
  strftime(buffer, bufferSize, "%Y-%m-%dT%H:%M:%SZ", &timeinfo);
}
