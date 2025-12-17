#include <WiFi.h>
#include <HTTPClient.h>
#include <PubSubClient.h> // TAMBAHAN: Library MQTT
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
  constexpr unsigned long WEIGHT_READ_INTERVAL    = 50;
  constexpr unsigned long LCD_UPDATE_INTERVAL     = 100;
  constexpr unsigned long WIFI_CHECK_INTERVAL     = 15000;
  constexpr unsigned long STATUS_MSG_DURATION     = 2000; 
  constexpr unsigned long CALIBRATION_VALUE       = 12.333372;
  constexpr float MIN_WEIGHT_THRESHOLD = 0.01f;

  constexpr int PIN_TOMBOL_1 = 27;
  constexpr int PIN_TOMBOL_2 = 26;
  constexpr int PIN_TOMBOL_3 = 25;
  constexpr int PIN_TOMBOL_4 = 33;
  constexpr int PIN_BUZZER   = 5;
  constexpr int HX711_DOUT   = 2;
  constexpr int HX711_SCK    = 4;
}

// ==================== KONFIGURASI SERVER (LARAVEL & MQTT) ====================
// URL Laravel (Sesuaikan IP Laptop/Server)
const char* serverName = "https://ecoscale.undip.us/api/receive-sampah"; 

// Konfigurasi MQTT (Broker Public HiveMQ)
const char* mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;
const char* mqtt_topic = "undip/scale/new";

// ==================== GLOBAL OBJECTS ====================
LiquidCrystal_I2C lcd(0x27, LCD_COLUMNS, LCD_ROWS);
LCDBigNumbers bigNumbers(&lcd, BIG_NUMBERS_FONT_2_COLUMN_3_ROWS_VARIANT_2);
HX711_ADC LoadCell(Config::HX711_DOUT, Config::HX711_SCK);

ezButton tombol[] = {
    ezButton(Config::PIN_TOMBOL_1), 
    ezButton(Config::PIN_TOMBOL_2), 
    ezButton(Config::PIN_TOMBOL_3), 
    ezButton(Config::PIN_TOMBOL_4)
};

// Network Objects
WiFiClient wifiClient;            // Client dasar untuk koneksi
PubSubClient mqttClient(wifiClient); // Client MQTT membungkus WiFiClient

// ==================== GLOBAL VARIABLES ====================
enum class AppState { IDLE, SELECTING_SUBTYPE, SENDING_DATA, SHOWING_STATUS };
AppState currentState = AppState::IDLE;

struct SampahType { char jenis[16]; char subJenis[16]; };
SampahType sampah;

char fakultas[8] = "FSM"; 
bool isOnline = false;
bool offlineMode = false;

float currentWeight = 0.0;
float lastDisplayedWeight = -1.00;
float weightBuffer[2] = {0.0f, 0.0f};
int bufferIndex = 0;
bool newDataReady = false;

// Timers
unsigned long lastWeightReadTime = 0;
unsigned long lastLCDUpdateTime = 0;
unsigned long lastWifiCheckTime = 0;
unsigned long statusMsgTimestamp = 0;

// --- ASET IKON ---
#define ICON_IDX_SIGNAL_1   0
#define ICON_IDX_SIGNAL_2   1
#define ICON_IDX_SIGNAL_3   2
#define ICON_IDX_SIGNAL_4   3
#define ICON_IDX_NO_INTERNET 4
byte wifiSignal_1[] = { B00000, B00000, B00000, B00000, B00000, B00000, B11000, B11000 }; 
byte wifiSignal_2[] = { B00000, B00000, B00000, B00000, B00011, B00011, B11011, B11011 }; 
byte wifiSignal_3[] = { B00000, B00000, B11000, B11000, B11000, B11000, B11000, B11000 }; 
byte wifiSignal_4[] = { B00011, B00011, B11011, B11011, B11011, B11011, B11011, B11011 }; 
byte noInternetIcon[] = { B10100, B01000, B10100, B00000, B00000, B00000, B11000, B11000 }; 

// ==================== FUNCTION DECLARATIONS ====================
void prosesTombol();
void handleKirimData();
bool sendToLaravel(); // Fungsi Kirim HTTP
bool sendToMQTT();    // Fungsi Kirim MQTT
float readSmoothedWeight();

void initializeSystem();
bool connectWiFi();
void connectMQTT();   // Fungsi Konek MQTT
bool syncTime();
bool checkNetworkHealth(); // Cek Ping
void manageWifiConnection();

void updateWeightDisplay(float weight);
void restoreDefaultDisplay();
void tampilkanSubJenisAnorganik();
void updateStatusIndicators();
void safeStringCopy(char* dest, const char* src, size_t destSize);

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  Serial.println("\nStarting Firmware (Laravel + MQTT Hybrid)...");
  
  esp_task_wdt_init(60, true); 
  esp_task_wdt_add(NULL);

  initializeSystem();

  // Setup MQTT Server
  mqttClient.setServer(mqtt_server, mqtt_port);

  offlineMode = false; 
  char statusMsg[16] = "Online Mode";

  do {
    // 1. Cek WiFi
    if (!connectWiFi()) {
      offlineMode = true;
      safeStringCopy(statusMsg, "WiFi Gagal!", sizeof(statusMsg));
      break; 
    }

    // 2. Cek Waktu (NTP)
    if (!syncTime()) {
      offlineMode = true;
      safeStringCopy(statusMsg, "NTP Gagal!", sizeof(statusMsg));
      break; 
    }

    // 3. Coba Konek MQTT (Sebagai indikator koneksi bagus)
    connectMQTT();
    if (!mqttClient.connected()) {
       Serial.println("Warning: MQTT Gagal saat startup, tapi lanjut dulu...");
    }

  } while (0); 

  lcd.clear();
  if (offlineMode) {
    lcd.setCursor(0, 1); lcd.print("Mode Offline");
    lcd.setCursor(0, 2); lcd.print(statusMsg);
    tone(Config::PIN_BUZZER, 500, 1000);
    delay(2000); 
  } else {
    lcd.setCursor(0, 1); lcd.print("Setup Sukses");
    lcd.setCursor(0, 2); lcd.print("System Ready");
    delay(1000);
  }

  lcd.clear();
  restoreDefaultDisplay();
  updateWeightDisplay(0.0);
  lastWeightReadTime = millis();
  lastLCDUpdateTime = millis();
}

// ==================== MAIN LOOP ====================
void loop() {
  esp_task_wdt_reset();
  
  // Input Handling
  for (int i = 0; i < 4; i++) tombol[i].loop();
  
  // Network & MQTT Maintenance
  manageWifiConnection();
  if (!offlineMode) {
      if (!mqttClient.connected()) {
        // Coba reconnect MQTT sesekali jika putus (jangan blocking)
        static unsigned long lastMqttRetry = 0;
        if (millis() - lastMqttRetry > 5000) {
           connectMQTT();
           lastMqttRetry = millis();
        }
      }
      mqttClient.loop();
  }

  switch (currentState) {
    case AppState::IDLE: {
      unsigned long currentMillis = millis();
      if (LoadCell.update()) newDataReady = true;
      
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
      prosesTombol();
      handleKirimData();
      updateStatusIndicators();
      break;
    }
    case AppState::SELECTING_SUBTYPE: { prosesTombol(); break; }
    case AppState::SENDING_DATA: { break; }
    case AppState::SHOWING_STATUS: {
      if (millis() - statusMsgTimestamp > Config::STATUS_MSG_DURATION) {
        restoreDefaultDisplay();
        currentState = AppState::IDLE;
      }
      break;
    }
  }
}

// ==================== NETWORK FUNCTIONS ====================

bool connectWiFi() {
  WiFi.mode(WIFI_STA); 
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD); // Ambil dari credentials.h
  
  lcd.clear(); lcd.setCursor(0, 1); lcd.print("Connecting WiFi...");
  
  int retry = 0;
  while (WiFi.status() != WL_CONNECTED && retry < 20) {
    lcd.setCursor(retry % 20, 2); lcd.print("."); 
    delay(500); retry++; esp_task_wdt_reset();
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✅ WiFi Connected");
    Serial.print("IP: "); Serial.println(WiFi.localIP());
    return true;
  }
  return false;
}

void connectMQTT() {
  if (mqttClient.connected()) return;
  
  Serial.print("🔗 Connecting MQTT...");
  String clientId = "ESP32Scale-" + String(random(0xffff), HEX);
  
  if (mqttClient.connect(clientId.c_str())) {
    Serial.println("Success!");
  } else {
    Serial.print("Failed, rc="); Serial.println(mqttClient.state());
  }
}

// --- PENGIRIMAN KE LARAVEL (Menggunakan logika kode baru Anda) ---
bool sendToLaravel() {
  if (WiFi.status() != WL_CONNECTED) return false;

  // 1. Siapkan Client Secure
  WiFiClientSecure clientSecure;
  clientSecure.setInsecure();          // Abaikan sertifikat
  clientSecure.setTimeout(15);         // Perpanjang timeout koneksi TCP ke 15 detik (bukan ms)

  HTTPClient http;
  
  // Debug
  Serial.println("\n--- 📦 LARAVEL POST ---");
  
  // 2. Mulai Koneksi dengan Timeout Ekstra
  if (!http.begin(clientSecure, serverName)) {
    Serial.println("❌ Gagal inisialisasi HTTP!");
    return false;
  }
  
  // Set timeout level HTTP juga
  http.setTimeout(15000); 

  // 3. Tambahkan Header Penting
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");
  http.addHeader("Connection", "close"); // PENTING: Minta server langsung tutup koneksi setelah reply

  // 4. Siapkan Data (Jenis Lengkap)
  String jenisFinal;
  if (strcmp(sampah.jenis, "Anorganik") == 0 && strcmp(sampah.subJenis, "--") != 0) {
    if (strcmp(sampah.subJenis, "Umum") == 0) jenisFinal = "Anorganik";
    else jenisFinal = String(sampah.subJenis);
  } else {
    jenisFinal = String(sampah.jenis);
  }

  // 5. Buat Payload
  String postData = "api_key=" + String(API_KEY) +
                    "&berat=" + String(currentWeight, 2) +
                    "&fakultas=" + String(fakultas) +
                    "&jenis=" + jenisFinal;

  Serial.println("Data: " + postData);
  
  // 6. Kirim (POST)
  int httpResponseCode = http.POST(postData);
  
  bool success = false;

  if (httpResponseCode > 0) {
     Serial.print("HTTP Code: "); Serial.println(httpResponseCode);
     String response = http.getString();
     // Serial.println("Response: " + response); // Uncomment jika ingin lihat pesan server

     // Kode 200/201 atau ada kata "berhasil"
     if (httpResponseCode == 200 || httpResponseCode == 201 || response.indexOf("berhasil") >= 0) {
        Serial.println("✅ Database OK");
        success = true;
     } else {
        Serial.println("⚠️ Terkirim tapi response aneh (Check Server)");
        success = true; // Tetap anggap sukses agar UI tidak error
     }
  } else {
     Serial.print("❌ HTTP Error: "); 
     Serial.print(httpResponseCode);
     Serial.print(" - ");
     Serial.println(http.errorToString(httpResponseCode));
     success = false;
  }
  
  // 7. Bersihkan Resource
  http.end();
  clientSecure.stop(); // Paksa putus koneksi secure
  
  return success;
}

// --- PENGIRIMAN KE MQTT ---
bool sendToMQTT() {
  if (!mqttClient.connected()) {
     connectMQTT();
     if (!mqttClient.connected()) return false;
  }

  // Siapkan Data Jenis
  char jenisLengkap[20];
  if (strcmp(sampah.jenis, "Anorganik") == 0 && strcmp(sampah.subJenis, "--") != 0) {
    if (strcmp(sampah.subJenis, "Umum") == 0) safeStringCopy(jenisLengkap, "Anorganik", sizeof(jenisLengkap));
    else safeStringCopy(jenisLengkap, sampah.subJenis, sizeof(jenisLengkap));
  } else {
    safeStringCopy(jenisLengkap, sampah.jenis, sizeof(jenisLengkap));
  }

  // Format JSON
  String mqttData = "{";
  mqttData += "\"weight\":" + String(currentWeight, 2) + ",";
  mqttData += "\"fakultas\":\"" + String(fakultas) + "\",";
  mqttData += "\"jenis\":\"" + String(jenisLengkap) + "\"";
  mqttData += "}";

  Serial.println("📡 MQTT Publish: " + mqttData);
  
  if (mqttClient.publish(mqtt_topic, mqttData.c_str())) {
    Serial.println("✅ MQTT Sent");
    return true;
  } else {
    Serial.println("❌ MQTT Failed");
    return false;
  }
}

// ==================== BUTTON & LOGIC ====================

void handleKirimData() {
  if (currentState != AppState::IDLE) return;

  if (tombol[3].isPressed()) {
    tone(Config::PIN_BUZZER, 2000, 100);

    if (offlineMode) {
      lcd.setCursor(0, 0); lcd.print("Gagal: Offline!   ");
      statusMsgTimestamp = millis(); currentState = AppState::SHOWING_STATUS;
      return;
    }

    currentState = AppState::SENDING_DATA; 
    lcd.setCursor(0, 0);
    
    if (strcmp(sampah.jenis, "--") == 0) {
      lcd.print("Error: Pilih Jenis!   ");
    } else {
      lcd.print("Status: Mengirim... ");
      
      // --- KIRIM KE DUA TUJUAN ---
      // Kita prioritaskan Laravel (Database) untuk status sukses/gagal di LCD
      bool laravelSuccess = sendToLaravel();
      
      // Kirim ke MQTT (Background process, tidak mempengaruhi LCD secara kritis)
      sendToMQTT(); 
      
      lcd.setCursor(0, 0);
      lcd.print(laravelSuccess ? "Status: Sukses!      " : "Status: Gagal!        ");
      
      if (laravelSuccess) {
        safeStringCopy(sampah.jenis, "--", sizeof(sampah.jenis));
        safeStringCopy(sampah.subJenis, "--", sizeof(sampah.subJenis));
      }
    }
    statusMsgTimestamp = millis();
    currentState = AppState::SHOWING_STATUS;
  }
}

// --- FUNGSI UTILITAS (Tidak banyak berubah) ---

bool syncTime() {
  configTime(7 * 3600, 0, "pool.ntp.org", "time.nist.gov");
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo, 3000)) return false; // Timeout dipercepat
  return true;
}

bool checkNetworkHealth() {
    // Cek ping ke Google DNS sebagai indikator internet
    return Ping.ping("8.8.8.8", 1);
}

void manageWifiConnection() {
  if (millis() - lastWifiCheckTime >= Config::WIFI_CHECK_INTERVAL) {
    if (WiFi.status() != WL_CONNECTED) {
      WiFi.reconnect(); 
      offlineMode = true; isOnline = false;
    } else {
      if (offlineMode == true) {
        // Jika sebelumnya offline, cek apakah sudah benar-benar pulih
        if(checkNetworkHealth()) {
           offlineMode = false;
           syncTime(); // Sync ulang waktu
        }
      }
    }
    lastWifiCheckTime = millis();
  }
}

float readSmoothedWeight() {
  float rawWeight = LoadCell.getData();
  float weightInKg = rawWeight / 1000.0;
  if (weightInKg < 0.05) weightInKg = 0;
  weightBuffer[bufferIndex] = weightInKg;
  bufferIndex = (bufferIndex + 1) % 2;
  return (weightBuffer[0] + weightBuffer[1]) / 2.0f;
}

void prosesTombol() {
  if (tombol[0].isPressed()) {
    if (currentState == AppState::IDLE) {
      safeStringCopy(sampah.jenis, "Organik", sizeof(sampah.jenis));
      safeStringCopy(sampah.subJenis, "--", sizeof(sampah.subJenis));
      tone(Config::PIN_BUZZER, 2500, 100); restoreDefaultDisplay();
    } else if (currentState == AppState::SELECTING_SUBTYPE) {
      safeStringCopy(sampah.jenis, "Anorganik", sizeof(sampah.jenis));
      safeStringCopy(sampah.subJenis, "Umum", sizeof(sampah.subJenis));
      tone(Config::PIN_BUZZER, 2500, 100); restoreDefaultDisplay(); currentState = AppState::IDLE;
    }
  } else if (tombol[1].isPressed()) {
    if (currentState == AppState::IDLE) {
      currentState = AppState::SELECTING_SUBTYPE; tampilkanSubJenisAnorganik();
    } else if (currentState == AppState::SELECTING_SUBTYPE) {
      safeStringCopy(sampah.jenis, "Anorganik", sizeof(sampah.jenis));
      safeStringCopy(sampah.subJenis, "Botol", sizeof(sampah.subJenis));
      tone(Config::PIN_BUZZER, 2500, 100); restoreDefaultDisplay(); currentState = AppState::IDLE;
    }
  } else if (tombol[2].isPressed()) {
    if (currentState == AppState::IDLE) {
      safeStringCopy(sampah.jenis, "Residu", sizeof(sampah.jenis));
      safeStringCopy(sampah.subJenis, "--", sizeof(sampah.subJenis));
      tone(Config::PIN_BUZZER, 2500, 100); restoreDefaultDisplay();
    } else if (currentState == AppState::SELECTING_SUBTYPE) {
      safeStringCopy(sampah.jenis, "Anorganik", sizeof(sampah.jenis));
      safeStringCopy(sampah.subJenis, "Kertas", sizeof(sampah.subJenis));
      tone(Config::PIN_BUZZER, 2500, 100); restoreDefaultDisplay(); currentState = AppState::IDLE;
    }
  }
}

void tampilkanSubJenisAnorganik() {
  lcd.clear(); lcd.setCursor(2, 0); lcd.print("Pilih Sub-jenis:");
  lcd.setCursor(0, 1); lcd.print(" 1.Umum     2.Botol");
  lcd.setCursor(0, 2); lcd.print(" 3.Kertas");
}

void restoreDefaultDisplay() {
  lcd.clear(); lcd.setCursor(0, 0);
  char displayText[21];
  if (strcmp(sampah.jenis, "Anorganik") == 0 && strcmp(sampah.subJenis, "--") != 0) {
    if (strcmp(sampah.subJenis, "Umum") == 0) snprintf(displayText, sizeof(displayText), "Jenis: Anorganik");
    else snprintf(displayText, sizeof(displayText), "Jenis: %s", sampah.subJenis);
  } else {
    snprintf(displayText, sizeof(displayText), "Jenis: %s", sampah.jenis);
  }
  lcd.print(displayText); lcd.setCursor(17, 1); lcd.print("kg"); lastDisplayedWeight = -1.00;
}

void updateWeightDisplay(float weight) {
  char weightString[10]; snprintf(weightString, sizeof(weightString), "%6.2f", weight);
  bigNumbers.setBigNumberCursor(1, 1); bigNumbers.print(weightString);
}

void updateStatusIndicators() {
  const unsigned long DISPLAY_UPDATE_INTERVAL = 1000; 
  static unsigned long lastDisplayUpdateTime = 0;
  static bool blinkerState = false;

  if (millis() - lastDisplayUpdateTime >= DISPLAY_UPDATE_INTERVAL) {
    char signalText[5];
    if (WiFi.status() == WL_CONNECTED && !offlineMode) {
      long rssi = WiFi.RSSI(); snprintf(signalText, sizeof(signalText), "%3ld", rssi);
    } else {
      safeStringCopy(signalText, "OFF", sizeof(signalText));
    }
    lcd.setCursor(17, 3); lcd.print(signalText);

    blinkerState = !blinkerState;
    if (offlineMode) { 
      lcd.setCursor(0, 3); lcd.write(ICON_IDX_NO_INTERNET); 
    } else if (mqttClient.connected()) {
      lcd.setCursor(0, 3); lcd.print(" "); // Bersih jika MQTT connected
    } else {
      lcd.setCursor(0, 3); lcd.print(blinkerState ? "-" : " "); // Kedip jika MQTT putus tapi WiFi on
    }
    lastDisplayUpdateTime = millis();
  }

  const unsigned long PING_CHECK_INTERVAL = 10000;
  static unsigned long lastPingTime = 0;
  if (!offlineMode && millis() - lastPingTime >= PING_CHECK_INTERVAL) {
    if (WiFi.status() == WL_CONNECTED) isOnline = checkNetworkHealth();
    else isOnline = false;
    lastPingTime = millis();
  }
}

void initializeSystem() {
  safeStringCopy(sampah.jenis, "--", sizeof(sampah.jenis));
  safeStringCopy(sampah.subJenis, "--", sizeof(sampah.subJenis));
  pinMode(Config::PIN_BUZZER, OUTPUT);
  lcd.init(); lcd.backlight(); lcd.clear();
  lcd.createChar(ICON_IDX_SIGNAL_1, wifiSignal_1);
  lcd.createChar(ICON_IDX_SIGNAL_2, wifiSignal_2);
  lcd.createChar(ICON_IDX_SIGNAL_3, wifiSignal_3);
  lcd.createChar(ICON_IDX_SIGNAL_4, wifiSignal_4);
  lcd.createChar(ICON_IDX_NO_INTERNET, noInternetIcon);
  bigNumbers.begin();
  LoadCell.begin(); EEPROM.begin(512); LoadCell.start(2000, true);
  if (LoadCell.getTareTimeoutFlag()) {
    lcd.clear(); lcd.print("HX711 Error!"); while (1) { esp_task_wdt_reset(); delay(100); } 
  } else {
    LoadCell.setCalFactor(Config::CALIBRATION_VALUE);
    LoadCell.setSamplesInUse(1);
  }
}

void safeStringCopy(char* dest, const char* src, size_t destSize) {
  strncpy(dest, src, destSize - 1); dest[destSize - 1] = '\0';
}