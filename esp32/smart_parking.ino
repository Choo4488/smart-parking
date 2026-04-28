/**
 * Smart Parking – ESP32-S3  (FreeRTOS)
 * ─────────────────────────────────────────────────────────────────
 * Task 1  RFID Auth      Priority 3  ตรวจบัตร RC522 (poll 100 ms)
 * Task 2  Gate Control   Priority 3  ควบคุม Servo Entry/Exit
 * Task 3  Slot Monitor   Priority 2  HC-SR04 x3  ทุก 150 ms
 * Task 4  Fire Alert     Priority 4  MQ-5 ADC    ทุก 200 ms  (REALTIME)
 * Task 5  IoT Publish    Priority 1  MQTT JSON   ทุก 2000 ms
 *
 * Semaphore / Mutex:
 *   mutServoEntry  Mutex          Task1 & Task2 แย่ง servo entry
 *   mutServoExit   Mutex          Task2 & Task4 แย่ง servo exit
 *   semFire        Binary Sem     Task4 give → Task2 take
 *   mutState       Mutex          Task3/4 write, Task5 read
 *
 * Library ที่ต้องติดตั้ง:
 *   PubSubClient, ArduinoJson, ESP32Servo, MFRC522
 * ─────────────────────────────────────────────────────────────────
 */

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>
#include <SPI.h>
#include <MFRC522.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

// ── WiFi ──────────────────────────────────────────────────────────
const char* WIFI_SSID = "choo";
const char* WIFI_PASS = "12345678";

// ── HiveMQ Cloud ────────────────────────────────────────────────
const char* MQTT_HOST  = "c00f55e87aaa4f459faa7dd4b7ad4273.s1.eu.hivemq.cloud";
const int   MQTT_PORT  = 8883;
const char* MQTT_USER  = "esp32s3";
const char* MQTT_PASS  = "esp1234A";
const char* MQTT_TOPIC = "smartparking/status";

// ── Pins ──────────────────────────────────────────────────────────
// RFID RC522  (SPI: SCK=12, MISO=13, MOSI=11)
#define RFID_SS    10
#define RFID_RST    9

// IR beam-break
#define IR_ENTRY    4
#define IR_EXIT     5

// HC-SR04 x3
#define TRIG1  15
#define ECHO1  16
#define TRIG2  17
#define ECHO2  18
#define TRIG3  19
#define ECHO3  20

// MQ-5 (ADC) - ESP32-S3 should use ADC-capable GPIO (1-10, 11-20)
#define MQ5_PIN    1

// Servo
#define SERVO_ENTRY_PIN  6
#define SERVO_EXIT_PIN   7

// LED per slot  (HIGH = มีรถ)
#define LED_SLOT1  38
#define LED_SLOT2  39
#define LED_SLOT3  40
#define LED_FIRE   41

// Buzzer
#define BUZZER_PIN 42

// ── ค่าตั้ง ───────────────────────────────────────────────────────
#define DIST_CM       50.0f   // ≤ 50 cm = มีรถ
#define MQ5_THRESH    2000    // ADC > 2000 = แจ้งเตือน
#define TOTAL_SLOTS   2
#define PUBLISH_MS    2000
#define GATE_OPEN_MS  3000    // เวลา servo ค้างเปิด (ms)
#define IR_DEBOUNCE_US 120000 // 120ms debounce for IR ISR
#define FIRE_ON_COUNT      3  // require N consecutive high samples to trigger
#define FIRE_OFF_COUNT     4  // require N consecutive low samples to clear

// ── Authorized RFID UIDs ──────────────────────────────────────────
const byte AUTH_UIDS[][4] = {
  { 0xDE, 0xAD, 0xBE, 0xEF },   // ← เปลี่ยนเป็น UID บัตรจริง
  { 0x01, 0x02, 0x03, 0x04 },
};
const int AUTH_COUNT = sizeof(AUTH_UIDS) / 4;

// ── IR Event type ─────────────────────────────────────────────────
typedef enum : uint8_t { EVT_IR_ENTRY = 1, EVT_IR_EXIT = 2 } IREvent_t;

// ── Shared state ──────────────────────────────────────────────────
struct ParkingState {
  bool occupied[TOTAL_SLOTS];
  float distanceCm[TOTAL_SLOTS];
  bool fireAlert;
  int gasValue;
  bool gateEntryOpen;
  bool gateExitOpen;
  uint32_t irEntryCount;
  uint32_t irExitCount;
  char lastIrEvent[8];
  bool lastRfidAuthorized;
  char lastRfidUid[16];
  uint32_t lastRfidSeenAt;
} parking = {};

// ── FreeRTOS handles ──────────────────────────────────────────────
static QueueHandle_t     qIR;
static SemaphoreHandle_t mutServoEntry, mutServoExit, semFire, mutState;

// ── Hardware objects ──────────────────────────────────────────────
static MFRC522           rfid(RFID_SS, RFID_RST);
static Servo             servoEntry, servoExit;
static WiFiClientSecure  wifiClient;
static PubSubClient      mqttClient(wifiClient);

volatile bool authFlag = false;
volatile uint32_t lastIrEntryUs = 0;
volatile uint32_t lastIrExitUs  = 0;

// ── Pin arrays ────────────────────────────────────────────────────
static const uint8_t TRIGS[TOTAL_SLOTS] = { TRIG1, TRIG2, TRIG3 };
static const uint8_t ECHOS[TOTAL_SLOTS] = { ECHO1, ECHO2, ECHO3 };
static const uint8_t LEDS [TOTAL_SLOTS] = { LED_SLOT1, LED_SLOT2, LED_SLOT3 };

// ─────────────────────────────────────────────────────────────────
// ISR – IR Entry / Exit
// ─────────────────────────────────────────────────────────────────
void IRAM_ATTR isrIREntry() {
  uint32_t nowUs = micros();
  if ((nowUs - lastIrEntryUs) < IR_DEBOUNCE_US) return;
  lastIrEntryUs = nowUs;

  BaseType_t hpTaskWoken = pdFALSE;
  IREvent_t e = EVT_IR_ENTRY;
  xQueueSendFromISR(qIR, &e, &hpTaskWoken);
  if (hpTaskWoken) portYIELD_FROM_ISR();
}
void IRAM_ATTR isrIRExit() {
  uint32_t nowUs = micros();
  if ((nowUs - lastIrExitUs) < IR_DEBOUNCE_US) return;
  lastIrExitUs = nowUs;

  BaseType_t hpTaskWoken = pdFALSE;
  IREvent_t e = EVT_IR_EXIT;
  xQueueSendFromISR(qIR, &e, &hpTaskWoken);
  if (hpTaskWoken) portYIELD_FROM_ISR();
}

// ─────────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────────
bool checkUID(byte* uid, byte size) {
  if (size != 4) return false;
  for (int i = 0; i < AUTH_COUNT; i++)
    if (memcmp(uid, AUTH_UIDS[i], 4) == 0) return true;
  return false;
}

void uidToString(byte* uid, byte size, char* out, size_t outSize) {
  if (outSize == 0) return;
  out[0] = '\0';
  size_t pos = 0;
  for (byte i = 0; i < size && pos + 3 < outSize; i++) {
    int written = snprintf(out + pos, outSize - pos, "%02X", uid[i]);
    if (written <= 0) break;
    pos += (size_t) written;
    if (i < size - 1 && pos + 1 < outSize) {
      out[pos++] = ':';
      out[pos] = '\0';
    }
  }
}

float readDist(uint8_t trig, uint8_t echo) {
  digitalWrite(trig, LOW);  delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long d = pulseIn(echo, HIGH, 30000);
  return (d == 0) ? 999.0f : d * 0.0343f / 2.0f;
}

void servoOpen (Servo& s) { s.write(90); }
void servoClose(Servo& s) { s.write(0);  }

void reconnectWifi() {
  Serial.printf("WiFi %s...", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(pdMS_TO_TICKS(500));
    Serial.print(".");
  }
  Serial.printf(" OK  IP:%s\n", WiFi.localIP().toString().c_str());
}

void reconnectMQTT() {
  while (!mqttClient.connected()) {
    Serial.print("MQTT...");
    String id = "sp_" + String(random(0xffff), HEX);
    bool ok;
    if (strlen(MQTT_USER) > 0) {
      ok = mqttClient.connect(id.c_str(), MQTT_USER, MQTT_PASS);
    } else {
      ok = mqttClient.connect(id.c_str());
    }
    if (ok) {
      Serial.println(" Connected!");
    } else {
      Serial.printf(" fail rc=%d retry 5s\n", mqttClient.state());
      vTaskDelay(pdMS_TO_TICKS(5000));
    }
  }
}

// ═════════════════════════════════════════════════════════════════
// Task 1 – RFID Auth  (Priority 3, Core 1)
// ═════════════════════════════════════════════════════════════════
void taskRFIDAuth(void*) {
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(100));   // poll ทุก 100 ms

    if (!rfid.PICC_IsNewCardPresent()) continue;
    if (!rfid.PICC_ReadCardSerial())   continue;

    bool ok = checkUID(rfid.uid.uidByte, rfid.uid.size);
    char uidText[16];
    uidToString(rfid.uid.uidByte, rfid.uid.size, uidText, sizeof(uidText));
    rfid.PICC_HaltA();
    rfid.PCD_StopCrypto1();

    xSemaphoreTake(mutState, portMAX_DELAY);
    parking.lastRfidAuthorized = ok;
    strncpy(parking.lastRfidUid, uidText, sizeof(parking.lastRfidUid) - 1);
    parking.lastRfidUid[sizeof(parking.lastRfidUid) - 1] = '\0';
    parking.lastRfidSeenAt = millis() / 1000;
    xSemaphoreGive(mutState);

    if (ok) {
      authFlag = true;
      Serial.printf("[RFID] Authorized %s\n", uidText);
    } else {
      Serial.printf("[RFID] Unauthorized %s\n", uidText);
    }
  }
}

// ═════════════════════════════════════════════════════════════════
// Task 2 – Gate Control  (Priority 3, Core 1)
// ═════════════════════════════════════════════════════════════════
void taskGateControl(void*) {
  IREvent_t evt;
  for (;;) {
    // ตรวจ fire semaphore ก่อน (non-blocking)
    if (xSemaphoreTake(semFire, 0) == pdTRUE) {
      // ล็อค Entry, เปิด Exit ระบายรถ
      if (xSemaphoreTake(mutServoEntry, portMAX_DELAY) == pdTRUE) {
        servoClose(servoEntry);
        xSemaphoreGive(mutServoEntry);
      }
      if (xSemaphoreTake(mutServoExit, portMAX_DELAY) == pdTRUE) {
        servoOpen(servoExit);
        xSemaphoreGive(mutServoExit);
      }
      xSemaphoreTake(mutState, portMAX_DELAY);
      parking.gateEntryOpen = false;
      parking.gateExitOpen  = true;
      xSemaphoreGive(mutState);
      Serial.println("[Gate] FIRE – Entry locked, Exit open");
    }

    // รอ IR event
    if (xQueueReceive(qIR, &evt, pdMS_TO_TICKS(50)) != pdTRUE) continue;

    xSemaphoreTake(mutState, portMAX_DELAY);
    if (evt == EVT_IR_ENTRY) {
      parking.irEntryCount++;
      strncpy(parking.lastIrEvent, "entry", sizeof(parking.lastIrEvent) - 1);
    } else {
      parking.irExitCount++;
      strncpy(parking.lastIrEvent, "exit", sizeof(parking.lastIrEvent) - 1);
    }
    parking.lastIrEvent[sizeof(parking.lastIrEvent) - 1] = '\0';
    xSemaphoreGive(mutState);

    if (evt == EVT_IR_ENTRY && authFlag) {
      authFlag = false;
      if (xSemaphoreTake(mutServoEntry, portMAX_DELAY) == pdTRUE) {
        servoOpen(servoEntry);
        xSemaphoreTake(mutState, portMAX_DELAY);
        parking.gateEntryOpen = true;
        xSemaphoreGive(mutState);

        vTaskDelay(pdMS_TO_TICKS(GATE_OPEN_MS));  // รอรถผ่าน

        servoClose(servoEntry);
        xSemaphoreTake(mutState, portMAX_DELAY);
        parking.gateEntryOpen = false;
        xSemaphoreGive(mutState);
        xSemaphoreGive(mutServoEntry);
      }
      Serial.println("[Gate] Entry opened");
    }
    else if (evt == EVT_IR_EXIT) {
      if (xSemaphoreTake(mutServoExit, portMAX_DELAY) == pdTRUE) {
        servoOpen(servoExit);
        xSemaphoreTake(mutState, portMAX_DELAY);
        parking.gateExitOpen = true;
        xSemaphoreGive(mutState);

        vTaskDelay(pdMS_TO_TICKS(GATE_OPEN_MS));

        servoClose(servoExit);
        xSemaphoreTake(mutState, portMAX_DELAY);
        parking.gateExitOpen = false;
        xSemaphoreGive(mutState);
        xSemaphoreGive(mutServoExit);
      }
      Serial.println("[Gate] Exit opened");
    }
  }
}

// ═════════════════════════════════════════════════════════════════
// Task 3 – Slot Monitor  (Priority 2, Core 1)
// ═════════════════════════════════════════════════════════════════
void taskSlotMonitor(void*) {
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(150));

    bool occ[TOTAL_SLOTS];
    float dist[TOTAL_SLOTS];
    for (int i = 0; i < TOTAL_SLOTS; i++) {
      dist[i] = readDist(TRIGS[i], ECHOS[i]);
      occ[i] = (dist[i] <= DIST_CM);
      digitalWrite(LEDS[i], occ[i] ? HIGH : LOW);
    }

    xSemaphoreTake(mutState, portMAX_DELAY);
    for (int i = 0; i < TOTAL_SLOTS; i++) {
      parking.occupied[i] = occ[i];
      parking.distanceCm[i] = dist[i];
    }
    xSemaphoreGive(mutState);
  }
}

// ═════════════════════════════════════════════════════════════════
// Task 4 – Fire Alert  (Priority 4 – REALTIME, Core 1)
// ═════════════════════════════════════════════════════════════════
void taskFireAlert(void*) {
  uint8_t highCount = 0;
  uint8_t lowCount  = 0;
  bool fireStable   = false;

  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(200));

    int  gas  = analogRead(MQ5_PIN);
    bool high = (gas > MQ5_THRESH);

    if (high) {
      if (highCount < 255) highCount++;
      lowCount = 0;
    } else {
      if (lowCount < 255) lowCount++;
      highCount = 0;
    }

    if (!fireStable && highCount >= FIRE_ON_COUNT) {
      fireStable = true;
      xSemaphoreGive(semFire);  // signal Task 2 when state turns ON
      Serial.printf("[Fire] GAS=%d  DANGER!\n", gas);
    } else if (fireStable && lowCount >= FIRE_OFF_COUNT) {
      fireStable = false;
      Serial.printf("[Fire] GAS=%d  cleared\n", gas);
    }

    xSemaphoreTake(mutState, portMAX_DELAY);
    parking.fireAlert = fireStable;
    parking.gasValue = gas;
    xSemaphoreGive(mutState);

    digitalWrite(LED_FIRE,   fireStable ? HIGH : LOW);
    digitalWrite(BUZZER_PIN, fireStable ? HIGH : LOW);
  }
}

// ═════════════════════════════════════════════════════════════════
// Task 5 – IoT Publish  (Priority 1, Core 0)
// ═════════════════════════════════════════════════════════════════
void taskIoTPublish(void*) {
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(PUBLISH_MS));

    // Reconnect ถ้าหลุด
    if (WiFi.status() != WL_CONNECTED) reconnectWifi();
    if (!mqttClient.connected())        reconnectMQTT();
    mqttClient.loop();

    // อ่าน shared state (snapshot)
    xSemaphoreTake(mutState, portMAX_DELAY);
    ParkingState snap = parking;
    xSemaphoreGive(mutState);

    // นับ slot
    int occupied = 0;
    for (int i = 0; i < TOTAL_SLOTS; i++) if (snap.occupied[i]) occupied++;
    int freeSlots = TOTAL_SLOTS - occupied;

    // Build JSON
    StaticJsonDocument<1024> doc;
    doc["slots"]["total"]    = TOTAL_SLOTS;
    doc["slots"]["occupied"] = occupied;
    doc["slots"]["free"]     = freeSlots;

    JsonArray detail = doc.createNestedArray("slot_detail");
    for (int i = 0; i < TOTAL_SLOTS; i++)
      detail.add(snap.occupied[i] ? "occupied" : "free");

    doc["gate_entry"] = snap.gateEntryOpen ? "open" : "closed";
    doc["gate_exit"]  = snap.gateExitOpen  ? "open" : "closed";
    doc["fire_alert"] = snap.fireAlert;
    doc["timestamp"]  = millis() / 1000;

    JsonArray distanceArr = doc.createNestedArray("distance_cm");
    for (int i = 0; i < TOTAL_SLOTS; i++) {
      distanceArr.add((double) snap.distanceCm[i]);
    }

    doc["sensors"]["mq5_gas_value"] = snap.gasValue;
    doc["sensors"]["mq5_threshold"] = MQ5_THRESH;
    doc["sensors"]["ir_entry_count"] = snap.irEntryCount;
    doc["sensors"]["ir_exit_count"] = snap.irExitCount;
    doc["sensors"]["last_ir_event"] = snap.lastIrEvent;

    doc["rfid"]["last_uid"] = snap.lastRfidUid;
    doc["rfid"]["authorized"] = snap.lastRfidAuthorized;
    doc["rfid"]["last_seen_at"] = snap.lastRfidSeenAt;

    char buf[1024];
    serializeJson(doc, buf);

    bool ok = mqttClient.publish(MQTT_TOPIC, buf, false);
    Serial.printf("[IoT] %s  occ=%d/%d fire=%s\n",
      ok ? "OK" : "FAIL", occupied, TOTAL_SLOTS,
      snap.fireAlert ? "YES" : "no");
  }
}

// ─────────────────────────────────────────────────────────────────
// Setup
// ─────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);

  // Pin modes
  pinMode(IR_ENTRY,   INPUT_PULLUP);
  pinMode(IR_EXIT,    INPUT_PULLUP);
  pinMode(MQ5_PIN,    INPUT);
  pinMode(LED_FIRE,   OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  for (int i = 0; i < TOTAL_SLOTS; i++) {
    pinMode(TRIGS[i], OUTPUT);
    pinMode(ECHOS[i], INPUT);
    pinMode(LEDS[i],  OUTPUT);
  }

  // RFID
  // Keep SPI pins explicit for ESP32-S3 + RC522 wiring.
  SPI.begin(12, 13, 11, RFID_SS);
  rfid.PCD_Init();
  Serial.println("[RFID] Ready");

  // Servo
  servoEntry.attach(SERVO_ENTRY_PIN);
  servoExit.attach(SERVO_EXIT_PIN);
  servoClose(servoEntry);
  servoClose(servoExit);

  // WiFi + MQTT
  WiFi.mode(WIFI_STA);
  reconnectWifi();
  analogSetPinAttenuation(MQ5_PIN, ADC_11db); // expand measurable voltage range
  wifiClient.setInsecure();
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setBufferSize(512);
  reconnectMQTT();

  // FreeRTOS primitives
  qIR           = xQueueCreate(10, sizeof(IREvent_t));
  mutServoEntry = xSemaphoreCreateMutex();
  mutServoExit  = xSemaphoreCreateMutex();
  semFire       = xSemaphoreCreateBinary();
  mutState      = xSemaphoreCreateMutex();

  // GPIO ISR
  attachInterrupt(digitalPinToInterrupt(IR_ENTRY), isrIREntry, FALLING);
  attachInterrupt(digitalPinToInterrupt(IR_EXIT),  isrIRExit,  FALLING);

  // Create tasks
  //                          name        stack  param prio  handle core
  xTaskCreatePinnedToCore(taskRFIDAuth,   "RFID", 4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(taskGateControl,"Gate", 4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(taskSlotMonitor,"Slot", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(taskFireAlert,  "Fire", 4096, NULL, 4, NULL, 1);
  xTaskCreatePinnedToCore(taskIoTPublish, "IoT",  8192, NULL, 1, NULL, 0);

  Serial.println("=== Smart Parking FreeRTOS Ready ===");
}

// ─────────────────────────────────────────────────────────────────
// Loop – ว่างเปล่า FreeRTOS scheduler จัดการแทน
// ─────────────────────────────────────────────────────────────────
void loop() {
  vTaskDelay(portMAX_DELAY);
}
