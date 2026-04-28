# Smart Parking Dashboard

## โครงสร้างโฟลเดอร์

```
smart-parking/
├── dashboard/
│   └── index.html          ← เปิดบนมือถือ/เบราว์เซอร์ได้เลย
├── mock-data/
│   └── mock_generator.js   ← สร้างข้อมูลทดสอบส่งไป MQTT

```

## การตั้งค่า HiveMQ Cloud

ใส่ข้อมูล HiveMQ Cloud ในแต่ละไฟล์:

| ค่า | คำอธิบาย |
|-----|-----------|
| `YOUR_CLUSTER.s1.eu.hivemq.cloud` | Cluster URL จาก HiveMQ Console |
| `YOUR_USERNAME` | Username ที่สร้างไว้ |
| `YOUR_PASSWORD` | Password |

## วิธีใช้งาน

### 1. Dashboard (index.html)
- เปิดไฟล์ `dashboard/index.html` ในเบราว์เซอร์
- กด ⚙ ที่มุมขวาบน → ใส่ HiveMQ credentials → กด Connect
- ค่าจะถูกบันทึกใน localStorage อัตโนมัติ

### 2. Mock Generator
```bash
cd mock-data
npm install mqtt
node mock_generator.js
```

### 3. Node-RED
1. เปิด Node-RED → Menu → Import → วางเนื้อหาจาก `flow.json`
2. ดับเบิลคลิก node `mqtt-broker` → ใส่ credentials
3. Deploy

## MQTT Topic & JSON Format

**Topic:** `smartparking/status`

```json
{
  "slots":       { "total": 3, "occupied": 2, "free": 1 },
  "slot_detail": ["occupied", "free", "occupied"],
  "gate_entry":  "closed",
  "gate_exit":   "open",
  "fire_alert":  false
}
```

## Ports

| Protocol | Port |
|----------|------|
| MQTT over TLS (TCP) | 8883 |
| MQTT over WebSocket TLS | 8884 |

Dashboard ใช้ **8884 (WSS)** · Mock Generator / ESP32 ใช้ **8883 (MQTTS)**

---

## Smart Parking System - Key Requirement Mapping

### +5% Bonus: IR Interrupt with FreeRTOS Queue

This project uses **GPIO Interrupt** (not polling) for IR sensors:

- `IR_ENTRY` and `IR_EXIT` use `attachInterrupt(..., FALLING)`
- ISR sends event to FreeRTOS queue via `xQueueSendFromISR()`
- `taskGateControl` consumes events with `xQueueReceive()`
- Gate servo is opened immediately when event arrives

Code reference (ESP32):

```cpp
attachInterrupt(digitalPinToInterrupt(IR_ENTRY), isrIREntry, FALLING);
attachInterrupt(digitalPinToInterrupt(IR_EXIT),  isrIRExit,  FALLING);
```

### FreeRTOS Tasks and Priorities

| Task | Priority | Responsibility |
|------|----------|----------------|
| Task 1 (`taskRFIDAuth`) | 3 (High) | RFID authentication for entry |
| Task 2 (`taskGateControl`) | 3 (High) | Entry/Exit gate servo control |
| Task 3 (`taskSlotMonitor`) | 2 (Medium) | Ultrasonic slot occupancy monitoring |
| Task 4 (`taskFireAlert`) | 4 (Highest) | MQ-5 gas/fire alert (real-time safety) |
| Task 5 (`taskIoTPublish`) | 1 (Low) | Publish MQTT JSON to dashboard/mobile |

### Why interrupt is used here

- Faster reaction than continuous polling
- Lower CPU overhead
- Better real-time behavior for gate control
- Scales better when adding more tasks
