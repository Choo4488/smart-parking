/**
 * Smart Parking – Mock Data Generator
 * ─────────────────────────────────────
 * Publishes random parking status to HiveMQ Cloud every 2 seconds.
 *
 * Usage:
 *   npm install mqtt
 *   node mock_generator.js
 */

const mqtt = require('mqtt');

// ── Config ── HiveMQ Cloud
const CONFIG = {
  host: '2ca71896352241b183bcb6c58ce3f5e9.s1.eu.hivemq.cloud',
  port: 8883,   // TCP TLS
  user: 'esp32s3',
  pass: 'esp1234A',
  topic: 'smartparking/status'
};
// ─────────────────────────────────────────────────────────────────

const TOTAL_SLOTS = 2;
const INTERVAL_MS = 2000;

const brokerUrl = `mqtts://${CONFIG.host}:${CONFIG.port}`;

console.log(`Connecting to ${brokerUrl} …`);

const client = mqtt.connect(brokerUrl, {
  username: CONFIG.user || undefined,
  password: CONFIG.pass || undefined,
  clientId: 'mock_generator_' + Math.random().toString(16).slice(2, 8),
  clean: true,
  rejectUnauthorized: true
});

client.on('connect', () => {
  console.log('Connected to HiveMQ Cloud');
  startPublishing();
});

client.on('error', err => {
  console.error('❌ MQTT error:', err.message);
});

client.on('offline', () => console.log('⚠  Broker offline'));

// ── Random data helpers ──────────────────────────────────────────
function randomSlot() {
  return Math.random() < 0.5 ? 'occupied' : 'free';
}

function generatePayload() {
  const slotDetail = Array.from({ length: TOTAL_SLOTS }, () => randomSlot());
  const occupied = slotDetail.filter(s => s === 'occupied').length;
  const free = TOTAL_SLOTS - occupied;
  const distanceCm = slotDetail.map(s => s === 'occupied'
    ? Number((8 + Math.random() * 14).toFixed(1))
    : Number((65 + Math.random() * 55).toFixed(1)));

  // Fire alert: ~15% chance
  const fireAlert = Math.random() < 0.15;
  const gasValue = fireAlert
    ? 2300 + Math.floor(Math.random() * 500)
    : 900 + Math.floor(Math.random() * 700);

  // Gates: random open/closed
  const gateEntry = Math.random() < 0.5 ? 'open' : 'closed';
  const gateExit = Math.random() < 0.5 ? 'open' : 'closed';
  const rfidAuthorized = Math.random() < 0.8;
  const fakeUid = ['DE', 'AD', 'BE', 'EF'].map(part =>
    Math.random() < 0.35 ? part : Math.floor(Math.random() * 255).toString(16).padStart(2, '0').toUpperCase()
  ).join(':');

  return {
    slots: {
      total: TOTAL_SLOTS,
      occupied: occupied,
      free: free
    },
    slot_detail: slotDetail,
    distance_cm: distanceCm,
    gate_entry: gateEntry,
    gate_exit: gateExit,
    fire_alert: fireAlert,
    sensors: {
      mq5_gas_value: gasValue,
      mq5_threshold: 2000,
      ir_entry_count: Math.floor(Math.random() * 20),
      ir_exit_count: Math.floor(Math.random() * 20),
      last_ir_event: Math.random() < 0.5 ? 'entry' : 'exit'
    },
    rfid: {
      last_uid: fakeUid,
      authorized: rfidAuthorized,
      last_seen_at: Math.floor(Date.now() / 1000)
    }
  };
}

// ── Publish loop ─────────────────────────────────────────────────
function startPublishing() {
  setInterval(() => {
    const payload = generatePayload();
    const json = JSON.stringify(payload);

    client.publish(CONFIG.topic, json, { qos: 1 }, err => {
      if (err) {
        console.error('Publish error:', err.message);
      } else {
        const slots = payload.slot_detail.map((s, i) =>
          `Slot${i + 1}:${s === 'occupied' ? '🔴' : '🟢'}`
        ).join('  ');

        console.log(
          `[${new Date().toLocaleTimeString()}]  ` +
          `${slots}  ` +
          `Entry:${payload.gate_entry}  Exit:${payload.gate_exit}  ` +
          `Fire:${payload.fire_alert ? '🔥 YES' : 'No'}`
        );
      }
    });
  }, INTERVAL_MS);
}
