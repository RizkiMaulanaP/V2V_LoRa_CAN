#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "HT_TinyGPS++.h"
#include <SPI.h>
#include <BMI160Gen.h>
#include "driver/twai.h"

// ==========================================
// LORA
// ==========================================
#define RF_FREQUENCY                921000000
#define TX_OUTPUT_POWER             20     // dBm (SX1262 max 22; raised from 7 for link budget)
#define LORA_BANDWIDTH              0
#define LORA_SPREADING_FACTOR       9      // raised from 7: +~6 dB sensitivity
#define LORA_CODINGRATE             4      // 4/8 (was 1=4/5): more FEC vs interference
#define LORA_PREAMBLE_LENGTH        8
#define LORA_SYMBOL_TIMEOUT         0
#define LORA_FIX_LENGTH_PAYLOAD_ON  false
#define LORA_IQ_INVERSION_ON        false
#define LORA_TX_INTERVAL            2000   // ms base interval
#define EMERGENCY_COOLDOWN          3000   // ms
#define NODE_ID  5


// ==========================================
// CAN BUS / OBD-II — ESP32-S3 TWAI
// ==========================================
#define TWAI_TX_GPIO        GPIO_NUM_4
#define TWAI_RX_GPIO        GPIO_NUM_5
#define OBD_REQUEST_ID      0x7DF
#define OBD_RESPONSE_ID     0x7E8
#define OBD_MODE_CURRENT    0x01
#define OBD_PID_SPEED       0x0D    // km/h               (1 byte)
#define OBD_PID_RPM         0x0C    // ((A*256)+B)/4 rpm  (2 bytes)
#define OBD_PID_COOLANT     0x05    // A - 40 °C          (1 byte)
#define CAN_REQUEST_INTERVAL 250   // ms per PID slot; cycle = {SPD,RPM,SPD,COOL}

// ==========================================
// GPS
// ==========================================
#define VGNSS_CTRL  3

// ==========================================
// IMU — BMI160 via SPI (CS=15, shared default SPI bus)
// ==========================================
#define IMU_CS_PIN           15
#define IMU_ACCEL_RANGE_G    8       // ±8g
#define IMU_GYRO_RANGE_DPS   500     // ±500°/s
#define IMU_SAMPLE_MS        10      // 100 Hz

// ==========================================
// SERIAL PROTOCOL — USB CDC
// Frame:   [0xAA | type | ts(4) | payload | checksum | 0x55]
// Command: [0xBB | cmd  | payload | checksum | 0x55]
// Checksum: XOR of all bytes between start and end (exclusive)
// ==========================================
#define FRAME_START     0xAA
#define FRAME_END       0x55
#define FRAME_IMU       0x01
#define FRAME_GPS       0x02
#define FRAME_OBD       0x03
#define FRAME_LORA_RX   0x04
#define FRAME_HELLO     0x05   // Sent in reply to CMD_HELLO
#define FRAME_OBD_EXT   0x06   // Extra OBD telemetry (RPM, coolant) — not filtered

#define CMD_START           0xBB
#define CMD_BROADCAST       0x01   // Immediate LoRa alert TX with provided data
#define CMD_UPDATE_STATE    0x02   // Update state used in next periodic TX
#define CMD_HELLO           0x03   // Host handshake / heartbeat

#define FW_VERSION          2      // protocol v2: V2V_Payload carries heading + tx_ts
#define HOST_TIMEOUT_MS     3000   // No host CMD for this long → disconnect

// ==========================================
// FRAME STRUCTS
// ==========================================
struct __attribute__((packed)) IMUFrame {
    uint8_t  start;
    uint8_t  type;
    uint32_t timestamp_ms;
    float    ax, ay, az;   // m/s²
    float    gx, gy, gz;   // rad/s
    uint8_t  checksum;
    uint8_t  end;
};  // 32 bytes

struct __attribute__((packed)) GPSFrame {
    uint8_t  start;
    uint8_t  type;
    uint32_t timestamp_ms;
    double   latitude;
    double   longitude;
    float    speed_kmh;
    float    hdop;
    uint8_t  satellites;
    uint8_t  fix_valid;
    uint8_t  checksum;
    uint8_t  end;
};  // 34 bytes

struct __attribute__((packed)) OBDFrame {
    uint8_t  start;
    uint8_t  type;
    uint32_t timestamp_ms;
    uint8_t  speed_kmh;
    uint8_t  checksum;
    uint8_t  end;
};  // 9 bytes

struct __attribute__((packed)) OBDExtFrame {
    uint8_t  start;
    uint8_t  type;
    uint32_t timestamp_ms;
    uint16_t engine_rpm;       // raw RPM
    int16_t  coolant_c;        // °C (signed)
    uint8_t  checksum;
    uint8_t  end;
};  // 12 bytes

struct __attribute__((packed)) HelloFrame {
    uint8_t  start;
    uint8_t  type;
    uint32_t timestamp_ms;
    uint8_t  node_id;
    uint8_t  fw_version;
    uint8_t  checksum;
    uint8_t  end;
};  // 10 bytes

struct __attribute__((packed)) LoRaRxFrame {
    uint8_t  start;
    uint8_t  type;
    uint32_t timestamp_ms;      // local rx ts (millis at receive)
    uint32_t remote_tx_ts_ms;   // sender's millis at TX (from V2V_Payload)
    uint8_t  node_id;
    float    latitude;
    float    longitude;
    float    heading_deg;       // 0..360
    uint8_t  speed;
    uint8_t  alert_type;
    int16_t  rssi;
    int8_t   snr;
    uint8_t  checksum;
    uint8_t  end;
};  // 30 bytes

struct __attribute__((packed)) CmdFrame {
    uint8_t  start;        // CMD_START
    uint8_t  cmd_type;
    uint8_t  alert_type;
    float    latitude;
    float    longitude;
    float    heading_deg;  // host's fused heading (0..360)
    uint8_t  speed_kmh;
    uint8_t  checksum;
    uint8_t  end;          // FRAME_END
};  // 18 bytes

struct __attribute__((packed)) V2V_Payload {
    uint8_t  node_id;
    uint32_t tx_ts_ms;     // sender's millis() at broadcast
    float    latitude;
    float    longitude;
    float    heading_deg;  // 0..360
    uint8_t  speed;
    uint8_t  alert_type;
};  // 19 bytes

// ==========================================
// GLOBALS
// ==========================================
TinyGPSPlus GPS;

// BMI160 scaling: raw int16 → SI units (m/s² and rad/s)
static const float IMU_ACCEL_SCALE =
    (float)IMU_ACCEL_RANGE_G * 9.80665f / 32768.0f;
static const float IMU_GYRO_SCALE  =
    (float)IMU_GYRO_RANGE_DPS * ((float)M_PI / 180.0f) / 32768.0f;

V2V_Payload my_data;
V2V_Payload surrounding_vehicles[20];

static RadioEvents_t RadioEvents;

bool canReady      = false;
bool canSpeedValid = false;
bool gpsFixed      = false;
bool imuReady      = false;
bool isTransmitting = false;
bool cadPending     = false;   // true while a Channel-Activity-Detect (pre-TX listen) is in flight
bool hostConnected  = false;
unsigned long lastHostActivity = 0;

unsigned long lastCANRequest      = 0;
unsigned long lastIMUSample       = 0;
unsigned long lastSendTime        = 0;
unsigned long lastEmergencyTime   = 0;
int           randomJitter        = 0;

// PID cycling for OBD requests
static const uint8_t CAN_PID_CYCLE[] = {
    OBD_PID_SPEED, OBD_PID_RPM, OBD_PID_SPEED, OBD_PID_COOLANT
};
static uint8_t canPidStep = 0;

// Latest extended telemetry (pure pass-through, not used in fusion)
uint16_t lastRPM      = 0;
int16_t  lastCoolantC = 0;
bool     hostOverridesPose = false;   // true once host pushes CMD_UPDATE_STATE

static uint8_t cmdBuf[sizeof(CmdFrame)];
static uint8_t cmdIdx    = 0;
static bool    cmdActive = false;

// ==========================================
// UTILITY
// ==========================================
static uint8_t xorChecksum(const uint8_t *buf, size_t len) {
    uint8_t cs = 0;
    for (size_t i = 0; i < len; i++) cs ^= buf[i];
    return cs;
}

// ==========================================
// FRAME SENDERS
// ==========================================
void sendHelloFrame() {
    HelloFrame f;
    f.start        = FRAME_START;
    f.type         = FRAME_HELLO;
    f.timestamp_ms = millis();
    f.node_id      = NODE_ID;
    f.fw_version   = FW_VERSION;
    f.checksum     = xorChecksum((uint8_t*)&f + 1, sizeof(f) - 3);
    f.end          = FRAME_END;
    // HELLO always goes out — it's the response to the host's handshake
    Serial.write((uint8_t*)&f, sizeof(f));
}

void sendIMUFrame(float ax, float ay, float az, float gx, float gy, float gz) {
    if (!hostConnected) return;
    IMUFrame f;
    f.start        = FRAME_START;
    f.type         = FRAME_IMU;
    f.timestamp_ms = millis();
    f.ax = ax; f.ay = ay; f.az = az;
    f.gx = gx; f.gy = gy; f.gz = gz;
    f.checksum     = xorChecksum((uint8_t*)&f + 1, sizeof(f) - 3);
    f.end          = FRAME_END;
    Serial.write((uint8_t*)&f, sizeof(f));
}

void sendGPSFrame() {
    if (!hostConnected) return;
    GPSFrame f;
    f.start        = FRAME_START;
    f.type         = FRAME_GPS;
    f.timestamp_ms = millis();
    f.latitude     = GPS.location.lat();
    f.longitude    = GPS.location.lng();
    f.speed_kmh    = GPS.speed.kmph();
    f.hdop         = GPS.hdop.hdop();
    f.satellites   = GPS.satellites.value();
    f.fix_valid    = GPS.location.isValid() ? 1 : 0;
    f.checksum     = xorChecksum((uint8_t*)&f + 1, sizeof(f) - 3);
    f.end          = FRAME_END;
    Serial.write((uint8_t*)&f, sizeof(f));
}

void sendOBDFrame(uint8_t speed) {
    if (!hostConnected) return;
    OBDFrame f;
    f.start        = FRAME_START;
    f.type         = FRAME_OBD;
    f.timestamp_ms = millis();
    f.speed_kmh    = speed;
    f.checksum     = xorChecksum((uint8_t*)&f + 1, sizeof(f) - 3);
    f.end          = FRAME_END;
    Serial.write((uint8_t*)&f, sizeof(f));
}

void sendOBDExtFrame(uint16_t rpm, int16_t coolant_c) {
    if (!hostConnected) return;
    OBDExtFrame f;
    f.start        = FRAME_START;
    f.type         = FRAME_OBD_EXT;
    f.timestamp_ms = millis();
    f.engine_rpm   = rpm;
    f.coolant_c    = coolant_c;
    f.checksum     = xorChecksum((uint8_t*)&f + 1, sizeof(f) - 3);
    f.end          = FRAME_END;
    Serial.write((uint8_t*)&f, sizeof(f));
}

void sendLoRaRxFrame(const V2V_Payload &p, int16_t rssi, int8_t snr) {
    if (!hostConnected) return;
    LoRaRxFrame f;
    f.start            = FRAME_START;
    f.type             = FRAME_LORA_RX;
    f.timestamp_ms     = millis();
    f.remote_tx_ts_ms  = p.tx_ts_ms;
    f.node_id          = p.node_id;
    f.latitude         = p.latitude;
    f.longitude        = p.longitude;
    f.heading_deg      = p.heading_deg;
    f.speed            = p.speed;
    f.alert_type       = p.alert_type;
    f.rssi             = rssi;
    f.snr              = snr;
    f.checksum         = xorChecksum((uint8_t*)&f + 1, sizeof(f) - 3);
    f.end              = FRAME_END;
    Serial.write((uint8_t*)&f, sizeof(f));
}

// ==========================================
// COMMAND RECEIVER (microcomputer → MCU)
// ==========================================
void handleCommand(const uint8_t *buf, uint8_t len) {
    if (len < (uint8_t)sizeof(CmdFrame)) return;
    const CmdFrame *cmd = (const CmdFrame*)buf;
    if (cmd->end != FRAME_END) return;
    if (xorChecksum(buf + 1, len - 3) != cmd->checksum) return;

    // Any valid command counts as host activity
    hostConnected    = true;
    lastHostActivity = millis();

    switch (cmd->cmd_type) {
        case CMD_HELLO:
            sendHelloFrame();
            break;

        case CMD_BROADCAST:
            my_data.alert_type  = cmd->alert_type;
            my_data.latitude    = cmd->latitude;
            my_data.longitude   = cmd->longitude;
            my_data.heading_deg = cmd->heading_deg;
            my_data.speed       = cmd->speed_kmh;
            hostOverridesPose   = true;
            // Host-driven alert: send promptly (skip CAD), but not mid-TX/CAD.
            if (!isTransmitting && !cadPending) {
                isTransmitting    = true;
                my_data.tx_ts_ms  = millis();
                Radio.Send((uint8_t*)&my_data, sizeof(V2V_Payload));
                lastSendTime      = millis();
            }
            break;

        case CMD_UPDATE_STATE:
            my_data.latitude    = cmd->latitude;
            my_data.longitude   = cmd->longitude;
            my_data.heading_deg = cmd->heading_deg;
            my_data.speed       = cmd->speed_kmh;
            my_data.alert_type  = cmd->alert_type;
            hostOverridesPose   = true;
            break;
    }
}

void pollCommands() {
    while (Serial.available()) {
        uint8_t b = (uint8_t)Serial.read();

        if (!cmdActive) {
            if (b == CMD_START) {
                cmdActive    = true;
                cmdIdx       = 0;
                cmdBuf[cmdIdx++] = b;
            }
        } else {
            if (cmdIdx < sizeof(cmdBuf)) {
                cmdBuf[cmdIdx++] = b;
            }
            if (b == FRAME_END || cmdIdx >= sizeof(cmdBuf)) {
                handleCommand(cmdBuf, cmdIdx);
                cmdActive = false;
                cmdIdx    = 0;
            }
        }
    }
}

// ==========================================
// LORA BROADCAST
// ==========================================
void BroadcastData() {
    isTransmitting    = true;
    my_data.tx_ts_ms  = millis();
    Radio.Send((uint8_t*)&my_data, sizeof(V2V_Payload));
}

// ==========================================
// TWAI (CAN) HELPERS
// ==========================================
static bool twaiSendOBDRequest(uint8_t pid) {
    twai_message_t msg = {};
    msg.identifier       = OBD_REQUEST_ID;
    msg.extd             = 0;
    msg.rtr              = 0;
    msg.data_length_code = 8;
    msg.data[0] = 0x02;
    msg.data[1] = OBD_MODE_CURRENT;
    msg.data[2] = pid;
    msg.data[3] = 0x55;
    msg.data[4] = 0x55;
    msg.data[5] = 0x55;
    msg.data[6] = 0x55;
    msg.data[7] = 0x55;
    return twai_transmit(&msg, 0) == ESP_OK;
}

// ==========================================
// SETUP
// ==========================================
void setup() {
    // USB CDC — baud rate ignored by CDC but required by API
    Serial.begin(921600);
    Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);

    my_data.node_id     = NODE_ID;
    my_data.tx_ts_ms    = 0;
    my_data.latitude    = 0.0f;
    my_data.longitude   = 0.0f;
    my_data.heading_deg = 0.0f;
    my_data.speed       = 0;
    my_data.alert_type  = 0;

    // --- GPS ---
    pinMode(VGNSS_CTRL, OUTPUT);
    digitalWrite(VGNSS_CTRL, HIGH);
    Serial1.begin(115200, SERIAL_8N1, 33, 34);

    // --- IMU (BMI160 over SPI, CS=15 on default SPI bus) ---
    SPI.begin();
    pinMode(IMU_CS_PIN, OUTPUT);
    digitalWrite(IMU_CS_PIN, HIGH);
    if (BMI160.begin(BMI160GenClass::SPI_MODE, IMU_CS_PIN)) {
        BMI160.setAccelerometerRange(IMU_ACCEL_RANGE_G);
        BMI160.setGyroRange(IMU_GYRO_RANGE_DPS);
        imuReady = true;
    }

    // --- LoRa ---
    RadioEvents.TxDone    = OnTxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxDone    = OnRxDone;
    RadioEvents.RxTimeout = OnRxTimeout;
    RadioEvents.RxError   = OnRxError;
    RadioEvents.CadDone   = OnCadDone;

    Radio.Init(&RadioEvents);
    Radio.SetChannel(RF_FREQUENCY);
    Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                      LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                      LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                      true, 0, 0, LORA_IQ_INVERSION_ON, 3000);
    Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                      LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                      LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                      0, true, 0, 0, LORA_IQ_INVERSION_ON, true);

    // --- CAN Bus via ESP32-S3 TWAI (TX=GPIO4, RX=GPIO5) ---
    Radio.Standby();
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        TWAI_TX_GPIO, TWAI_RX_GPIO, TWAI_MODE_NORMAL);
    twai_timing_config_t  t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t  f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    pinMode(TWAI_TX_GPIO, OUTPUT);
    pinMode(TWAI_RX_GPIO, OUTPUT);
    for (int i = 0; i < 10; i++) {
    digitalWrite(TWAI_TX_GPIO, !digitalRead(TWAI_TX_GPIO));
    digitalWrite(TWAI_RX_GPIO, !digitalRead(TWAI_RX_GPIO));
    delay(50);
}

    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK &&
        twai_start() == ESP_OK) {
        canReady = true;
    }

    randomSeed(analogRead(0));
    Radio.Rx(0);
}

// ==========================================
// LOOP
// ==========================================
void loop() {
    Radio.IrqProcess();

    // --- A. COMMANDS FROM MICROCOMPUTER ---
    pollCommands();

    // Drop the connection if the host has gone silent
    if (hostConnected && (millis() - lastHostActivity > HOST_TIMEOUT_MS)) {
        hostConnected     = false;
        hostOverridesPose = false;   // fall back to raw GPS for my_data
    }

    // --- B. IMU @ 100 Hz ---
    if (imuReady && (millis() - lastIMUSample >= IMU_SAMPLE_MS)) {
        lastIMUSample = millis();

        int rax, ray, raz, rgx, rgy, rgz;
        BMI160.readMotionSensor(rax, ray, raz, rgx, rgy, rgz);

        sendIMUFrame(rax * IMU_ACCEL_SCALE,
                     ray * IMU_ACCEL_SCALE,
                     raz * IMU_ACCEL_SCALE,
                     rgx * IMU_GYRO_SCALE,
                     rgy * IMU_GYRO_SCALE,
                     rgz * IMU_GYRO_SCALE);
    }

    // --- C. GPS ---
    while (Serial1.available() > 0) {
        GPS.encode(Serial1.read());
    }

    if (GPS.location.isUpdated()) {
        if (!gpsFixed && GPS.location.isValid()) gpsFixed = true;

        // Raw-GPS fallback for my_data — only when the host hasn't taken over.
        // Once host pushes CMD_UPDATE_STATE we stop clobbering with raw fixes.
        // Only copy position from a *valid* fix, so we never broadcast garbage
        // coordinates before lock (the source of the wild outliers in logs).
        if (!hostOverridesPose && GPS.location.isValid()) {
            my_data.latitude    = (float)GPS.location.lat();
            my_data.longitude   = (float)GPS.location.lng();
            if (GPS.course.isValid()) {
                my_data.heading_deg = (float)GPS.course.deg();
            }
            if (!canSpeedValid) {
                my_data.speed = (uint8_t)GPS.speed.kmph();
            }
        }

        sendGPSFrame();
    }

    // --- D. OBD VIA TWAI — cycle speed / RPM / speed / coolant ---
    if (canReady) {
        if (millis() - lastCANRequest >= CAN_REQUEST_INTERVAL) {
            uint8_t pid = CAN_PID_CYCLE[canPidStep];
            twaiSendOBDRequest(pid);
            canPidStep    = (canPidStep + 1) % (sizeof(CAN_PID_CYCLE) / sizeof(CAN_PID_CYCLE[0]));
            lastCANRequest = millis();
        }

        twai_message_t rxMsg;
        while (twai_receive(&rxMsg, 0) == ESP_OK) {
            if (rxMsg.rtr || rxMsg.identifier != OBD_RESPONSE_ID) continue;
            if (rxMsg.data_length_code < 3 || rxMsg.data[1] != 0x41) continue;

            uint8_t pid = rxMsg.data[2];
            if (pid == OBD_PID_SPEED && rxMsg.data_length_code >= 4) {
                if (!hostOverridesPose) my_data.speed = rxMsg.data[3];
                canSpeedValid = true;
                sendOBDFrame(rxMsg.data[3]);
            } else if (pid == OBD_PID_RPM && rxMsg.data_length_code >= 5) {
                lastRPM = ((uint16_t)rxMsg.data[3] * 256u + rxMsg.data[4]) / 4u;
                sendOBDExtFrame(lastRPM, lastCoolantC);
            } else if (pid == OBD_PID_COOLANT && rxMsg.data_length_code >= 4) {
                lastCoolantC = (int16_t)rxMsg.data[3] - 40;
                sendOBDExtFrame(lastRPM, lastCoolantC);
            }
        }
    }

    // --- E. PERIODIC LORA BROADCAST (fallback when microcomputer not connected) ---
    // Only broadcast once we have a usable pose (GPS lock or host-supplied),
    // so peers never receive a pre-fix placeholder position.
    bool poseValid = gpsFixed || hostOverridesPose;
    if (!isTransmitting && !cadPending && poseValid &&
        (millis() - lastSendTime >= (unsigned long)(LORA_TX_INTERVAL + randomJitter))) {
        if (!hostOverridesPose) {
            // Local heuristic only when host isn't dictating alert_type
            bool isTrafficJam  = (my_data.speed < 10 && gpsFixed);
            my_data.alert_type = isTrafficJam ? 1 : 0;
        }
        // Listen-before-talk: run CAD first; the actual Send happens in
        // OnCadDone() only if the channel is clear (collision avoidance).
        cadPending = true;
        Radio.StartCad();
    }
}

// ==========================================
// LORA CALLBACKS
// ==========================================
void OnTxDone(void) {
    isTransmitting = false;
    Radio.Rx(0);
}

void OnTxTimeout(void) {
    isTransmitting = false;
    Radio.Rx(0);
}

// Channel-Activity-Detect result for a pending broadcast (listen-before-talk).
void OnCadDone(bool channelActivityDetected) {
    cadPending = false;
    if (channelActivityDetected) {
        // Channel busy: don't transmit. Retry after a short random back-off
        // (not a full interval) so the two nodes desynchronise.
        lastSendTime = millis() - (unsigned long)LORA_TX_INTERVAL + random(30, 250);
        randomJitter = 0;
        Radio.Rx(0);
    } else {
        // Channel clear: send now.
        BroadcastData();
        lastSendTime = millis();
        randomJitter = random(0, 500);
    }
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
    if (size == sizeof(V2V_Payload)) {
        V2V_Payload incoming;
        memcpy(&incoming, payload, sizeof(V2V_Payload));

        if (incoming.node_id != my_data.node_id && incoming.node_id < 20) {
            surrounding_vehicles[incoming.node_id] = incoming;
            sendLoRaRxFrame(incoming, rssi, snr);
        }
    }
    Radio.Rx(0);
}

void OnRxTimeout(void) { Radio.Rx(0); }
void OnRxError(void)   { Radio.Rx(0); }

// ==========================================
// NODE CONFIG (adjust per unit)
// ==========================================
