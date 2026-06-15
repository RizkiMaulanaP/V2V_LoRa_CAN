#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "HT_TinyGPS++.h"
#include <SPI.h>
#include <mcp_can.h>
#include <BMI160Gen.h>
#include "HT_st7735.h"

// ==========================================
// LORA
// ==========================================
#define RF_FREQUENCY                921000000
#define TX_OUTPUT_POWER             20     // dBm (SX1262 max 22; raised from 7 for link budget)
#define LORA_BANDWIDTH              0
#define LORA_SPREADING_FACTOR       9      // SF8: ~2x less airtime than SF9 (−3 dB range)
#define LORA_CODINGRATE             4      // 4/8 (was 1=4/5): more FEC vs interference
#define LORA_PREAMBLE_LENGTH        8
#define LORA_SYMBOL_TIMEOUT         0
#define LORA_FIX_LENGTH_PAYLOAD_ON  false
#define LORA_IQ_INVERSION_ON        false

// ── CAM-style transmit triggering (ETSI-inspired) ──────────────────────────
// Broadcast when the pose has changed meaningfully OR the heartbeat is due,
// bounded by [T_MIN, T_MAX]. A fast/turning car beacons more often; a steady
// car backs off. Checked every CAM_CHECK_MS.
#define CAM_T_MIN_MS    500    // never faster than this (duty-cycle floor)
#define CAM_T_MAX_MS    2000   // heartbeat: send at least this often
#define CAM_CHECK_MS    100    // how often the trigger conditions are evaluated
#define CAM_DIST_M      4.0f   // position change that forces a broadcast
#define CAM_HEAD_DEG    4.0f   // heading change that forces a broadcast
#define CAM_SPEED_KMH   2      // speed change (km/h) that forces a broadcast
#define EMERGENCY_COOLDOWN          3000   // ms
#define NODE_ID  1


// ==========================================
// SHARED SPI BUS  (MCP2515 CAN + BMI160 IMU)
// SCK=GP9   MISO=GP11 (SO)   MOSI=GP10 (SI)
// ==========================================
#define SPI_SCK_PIN         9
#define SPI_MISO_PIN        11
#define SPI_MOSI_PIN        10

// ==========================================
// CAN BUS / OBD-II   (MCP2515, CS0=GP15, INT=GP16)
// ==========================================
#define CAN_CS_PIN          15
#define CAN_INT_PIN         16
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
// STATUS DISPLAY + BUZZER (proto board)
// Built-in ST7735 TFT shares Vext/VGNSS_CTRL (GPIO3).
// ==========================================
#define BUZZER_PIN   4      // active buzzer — single beep on config error

// Activity LEDs — blink once per detected event (non-blocking).
#define LED_CAN_PIN   17    // LED1: CAN frame received
#define LED_LORA_PIN  6     // LED2: LoRa packet TX/RX
#define LED_GPS_PIN   5     // LED3: GPS sentence decoded
#define LED_BLINK_MS  30    // on-time per blink

// ==========================================
// IMU — BMI160 over SPI (CS = CS1 = GP7)
// Shares the SCK/MISO/MOSI bus with the CAN controller.
// ==========================================
#define IMU_CS_PIN           7
#define IMU_ACCEL_RANGE_G    8       // ±8 g
#define IMU_GYRO_RANGE_DPS   500     // ±500 °/s
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
#define CMD_BUZZER          0x04   // Drive GP4 buzzer; pattern id in alert_type

// V2V alert_type encoding (broadcast in V2V_Payload, shared with host/app).
#define ALERT_NORMAL        0
#define ALERT_TRAFFIC_JAM   1
#define ALERT_BRAKE         2      // hard braking detected (EEBL)

// Buzzer patterns (must match v2v_warnings.py BUZZER_* / CMD_BUZZER payload).
#define BUZZ_OFF            0
#define BUZZ_WARNING        1      // single 1000 ms beep
#define BUZZ_DANGER         2      // rapid beep, ~2 s total
#define BUZZ_GPS            3      // distinct short double-chirp

// Hard-brake detection (sender side, drives ALERT_BRAKE broadcast).
#define BRAKE_DECEL_KMHS    18     // km/h drop per second that counts as hard braking
#define BRAKE_HOLD_MS       2000   // keep broadcasting ALERT_BRAKE this long after

#define FW_VERSION          3      // protocol v3: 11-byte compact over-air payload
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

// Internal full-precision representation of own/neighbour state.
struct __attribute__((packed)) V2V_Payload {
    uint8_t  node_id;
    uint32_t tx_ts_ms;     // sender's millis() at broadcast
    float    latitude;
    float    longitude;
    float    heading_deg;  // 0..360
    uint8_t  speed;
    uint8_t  alert_type;
};  // 19 bytes (internal only — NOT the over-air format)

// ── Compact over-air payload (11 bytes, global reference) ──────────────────
// Cuts LoRa airtime vs the 19-byte struct. Layout (little-endian per field):
//   [0]      node_id(6b) | alert(2b)
//   [1]      heading  (v*360/256, 1.41° step)
//   [2]      speed    (km/h)
//   [3..5]   latitude  24-bit over -90..+90   (~1.2 m)
//   [6..8]   longitude 24-bit over -180..+180 (~2.4 m)
//   [9..10]  tx_ts (millis & 0xFFFF, wraps ~65 s)
// Global reference → no shared/region config needed; both nodes just agree on
// this fixed scale (which they do by running the same firmware).
#define V2V_AIR_SIZE   11
#define V2V_FIX24_MAX  16777215.0   // 2^24 - 1
// Pack/unpack helpers are defined further down (after the GLOBALS section) to
// keep them below the CanStatus enum — otherwise Arduino's auto-generated
// prototypes land before that enum and fail to compile.

// ==========================================
// GLOBALS
// ==========================================
MCP_CAN     CAN(CAN_CS_PIN);
TinyGPSPlus GPS;
HT_st7735   st7735;
// IMU uses the library's global `BMI160` instance (from BMI160Gen.h)

// Raw 16-bit -> physical-unit scale factors (derived from the ranges above)
static const float IMU_ACCEL_SCALE = (float)IMU_ACCEL_RANGE_G  / 32768.0f * 9.80665f;              // -> m/s²
static const float IMU_GYRO_SCALE  = (float)IMU_GYRO_RANGE_DPS / 32768.0f * (float)(M_PI / 180.0); // -> rad/s

V2V_Payload my_data;
V2V_Payload surrounding_vehicles[20];

static RadioEvents_t RadioEvents;

// CAN configuration result (more specific than a plain ok/fail)
enum CanStatus {
    CAN_STAT_OK = 0,      // MCP2515 found AND a frame was acknowledged on the bus
    CAN_STAT_NO_MODULE,   // MCP2515 itself didn't respond over SPI (unrecognized)
    CAN_STAT_NO_BUS       // MCP2515 ok, but no node acknowledged -> bus not connected
};
CanStatus canStatus = CAN_STAT_NO_MODULE;
uint8_t   canStatReg   = 0xFF;   // CANSTAT read at 1 MHz  (slow, reliable)
uint8_t   canStatReg10 = 0xFF;   // CANSTAT read at 10 MHz (the library's actual speed)
uint8_t   imuChipId    = 0x00;   // BMI160 CHIP_ID read at boot (0xD1 expected)

bool canReady      = false;
bool canSpeedValid = false;
bool gpsFixed      = false;
bool imuReady      = false;
bool isTransmitting = false;
bool hostConnected  = false;
bool hostOverridesPose = false;   // true once host pushes CMD_UPDATE_STATE
unsigned long lastHostActivity = 0;

unsigned long lastCANRequest = 0;
unsigned long lastIMUSample  = 0;
unsigned long lastSendTime   = 0;
unsigned long lastEmergencyTime = 0;
int           randomJitter      = 0;

// CAM-trigger state: pose/time at the last broadcast + last evaluation tick.
unsigned long lastCamCheck   = 0;
float         lastTxLat      = 0.0f;
float         lastTxLon      = 0.0f;
float         lastTxHeading  = 0.0f;
uint8_t       lastTxSpeed    = 0;

// Activity-LED auto-off deadlines (0 = LED currently off).
unsigned long ledCanOffAt  = 0;
unsigned long ledLoraOffAt = 0;
unsigned long ledGpsOffAt  = 0;

// Non-blocking buzzer pattern player (GP4).
uint8_t       buzzPattern   = BUZZ_OFF;  // pattern currently playing
uint8_t       buzzStep      = 0;         // index into the pattern's on/off steps
unsigned long buzzStepUntil = 0;         // when the current step ends

// Hard-brake detection (sender side).
uint8_t       lastBrakeSpeed   = 0;      // km/h at previous OBD speed sample
unsigned long lastBrakeSpeedTs = 0;      // millis of that sample
unsigned long brakeHoldUntil   = 0;      // broadcast ALERT_BRAKE until this time

// PID cycling for OBD requests
static const uint8_t CAN_PID_CYCLE[] = {
    OBD_PID_SPEED, OBD_PID_RPM, OBD_PID_SPEED, OBD_PID_COOLANT
};
static uint8_t canPidStep = 0;

// Latest extended telemetry (pure pass-through, not used in fusion)
uint16_t lastRPM      = 0;
int16_t  lastCoolantC = 0;

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
            // Host-driven alert: send promptly, but not mid-TX.
            if (!isTransmitting) {
                BroadcastData();   // packs compact, sends, snapshots CAM state
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

        case CMD_BUZZER:
            // Host-decided warning annunciation; pattern id rides in alert_type.
            startBuzzer(cmd->alert_type);
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
// CAN SPI LINK SELF-TEST
// Talks to the MCP2515 directly (bypassing the library) to tell a dead
// SPI link from a present-but-unconfigurable chip. Resets the chip, then
// reads CANSTAT — which must read 0x80 (config mode) right after reset.
//   0x00 / 0xFF -> no MISO data: wiring, power, or level-shifter problem
//   0x80        -> SPI link is fine; chip is alive
// ==========================================
static uint8_t canReadCanstat(uint32_t hz) {
    SPI.beginTransaction(SPISettings(hz, MSBFIRST, SPI_MODE0));
    digitalWrite(CAN_CS_PIN, LOW);
    SPI.transfer(0x03);     // READ instruction
    SPI.transfer(0x0E);     // CANSTAT register address
    uint8_t v = SPI.transfer(0x00);
    digitalWrite(CAN_CS_PIN, HIGH);
    SPI.endTransaction();
    return v;
}

void canSpiSelfTest() {
    pinMode(CAN_CS_PIN, OUTPUT);
    digitalWrite(CAN_CS_PIN, HIGH);

    // RESET instruction (0xC0) at a safe 1 MHz
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    digitalWrite(CAN_CS_PIN, LOW);
    SPI.transfer(0xC0);
    digitalWrite(CAN_CS_PIN, HIGH);
    SPI.endTransaction();
    delay(10);

    // Read CANSTAT at both speeds. After reset both must read 0x80.
    // If 1 MHz = 0x80 but 10 MHz != 0x80, the SPI link can't sustain the
    // library's 10 MHz clock (series-resistor level shifting too slow).
    canStatReg   = canReadCanstat(1000000);
    canStatReg10 = canReadCanstat(10000000);
}

// ==========================================
// CAN INITIALISATION + BUS PROBE
// Separates "MCP2515 not found" from "bus not connected".
// ==========================================
CanStatus initCAN() {
    // Raw SPI link check first (records CANSTAT for the diagnostic screen).
    canSpiSelfTest();

    // Step 1: does the MCP2515 respond over SPI at all?
    if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) {
        return CAN_STAT_NO_MODULE;
    }
    CAN.setMode(MCP_NORMAL);

    // Step 2: probe the bus. A CAN frame is ACKed at bit level by *any* other
    // node; with nothing connected there is no ACK, so the frame never leaves
    // the buffer and the TX error counter climbs.
    uint8_t req[8] = {0x02, OBD_MODE_CURRENT, OBD_PID_SPEED,
                      0x55, 0x55, 0x55, 0x55, 0x55};
    if (CAN.sendMsgBuf(OBD_REQUEST_ID, 0, 8, req) != CAN_OK ||
        CAN.errorCountTX() > 0) {
        return CAN_STAT_NO_BUS;
    }
    return CAN_STAT_OK;
}

// ==========================================
// LORA BROADCAST
// ==========================================
// ==========================================
// ACTIVITY LEDs (non-blocking single blink)
// ==========================================
void blinkLed(uint8_t pin, unsigned long &offAt) {
    digitalWrite(pin, HIGH);
    offAt = millis() + LED_BLINK_MS;
}

// Turn each activity LED back off once its blink window has elapsed.
void serviceLeds() {
    unsigned long now = millis();
    if (ledCanOffAt  && now >= ledCanOffAt)  { digitalWrite(LED_CAN_PIN,  LOW); ledCanOffAt  = 0; }
    if (ledLoraOffAt && now >= ledLoraOffAt) { digitalWrite(LED_LORA_PIN, LOW); ledLoraOffAt = 0; }
    if (ledGpsOffAt  && now >= ledGpsOffAt)  { digitalWrite(LED_GPS_PIN,  LOW); ledGpsOffAt  = 0; }
}

// ==========================================
// BUZZER PATTERN PLAYER (non-blocking, GP4)
// ==========================================
// Each pattern is a list of on/off durations (ms); even steps = ON, odd = OFF.
// Warning: two quick beeps.
static const uint16_t BUZZ_PAT_WARNING[] = {90, 110, 90};
// Danger: the same two-beep burst repeated for ~2.5 s (burst = on,off,on,gap).
static const uint16_t BUZZ_PAT_DANGER[]  = {90, 110, 90, 320,
                                            90, 110, 90, 320,
                                            90, 110, 90, 320,
                                            90, 110, 90, 320,
                                            90, 110, 90};
static const uint16_t BUZZ_PAT_GPS[]     = {150, 120, 150};              // double chirp

static const uint16_t *buzzSteps = nullptr;
static uint8_t          buzzLen  = 0;

void startBuzzer(uint8_t pattern) {
    buzzPattern = pattern;
    buzzStep    = 0;
    switch (pattern) {
        case BUZZ_WARNING: buzzSteps = BUZZ_PAT_WARNING;
                           buzzLen = sizeof(BUZZ_PAT_WARNING)/sizeof(uint16_t); break;
        case BUZZ_DANGER:  buzzSteps = BUZZ_PAT_DANGER;
                           buzzLen = sizeof(BUZZ_PAT_DANGER)/sizeof(uint16_t);  break;
        case BUZZ_GPS:     buzzSteps = BUZZ_PAT_GPS;
                           buzzLen = sizeof(BUZZ_PAT_GPS)/sizeof(uint16_t);     break;
        default:           buzzSteps = nullptr; buzzLen = 0;
                           digitalWrite(BUZZER_PIN, LOW); return;
    }
    digitalWrite(BUZZER_PIN, HIGH);          // step 0 is always ON
    buzzStepUntil = millis() + buzzSteps[0];
}

// Advance the buzzer pattern; call every loop().
void serviceBuzzer() {
    if (buzzSteps == nullptr) return;
    if (millis() < buzzStepUntil) return;
    buzzStep++;
    if (buzzStep >= buzzLen) {                // pattern finished
        digitalWrite(BUZZER_PIN, LOW);
        buzzSteps = nullptr;
        buzzPattern = BUZZ_OFF;
        return;
    }
    digitalWrite(BUZZER_PIN, (buzzStep & 1) ? LOW : HIGH);  // even=ON, odd=OFF
    buzzStepUntil = millis() + buzzSteps[buzzStep];
}

// Flag our own hard braking from the OBD speed trend → broadcast ALERT_BRAKE.
void detectHardBrake(uint8_t speed_kmh) {
    unsigned long now = millis();
    if (lastBrakeSpeedTs != 0) {
        unsigned long dt = now - lastBrakeSpeedTs;
        if (dt > 0 && dt < 3000 && speed_kmh < lastBrakeSpeed) {
            // decel rate in km/h per second
            uint32_t decel = (uint32_t)(lastBrakeSpeed - speed_kmh) * 1000u / dt;
            if (decel >= BRAKE_DECEL_KMHS) brakeHoldUntil = now + BRAKE_HOLD_MS;
        }
    }
    lastBrakeSpeed   = speed_kmh;
    lastBrakeSpeedTs = now;
}

// ── Compact over-air payload codec (see layout comment near V2V_AIR_SIZE) ──
static uint32_t v2v_enc_axis(double v, double lo, double span) {
    double u = (v - lo) / span * V2V_FIX24_MAX;
    if (u < 0) u = 0;
    if (u > V2V_FIX24_MAX) u = V2V_FIX24_MAX;
    return (uint32_t)(u + 0.5);
}
static double v2v_dec_axis(uint32_t u, double lo, double span) {
    return lo + ((double)u / V2V_FIX24_MAX) * span;
}

// Pack a V2V_Payload into out[V2V_AIR_SIZE].
static void v2v_air_pack(uint8_t *out, const V2V_Payload *p) {
    out[0] = (uint8_t)(((p->node_id & 0x3F) << 2) | (p->alert_type & 0x03));
    out[1] = (uint8_t)(lround(p->heading_deg / 360.0 * 256.0) & 0xFF);
    out[2] = p->speed;
    uint32_t la = v2v_enc_axis(p->latitude,  -90.0, 180.0);
    uint32_t lo = v2v_enc_axis(p->longitude, -180.0, 360.0);
    out[3] = la & 0xFF; out[4] = (la >> 8) & 0xFF; out[5] = (la >> 16) & 0xFF;
    out[6] = lo & 0xFF; out[7] = (lo >> 8) & 0xFF; out[8] = (lo >> 16) & 0xFF;
    uint16_t ts = (uint16_t)(p->tx_ts_ms & 0xFFFF);
    out[9] = ts & 0xFF; out[10] = (ts >> 8) & 0xFF;
}

// Unpack air bytes into a V2V_Payload (tx_ts_ms holds the 16-bit value).
static void v2v_air_unpack(const uint8_t *in, V2V_Payload *p) {
    p->node_id    = (in[0] >> 2) & 0x3F;
    p->alert_type = in[0] & 0x03;
    p->heading_deg = (float)in[1] * (360.0f / 256.0f);
    p->speed      = in[2];
    uint32_t la = (uint32_t)in[3] | ((uint32_t)in[4] << 8) | ((uint32_t)in[5] << 16);
    uint32_t lo = (uint32_t)in[6] | ((uint32_t)in[7] << 8) | ((uint32_t)in[8] << 16);
    p->latitude  = (float)v2v_dec_axis(la, -90.0, 180.0);
    p->longitude = (float)v2v_dec_axis(lo, -180.0, 360.0);
    p->tx_ts_ms  = (uint32_t)(in[9] | ((uint16_t)in[10] << 8));
}

// Small-distance metres between two lat/lon (equirectangular; fine for <km).
float posDeltaMeters(float lat1, float lon1, float lat2, float lon2) {
    const float R = 6371000.0f, D2R = 0.01745329252f;
    float dlat = (lat2 - lat1) * D2R;
    float dlon = (lon2 - lon1) * D2R * cosf(lat1 * D2R);
    return R * sqrtf(dlat * dlat + dlon * dlon);
}

void BroadcastData() {
    // Hard braking is safety-critical: it overrides whatever alert is staged.
    if (millis() < brakeHoldUntil) my_data.alert_type = ALERT_BRAKE;
    isTransmitting    = true;
    my_data.tx_ts_ms  = millis();
    uint8_t air[V2V_AIR_SIZE];
    v2v_air_pack(air, &my_data);
    Radio.Send(air, V2V_AIR_SIZE);
    blinkLed(LED_LORA_PIN, ledLoraOffAt);   // LoRa TX activity

    // Snapshot pose/time for the CAM change-detection trigger.
    lastSendTime   = millis();
    lastTxLat      = my_data.latitude;
    lastTxLon      = my_data.longitude;
    lastTxHeading  = my_data.heading_deg;
    lastTxSpeed    = my_data.speed;
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

    // --- Buzzer ---
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);

    // --- Activity LEDs ---
    pinMode(LED_CAN_PIN,  OUTPUT);
    pinMode(LED_LORA_PIN, OUTPUT);
    pinMode(LED_GPS_PIN,  OUTPUT);
    digitalWrite(LED_CAN_PIN,  LOW);
    digitalWrite(LED_LORA_PIN, LOW);
    digitalWrite(LED_GPS_PIN,  LOW);

    // --- GPS power (also powers the built-in TFT via Vext) ---
    pinMode(VGNSS_CTRL, OUTPUT);
    digitalWrite(VGNSS_CTRL, HIGH);
    Serial1.begin(115200, SERIAL_8N1, 33, 34);

    // --- Status display (init before the SPI radio/CAN devices) ---
    st7735.st7735_init();
    st7735.st7735_fill_screen(ST7735_BLACK);
    st7735.st7735_write_str(0, 0, (String)"V2V Node " + NODE_ID,
                            Font_7x10, ST7735_WHITE, ST7735_BLACK);
    st7735.st7735_write_str(0, 20, "Configuring...",
                            Font_7x10, ST7735_YELLOW, ST7735_BLACK);

    // --- Shared SPI bus for CAN + IMU (configure before initializing either) ---
    SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN);

    // --- IMU (BMI160 over SPI, CS = GP7; -1 = no interrupt pin, we poll) ---
    imuReady  = BMI160.begin(BMI160GenClass::SPI_MODE, IMU_CS_PIN, -1);
    imuChipId = BMI160.getDeviceID();   // 0xD1 expected
    if (imuReady) {
        BMI160.setAccelerometerRange(IMU_ACCEL_RANGE_G);
        BMI160.setGyroRange(IMU_GYRO_RANGE_DPS);
    }

    // --- LoRa ---
    RadioEvents.TxDone    = OnTxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxDone    = OnRxDone;
    RadioEvents.RxTimeout = OnRxTimeout;
    RadioEvents.RxError   = OnRxError;

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

    // --- CAN Bus (MCP2515) ---
    Radio.Standby();
    pinMode(CAN_INT_PIN, INPUT);
    canStatus = initCAN();
    // Keep polling whenever the chip itself responds; "no bus" may just mean
    // the vehicle isn't powered yet and can recover at runtime.
    canReady  = (canStatus != CAN_STAT_NO_MODULE);

    randomSeed(analogRead(0));
    Radio.Rx(0);

    // --- Report configuration result on the TFT (beep once if anything failed) ---
    showConfigStatus();
}

// ==========================================
// CONFIG STATUS SCREEN
// Lists each subsystem; names exactly what's missing/not
// working and sounds a single beep on any error or warning.
// ==========================================
void showConfigStatus() {
    // Build one coloured line per subsystem.
    String   imuLine;
    uint16_t imuColor;
    if (imuReady) {
        // Live sample: at rest one axis should read ~±4096 (1 g at ±8 g).
        // All-zero here means init claimed OK but the sensor isn't producing data.
        int ax, ay, az, gx, gy, gz;
        BMI160.readMotionSensor(ax, ay, az, gx, gy, gz);
        imuLine  = (String)"OK IMU az=" + az;
        imuColor = ST7735_GREEN;
    } else {
        // begin() failed: show what CHIP_ID actually read (0xD1 = chip present).
        imuLine  = (String)"X IMU id=0x" + String(imuChipId, HEX);
        imuColor = ST7735_RED;
    }

    String   canLine;
    uint16_t canColor;
    switch (canStatus) {
        case CAN_STAT_OK:
            canLine = "OK  CAN";                canColor = ST7735_GREEN;  break;
        case CAN_STAT_NO_BUS:
            canLine = "!   CAN: no bus link";   canColor = ST7735_YELLOW; break;
        case CAN_STAT_NO_MODULE:
        default:
            canLine = "X   CAN: no MCP2515";    canColor = ST7735_RED;    break;
    }

    bool err  = (!imuReady) || (canStatus == CAN_STAT_NO_MODULE);
    bool warn = (canStatus == CAN_STAT_NO_BUS);

    st7735.st7735_fill_screen(ST7735_BLACK);
    st7735.st7735_write_str(0, 0, (String)"V2V Node " + NODE_ID,
                            Font_7x10, ST7735_WHITE, ST7735_BLACK);
    st7735.st7735_write_str(0, 12,
                            err ? "CONFIG ERROR" : (warn ? "CONFIG WARN" : "Config OK"),
                            Font_7x10,
                            err ? ST7735_RED : (warn ? ST7735_YELLOW : ST7735_GREEN),
                            ST7735_BLACK);

    st7735.st7735_write_str(0, 28, imuLine, Font_7x10, imuColor, ST7735_BLACK);
    st7735.st7735_write_str(0, 40, canLine, Font_7x10, canColor, ST7735_BLACK);

    // When the chip is unrecognized, show CANSTAT read at both SPI speeds:
    //   1M=0x80 10M=0x80  -> link fine; failure is elsewhere
    //   1M=0x80 10M!=0x80 -> chip ok, but 10 MHz link too slow (series-R)
    //   1M=0x00/0xFF      -> SPI link dead (wiring/power)
    if (canStatus == CAN_STAT_NO_MODULE) {
        String dl = (String)"1M:0x" + String(canStatReg, HEX)
                    + " 10M:0x" + String(canStatReg10, HEX);
        bool clockLimited = (canStatReg == 0x80 && canStatReg10 != 0x80);
        st7735.st7735_write_str(0, 52, dl, Font_7x10,
                                clockLimited ? ST7735_YELLOW : ST7735_RED, ST7735_BLACK);
    }

    if (err || warn) {
        // Single notification beep
        digitalWrite(BUZZER_PIN, HIGH);
        delay(300);
        digitalWrite(BUZZER_PIN, LOW);
    }
}

// ==========================================
// LOOP
// ==========================================
void loop() {
    Radio.IrqProcess();
    serviceLeds();
    serviceBuzzer();

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

        int axR, ayR, azR, gxR, gyR, gzR;
        BMI160.readMotionSensor(axR, ayR, azR, gxR, gyR, gzR);
        // Scale raw 16-bit readings -> accel in m/s², gyro in rad/s
        sendIMUFrame(axR * IMU_ACCEL_SCALE, ayR * IMU_ACCEL_SCALE, azR * IMU_ACCEL_SCALE,
                     gxR * IMU_GYRO_SCALE,  gyR * IMU_GYRO_SCALE,  gzR * IMU_GYRO_SCALE);
    }

    // --- C. GPS ---
    while (Serial1.available() > 0) {
        if (GPS.encode(Serial1.read())) blinkLed(LED_GPS_PIN, ledGpsOffAt);  // GPS activity
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

    // --- D. OBD VIA CAN — cycle speed / RPM / speed / coolant ---
    if (canReady) {
        if (millis() - lastCANRequest >= CAN_REQUEST_INTERVAL) {
            uint8_t pid = CAN_PID_CYCLE[canPidStep];
            uint8_t req[8] = {0x02, OBD_MODE_CURRENT, pid,
                              0x55, 0x55, 0x55, 0x55, 0x55};
            CAN.sendMsgBuf(OBD_REQUEST_ID, 0, 8, req);
            canPidStep     = (canPidStep + 1) % (sizeof(CAN_PID_CYCLE) / sizeof(CAN_PID_CYCLE[0]));
            lastCANRequest = millis();
        }

        if (digitalRead(CAN_INT_PIN) == LOW) {
            uint32_t canId;
            uint8_t  canLen, canBuf[8];
            if (CAN.readMsgBuf(&canId, &canLen, canBuf) == CAN_OK) {
                blinkLed(LED_CAN_PIN, ledCanOffAt);   // CAN activity
                if (canId == OBD_RESPONSE_ID && canLen >= 3 && canBuf[1] == 0x41) {
                    uint8_t pid = canBuf[2];
                    if (pid == OBD_PID_SPEED && canLen >= 4) {
                        if (!hostOverridesPose) my_data.speed = canBuf[3];
                        canSpeedValid = true;
                        detectHardBrake(canBuf[3]);   // EEBL: flag rapid decel
                        sendOBDFrame(canBuf[3]);
                    } else if (pid == OBD_PID_RPM && canLen >= 5) {
                        lastRPM = ((uint16_t)canBuf[3] * 256u + canBuf[4]) / 4u;
                        sendOBDExtFrame(lastRPM, lastCoolantC);
                    } else if (pid == OBD_PID_COOLANT && canLen >= 4) {
                        lastCoolantC = (int16_t)canBuf[3] - 40;
                        sendOBDExtFrame(lastRPM, lastCoolantC);
                    }
                }
            }
        }
    }

    // --- E. CAM-STYLE LORA BROADCAST ---------------------------------------
    // Send when the pose changed meaningfully (distance / heading / speed) or
    // the heartbeat is due, bounded by [CAM_T_MIN_MS, CAM_T_MAX_MS]. A fast or
    // turning car beacons more often; a steady car backs off. Only broadcast
    // with a usable pose so peers never get a pre-fix placeholder.
    bool poseValid = gpsFixed || hostOverridesPose;
    if (!isTransmitting && poseValid && millis() - lastCamCheck >= CAM_CHECK_MS) {
        lastCamCheck = millis();
        unsigned long dt = millis() - lastSendTime;

        float dPos  = posDeltaMeters(lastTxLat, lastTxLon,
                                     my_data.latitude, my_data.longitude);
        float dHead = fabsf(my_data.heading_deg - lastTxHeading);
        if (dHead > 180.0f) dHead = 360.0f - dHead;          // wrap
        int   dSpd  = abs((int)my_data.speed - (int)lastTxSpeed);

        bool changed = (dPos  > CAM_DIST_M) ||
                       (dHead > CAM_HEAD_DEG) ||
                       (dSpd >= CAM_SPEED_KMH);
        // Heartbeat gets a little jitter so two idle cars don't lock-step.
        bool heartbeat = dt >= (unsigned long)(CAM_T_MAX_MS + randomJitter);

        if ((changed && dt >= CAM_T_MIN_MS) || heartbeat) {
            if (!hostOverridesPose) {
                bool isTrafficJam  = (my_data.speed < 10 && gpsFixed);
                my_data.alert_type = isTrafficJam ? ALERT_TRAFFIC_JAM : ALERT_NORMAL;
            }
            BroadcastData();                 // also snapshots pose/time
            randomJitter = random(0, 300);   // re-roll heartbeat jitter
        }
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

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
    blinkLed(LED_LORA_PIN, ledLoraOffAt);   // LoRa RX activity
    if (size == V2V_AIR_SIZE) {
        V2V_Payload incoming;
        v2v_air_unpack(payload, &incoming);

        if (incoming.node_id != my_data.node_id && incoming.node_id < 20) {
            surrounding_vehicles[incoming.node_id] = incoming;
            sendLoRaRxFrame(incoming, rssi, snr);   // serial format unchanged
        }
    }
    Radio.Rx(0);
}

void OnRxTimeout(void) { Radio.Rx(0); }
void OnRxError(void)   { Radio.Rx(0); }

// ==========================================
// NODE CONFIG (adjust per unit)
// ==========================================
