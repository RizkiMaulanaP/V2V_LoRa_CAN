// ─────────────────────────────────────────────────────────────────────────────
// ESP32-S3 TWAI (CAN) standalone test
//   TX  : GPIO4   (to transceiver CTX)
//   RX  : GPIO5   (from transceiver CRX)
//   Bit : 500 kbit/s   — change TWAI_TIMING_CONFIG_*KBITS() if needed
//
// Modes (pick ONE via the defines below):
//   MODE_NORMAL       — Standard bus operation (requires a transceiver +
//                       at least one other ACK'ing node, or NO_ACK + sniffer)
//   MODE_NO_ACK       — TX without requiring an ACK from another node
//                       (useful for bench-testing with only one ESP32 on bus)
//   MODE_LISTEN_ONLY  — Pure receive, no ACKs, no TX (sniffer)
//
// Features:
//   • Sends an OBD-II speed request (0x7DF) every TX_INTERVAL_MS
//   • Receives and prints every incoming frame
//   • Decodes OBD-II speed responses from 0x7E8
//   • Periodic status: state, TX/RX queue depths, bus error counters
//   • Bus-off recovery attempts
//
// Wiring with a SN65HVD230 / TJA1050 / MCP2562 transceiver:
//   ESP32 GPIO4  → transceiver CTX/TXD
//   ESP32 GPIO5  → transceiver CRX/RXD
//   Transceiver CANH / CANL → bus, with 120 Ω termination at both ends.
// ─────────────────────────────────────────────────────────────────────────────

#include "Arduino.h"
#include "driver/twai.h"

// ── Pin / timing config ──────────────────────────────────────────────────────
#define TWAI_TX_GPIO        GPIO_NUM_5
#define TWAI_RX_GPIO        GPIO_NUM_4
#define TWAI_BITRATE_CONFIG TWAI_TIMING_CONFIG_500KBITS()

// ── Choose exactly one mode ──────────────────────────────────────────────────
#define MODE_NORMAL       0
#define MODE_NO_ACK       1
#define MODE_LISTEN_ONLY  0

#if (MODE_NORMAL + MODE_NO_ACK + MODE_LISTEN_ONLY) != 1
#error "Pick exactly one of MODE_NORMAL / MODE_NO_ACK / MODE_LISTEN_ONLY"
#endif

// ── Timings ──────────────────────────────────────────────────────────────────
#define TX_INTERVAL_MS      500     // OBD speed request cadence
#define STATUS_INTERVAL_MS  2000    // Status print cadence

// ── OBD-II ───────────────────────────────────────────────────────────────────
#define OBD_REQUEST_ID      0x7DF
#define OBD_RESPONSE_ID     0x7E8
#define OBD_MODE_CURRENT    0x01
#define OBD_PID_SPEED       0x0D

// ── State counters ───────────────────────────────────────────────────────────
static uint32_t txCount      = 0;
static uint32_t txFailCount  = 0;
static uint32_t rxCount      = 0;
static uint32_t obdCount     = 0;
static unsigned long lastTx     = 0;
static unsigned long lastStatus = 0;

// ─────────────────────────────────────────────────────────────────────────────
static const char* stateStr(twai_state_t s) {
    switch (s) {
        case TWAI_STATE_STOPPED:    return "STOPPED";
        case TWAI_STATE_RUNNING:    return "RUNNING";
        case TWAI_STATE_BUS_OFF:    return "BUS_OFF";
        case TWAI_STATE_RECOVERING: return "RECOVERING";
        default:                    return "UNKNOWN";
    }
}

static twai_mode_t pickMode() {
#if MODE_LISTEN_ONLY
    return TWAI_MODE_LISTEN_ONLY;
#elif MODE_NO_ACK
    return TWAI_MODE_NO_ACK;
#else
    return TWAI_MODE_NORMAL;
#endif
}

static const char* modeStr() {
#if MODE_LISTEN_ONLY
    return "LISTEN_ONLY";
#elif MODE_NO_ACK
    return "NO_ACK";
#else
    return "NORMAL";
#endif
}

// Forward declaration — definition is below sendOBDRequest()
static void diagnoseTxFailure(esp_err_t r);

// ─────────────────────────────────────────────────────────────────────────────
static void sendOBDRequest() {
#if MODE_LISTEN_ONLY
    return;   // can't transmit in listen-only mode
#else
    twai_message_t msg = {};
    msg.identifier       = OBD_REQUEST_ID;
    msg.extd             = 0;
    msg.rtr              = 0;
    msg.data_length_code = 8;
    msg.data[0] = 0x02;
    msg.data[1] = OBD_MODE_CURRENT;
    msg.data[2] = OBD_PID_SPEED;
    msg.data[3] = 0x55;
    msg.data[4] = 0x55;
    msg.data[5] = 0x55;
    msg.data[6] = 0x55;
    msg.data[7] = 0x55;

    esp_err_t r = twai_transmit(&msg, pdMS_TO_TICKS(50));
    if (r == ESP_OK) {
        txCount++;
        Serial.printf("[TX] id=0x%03X  dlc=%u  payload=02 01 0D ...\n",
                      (unsigned)msg.identifier, msg.data_length_code);
    } else {
        txFailCount++;
        diagnoseTxFailure(r);
    }
#endif
}

// Called after every twai_transmit() failure to pinpoint the cause.
// The most common case in MODE_NO_ACK is ESP_ERR_TIMEOUT, which means the
// frame was queued but never cleared the controller — i.e., the controller
// is not seeing its own bits coming back on RX. That points to wiring, not
// software.
static void diagnoseTxFailure(esp_err_t r) {
    twai_status_info_t st;
    twai_get_status_info(&st);

    Serial.printf("[TX-FAIL] err=%s  state=%s  tx_q=%lu  "
                  "tx_err=%lu rx_err=%lu  bus_err=%lu  arb_lost=%lu\n",
                  esp_err_to_name(r), stateStr(st.state),
                  (unsigned long)st.msgs_to_tx,
                  (unsigned long)st.tx_error_counter,
                  (unsigned long)st.rx_error_counter,
                  (unsigned long)st.bus_error_count,
                  (unsigned long)st.arb_lost_count);

    // Print actionable hints on the first failure only so we don't spam.
    static bool hinted = false;
    if (hinted) return;
    hinted = true;

    Serial.println("[HINT] First TX failure — likely causes:");
    if (r == ESP_ERR_TIMEOUT) {
        Serial.println("  • Frame was queued but couldn't clear the controller.");
        Serial.println("  • In MODE_NO_ACK the controller still listens to its");
        Serial.println("    own TX on RX. If RX never sees a falling edge, the");
        Serial.println("    bit is treated as a bus error and TX is retried.");
        Serial.println("  Check (in order):");
        Serial.println("   1. Transceiver powered? (VCC = 3.3 V for SN65HVD230,");
        Serial.println("      5 V for TJA1050).");
        Serial.println("   2. ESP TX→transceiver TXD, ESP RX←transceiver RXD?");
        Serial.printf ("      Current: TX=GPIO%d  RX=GPIO%d\n",
                       (int)TWAI_TX_GPIO, (int)TWAI_RX_GPIO);
        Serial.println("   3. 120 Ω termination across CANH/CANL at both ends?");
        Serial.println("   4. Standby pin (Rs/STBY) tied LOW so transceiver is");
        Serial.println("      in normal mode? (Pulled high = standby = no TX).");
        Serial.println("   5. NO transceiver on the bench? Jumper TX↔RX GPIOs");
        Serial.printf ("      directly (GPIO%d↔GPIO%d) — controller will then\n",
                       (int)TWAI_TX_GPIO, (int)TWAI_RX_GPIO);
        Serial.println("      hear its own frames and TX will complete.");
    } else if (r == ESP_ERR_INVALID_STATE) {
        Serial.println("  • Driver isn't RUNNING — likely BUS_OFF.");
        Serial.println("  • Status above shows current state; recovery happens");
        Serial.println("    automatically from the periodic status check.");
    } else if (r == ESP_FAIL) {
        Serial.println("  • TX queue full — loop isn't draining fast enough.");
    } else {
        Serial.println("  • Unexpected error — check esp_err code in docs.");
    }
}

static void printFrame(const twai_message_t& m) {
    Serial.printf("[RX] id=0x%03X%s%s  dlc=%u  data=",
                  (unsigned)m.identifier,
                  m.extd ? " (ext)" : "",
                  m.rtr  ? " (rtr)" : "",
                  m.data_length_code);
    for (int i = 0; i < m.data_length_code; i++) {
        Serial.printf("%02X ", m.data[i]);
    }

    // Decode OBD-II speed response: 0x7E8, [len, 0x41, 0x0D, speed, ...]
    if (!m.rtr && m.identifier == OBD_RESPONSE_ID &&
        m.data_length_code >= 4 &&
        m.data[1] == 0x41 && m.data[2] == OBD_PID_SPEED) {
        obdCount++;
        Serial.printf(" → OBD speed = %u km/h", m.data[3]);
    }
    Serial.println();
}

static void printStatus() {
    twai_status_info_t st;
    if (twai_get_status_info(&st) != ESP_OK) {
        Serial.println("[STATUS] twai_get_status_info() failed");
        return;
    }
    Serial.printf(
        "[STATUS] state=%s  tx=%lu (fail=%lu)  rx=%lu  obd_decoded=%lu  "
        "tx_q=%lu rx_q=%lu  tx_err=%lu rx_err=%lu  bus_err=%lu "
        "arb_lost=%lu  rx_missed=%lu\n",
        stateStr(st.state),
        (unsigned long)txCount, (unsigned long)txFailCount,
        (unsigned long)rxCount, (unsigned long)obdCount,
        (unsigned long)st.msgs_to_tx, (unsigned long)st.msgs_to_rx,
        (unsigned long)st.tx_error_counter, (unsigned long)st.rx_error_counter,
        (unsigned long)st.bus_error_count,
        (unsigned long)st.arb_lost_count,
        (unsigned long)st.rx_missed_count
    );

    // Auto-attempt bus-off recovery
    if (st.state == TWAI_STATE_BUS_OFF) {
        Serial.println("[STATUS] BUS_OFF — initiating recovery");
        twai_initiate_recovery();
    } else if (st.state == TWAI_STATE_STOPPED) {
        Serial.println("[STATUS] STOPPED — restarting driver");
        twai_start();
    }
}

// ─────────────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    // Give USB CDC a moment so the boot banner isn't lost
    unsigned long t0 = millis();
    while (!Serial && (millis() - t0) < 2000) { delay(10); }

    Serial.println();
    Serial.println("=== ESP32-S3 TWAI test ===");
    Serial.printf("TX  GPIO  : %d\n", (int)TWAI_TX_GPIO);
    Serial.printf("RX  GPIO  : %d\n", (int)TWAI_RX_GPIO);
    Serial.printf("Bitrate   : 500 kbit/s\n");
    Serial.printf("Mode      : %s\n", modeStr());

    // ── GPIO sanity check ────────────────────────────────────────────────────
    // Drive a 10 Hz square wave on TX and RX for ~1 s before handing the pins
    // to the TWAI peripheral. Scope at 50 ms/div, any trigger mode — both pins
    // should toggle 0 V ↔ 3.3 V. If a pin stays flat, that pin is not bonded
    // out / is fighting an external load and TWAI will never work on it.
    Serial.println("[CHECK] Toggling TX/RX as plain GPIO for 1 s ...");
    pinMode(TWAI_TX_GPIO, OUTPUT);
    pinMode(TWAI_RX_GPIO, OUTPUT);
    for (int i = 0; i < 20; i++) {
        digitalWrite(TWAI_TX_GPIO, i & 1);
        digitalWrite(TWAI_RX_GPIO, !(i & 1));
        delay(50);
    }
    digitalWrite(TWAI_TX_GPIO, HIGH);   // leave recessive (high) for TWAI
    digitalWrite(TWAI_RX_GPIO, HIGH);
    Serial.println("[CHECK] Toggle done. Installing TWAI driver ...");

    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        TWAI_TX_GPIO, TWAI_RX_GPIO, pickMode());
    twai_timing_config_t  t_config = TWAI_BITRATE_CONFIG;
    twai_filter_config_t  f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t r = twai_driver_install(&g_config, &t_config, &f_config);
    if (r != ESP_OK) {
        Serial.printf("[FATAL] twai_driver_install: %s\n", esp_err_to_name(r));
        while (true) delay(1000);
    }
    r = twai_start();
    if (r != ESP_OK) {
        Serial.printf("[FATAL] twai_start: %s\n", esp_err_to_name(r));
        while (true) delay(1000);
    }
    Serial.println("[OK] TWAI driver running. Listening for frames ...");
    Serial.println();
}

// ─────────────────────────────────────────────────────────────────────────────
void loop() {
    // 1. Drain all pending RX frames (non-blocking)
    twai_message_t rxMsg;
    while (twai_receive(&rxMsg, 0) == ESP_OK) {
        rxCount++;
        printFrame(rxMsg);
    }

    // 2. Periodically send an OBD speed request
    if (millis() - lastTx >= TX_INTERVAL_MS) {
        lastTx = millis();
        sendOBDRequest();
    }

    // 3. Periodic status / health line
    if (millis() - lastStatus >= STATUS_INTERVAL_MS) {
        lastStatus = millis();
        printStatus();
    }
}
