// ============================================================
// v2v_payload_compact.h  —  DRAFT compact V2V over-air payload
//
// 11-byte packed replacement for the 19-byte `V2V_Payload`.
// At SF9/BW125/CR4-8 this drops one LoRa symbol group:
//     19 B  → 247 ms airtime
//     11 B  → 181 ms airtime   (−27 %)
// (LoRa airtime is step-quantised; 9..12 B all cost 181 ms, so the 16-bit
//  tx timestamp below is effectively "free" — keep it.)
//
// This file is a DRAFT for review — it is NOT included by the firmware yet.
// Both the firmware and python-app/v2v_payload_compact.py MUST use the SAME
// constants and byte order (little-endian) or the fields will decode wrong.
//
// WIRE LAYOUT (11 bytes, little-endian within each multi-byte field):
//   byte 0      : node_id (bits 7..2, 0..63) | alert (bits 1..0, 0..3)
//   byte 1      : heading  (0..255  → deg = v * 360/256,  1.41° step)
//   byte 2      : speed    (km/h, 0..255)
//   bytes 3..5  : lat fixed-point (24-bit) over [REF_LAT ± LAT_SPAN/2]
//   bytes 6..8  : lon fixed-point (24-bit) over [REF_LON ± LON_SPAN/2]
//   bytes 9..10 : tx_ts (millis & 0xFFFF, wraps ~65 s — for link latency)
//
// POSITION ENCODING
//   A compile-time reference window (not a per-packet delta, so packet loss
//   never breaks decoding). With the defaults below — a ±1° window — the
//   resolution is ~1.3 cm, covering ~±111 km around the reference. Widen the
//   span for more coverage (resolution scales linearly); set REF=0 and
//   SPAN=180/360 for absolute global (~1.2 m lat / 2.4 m lon).
// ============================================================
#ifndef V2V_PAYLOAD_COMPACT_H
#define V2V_PAYLOAD_COMPACT_H

#include <stdint.h>
#include <math.h>

// ---- reference window (MUST match the Python side) -------------------------
#define V2VC_REF_LAT     (-7.0)    // window centre (deg)
#define V2VC_REF_LON     (107.0)
#define V2VC_LAT_SPAN    (2.0)     // total window height (deg) → ±1° around ref
#define V2VC_LON_SPAN    (2.0)
#define V2VC_FIX24_MAX   (16777215.0)   // 2^24 - 1

#define V2V_COMPACT_SIZE 11

// Decoded view (same semantics as the old V2V_Payload fields).
typedef struct {
    uint8_t  node_id;       // 0..63
    uint8_t  alert_type;    // 0..3
    float    heading_deg;   // 0..360
    uint8_t  speed;         // km/h
    float    latitude;
    float    longitude;
    uint16_t tx_ts16;       // millis() & 0xFFFF
} V2VCompact;

// --- helpers ----------------------------------------------------------------
static inline uint32_t v2vc_enc_axis(double v, double ref, double span) {
    double lo = ref - span * 0.5;
    double u  = (v - lo) / span * V2VC_FIX24_MAX;
    if (u < 0) u = 0;
    if (u > V2VC_FIX24_MAX) u = V2VC_FIX24_MAX;
    return (uint32_t)(u + 0.5);
}
static inline double v2vc_dec_axis(uint32_t u, double ref, double span) {
    double lo = ref - span * 0.5;
    return lo + ((double)u / V2VC_FIX24_MAX) * span;
}

// Pack into out[11]; returns the byte count.
static inline uint8_t v2v_compact_pack(uint8_t *out, const V2VCompact *p) {
    out[0] = (uint8_t)(((p->node_id & 0x3F) << 2) | (p->alert_type & 0x03));

    int h = (int)lround(p->heading_deg / 360.0 * 256.0) & 0xFF;
    out[1] = (uint8_t)h;
    out[2] = p->speed;

    uint32_t la = v2vc_enc_axis(p->latitude,  V2VC_REF_LAT, V2VC_LAT_SPAN);
    uint32_t lo = v2vc_enc_axis(p->longitude, V2VC_REF_LON, V2VC_LON_SPAN);
    out[3] = (uint8_t)(la & 0xFF);
    out[4] = (uint8_t)((la >> 8) & 0xFF);
    out[5] = (uint8_t)((la >> 16) & 0xFF);
    out[6] = (uint8_t)(lo & 0xFF);
    out[7] = (uint8_t)((lo >> 8) & 0xFF);
    out[8] = (uint8_t)((lo >> 16) & 0xFF);

    out[9]  = (uint8_t)(p->tx_ts16 & 0xFF);
    out[10] = (uint8_t)((p->tx_ts16 >> 8) & 0xFF);
    return V2V_COMPACT_SIZE;
}

// Unpack from in[11].
static inline void v2v_compact_unpack(const uint8_t *in, V2VCompact *p) {
    p->node_id    = (in[0] >> 2) & 0x3F;
    p->alert_type = in[0] & 0x03;
    p->heading_deg = (float)in[1] * (360.0f / 256.0f);
    p->speed      = in[2];

    uint32_t la = (uint32_t)in[3] | ((uint32_t)in[4] << 8) | ((uint32_t)in[5] << 16);
    uint32_t lo = (uint32_t)in[6] | ((uint32_t)in[7] << 8) | ((uint32_t)in[8] << 16);
    p->latitude  = (float)v2vc_dec_axis(la, V2VC_REF_LAT, V2VC_LAT_SPAN);
    p->longitude = (float)v2vc_dec_axis(lo, V2VC_REF_LON, V2VC_LON_SPAN);

    p->tx_ts16 = (uint16_t)(in[9] | ((uint16_t)in[10] << 8));
}

#endif // V2V_PAYLOAD_COMPACT_H
