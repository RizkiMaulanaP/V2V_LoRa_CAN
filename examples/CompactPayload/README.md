# Compact V2V payload

A packed **11-byte** over-air payload replacing the 19-byte `V2V_Payload`, to
cut LoRa airtime (and reclaim duty-cycle room for shorter, CAM-style TX
intervals).

**Status: INTEGRATED into the firmware** ([../../V2V_LoRa_CAN.ino](../../V2V_LoRa_CAN.ino),
`FW_VERSION 3`) using the **global reference** (no hard-coded region). The two
files here are the standalone reference/codec:

- [`v2v_payload_compact.h`](v2v_payload_compact.h) — codec (regional-configurable reference)
- [`v2v_payload_compact.py`](v2v_payload_compact.py) — host/sim codec + round-trip self-test

The firmware inlines the same logic (`v2v_air_pack` / `v2v_air_unpack`) with the
global reference baked in. Only the **air** bytes changed — the MCU↔host serial
`LoRaRxFrame` and the Flutter JSON contract are unchanged, so host/sim/app need
no edits.

## Airtime (current radio: SF8 / BW125 / CR 4-8)

LoRa airtime is step-quantised by payload size and spreading factor:

| config | airtime |
|--------|---------|
| old (SF9, 19 B) | 247 ms |
| SF9, 11 B | 181 ms |
| **SF8, 11 B (current)** | **~107 ms (−57 % vs old)** |
| SF8, ≤8 B | ~90 ms |

## Wire layout (11 B, little-endian per field)

| bytes | field | encoding | resolution |
|------|-------|----------|-----------|
| 0 | node_id (6b) \| alert (2b) | `(id<<2)\|alert` | 0..63 / 0..3 |
| 1 | heading | `v*360/256` | 1.41° |
| 2 | speed | km/h | 1 km/h |
| 3–5 | latitude | 24-bit over −90..+90 | ~1.2 m |
| 6–8 | longitude | 24-bit over −180..+180 | ~2.4 m |
| 9–10 | tx_ts | `millis() & 0xFFFF` | 1 ms (wraps 65 s) |

**Global reference:** lat/lon are absolute (no shared window/region config), so
nodes interoperate anywhere just by running the same firmware. Resolution
(~1–2 m) is below GPS noise. If you later move to RTK/cm-grade GPS, switch the
firmware `v2v_enc_axis/v2v_dec_axis` ranges to a regional window (see the
standalone `.h`/`.py` here, which keep the `REF/SPAN` knobs) for cm precision.

Verified round-trip (global): ≤ **1.11 m** position, ≤ **0.10°** heading.

## Notes / limits

- Both nodes must run the matching firmware — SF *and* air format changed; not
  backward-compatible with FW_VERSION 2.
- `tx_ts` wraps ~65 s — adequate for one-way latency, not absolute time.
- No generic compressor (gzip/etc.): payload is too small and would be
  variable-length, breaking fixed airtime budgeting. Domain quantisation is the
  right tool.
