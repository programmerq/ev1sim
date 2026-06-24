/*
 * gm8192_frame: generic GM 8192-baud serial frame envelope.
 *
 * Implements the framing layer of the GM Class 1 / ALDL serial protocol
 * (8192 baud, 10-bit UART words, single-wire multi-drop). The wire format
 * between Idle Lines is:
 *
 *   [ID] [Length = 0x55 + N] [Payload x N] [SumCheck]
 *
 * where N is the count of Payload bytes (everything between Length and
 * SumCheck). N is constrained to [0, 64], so Length is in [0x55, 0x95].
 * N=0 is legal: the EV1 frame spec uses payload-less "trigger" polls
 * (L=0x55, the noise-resistant 0101 0101 sentinel).
 * @source:redux bus/messages/uart/30_ccu_data_poll.yaml,
 *               bus/messages/uart/a8_htcm_data_poll.yaml (N=0 polls)
 * SumCheck is the two's complement of the byte sum of (ID + Length +
 * Payload) so that the entire frame sums to 0 mod 256.
 *
 * Reference: Delco/Kokomo EE-1800-003 Rev A (1979-04-07), EE-1810-004
 *            Rev A (1979-04-17), and XDE 5024 (1982-11-17), as transcribed
 *            at http://www.calibra-classic.org/pages/ecu2pc1.htm; EV1
 *            message set per the EV1 frame spec as digitized in
 *            ev1-manual-redux bus/.
 *
 * This module owns the envelope (ID, Length, Payload slice, SumCheck).
 * Vehicle modules own their own payload semantics (byte layout, scaling,
 * sequence counters) inside the Payload slice. In particular the ALDL
 * sub-protocol's Mode byte is NOT an envelope field: ALDL messages carry
 * their Mode Number as payload[0] (the XDE-5024 "N = Mode + data" length
 * arithmetic is exactly this envelope with the Mode counted inside N).
 * Periodic EV1 messages have no Mode byte at all.
 *
 * Idle Line handling: the spec defines an Idle Line as >=10 consecutive
 * logic-1 bit times before the next Start Bit. Host-side byte streams
 * cannot count bit times, so we treat Idle Line as a state-machine
 * boundary: gm8192_decode_next walks a ring buffer byte-by-byte looking
 * for the next valid (ID, Length, SumCheck) triple. Garbage between
 * frames is dropped on the floor.
 *
 * Pure C99 (no heap, no globals, AVR-friendly). The encoder is small
 * enough to live in firmware targets.
 */

#ifndef ELECTRICSIM_SRC_IO_GM8192_GM8192_FRAME_H_
#define ELECTRICSIM_SRC_IO_GM8192_GM8192_FRAME_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------------- */
/* Wire constants                                                             */
/* ------------------------------------------------------------------------- */

/* Length-byte arithmetic: L = 0x55 + N where N counts Payload bytes.
 * N=0  -> payload-less trigger frame -> L=0x55.
 * N=65 -> L=0x96 — the ceiling. The ALDL Mode-1/2 RESPONSE is the
 * largest frame in the dialect: Mode byte + 64 data bytes = N 65.
 * @source:redux bus/aldl_modes.yaml byte_layout length rows
 * ("$55 + 1 + 64 = $96"). The EV1 periodic set tops out at N=11; the
 * ceiling exists so a real tester's maximal ALDL responses survive
 * decode at hardware-interop time (the old N=64 cap rejected L=$96 —
 * notes/manual_supplements.yaml#2026-06-11-pr110-review-followups #3). */
#define GM8192_LENGTH_BIAS    0x55u  /* = 85 */
#define GM8192_LENGTH_MIN     0x55u  /* N = 0 */
#define GM8192_LENGTH_MAX     0x96u  /* N = 65 (max ALDL Mode-1/2 response) */
#define GM8192_MAX_PAYLOAD    65u
#define GM8192_HEADER_LEN     2u     /* ID + Length */
#define GM8192_TRAILER_LEN    1u     /* SumCheck */
#define GM8192_MIN_FRAME_LEN  3u     /* ID + Length + SumCheck (N=0) */
#define GM8192_MAX_FRAME_LEN  68u    /* ID + Length + 65 payload + SumCheck */

/* Sentinel returned by gm8192_n_from_length for an illegal Length byte
 * (0 is a *valid* N, so it can no longer double as the error value). */
#define GM8192_N_INVALID      0xFFu

/* ALDL messages spend payload[0] on the Mode Number, leaving this many
 * data bytes after the Mode in a maximal frame — 64, the spec's Mode-1/2
 * response block size (@source:redux bus/aldl_modes.yaml: "Contents of
 * the 64 memory locations starting at the requested address"). */
#define GM8192_ALDL_MAX_DATA_LEN 64u

/* IDs 0x00 and 0xFF are reserved/illegal per spec. 254 IDs available. */
#define GM8192_ID_RESERVED_LOW  0x00u
#define GM8192_ID_RESERVED_HIGH 0xFFu

/* ------------------------------------------------------------------------- */
/* ALDL ID block                                                              */
/* ------------------------------------------------------------------------- */

/* All ALDL (scan-tool session) traffic lives on the 0xF0–0xF8 ID block so
 * consumers can distinguish it from mode-less periodic traffic by ID alone.
 * The EV1 frame spec defines $F0 (master presence check) and $F1 (tester
 * data response); @source:redux bus/messages/uart/f0_aldl_presence_check.yaml,
 * f1_aldl_data_response.yaml.
 *
 * The per-slave session IDs below are ours: how a real tester addressed an
 * individual EV1 ECU is not documented in the spec material we have.
 * @design 2026-06-09 claude — one ID per slave (request and response share
 * the ID; direction is implied by bus mastership). Revisit if a better
 * reference surfaces — the open questions are catalogued in
 * notes/manual_supplements.yaml#2026-06-09-aldl-id-block-f2-f7. */
#define GM8192_ALDL_ID_PRESENCE      0xF0u  /* master presence check (spec) */
#define GM8192_ALDL_ID_DATA_RESPONSE 0xF1u  /* tester data response (spec) */
#define GM8192_ALDL_ID_BPM           0xF2u  /* reserved; no responder yet */
#define GM8192_ALDL_ID_BTCM          0xF3u
#define GM8192_ALDL_ID_PCM           0xF4u  /* PIM in this codebase */
#define GM8192_ALDL_ID_CCU           0xF5u  /* RSA in this codebase */
#define GM8192_ALDL_ID_HTCM          0xF6u
#define GM8192_ALDL_ID_DSCM          0xF7u  /* IPC in this codebase */
/* @inferred 2026-06-20 claude — the SIR/SDM diagnostic-mode spec (XDE-5024) is
 * not bundled, so the SDM's wire identity is an engineered choice: 0xF8 extends
 * the ALDL session-ID block by one past DSCM (0xF7). SDM is the first module
 * added beyond the original spec set, so the F0–F7 range checks in
 * aldl_responder.c (observe_tester_traffic) and scan_tool_aldl.c
 * (decode_response) were widened to F0–F8 to match. Request and response share
 * the ID, as for the 0xF2–0xF7 slaves.
 * notes/manual_supplements.yaml#2026-06-20-sdm-aldl-id-and-responder */
#define GM8192_ALDL_ID_SDM           0xF8u  /* SDM (SIR/airbag); @inferred */

/* ------------------------------------------------------------------------- */
/* ALDL Mode Numbers (payload[0] of ALDL-block frames; not envelope fields)   */
/* ------------------------------------------------------------------------- */

/* ALDL sub-protocol modes (per spec). These are values of payload[0] in
 * frames belonging to the ALDL ID block, never an envelope field. */
#define GM8192_MODE_EXIT_NORMAL     0x00u
#define GM8192_MODE_FIXED_STREAM    0x01u
#define GM8192_MODE_MEMORY_DUMP     0x02u
#define GM8192_MODE_RAM_READ        0x03u
#define GM8192_MODE_FUNC_MOD        0x04u
#define GM8192_MODE_COMMAND_MESSAGE 0x07u

/* @inferred 2026-06-04 — ALDL "clear malfunction codes" is decimal Mode 10
 * (= 0x0A), added to the GM ALDL command set in a spec revision later than
 * XDE 5024 (1982-11-17). The widely-cited transcriptions (calibra-classic,
 * troublecodes.net GM tables) list Mode $0A as the clear-DTCs request; 0x0A
 * is otherwise unallocated in this dialect (0x10 is EV1_PERIODIC_FULL). The
 * targeted ECU clears its DTC / malfunction-history registry and replies with
 * Mode 0x0A echoed plus a one-byte status. */
#define GM8192_MODE_CLEAR_DTCS      0x0Au

/* Dialect version.
 *   v1 — pre-GM-8192 invented framing (function code + XOR-8 trailer).
 *   v2 — GM-8192 envelope (this header), peers self-broadcast at 200 ms.
 *   v3 — adds the invented EV1 master poll-and-respond mode (0x11) so
 *        BPM can drive the bus as the spec describes.
 *   v4 — the real EV1 message set (issue #94): mode-less periodic
 *        frames at their natural lengths on the spec's IDs and
 *        cadences ($40/$41, $90/$91, $30/$31, $A8/$A9, $20/$21 at
 *        120/960 ms; $F0/$F1 ALDL presence + mastership handover),
 *        decodable by period-correct tools (Peter Ohler's Palm EV1
 *        Dash). The v3 broadcasts and Mode 0x10/0x11 markers are gone;
 *        ALDL sessions ride per-slave IDs 0xF2-0xF8 with the Mode
 *        number as payload[0]. See docs/gm8192_protocol.md.
 * The envelope itself is mode-less; ALDL frames spend payload[0] on
 * their Mode number, which is byte-identical on the wire to the old
 * [ID][L][Mode][Data] formulation (L arithmetic unchanged: 85 == 0x55). */
#define EV1_GM8192_DIALECT_VERSION 4u

/* ------------------------------------------------------------------------- */
/* Status codes                                                               */
/* ------------------------------------------------------------------------- */

typedef enum {
  GM8192_OK = 0,
  GM8192_ERR_NULL,             /* required pointer is NULL */
  GM8192_ERR_BUF_TOO_SMALL,    /* encoder output buffer cannot hold the frame */
  GM8192_ERR_TRUNCATED,        /* not enough bytes available for a complete frame */
  GM8192_ERR_BAD_ID,           /* ID byte is 0x00 or 0xFF */
  GM8192_ERR_BAD_LENGTH,       /* Length byte outside [0x55, 0x95] */
  GM8192_ERR_BAD_N,            /* encoder: n > GM8192_MAX_PAYLOAD */
  GM8192_ERR_BAD_SUMCHECK,     /* checksum mismatch */
} gm8192_status_t;

/* ------------------------------------------------------------------------- */
/* Decoded-frame view                                                         */
/* ------------------------------------------------------------------------- */

/* A decoded frame. `payload` points into the caller-supplied input buffer;
 * the caller must keep that buffer alive while the view is in use. */
typedef struct {
  uint8_t        id;
  uint8_t        length;     /* on-wire Length byte = 0x55 + n */
  uint8_t        n;          /* payload byte count, in [0, 64] */
  const uint8_t* payload;    /* slice of input buffer, NULL iff n == 0 */
  uint8_t        sum_check;  /* on-wire SumCheck byte */
} gm8192_frame_t;

/* ------------------------------------------------------------------------- */
/* Primitives                                                                 */
/* ------------------------------------------------------------------------- */

/* Two's-complement byte-sum check. Returns the value that, appended to
 * `bytes[0..len)`, makes the total sum zero mod 256. */
uint8_t gm8192_sum_check(const uint8_t* bytes, size_t len);

/* True iff `id` is a legal Message ID per spec (i.e., not 0x00 or 0xFF). */
bool gm8192_id_is_legal(uint8_t id);

/* Compute Length byte from N. N must be in [0, 64]. Returns 0 if N is
 * out of range (0 is never a legal Length byte). */
uint8_t gm8192_length_from_n(uint8_t n);

/* Compute N from Length byte. Returns GM8192_N_INVALID if the Length byte
 * is outside [0x55, 0x95]. */
uint8_t gm8192_n_from_length(uint8_t length);

/* ------------------------------------------------------------------------- */
/* Encode                                                                     */
/* ------------------------------------------------------------------------- */

/* Encode a frame into out_buf.
 *
 *   id        Message ID (must be legal).
 *   payload   Pointer to n bytes of payload (may be NULL iff n == 0).
 *   n         Payload byte count. Must be in [0, 64].
 *   out_buf   Destination buffer; receives 3 + n bytes on success.
 *   out_cap   Capacity of out_buf.
 *   out_len   On success, set to the number of bytes written.
 *
 * Returns GM8192_OK on success, otherwise an error code and out_buf state
 * is unspecified. */
gm8192_status_t gm8192_encode(uint8_t id,
                              const uint8_t* payload,
                              uint8_t n,
                              uint8_t* out_buf,
                              size_t out_cap,
                              size_t* out_len);

/* ------------------------------------------------------------------------- */
/* Decode                                                                     */
/* ------------------------------------------------------------------------- */

/* Decode a single frame starting at the first byte of in_buf.
 *
 *   in_buf            Input buffer. The frame is expected to start at offset 0.
 *   in_len            Bytes available in in_buf.
 *   out               Receives the decoded frame view. `out->payload` points
 *                     into in_buf and is valid for as long as in_buf is.
 *   bytes_consumed    On GM8192_OK, set to the frame length (3 + n).
 *                     On GM8192_ERR_TRUNCATED, set to 0 (no progress; caller
 *                     should wait for more bytes). On other errors, set to 0.
 *
 * Returns:
 *   GM8192_OK             frame decoded into *out.
 *   GM8192_ERR_NULL       in_buf, out, or bytes_consumed is NULL.
 *   GM8192_ERR_TRUNCATED  in_len < required frame length.
 *   GM8192_ERR_BAD_ID     in_buf[0] is 0x00 or 0xFF.
 *   GM8192_ERR_BAD_LENGTH in_buf[1] is outside [0x55, 0x95].
 *   GM8192_ERR_BAD_SUMCHECK trailing SumCheck does not match. */
gm8192_status_t gm8192_decode(const uint8_t* in_buf,
                              size_t in_len,
                              gm8192_frame_t* out,
                              size_t* bytes_consumed);

/* Scan `ring[0..ring_len)` for the next valid frame.
 *
 * On success, *bytes_consumed is the number of bytes the caller should
 * advance past (skipped garbage + frame length).
 *
 * If no complete valid frame is present yet, returns GM8192_ERR_TRUNCATED
 * with *bytes_consumed set to the number of bytes the caller MAY safely
 * drop (i.e., bytes definitively before any plausible frame start). The
 * remaining (ring_len - *bytes_consumed) bytes should be retained until
 * the next call.
 *
 * On a malformed but framed candidate (bad sumcheck), the candidate's
 * leading byte is treated as garbage and scanning continues. */
gm8192_status_t gm8192_decode_next(const uint8_t* ring,
                                   size_t ring_len,
                                   gm8192_frame_t* out,
                                   size_t* bytes_consumed);

#ifdef __cplusplus
}  /* extern "C" */
#endif

#endif /* ELECTRICSIM_SRC_IO_GM8192_GM8192_FRAME_H_ */
