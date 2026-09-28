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
 * SumCheck). The Length byte is one byte, so N spans [0, 170] and Length
 * spans [0x55, 0xFF] — the whole range the field can express.
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
 * N=0   -> payload-less trigger frame -> L=0x55.
 * N=170 -> L=0xFF — the ceiling, and it is the LENGTH BYTE'S OWN
 * arithmetic ceiling (0xFF - 0x55), NOT an assumption about which
 * message happens to be the longest one anybody has seen.
 *
 * Why the field's own maximum and not a message-set maximum:
 * @source:manual EV1 Bus Reference (Serial Data Link Spec 5191 Rev 13),
 *   originals/EV1 Bus/bus-003.jpg (printed page 9-3), section 9.4.1.2
 *   "ALDL Data Response (Service Mode Only)" prints "Transmission Length:
 *   3 + N Bytes" and, under the byte table, "N = Mode Dependent (1 or
 *   greater)". The spec that owns the longest message on this bus states
 *   NO upper bound on N. Every ceiling below what the Length byte can
 *   express is therefore an invention of ours, and this codec has now
 *   shipped two of them:
 *     - N<=64 (v3), refuted 2026-06-11 by the ALDL Mode-1/2 response;
 *     - N<=65 (2026-06-11..2026-09-08), refuted by real hardware.
 *   The N<=65 figure came from a SECONDARY source, the calibra-classic
 *   XDE-5024 transcription mirrored at @source:redux bus/aldl_modes.yaml
 *   ("$55 + 1 + 64 = $96") — whose own note says "Calibra spec: 64 data
 *   bytes. EV1 application byte count likely differs."  It does.
 * @inferred 2026-09-08 claude — a logic-analyzer capture off a period
 *   S-10 Electric (the GM electric pickup sharing this bus dialect),
 *   tagged "BPCM Data 3" and taken 2026-07-16T23:47:30Z, carries TWELVE
 *   byte-identical $F1 (ALDL Data Response) frames of 71 wire bytes at
 *   L=0x99, i.e. N=68 — three bytes past the old ceiling. The capture
 *   device flagged all twelve good and each sums to 0 mod 256, so they
 *   are content, not corruption; the first is at capture timestamp
 *   1772755666 us. The frame is replayed byte-for-byte by
 *   test_decode_real_s10e_oversize_f1 in this directory's tests, so the
 *   evidence travels with the code rather than living only in a
 *   capture archive.
 *   Their payload[0] is Mode 0x01 (GM8192_MODE_FIXED_STREAM) and the
 *   same Mode-1 response recurs across the corpus at N = 6/11/29/68, so
 *   68 is a sample, not a bound. Rather than chase the sample, the
 *   ceiling is set where the wire itself stops: L cannot exceed 0xFF.
 *   Refuting THIS ceiling would require a frame the Length byte cannot
 *   encode.  Cost of the widen is buffer size, not behaviour — every
 *   `uint8_t wire[GM8192_MAX_FRAME_LEN]` rebuild buffer in the fleet
 *   grows from 68 to 173 bytes; the EV1 periodic set still tops out at
 *   N=11.  The refuted-twice history above IS the durable record; it is
 *   stated here rather than deferred to a pointer, so a reader of this
 *   header alone can see why a narrower ceiling must not come back.
 */
#define GM8192_LENGTH_BIAS    0x55u  /* = 85 */
#define GM8192_LENGTH_MIN     0x55u  /* N = 0 */
#define GM8192_LENGTH_MAX     0xFFu  /* N = 170 — the Length byte's own max */
#define GM8192_MAX_PAYLOAD    170u   /* 0xFF - 0x55 */
#define GM8192_HEADER_LEN     2u     /* ID + Length */
#define GM8192_TRAILER_LEN    1u     /* SumCheck */
#define GM8192_MIN_FRAME_LEN  3u     /* ID + Length + SumCheck (N=0) */
#define GM8192_MAX_FRAME_LEN  173u   /* ID + Length + 170 payload + SumCheck */

/* The three size constants above are ONE fact written three ways. Thirty-nine
 * production sites across the fleet (sixty-nine counting test code) rebuild a
 * wire slice into a `uint8_t wire[GM8192_MAX_FRAME_LEN]` from a decoded
 * frame's `n` and bounds-check it against GM8192_MAX_FRAME_LEN rather than
 * against GM8192_MAX_PAYLOAD; those checks are only safe while the identity
 * below holds. Desyncing the constants by hand would silently overflow every
 * one of them, so the two preprocessor checks below break the build instead.
 * @design 2026-09-08 claude. */
#if GM8192_MAX_PAYLOAD != (GM8192_LENGTH_MAX - GM8192_LENGTH_BIAS)
#error "GM8192_MAX_PAYLOAD must equal GM8192_LENGTH_MAX - GM8192_LENGTH_BIAS"
#endif
#if GM8192_MAX_FRAME_LEN != \
    (GM8192_HEADER_LEN + GM8192_MAX_PAYLOAD + GM8192_TRAILER_LEN)
#error "GM8192_MAX_FRAME_LEN must equal header + GM8192_MAX_PAYLOAD + trailer"
#endif

/* Sentinel returned by gm8192_n_from_length for an illegal Length byte
 * (0 is a *valid* N, so it can no longer double as the error value).
 * 0xFF stays outside [0, GM8192_MAX_PAYLOAD=170] so it is still
 * unambiguous after the 2026-09-08 ceiling widen. */
#define GM8192_N_INVALID      0xFFu

/* The ALDL Mode-2 memory-dump block size: "Contents of the 64 memory
 * locations starting at the requested address" (@source:redux
 * bus/aldl_modes.yaml). This is a MODE-2 payload fact, not an envelope
 * ceiling — a Mode-2 response is N = 1 (Mode) + 64 = 65, comfortably
 * inside GM8192_MAX_PAYLOAD. It sized the envelope ceiling until
 * 2026-09-08; conflating the two is what let a real N=68 frame be
 * rejected as garbage (see GM8192_LENGTH_MAX above). */
#define GM8192_ALDL_MAX_DATA_LEN 64u

/* IDs 0x00 and 0xFF are reserved/illegal per spec. 254 IDs available. */
#define GM8192_ID_RESERVED_LOW  0x00u
#define GM8192_ID_RESERVED_HIGH 0xFFu

/* ------------------------------------------------------------------------- */
/* ALDL ID block                                                              */
/* ------------------------------------------------------------------------- */

/* All ALDL (scan-tool session) traffic lives on the 0xF0–0xF7 ID block plus
 * the SDM's $FA, so consumers can distinguish it from mode-less periodic
 * traffic by ID alone (gm8192_is_aldl_id below).
 * The EV1 frame spec defines $F0 (master presence check) and $F1 (tester
 * data response); @source:redux bus/messages/uart/f0_aldl_presence_check.yaml,
 * f1_aldl_data_response.yaml.
 *
 * The per-slave session IDs 0xF2-0xF7 are ours: how a real tester addressed an
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
/* SDM (SIR/airbag): $FA, the ALDL id of every Delco SIR module in GM's own
 * datastream specs (A242/A252/A295/A296/A300: "FA 57 01 00 AE" = mode $01
 * message $00). A '97 Delco SDM on the bench answers that exact request, and
 * the EV1 SIR manual puts the SDM's serial data on DLC terminal 9 (SIR
 * md:474), the Delco SIR serial line. Request and response share the ID.
 * @source:manual SIR md:474 + GM Delco SIR datastream specs;
 * notes/manual_supplements.yaml#2026-06-20-sdm-aldl-id-and-responder */
#define GM8192_ALDL_ID_SDM           0xFAu  /* SDM (SIR/airbag), Delco SIR id */

/* True for every ID in the ALDL session space: the 0xF0-0xF7 block plus the
 * SDM's $FA. 0xF8/0xF9 are NOT ALDL ids. */
static inline bool gm8192_is_aldl_id(uint8_t id) {
  return (id >= GM8192_ALDL_ID_PRESENCE && id <= GM8192_ALDL_ID_DSCM) ||
         id == GM8192_ALDL_ID_SDM;
}

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
 *        ALDL sessions ride per-slave IDs 0xF2-0xF7 + $FA with the Mode
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
  GM8192_ERR_BAD_LENGTH,       /* Length byte outside [0x55, 0xFF] — i.e.
                                  below GM8192_LENGTH_MIN, since 0xFF is the
                                  byte's own maximum */
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
  uint8_t        n;          /* payload byte count, in [0, GM8192_MAX_PAYLOAD] */
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

/* Compute Length byte from N. N must be in [0, GM8192_MAX_PAYLOAD].
 * Returns 0 if N is out of range (0 is never a legal Length byte). */
uint8_t gm8192_length_from_n(uint8_t n);

/* Compute N from Length byte. Returns GM8192_N_INVALID if the Length byte
 * is outside [GM8192_LENGTH_MIN, GM8192_LENGTH_MAX] = [0x55, 0xFF]. */
uint8_t gm8192_n_from_length(uint8_t length);

/* ------------------------------------------------------------------------- */
/* Encode                                                                     */
/* ------------------------------------------------------------------------- */

/* Encode a frame into out_buf.
 *
 *   id        Message ID (must be legal).
 *   payload   Pointer to n bytes of payload (may be NULL iff n == 0).
 *   n         Payload byte count. Must be in [0, GM8192_MAX_PAYLOAD].
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
 *   GM8192_ERR_BAD_LENGTH in_buf[1] is below 0x55 (GM8192_LENGTH_MIN).
 *   GM8192_ERR_BAD_SUMCHECK trailing SumCheck does not match. */
gm8192_status_t gm8192_decode(const uint8_t* in_buf,
                              size_t in_len,
                              gm8192_frame_t* out,
                              size_t* bytes_consumed);

/* Scan `ring[0..ring_len)` for the next valid frame.
 *
 * Returns the EARLIEST complete, checksum-valid frame in the ring, with
 * *bytes_consumed set to the number of bytes the caller should advance past
 * (skipped garbage + frame length).
 *
 * If no complete valid frame is present yet, returns GM8192_ERR_TRUNCATED
 * with *bytes_consumed set to the number of bytes the caller MAY safely
 * drop (i.e., bytes definitively before any plausible frame start). The
 * remaining (ring_len - *bytes_consumed) bytes should be retained until
 * the next call.
 *
 * On a malformed but framed candidate (bad sumcheck), the candidate's
 * leading byte is treated as garbage and scanning continues.
 *
 * A candidate header that would need MORE bytes than the ring holds also
 * does not end the scan — the scan continues past it and the candidate is
 * only reported (as the TRUNCATED offset) if no complete valid frame turns
 * up anywhere. That changed on 2026-09-08 with the Length-ceiling widen, and
 * it had to: with L legal across [0x55, 0xFF], two-thirds of all byte pairs
 * are a syntactically plausible (ID, Length) header, so stopping at the
 * first truncated candidate let ordinary garbage stall resync until up to
 * 173 more bytes arrived. Three existing tests caught it —
 * test_decode_next_resync, test_decode_next_truncated_header, and the
 * bad-frame-then-good-frame case in test_gm8192_rx_framer.cpp.
 *
 * The trade this makes, stated plainly: when the ring ends mid-frame, the
 * scan now also looks INSIDE that partial tail, so a chance sumcheck match
 * there could be emitted as a frame instead of the real frame the caller
 * would have assembled from the next read. That risk already existed for
 * every garbage region the scan walks; this widens it to the tail, and it is
 * pinned in both directions by
 * test_decode_next_chance_valid_window_inside_a_partial_frame. It is
 * accepted because the alternative — stalling on garbage — DELAYS real
 * frames in a bounded buffer and loses them only on overrun
 * (Gm8192RxFramer drops OLDEST bytes when it overruns). The structural fix
 * is Idle-Line framing (>=10 logic-1 bit times is what real hardware frames
 * on, and what a byte stream throws away), not a better byte-scan
 * heuristic. @design 2026-09-08 claude.
 *
 * COST, degraded case. The scan no longer stops at the first truncated
 * candidate, so on a buffer that yields no frame it now visits every offset
 * and re-derives a SumCheck at each one that names a frame small enough to
 * fit. Worst case is bounded by sum over offsets of min(GM8192_MAX_PAYLOAD +
 * 1, remaining - 1) byte-additions: 20381 for the 205-byte receive-assembly
 * buffer the firmware modules use (GM8192_MAX_FRAME_LEN + a 32-byte
 * interrupt ring), 73185 for Gm8192RxFramer's 512-byte buffer. On the
 * emulated AVR the summing loop compiles to 9 cycles per byte at -Os, so the
 * firmware worst case is ~183k cycles ~= 11.5 ms at 16 MHz -- longer than
 * the 10 ms tick that calls it.
 *
 * Why that is acceptable rather than a defect: it needs 205 bytes in which
 * EVERY offset names a maximal frame and NONE checksums, which real traffic
 * does not produce (the EV1 periodic set tops out at a 14-byte frame, so the
 * common path visits a handful of offsets). The drain runs in the main loop,
 * not the receive interrupt, so an over-long pass delays the next supervisor
 * tick; it cannot drop wire bytes, because the 32-slot interrupt ring holds
 * 31 bytes = ~38 ms at 8192 baud (1.22 ms/byte), comfortably more than the
 * overrun. Idle-Line framing removes the bound entirely by capping the scan
 * at one frame's worth of bytes between boundaries. @design 2026-09-09
 * claude. */
gm8192_status_t gm8192_decode_next(const uint8_t* ring,
                                   size_t ring_len,
                                   gm8192_frame_t* out,
                                   size_t* bytes_consumed);

#ifdef __cplusplus
}  /* extern "C" */
#endif

#endif /* ELECTRICSIM_SRC_IO_GM8192_GM8192_FRAME_H_ */
