/*
 * gm8192_frame: implementation. See gm8192_frame.h.
 */

#include "gm8192_frame.h"

#include <string.h>

uint8_t gm8192_sum_check(const uint8_t* bytes, size_t len) {
  uint8_t s = 0;
  if (bytes != NULL) {
    for (size_t i = 0; i < len; ++i) {
      s = (uint8_t)(s + bytes[i]);
    }
  }
  return (uint8_t)(0u - s);
}

bool gm8192_id_is_legal(uint8_t id) {
  return id != GM8192_ID_RESERVED_LOW && id != GM8192_ID_RESERVED_HIGH;
}

uint8_t gm8192_length_from_n(uint8_t n) {
  if (n > GM8192_MAX_PAYLOAD) {
    return 0u;
  }
  return (uint8_t)(GM8192_LENGTH_BIAS + n);
}

uint8_t gm8192_n_from_length(uint8_t length) {
  if (length < GM8192_LENGTH_MIN || length > GM8192_LENGTH_MAX) {
    return GM8192_N_INVALID;
  }
  return (uint8_t)(length - GM8192_LENGTH_BIAS);
}

gm8192_status_t gm8192_encode(uint8_t id,
                              const uint8_t* payload,
                              uint8_t n,
                              uint8_t* out_buf,
                              size_t out_cap,
                              size_t* out_len) {
  if (out_buf == NULL || out_len == NULL) {
    return GM8192_ERR_NULL;
  }
  if (payload == NULL && n != 0u) {
    return GM8192_ERR_NULL;
  }
  if (!gm8192_id_is_legal(id)) {
    return GM8192_ERR_BAD_ID;
  }
  if (n > GM8192_MAX_PAYLOAD) {
    return GM8192_ERR_BAD_N;
  }

  const size_t frame_len = (size_t)GM8192_HEADER_LEN + (size_t)n + (size_t)GM8192_TRAILER_LEN;
  if (out_cap < frame_len) {
    return GM8192_ERR_BUF_TOO_SMALL;
  }

  out_buf[0] = id;
  out_buf[1] = (uint8_t)(GM8192_LENGTH_BIAS + n);
  if (n > 0u) {
    memcpy(&out_buf[2], payload, n);
  }
  /* SumCheck covers ID + Length + Payload. */
  out_buf[frame_len - 1u] = gm8192_sum_check(out_buf, frame_len - 1u);

  *out_len = frame_len;
  return GM8192_OK;
}

gm8192_status_t gm8192_decode(const uint8_t* in_buf,
                              size_t in_len,
                              gm8192_frame_t* out,
                              size_t* bytes_consumed) {
  if (in_buf == NULL || out == NULL || bytes_consumed == NULL) {
    return GM8192_ERR_NULL;
  }
  *bytes_consumed = 0u;

  /* Check what we have, not what we don't. This ordering lets a streaming
   * resync walk over single garbage bytes even when fewer than 3 bytes
   * remain: a single 0xFF byte is BAD_ID, not TRUNCATED. */
  if (in_len < 1u) {
    return GM8192_ERR_TRUNCATED;
  }
  const uint8_t id = in_buf[0];
  if (!gm8192_id_is_legal(id)) {
    return GM8192_ERR_BAD_ID;
  }

  if (in_len < 2u) {
    return GM8192_ERR_TRUNCATED;
  }
  const uint8_t length_byte = in_buf[1];
  const uint8_t n = gm8192_n_from_length(length_byte);
  if (n == GM8192_N_INVALID) {
    return GM8192_ERR_BAD_LENGTH;
  }

  const size_t frame_len = (size_t)GM8192_HEADER_LEN + (size_t)n + (size_t)GM8192_TRAILER_LEN;
  if (in_len < frame_len) {
    return GM8192_ERR_TRUNCATED;
  }

  const uint8_t expected_sum = gm8192_sum_check(in_buf, frame_len - 1u);
  const uint8_t actual_sum = in_buf[frame_len - 1u];
  if (expected_sum != actual_sum) {
    return GM8192_ERR_BAD_SUMCHECK;
  }

  out->id = id;
  out->length = length_byte;
  out->n = n;
  out->payload = (n > 0u) ? &in_buf[2] : NULL;
  out->sum_check = actual_sum;

  *bytes_consumed = frame_len;
  return GM8192_OK;
}

gm8192_status_t gm8192_decode_next(const uint8_t* ring,
                                   size_t ring_len,
                                   gm8192_frame_t* out,
                                   size_t* bytes_consumed) {
  if (ring == NULL || out == NULL || bytes_consumed == NULL) {
    return GM8192_ERR_NULL;
  }

  /* Offset of the FIRST candidate header that could still become a frame if
   * more bytes arrive; ring_len means "none seen". Scanning does not stop
   * there — see the contract note in gm8192_frame.h. */
  size_t first_truncated = ring_len;

  for (size_t i = 0u; i < ring_len; ++i) {
    gm8192_frame_t candidate;
    size_t consumed = 0u;
    const gm8192_status_t s = gm8192_decode(&ring[i], ring_len - i, &candidate, &consumed);
    if (s == GM8192_OK) {
      *out = candidate;
      *bytes_consumed = i + consumed;
      return GM8192_OK;
    }
    if (s == GM8192_ERR_TRUNCATED && first_truncated == ring_len) {
      first_truncated = i;
    }
    /* BAD_ID / BAD_LENGTH / BAD_SUMCHECK / TRUNCATED -> try the next offset. */
  }

  /* No complete, checksum-valid frame anywhere in the ring. Hand back the
   * earliest offset that could still become one (== ring_len when there is
   * none, i.e. the caller may drop everything). */
  *bytes_consumed = first_truncated;
  return GM8192_ERR_TRUNCATED;
}
