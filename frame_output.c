/*
 * Frame output in iridium-toolkit RAW format
 * Port of gr-iridium's iridium_frame_printer_impl.cc
 *
 * Format:
 *   RAW: {file_info} {timestamp_ms:012.4f} {freq_hz:010d} N:{mag:05.2f}{noise:+06.2f}
 *        I:{id:011d} {conf:3d}% {level:.5f} {payload_symbols:3d} {bits...}
 *
 * Original work Copyright 2021 gr-iridium author
 * Modifications Copyright 2026 CEMAXECUTER LLC
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

/*
 * Frame output in iridium-toolkit RAW format
 *
 * Format:
 *   RAW: {file_info} {timestamp_ms:012.4f} {freq_hz:010d} N:{mag:05.2f}{noise:+06.2f}
 *        I:{id:011d} {conf:3d}% {level:.5f} {payload_symbols:3d} {bits...}
 */

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <string.h>

#ifdef HAVE_ZMQ
#include <zmq.h>
#endif

#include "frame_output.h"
#include "iridium.h"

extern int diagnostic_mode;
extern int parsed_mode;
extern int acars_enabled;
extern int precise_timing;  /* --timing flag: emit first_symbol_ns to stderr */
#ifdef HAVE_UHD
extern int usrp_pps_ref;    /* --pps-ref: hardware GPS-disciplined timestamp */
#else
static const int usrp_pps_ref = 0;
#endif

static const char *out_file_info = NULL;
static uint64_t t0 = 0;
static int initialized = 0;

/* ---- Line buffer for ZMQ publishing ---- */

#define LINE_BUF_SIZE 8192
static char line_buf[LINE_BUF_SIZE];
static int line_pos;

static void buf_start(void) { line_pos = 0; }

__attribute__((format(printf, 1, 2)))
static void buf_printf(const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(line_buf + line_pos, LINE_BUF_SIZE - line_pos, fmt, ap);
    va_end(ap);
    if (n > 0) {
        line_pos += n;
        if (line_pos >= LINE_BUF_SIZE)
            line_pos = LINE_BUF_SIZE - 1;
    }
}

static void buf_char(int c) {
    if (line_pos < LINE_BUF_SIZE - 1)
        line_buf[line_pos++] = (char)c;
}

/* ---- ZMQ PUB socket ---- */

#ifdef HAVE_ZMQ
static void *zmq_context = NULL;
static void *zmq_pub_socket = NULL;
#define ZMQ_ACTIVE (zmq_pub_socket != NULL)
#else
#define ZMQ_ACTIVE 0
#endif

static void buf_flush(int to_stdout) {
    line_buf[line_pos] = '\0';

    if (to_stdout && line_pos > 0) {
        fwrite(line_buf, 1, line_pos, stdout);
        fflush(stdout);
    }

#ifdef HAVE_ZMQ
    if (zmq_pub_socket && line_pos > 0) {
        /* ZMQ messages are framed, strip trailing newline */
        int len = line_pos;
        if (len > 0 && line_buf[len - 1] == '\n')
            len--;
        zmq_send(zmq_pub_socket, line_buf, len, 0);
    }
#endif
}

/* ---- Public API ---- */

void frame_output_init(const char *fi)
{
    out_file_info = fi;
}

#ifdef HAVE_ZMQ
int frame_output_zmq_init(const char *endpoint)
{
    zmq_context = zmq_ctx_new();
    if (!zmq_context)
        return -1;

    zmq_pub_socket = zmq_socket(zmq_context, ZMQ_PUB);
    if (!zmq_pub_socket) {
        zmq_ctx_destroy(zmq_context);
        zmq_context = NULL;
        return -1;
    }

    if (zmq_bind(zmq_pub_socket, endpoint) != 0) {
        zmq_close(zmq_pub_socket);
        zmq_ctx_destroy(zmq_context);
        zmq_pub_socket = NULL;
        zmq_context = NULL;
        return -1;
    }

    return 0;
}

void frame_output_zmq_shutdown(void)
{
    if (zmq_pub_socket) {
        zmq_close(zmq_pub_socket);
        zmq_pub_socket = NULL;
    }
    if (zmq_context) {
        zmq_ctx_destroy(zmq_context);
        zmq_context = NULL;
    }
}
#endif

static void ensure_initialized(uint64_t timestamp)
{
    if (initialized)
        return;

    t0 = (timestamp / 1000000000ULL) * 1000000000ULL;

    if (out_file_info == NULL || out_file_info[0] == '\0') {
        static char auto_info[64];
        snprintf(auto_info, sizeof(auto_info),
                 "i-%" PRIu64 "-t1", (uint64_t)(t0 / 1000000000ULL));
        out_file_info = auto_info;
    }
    initialized = 1;
}

void frame_output_print(demod_frame_t *frame)
{
    int suppress_stdout = diagnostic_mode || acars_enabled;

    /* Skip entirely if nothing would receive the output */
    if (suppress_stdout && !ZMQ_ACTIVE)
        return;

    ensure_initialized(frame->timestamp);

    /* Relative timestamp in milliseconds */
    double ts_ms = (double)(frame->timestamp - t0) / 1000000.0;

    /* Center frequency rounded to nearest Hz */
    int freq_hz = (int)(frame->center_frequency + 0.5);

    /* Payload symbols (after unique word) */
    int payload_syms = frame->n_payload_symbols;
    if (payload_syms < 0) payload_syms = 0;

    /* Build line into buffer */
    buf_start();
    buf_printf("RAW: %s %012.4f %010d N:%05.2f%+06.2f I:%011" PRIu64
               " %3d%% %.5f %3d ",
               out_file_info,
               ts_ms,
               freq_hz,
               frame->magnitude,
               frame->noise,
               frame->id,
               frame->confidence,
               frame->level,
               payload_syms);

    for (int i = 0; i < frame->n_bits; i++)
        buf_char('0' + frame->bits[i]);

    buf_char('\n');
    buf_flush(!suppress_stdout);

    /* When --timing (or --pps-ref which implies it) is active, emit the
     * first-symbol absolute wall-clock timestamp to stderr.  This is kept
     * separate from stdout so the machine-readable RAW format is unchanged.
     *
     * src=hw: timestamp anchored to hardware PPS (--pps-ref / VITA49 hw ts)
     * src=sw: timestamp from clock_gettime(CLOCK_REALTIME) software fallback
     *
     * bid (burst identifier): a receiver-independent 64-bit tag that lets two
     * PPS-locked receivers correlate the same-over-the-air burst.  It is a
     * FNV-1a 64-bit hash of three signal properties that are fixed by the
     * transmitter:
     *
     *   uw_sym  -- unique-word arrival time quantised to the nearest symbol
     *              boundary (symbol period = 40 000 ns = 1/25 000 s).
     *              Computed as: round((first_symbol_ns
     *                                 + preamble_symbols × 40 000) / 40 000)
     *              Preamble length: 16 symbols DL, 32 symbols UL.
     *              Tolerance: ±20 µs.  This comfortably absorbs:
     *                - PPS clock accuracy (~100 ns)
     *                - Propagation delay from receiver separation up to ~6 km
     *                  (6 km / c ≈ 20 µs)
     *                - Any sub-symbol timing recovery differences between
     *                  receivers
     *              Bit errors in the decoded payload are not used here at all.
     *
     *   ch2     -- 2-channel bin index relative to 1 616 MHz base.  Each bin
     *              spans 2 × 41 666.667 Hz = 83 333 Hz, giving ±41 667 Hz
     *              of frequency tolerance.  This absorbs:
     *                - SDR oscillator inaccuracy (RTL-SDR: ±20-50 kHz after
     *                  PLL; HackRF/B200: ±5-20 kHz)
     *                - Differential Doppler between receivers at different
     *                  locations (two receivers ~500 km apart on the same
     *                  Iridium pass differ by ~15-30 kHz; up to ~40 kHz for
     *                  extreme 1 000+ km separations)
     *                - PLL residual convergence differences between receivers
     *              Adjacent channels (41 667 Hz apart) remain distinguishable
     *              because two physically different channels land in different
     *              2-channel bins at least half the time; when they share a
     *              bin, the timing field (uw_sym) still separates simultaneous
     *              bursts on different channels unless they happen to land in
     *              the same symbol slot, which is rare.
     *
     *   dir     -- 0 = downlink, 1 = uplink.
     *
     * Both receivers must be PPS-locked (src=hw) for bid to match.
     *
     * See TIMING.md for full details on accuracy and the timing chain. */
    if (precise_timing && frame->first_symbol_ns != 0) {
        const char *src = usrp_pps_ref ? "hw" : "sw";

        /* UW absolute time = first preamble symbol + preamble length in ns */
        int preamble_syms = (frame->direction == DIR_UPLINK)
            ? 32 : IR_PREAMBLE_LENGTH_SHORT;   /* 32 UL, 16 DL */
        uint64_t uw_ns = frame->first_symbol_ns
                         + (uint64_t)preamble_syms * 40000ULL;
        /* Quantise to nearest symbol index (round by adding half-period) */
        uint64_t uw_sym = (uw_ns + 20000ULL) / 40000ULL;

        /* 2-channel bin (83 333 Hz/bin) for ±41 667 Hz Doppler/CFO tolerance */
        double ch_f = (frame->center_frequency - 1616000000.0) / 83333.333;
        int64_t ch2 = (int64_t)(ch_f >= 0.0 ? ch_f + 0.5 : ch_f - 0.5);

        int dir_bit = (frame->direction == DIR_UPLINK) ? 1 : 0;

        /* FNV-1a 64-bit hash over the three fields */
        uint64_t h = 14695981039346656037ULL;  /* FNV offset basis */
#define FNV_MIX64(val) do { \
            uint64_t _v = (uint64_t)(val); \
            for (int _j = 0; _j < 8; _j++) { \
                h ^= (_v & 0xFFU); \
                h *= 1099511628211ULL; \
                _v >>= 8; \
            } \
        } while (0)
        FNV_MIX64(uw_sym);
        FNV_MIX64((uint64_t)ch2);
        FNV_MIX64((uint64_t)dir_bit);
#undef FNV_MIX64

        fprintf(stderr, "# TIMING I:%011" PRIu64 " first_symbol_ns=%" PRIu64
                " src=%s bid=%016" PRIx64 "\n",
                frame->id, frame->first_symbol_ns, src, h);
    }
}

/* ---- Parsed IDA output (iridium-parser.py compatible) ---- */

void frame_output_print_ida(const ida_burst_t *burst)
{
    int suppress_stdout = diagnostic_mode;

    if (suppress_stdout && !ZMQ_ACTIVE)
        return;

    ensure_initialized(burst->timestamp);

    /* Derive parsed file_info: "i-XXXXX-t1" -> "p-XXXXX"
     * Matches iridium-parser.py Message.__init__ */
    char parsed_info[64];
    uint64_t start_ts = t0 / 1000000000ULL;
    snprintf(parsed_info, sizeof(parsed_info), "p-%" PRIu64, start_ts);

    /* Timestamp in milliseconds */
    double ts_ms = (double)(burst->timestamp - t0) / 1000000.0;

    /* Frequency */
    int freq_hz = (int)(burst->frequency + 0.5);

    /* Signal levels: leveldb = 20*log10(level), noise, snr=magnitude */
    double leveldb = (burst->level > 0) ? 20.0 * log10(burst->level) : -99.99;
    double noise = burst->noise;
    double snr = burst->magnitude;

    /* Direction */
    const char *dir = (burst->direction == DIR_UPLINK) ? "UL" : "DL";

    /* Symbol count (payload symbols after UW, minus 12 access code symbols) */
    int syms = burst->n_symbols;
    if (syms < 0) syms = 0;

    buf_start();

    /* Print header: IDA: name timestamp frequency confidence% level|noise|snr symbols direction */
    buf_printf("IDA: %s %014.4f %010d %3d%% %06.2f|%07.2f|%05.2f %3d %s ",
               parsed_info, ts_ms, freq_hz,
               burst->confidence, leveldb, noise, snr,
               syms, dir);

    /* LCW header (already padded to 110+1 chars) */
    buf_printf("%s", burst->lcw_header);

    /* IDA-specific fields from bch_stream */
    const uint8_t *bs = burst->bch_stream;
    int bch_len = burst->bch_len;

    if (bch_len < 20) {
        buf_char('\n');
        buf_flush(!suppress_stdout);
        return;
    }

    /* bits[0:3] */
    buf_printf("%c%c%c", '0' + bs[0], '0' + bs[1], '0' + bs[2]);
    /* cont=bits[3:4] */
    buf_printf(" cont=%c", '0' + bs[3]);
    /* bits[4:5] */
    buf_printf(" %c", '0' + bs[4]);
    /* ctr=bits[5:8] */
    buf_printf(" ctr=%c%c%c", '0' + bs[5], '0' + bs[6], '0' + bs[7]);
    /* bits[8:11] */
    buf_printf(" %c%c%c", '0' + bs[8], '0' + bs[9], '0' + bs[10]);
    /* len=da_len */
    buf_printf(" len=%02d", burst->da_len);
    /* 0:bits[16:20] (bitsparser.py hardcodes "0:" prefix) */
    buf_printf(" 0:%c%c%c%c", '0' + bs[16], '0' + bs[17], '0' + bs[18], '0' + bs[19]);

    /* Hex data payload */
    buf_printf(" [");
    if (burst->da_len > 0) {
        /* Check if trailing bytes are all zero */
        int all_zero = 1;
        for (int i = burst->da_len + 1; i < 20; i++) {
            if (burst->payload[i] != 0) { all_zero = 0; break; }
        }

        if (all_zero) {
            /* Print only da_len bytes */
            for (int i = 0; i < burst->da_len; i++) {
                if (i > 0) buf_char('.');
                buf_printf("%02x", burst->payload[i]);
            }
        } else {
            /* Print all 20 bytes with ! separator at da_len boundary */
            for (int i = 0; i < 20; i++) {
                if (i > 0) {
                    if (i == burst->da_len && burst->da_len > 0 && burst->da_len < 20)
                        buf_char('!');
                    else
                        buf_char('.');
                }
                buf_printf("%02x", burst->payload[i]);
            }
        }
    } else {
        /* len=0: print all 20 bytes */
        for (int i = 0; i < 20; i++) {
            if (i > 0) buf_char('.');
            buf_printf("%02x", burst->payload[i]);
        }
    }

    /* Pad hex data + bracket to 60 chars */
    /* Calculate current hex string length */
    int hexlen;
    if (burst->da_len > 0) {
        int all_zero = 1;
        for (int i = burst->da_len + 1; i < 20; i++) {
            if (burst->payload[i] != 0) { all_zero = 0; break; }
        }
        int nbytes = all_zero ? burst->da_len : 20;
        hexlen = nbytes * 3 - 1 + 1;  /* hex.hex...hex] */
    } else {
        hexlen = 20 * 3 - 1 + 1;
    }
    buf_printf("]");
    /* Pad to 60 chars total (including the closing bracket) */
    for (int i = hexlen; i < 60; i++)
        buf_char(' ');

    /* CRC section */
    if (burst->da_len > 0) {
        buf_printf(" %04x/%04x", burst->stored_crc, burst->computed_crc);
        if (burst->crc_ok)
            buf_printf(" CRC:OK");
        else
            buf_printf(" CRC:no");
    } else {
        buf_printf("  ---   ");
    }

    /* Trailing bits (after CRC) */
    if (bch_len > 9 * 20 + 16) {
        buf_printf(" ");
        for (int i = 9 * 20 + 16; i < bch_len; i++)
            buf_printf("%c", '0' + bs[i]);
    } else {
        buf_printf(" 0000");
    }

    /* SBD ASCII preview (if da_len > 0) */
    if (burst->da_len > 0 && bch_len >= 9 * 20) {
        buf_printf(" SBD: ");
        for (int i = 0; i < 20; i++) {
            /* Extract byte from bch_stream bits[1*20 .. 9*20] */
            int byte = 0;
            for (int b = 0; b < 8; b++)
                byte = (byte << 1) | bs[1 * 20 + i * 8 + b];
            if (byte >= 32 && byte < 127)
                buf_char(byte);
            else
                buf_char('.');
        }
    }

    buf_char('\n');
    buf_flush(!suppress_stdout);
}
