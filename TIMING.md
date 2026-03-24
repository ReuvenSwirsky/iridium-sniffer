# Precise Packet Timing with a PPS / 10 MHz Reference

## Overview

`iridium-sniffer` supports locking timestamps to a hardware PPS pulse so that
every detected Iridium burst is time-stamped with its **first preamble symbol's
absolute wall-clock time**, traceable to UTC.  The RAW output format printed to
stdout is **unchanged** — downstream tools (`iridium-parser.py`, etc.) continue
to work without modification.  Absolute timing information is emitted separately
to **stderr**.

---

## Prerequisites

| Hardware | Required |
|---|---|
| USRP with an external 10 MHz reference input | clock accuracy / phase stability |
| External PPS source (GPS receiver, GPSDO, or rubidium oscillator) connected to the USRP PPS input | absolute time-of-day anchoring |
| Matching `--clock-source` and `--time-source` CLI flags | tells the driver to use those inputs |

A GPSDO (e.g. Jackson Labs LC_XO or SBG GNSS-disciplined oscillators) provides
both references in a single unit.

---

## How to Enable Precise Timing

```bash
iridium-sniffer \
    -i usrp-B210-<SERIAL> \
    --clock-source=external \   # 10 MHz reference
    --time-source=external \    # PPS
    --pps-ref \                 # enable lock-wait + time-set
    -c 1622000000 \
    -r 10000000 | iridium-parser.py
```

On startup the program will:
1. Configure the USRP clock and time sources to the external inputs.
2. Poll the USRP sensors (`pps_locked`, `gps_locked`, or `ref_locked`) once per
   second for up to 30 seconds until lock is confirmed.
3. Call `uhd_usrp_set_time_next_pps(now+1, 0.0)` to latch the device's
   free-running sample counter to the next PPS edge, anchoring it to UTC.
4. Wait a further 2 seconds for the latch to propagate before streaming starts.

The `--pps-ref` flag has **no effect on non-USRP builds**; the compiler will
reject it with an error message if UHD support is not compiled in.

---

## Timing Chain — Stage by Stage

```
PPS pulse → USRP hardware latches sample counter to wall-clock second
               │
               ▼
uhd_rx_streamer_recv() → uhd_rx_metadata_time_spec()
   returns (full_secs, frac_secs) for the FIRST sample of each USB buffer
               │
               ▼  sample_buf_t::hw_timestamp_ns  (ns since Unix epoch)
               │
               ▼  burst_detect.c  burst_detector_feed()
   start_time_ns = hw_timestamp_ns of the very first buffer received
   (or clock_gettime(CLOCK_REALTIME) when no hardware timestamp is available)
               │
               ▼  burst detected at absolute sample index  burst.info.start
               │  (granularity = one FFT frame = fft_size / sample_rate ≈ 0.82 ms)
               │
   burst_data_t::start_time_ns  = start_time_ns
   burst_data_t::info.start     = sample index (at capture sample rate)
               │
               ▼  burst_downmix.c  burst_downmix_process()
   timestamp = start_time_ns + info.start / sample_rate * 1e9
   decimate_burst() adjusts timestamp for LPF group delay:
       timestamp += (ntaps/2) / sample_rate * 1e9
   find_burst_start() locates burst energy onset in the decimated frame
       → 'start' index (at 250 kHz output rate)
   frame->timestamp = timestamp + start / 250000 * 1e9   ← burst energy onset
               │
               ▼  correlate_sync() finds the unique word (UW) via FFT correlation
   uw_start  = sample index of UW first sample in the decimated frame
   preamble_start = uw_start − preamble_symbols × samples_per_symbol
   frame->first_symbol_ns = frame->timestamp
                           + preamble_start / 250000 * 1e9
               │
               ▼  qpsk_demod.c  copies first_symbol_ns unchanged
               │
               ▼  frame_output.c  frame_output_print()
   stdout:  unchanged RAW: line with relative timestamp in milliseconds
   stderr:  "# TIMING I:NNNNNNNNNNN first_symbol_ns=MMMMMMMMMMMMMMMM"
            (only when --pps-ref is active and a hw timestamp was available)
```

---

## Reading the Timestamps

### Standard RAW output (stdout — unchanged)

```
RAW: i-1711234567-t1 000000.0000 1622025000 N:22.50 -3.20 I:00000000010 100% 0.98765 179 01101...
                     ^^^^^^^^^^^^
                     Relative timestamp in milliseconds from session start (t0).
                     t0 is the Unix epoch second of the first burst, rounded down.
```

This timestamp is **relative** and suitable for iridium-toolkit's differential
frame analysis.  It does not change regardless of whether `--pps-ref` is used.

### Absolute timing output (stderr — only with `--pps-ref`)

```
# TIMING I:00000000010 first_symbol_ns=1711234567123456789
            ^^^^^^^^^^^                 ^^^^^^^^^^^^^^^^^^^
            Burst ID (matches the      Nanoseconds since Unix epoch (UTC)
            I: field in the RAW line)  of the first preamble symbol
```

You can correlate the two streams by matching the burst `I:` field:

```bash
iridium-sniffer ... --pps-ref 2>timing.log | iridium-parser.py

# in timing.log:
# TIMING I:00000000010 first_symbol_ns=1711234567123456789
# TIMING I:00000000020 first_symbol_ns=1711234567145678901
```

Convert to human-readable UTC in Python:
```python
import datetime
ns = 1711234567123456789
dt = datetime.datetime.utcfromtimestamp(ns / 1e9)
print(dt.isoformat())   # 2024-03-23T21:36:07.123456
```

Or with `date`:
```bash
date -d @$(echo "1711234567123456789 / 1000000000" | bc) --utc
```

---

## What `first_symbol_ns` Actually Represents

`first_symbol_ns` is the estimated wall-clock UTC time, in nanoseconds since the
Unix epoch (1970-01-01 00:00:00 UTC), at which the **first chip of the Iridium
preamble** arrived at the USRP antenna port.

The preamble immediately precedes the Unique Word (UW):

```
  ← Burst envelope →
  [ pre-burst guard ][ Preamble (16 DL / 64 UL symbols) ][ UW (12 sym) ][ Data ]
  ^                   ^
  burst energy onset  first_symbol_ns ← this sample
  (frame->timestamp)
```

Both DL and UL frames follow the same structure.  The UW starts at
`first_symbol_ns + preamble_symbols / 25000 * 1e9` ns:
- Downlink: 16 symbols → 640 µs after `first_symbol_ns`
- Uplink:   64 symbols → 2560 µs after `first_symbol_ns`

---

## Achievable Precision

| Source of error | Magnitude |
|---|---|
| USRP + GPSDO (e.g. Jackson Labs LC_XO) | < 50 ns absolute, < 10 ns jitter |
| USRP + external 10 MHz + GPS PPS | < 100 ns absolute |
| `set_time_next_pps` quantization | < 1 sample period (100 ns at 10 Msps) |
| UHD USB/PCIe transfer latency jitter | 1–5 µs (B-series), < 1 µs (X/N-series) |
| FFT burst-start detection granularity | ~0.82 ms (1 FFT frame at 10 Msps / 8192) |
| Decimated burst-start refinement | ~4 µs (1 decimated sample at 250 kHz) |
| **Sync-word correlation peak** | **< 1 decimated sample ≈ 4 µs** |
| RRC matched filter group delay | accounted for in `decimate_burst()` |

With a well-disciplined GPSDO the **combined absolute uncertainty** of
`first_symbol_ns` is typically **< 10 µs** at 10 Msps, dominated by the
decimated-domain burst-onset detection.  Relative timing *between successive
bursts* (same session) is significantly better because the `start_time_ns`
origin is shared.

---

## Computing the Exact Capture-Rate Sample Index

If you need to know which sample index in the original 10 Msps capture
corresponds to the first preamble symbol, use:

```
decimation_factor = capture_sample_rate / 250000
                  = 10000000 / 250000 = 40

absolute_first_symbol_sample =
    burst.info.start                         # FFT-domain burst onset (capture samples)
    + preamble_start_decimated * decimation  # preamble position back in capture domain
    + lp_filter_half_taps                    # add back the LPF group delay that was
                                             # subtracted during decimation
```

`preamble_start_decimated` (in the 250 kHz domain) equals
`uw_start - preamble_symbols * samples_per_symbol` where `samples_per_symbol`
is 10 at the default 250 kHz / 25 ksym/s rate.

In nanoseconds this equals `first_symbol_ns - start_time_ns` divided by the
capture sample period, which is equivalent to:

```python
first_sample_idx = int((first_symbol_ns - start_time_ns) * capture_rate / 1e9)
```

`start_time_ns` is the `hw_timestamp_ns` from the very first received buffer
(printed to stderr with `-v`, or derivable as
`t0_seconds * 1e9` from the RAW output file-info field).

---

## Internals — Code Locations

| Stage | File | Key variable |
|---|---|---|
| Hardware timestamp extraction | [usrp.c](usrp.c) `usrp_stream_thread()` | `s->hw_timestamp_ns` |
| PPS lock + set_time_next_pps | [usrp.c](usrp.c) `usrp_setup()` | `usrp_pps_ref` global |
| Base timestamp anchoring | [burst_detect.c](burst_detect.c) `burst_detector_feed()` | `d->start_time_ns` |
| Burst-onset → ns conversion | [burst_downmix.c](burst_downmix.c) `burst_downmix_process()` | `timestamp` local |
| LPF group-delay correction | [burst_downmix.c](burst_downmix.c) `decimate_burst()` | `delay_ns` |
| First-symbol timestamp | [burst_downmix.c](burst_downmix.c) `burst_downmix_process()` | `frame->first_symbol_ns` |
| Propagation through QPSK | [qpsk_demod.c](qpsk_demod.c) `qpsk_demod()` | `frame->first_symbol_ns` |
| TIMING stderr output | [frame_output.c](frame_output.c) `frame_output_print()` | `usrp_pps_ref` guard |
