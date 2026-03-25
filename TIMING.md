# Precise Packet Timing

## Overview

`iridium-sniffer` can emit the **absolute wall-clock time of the first preamble
symbol** of every decoded Iridium burst to stderr, leaving the stdout RAW
format completely unchanged so downstream tools (`iridium-parser.py`, etc.)
continue to work without modification.

Two options control this:

| Option | Works with | Timestamp accuracy |
|--------|-----------|--------------------|
| `--timing` | Any SDR, IQ file, VITA 49, ZMQ SUB | ~10 ms (software `clock_gettime`) |
| `--pps-ref` | USRP only (requires `--clock-source` + `--time-source=external/gpsdo`) | < 10 µs (hardware GPS-disciplined PPS) |

`--pps-ref` implies `--timing` automatically.

The output line includes a `src=` field that identifies the timestamp source:

```
# TIMING I:00000000010 first_symbol_ns=1711234567123456789 src=hw bid=a3f2c1d0e5b6a712
                                                            ^^^^^^     ^^^^^^^^^^^^^^^^
                                                            hw = hardware PPS
                                                            sw = software clock
                                                                       see below
```

---

## Quick Start

### Any SDR or IQ file — software clock (~10 ms accuracy)

```bash
iridium-sniffer -l -i soapy-0 --timing 2>timing.log | iridium-parser.py
```

### USRP with GPS-disciplined PPS — hardware accuracy (< 10 µs)

```bash
iridium-sniffer \
    -i usrp-B210-<SERIAL> \
    --clock-source=external \
    --time-source=external \
    --pps-ref \
    -c 1622000000 \
    -r 10000000 2>timing.log | iridium-parser.py
```

With `--pps-ref` the program will:
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
Timestamp origin (one of the following):
  hw_timestamp_ns   USRP+PPS: uhd_rx_metadata_time_spec() at start of each USB buffer
                    VITA 49: embedded timestamp from VRT packet header (future)
  clock_gettime()   all other inputs: software wall-clock at first buffer receipt
               │
               ▼  sample_buf_t::hw_timestamp_ns  (ns since Unix epoch, or 0=none)
               │
               ▼  burst_detect.c  burst_detector_feed() / burst_detector_feed_cf32()
  start_time_ns = hw_timestamp_ns if non-zero, else clock_gettime(CLOCK_REALTIME)
  (set once on the very first call; held in burst_detector_t::start_time_ns)
               │
               ▼  burst detected at absolute sample index  burst.info.start
               │  (granularity = one FFT frame = fft_size / sample_rate ≈ 0.82 ms)
               │
  burst_data_t::start_time_ns  = d->start_time_ns      ← origin tag follows burst
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
  frame->first_symbol_ns = (start_time_ns != 0)
      ? frame->timestamp + preamble_start / 250000 * 1e9
      : 0  ← always non-zero in practice (start_time_ns set from clock_gettime)
               │
               ▼  qpsk_demod.c  copies first_symbol_ns unchanged
               │
               ▼  frame_output.c  frame_output_print()
  stdout:  unchanged RAW: line with relative timestamp in milliseconds
  stderr:  "# TIMING I:NNNNNNNNNNN first_symbol_ns=MMMMMMMMMMMMMMMM src=hw|sw"
           src=hw when usrp_pps_ref is set (hardware GPS-disciplined PPS)
           src=sw when falling back to clock_gettime(CLOCK_REALTIME)
           emitted when --timing (or --pps-ref) is active and first_symbol_ns != 0
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
frame analysis.  It does not change regardless of whether `--timing` or
`--pps-ref` is used.

### Absolute timing output (stderr — with `--timing` or `--pps-ref`)

```
# TIMING I:00000000010 first_symbol_ns=1711234567123456789 src=hw bid=a3f2c1d0e5b6a712
            ^^^^^^^^^^^                 ^^^^^^^^^^^^^^^^^^^      ^^     ^^^^^^^^^^^^^^^^
            Burst ID (matches the      Nanoseconds since         hw     Cross-receiver
            I: field in the RAW line)  Unix epoch (UTC) of       sw     burst correlator
                                       the first preamble symbol         (see below)
```

You can correlate the two streams by matching the burst `I:` field:

```bash
iridium-sniffer ... --timing 2>timing.log | iridium-parser.py

# in timing.log:
# TIMING I:00000000010 first_symbol_ns=1711234567123456789 src=sw
# TIMING I:00000000020 first_symbol_ns=1711234567145678901 src=sw
```

With `--pps-ref` (USRP + hardware PPS) you instead see `src=hw`.

---

## Cross-Receiver Burst Correlation (`bid`)

The `bid` field is a **receiver-independent 64-bit burst identifier** that
allows two separate receivers, running completely independently, to discover
that they decoded the same over-the-air Iridium burst.

It is a FNV-1a 64-bit hash of three signal properties determined solely by
the transmitter — not by anything local to the receiver:

### Input 1 — `uw_sym`: UW arrival time (symbol-quantised)

```
uw_sym = round((first_symbol_ns + preamble_symbols × 40 000) / 40 000)
```

This converts the UW arrival time into an **absolute Iridium symbol index**
(one symbol = 40 000 ns = 1/25 000 s).  Both receivers will compute the
same integer so long as their PPS-anchored clocks agree to within **± 20 µs**
(half a symbol period).

| Error source | Magnitude | Within ± 20 µs? |
|---|---|---|
| GPSDO absolute accuracy | < 100 ns | ✓ |
| `set_time_next_pps` quantisation | < 1 sample (100 ns at 10 Msps) | ✓ |
| UHD USB transfer jitter | 1–5 µs (B-series) | ✓ |
| Decimated burst-onset granularity | ~4 µs (1 sample at 250 kHz) | ✓ |
| **Propagation delay from receiver separation** | **~3.3 µs/km** | **✓ up to ~6 km separation** |

For receivers separated by more than ~6 km, the far receiver will see the
burst arriving up to 20 µs later than the near receiver.  At extreme
separations (> 6 km) the `uw_sym` values may differ by 1, causing a `bid`
mismatch.  This is an inherent physical limit of a single symbol-period bin.

### Input 2 — `ch2`: frequency (2-channel bin)

```
ch2 = round((center_frequency − 1 616 000 000 Hz) / 83 333 Hz)
```

This maps the measured center frequency into a **2-channel bin** (two Iridium
channel spacings = 83 333 Hz wide, ± 41 667 Hz per side).  This wide bin
absorbs all realistic sources of per-receiver frequency error:

| Error source | Magnitude | Within ± 41 667 Hz? |
|---|---|---|
| RTL-SDR TCXO inaccuracy (before calibration) | ± 20–50 kHz | ✓ (marginal for cheap dongles) |
| HackRF / B200 oscillator residual after PLL | ± 5–20 kHz | ✓ |
| Differential Doppler, 500 km receiver separation | ~ 15–30 kHz | ✓ |
| Differential Doppler, 1 000+ km separation (extreme) | up to ±40 kHz | ✓ (just) |

Two adjacent Iridium channels (41 667 Hz apart) will occasionally share the
same `ch2` bin when one sits precisely on a bin boundary; in that case the
`uw_sym` timing field still separates simultaneous bursts because they arrive
at different symbol times.

### Input 3 — `dir`: link direction

`dir = 0` for downlink, `dir = 1` for uplink.  A downlink burst and an
uplink burst on the same frequency and time slot will never share a `bid`.

### Hash algorithm

FNV-1a 64-bit hash (offset basis `14695981039346656037`, prime
`1099511628211`) over the three 64-bit little-endian inputs in order:
`uw_sym`, `ch2`, `dir`.

### When `bid` will match across receivers

- Both receivers must be PPS-locked (`src=hw`).  A `src=sw` receiver has
  ~10 ms timing uncertainty — 250× the ±20 µs symbol bin — so `uw_sym`
  will almost never agree.
- Receiver separation must be small enough that propagation delay is
  < 20 µs, i.e., ≲ 6 km direct path difference.
- SDR oscillator error must be < 41 667 Hz including Doppler.

### Matching `bid` values across two log files

```bash
# Extract all bid values from receiver A's timing log
grep '^# TIMING' rx_a.log | awk '{print $NF}' | grep ^bid= | sort > bids_a.txt

# Extract from receiver B
grep '^# TIMING' rx_b.log | awk '{print $NF}' | grep ^bid= | sort > bids_b.txt

# Show bursts seen by both
comm -12 bids_a.txt bids_b.txt
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

| Source of error | With `--timing` (sw) | With `--pps-ref` (hw) |
|---|---|---|
| Timestamp origin | `clock_gettime(CLOCK_REALTIME)` on first buffer | USRP UHD hardware timespec per USB buffer |
| Absolute accuracy | ~10 ms (OS scheduler jitter) | < 100 ns with GPSDO |
| USRP + GPSDO (e.g. Jackson Labs LC_XO) | N/A | < 50 ns absolute, < 10 ns jitter |
| `set_time_next_pps` quantization | N/A | < 1 sample period (100 ns at 10 Msps) |
| UHD USB/PCIe transfer latency jitter | N/A | 1–5 µs (B-series), < 1 µs (X/N-series) |
| FFT burst-start detection granularity | ~0.82 ms (1 FFT frame at 10 Msps / 8192) | ~0.82 ms |
| Decimated burst-start refinement | ~4 µs (1 decimated sample at 250 kHz) | ~4 µs |
| **Sync-word correlation peak** | **< 1 decimated sample ≈ 4 µs** | **< 4 µs** |
| RRC matched filter group delay | accounted for in `decimate_burst()` | accounted for |

With a well-disciplined GPSDO (`--pps-ref`) the **combined absolute uncertainty**
is typically **< 10 µs**, dominated by decimated-domain burst-onset detection.

With software clock (`--timing` only) the absolute accuracy is dominated by
OS scheduling jitter (~1–10 ms), but **relative timing between successive
bursts within the same session** is much better because the `start_time_ns`
origin is a single `clock_gettime()` call shared by all subsequent bursts.

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
| TIMING stderr output | [frame_output.c](frame_output.c) `frame_output_print()` | `precise_timing` flag, `src=` qualifier, `bid=` hash |
