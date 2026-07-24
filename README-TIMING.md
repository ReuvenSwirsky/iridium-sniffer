# Timing & Timestamping (USRP)

This document describes how `iridium-sniffer` timestamps received bursts, the
timing precision you can expect, and the mechanisms that keep timestamps
accurate and drift-free when driving a USRP from an external timing source
(external 10 MHz reference + PPS, or a GPSDO).

## Summary

With an external reference/PPS (or GPSDO):

- The **sample clock is disciplined** by the external 10 MHz reference, so there
  is effectively no frequency drift (sub-ppb).
- Burst timestamps are **labeled to real UTC time-of-day**, aligned to the PPS
  edge, using the host clock only to identify the integer second.
- Timing is **re-anchored to hardware time on every buffer**, so dropped-sample
  (overflow) gaps self-correct instead of accumulating drift.
- Overflows are **counted and reported** instead of being silently ignored.

## How a burst gets its timestamp

1. The USRP RX streamer reports a `time_spec` (seconds + fractional seconds) for
   the first sample of every received buffer (`usrp.c`).
2. That value is converted to nanoseconds and carried on the sample buffer as
   `hw_timestamp_ns` (`sdr.h`).
3. The burst detector records one **anchor** per buffer that maps the absolute
   sample index of the buffer's first sample to its `hw_timestamp_ns`
   (`burst_detect.c`).
4. When a completed burst is emitted, its absolute time is computed from the
   **nearest anchor at or before the burst start**:

   ```
   burst_time_ns = anchor.hw_ts_ns
                 + (burst.start_sample - anchor.sample_index) / sample_rate * 1e9
   ```

5. The downmixer refines the intra-burst timing to the unique-word position and
   emits the final per-frame timestamp.

## UTC / PPS alignment

The PPS edge marks each true UTC second boundary precisely, but the device's
integer-seconds counter is arbitrary until it is labeled. On startup, when a PPS
time source is configured, `usrp.c` performs a one-time alignment
(`usrp_align_time_to_utc()`):

1. Set the time source to `external`/`gpsdo` (PPS).
2. Wait for a PPS edge so there is ~1 s of headroom.
3. Read the host UTC clock and **round to the nearest second** to get the true
   UTC label of the next PPS edge. Rounding (not floor) makes a host clock error
   of tens of milliseconds harmless near the boundary.
4. Arm `uhd_usrp_set_time_next_pps(next_second)` so the device counter takes on
   the correct UTC value at the next edge.
5. Start streaming on the following PPS edge with a timed stream command
   (`stream_now = 0`, `time_spec = next_second + 1`), so **buffer 0 lands exactly
   on a UTC second boundary**.

> Only the *integer second* comes from the host clock; all sub-second precision
> comes from the PPS. The host clock only needs to be within ±0.5 s of true UTC
> (trivially satisfied by NTP).

### Requirements

- An external PPS (via `external`) or a GPSDO (`gpsdo`) as the **time source**.
- An external 10 MHz reference (or GPSDO) as the **clock source** for a
  disciplined, drift-free sample clock.
- A host clock accurate to better than ±0.5 s (NTP).

If no PPS is detected, UTC labeling is disabled and the code falls back to
`stream_now` with a host wall-clock origin.

## Minute-aligned start (`--start-next-minute`)

The `--start-next-minute` flag delays the USRP capture until the top of the next
UTC minute, so the first sample lands exactly on a `:00` boundary. This is useful
for aligning captures across multiple receivers or with external schedules.

- **With a PPS time source:** the stream start second is rounded up from the
  PPS-labeled UTC time to the next whole minute and started with a timed stream
  command — minute-accurate to the PPS (tens of ns).
- **Without a PPS (internal clock, or PPS not detected):** the stream thread
  waits on the host clock until the top of the next UTC minute, then starts —
  accurate to the host clock (typically ms via NTP).

Example:

```
iridium-sniffer -i usrp-... --clock-source external --time-source external \
    --start-next-minute
```

## Timing precision (default settings)

| Parameter | Default | Notes |
|-----------|---------|-------|
| Sample rate | 10 MHz | 100 ns/sample |
| FFT size | 8192 | ~819 µs coarse detection frame |
| Downmix output rate | 250 kHz | 4 µs/sample fine alignment |

- **Relative precision:** a few microseconds per burst. Coarse detection is
  quantized to one FFT frame (~819 µs), but the downmixer re-aligns to the
  unique word at the 250 kHz output rate (4 µs/sample) plus a fractional
  correction.
- **Absolute accuracy:** limited by the PPS source accuracy (tens of ns for a
  GPSDO), plus a fixed, unmodeled antenna-to-timestamp latency offset.
- **Long-term drift:** none from the sample clock (disciplined by the external
  reference); dropped samples are corrected by re-anchoring (below).

## Drift protection

### Continuous re-anchoring

Rather than latching a single origin at sample 0 and counting samples forever,
the detector keeps a small FIFO of `(sample_index, hw_ts_ns)` anchors — one per
buffer — pruned to the span of the IQ ringbuffer. Each burst's time is derived
from the nearest preceding anchor.

Because each buffer's hardware `time_spec` already reflects true elapsed time,
any gap from dropped samples is absorbed at the very next anchor. Residual error
is bounded to the sub-buffer interval since the last anchor, instead of
accumulating for the rest of the capture.

This applies to both the int8 and float32 sample paths, and to both the CPU and
GPU processing paths. When no hardware timestamp is available (e.g. internal
clock source), the detector falls back to the original single-origin behavior
(host wall clock captured once at the first buffer).

### Overflow accounting

RX overflows (dropped samples) were previously ignored silently. They are now:

- Counted in the atomic `stat_n_overflows` (`main.c`, `usrp.c`).
- Logged to stderr under `-v` (`USRP: overflow (samples dropped)`).
- Reported in the status line as `ov: N` (shown only when non-zero).

Explicit correction of the missing sample count is unnecessary because
re-anchoring already self-corrects the timing; the counter exists for
visibility.

## Files changed

| File | Change |
|------|--------|
| `usrp.c` | `usrp_align_time_to_utc()`: PPS-edge sync + UTC labeling via `set_time_next_pps`; timed stream start on a PPS edge; `--start-next-minute` alignment (PPS-timed and host-clock fallback); overflow counting. |
| `burst_detect.c` | `ts_anchor_t` FIFO of per-buffer `(sample_index, hw_ts_ns)` anchors; anchor push in both feed paths; burst time re-derivation from nearest anchor in `emit_gone_bursts()`. |
| `main.c` | `stat_n_overflows` atomic counter and `ov: N` status-line field; `start_next_minute` global. |
| `options.c` | `--start-next-minute` flag. |
| `sdr.h` | `hw_timestamp_ns` field on the sample buffer (already present, carries the per-buffer hardware timestamp). |

## Limitations & notes

- UTC labeling relies on the host clock being within ±0.5 s of true UTC to pick
  the correct integer second. This is not the same as sub-second accuracy, which
  comes entirely from the PPS.
- A GPSDO's raw `gps_time` sensor reports GPS time (no leap seconds); this code
  deliberately labels from the host UTC clock instead, avoiding the GPS↔UTC
  leap-second offset.
- There is a fixed, unmodeled hardware latency between the antenna and the point
  where the timestamp is latched. If you need absolute accuracy better than that
  offset, characterize and subtract it separately.
- A burst that spans a dropped-sample gap (pathological, since Iridium bursts are
  short) may carry a small timing error; normal bursts starting after the gap are
  correct.
