#!/usr/bin/env python3
"""
compare_bursts.py -- match Iridium bursts captured by two receivers.

Each receiver's --save-bursts directory holds <name>.cf32 IQ files and
<name>.meta sidecars.  The meta files now carry a `demod_bits` field: the
DQPSK-decoded bit sequence.  Because DQPSK differential decoding removes each
receiver's absolute LO-phase offset, the *same* physical burst produces the
*same* bits on every receiver -- so we can correlate captures across sites even
though the file names, burst IDs, timestamps and per-burst frequency estimates
all differ.

The script pairs bursts from two directories (default "d" and "r") by sliding
one bit string against the other and taking the best bit-error rate (BER) over
the overlap.  Matches are the starting point for TDOA: once a burst pair is
identified, cross-correlate the two .cf32 files for the fine time offset.

Usage:
    python3 compare_bursts.py d r
    python3 compare_bursts.py d r --max-ber 0.10 --max-shift 96
"""

import argparse
import os
import sys
import glob

try:
    import numpy as np
except ImportError:
    sys.exit("error: this script requires numpy (pip install numpy)")


class Burst:
    __slots__ = ("path", "meta", "bits", "n_bits", "timestamp_ns",
                 "center_freq_hz", "direction", "burst_id")

    def __init__(self, path, meta, bits, n_bits):
        self.path = path            # path to the .cf32 file
        self.meta = meta            # raw key -> str dict
        self.bits = bits            # numpy uint8 array of 0/1, length n_bits
        self.n_bits = n_bits
        self.timestamp_ns = int(meta.get("timestamp_ns", "0"))
        self.center_freq_hz = float(meta.get("center_freq_hz", "0"))
        self.direction = meta.get("direction", "??")
        self.burst_id = meta.get("burst_id", "?")


def parse_meta(meta_path):
    """Parse a `key: value` meta file into a dict."""
    d = {}
    with open(meta_path, "r") as f:
        for line in f:
            line = line.strip()
            if not line or ":" not in line:
                continue
            key, _, val = line.partition(":")
            d[key.strip()] = val.strip()
    return d


def bits_from_hex(hexstr, n_bits):
    """Unpack a hex string into a numpy uint8 array of 0/1 truncated to n_bits."""
    raw = bytes.fromhex(hexstr)
    bits = np.unpackbits(np.frombuffer(raw, dtype=np.uint8))
    return bits[:n_bits]


def load_dir(dirname):
    """Load all bursts (with demod_bits) from a directory."""
    bursts = []
    skipped = 0
    for meta_path in sorted(glob.glob(os.path.join(dirname, "*.meta"))):
        meta = parse_meta(meta_path)
        hexstr = meta.get("demod_bits")
        n_bits = int(meta.get("demod_n_bits", "0"))
        if not hexstr or n_bits <= 0:
            skipped += 1
            continue
        bits = bits_from_hex(hexstr, n_bits)
        cf32 = meta_path[:-len(".meta")] + ".cf32"
        bursts.append(Burst(cf32, meta, bits, n_bits))
    return bursts, skipped


def best_alignment(a, b, max_shift, min_overlap):
    """
    Slide bit array `b` against `a` over [-max_shift, +max_shift] and return
    (best_ber, best_shift, best_overlap).  `shift` is the offset of `b`
    relative to `a`; positive means b is delayed.  Returns None if no shift
    yields at least `min_overlap` overlapping bits.
    """
    best = None
    la, lb = len(a), len(b)
    for shift in range(-max_shift, max_shift + 1):
        if shift >= 0:
            aa = a[shift:]
            bb = b[:len(aa)]
        else:
            bb = b[-shift:]
            aa = a[:len(bb)]
        n = min(len(aa), len(bb))
        if n < min_overlap:
            continue
        diffs = int(np.count_nonzero(aa[:n] != bb[:n]))
        ber = diffs / n
        if best is None or ber < best[0]:
            best = (ber, shift, n)
    return best


def main():
    ap = argparse.ArgumentParser(
        description="Match Iridium bursts between two receiver directories "
                    "by their DQPSK-decoded bits.")
    ap.add_argument("dir_d", nargs="?", default="d",
                    help="first burst directory (default: d)")
    ap.add_argument("dir_r", nargs="?", default="r",
                    help="second burst directory (default: r)")
    ap.add_argument("--max-ber", type=float, default=0.15,
                    help="max bit-error rate to accept a match (default: 0.15)")
    ap.add_argument("--max-shift", type=int, default=64,
                    help="max bit shift to search for alignment (default: 64)")
    ap.add_argument("--min-overlap", type=int, default=100,
                    help="min overlapping bits required (default: 100)")
    ap.add_argument("--freq-window", type=float, default=0.0,
                    help="if >0, only compare bursts whose center freqs differ "
                         "by less than this many Hz (prefilter, default: off)")
    ap.add_argument("--time-window", type=float, default=0.0,
                    help="if >0, only compare bursts whose timestamps differ by "
                         "less than this many ns (prefilter, default: off)")
    ap.add_argument("--show-unmatched", action="store_true",
                    help="also list bursts with no match")
    args = ap.parse_args()

    d_bursts, d_skip = load_dir(args.dir_d)
    r_bursts, r_skip = load_dir(args.dir_r)

    print(f"# loaded {len(d_bursts)} bursts from '{args.dir_d}' "
          f"({d_skip} without demod_bits)")
    print(f"# loaded {len(r_bursts)} bursts from '{args.dir_r}' "
          f"({r_skip} without demod_bits)")
    if not d_bursts or not r_bursts:
        print("# nothing to compare")
        return

    # Build all candidate pairs that pass the optional pre-filters and the
    # BER threshold, then greedily assign one-to-one matches best-first.
    candidates = []
    for i, db in enumerate(d_bursts):
        for j, rb in enumerate(r_bursts):
            if args.freq_window > 0 and \
                    abs(db.center_freq_hz - rb.center_freq_hz) > args.freq_window:
                continue
            if args.time_window > 0 and \
                    abs(db.timestamp_ns - rb.timestamp_ns) > args.time_window:
                continue
            res = best_alignment(db.bits, rb.bits, args.max_shift,
                                 args.min_overlap)
            if res is None:
                continue
            ber, shift, overlap = res
            if ber <= args.max_ber:
                candidates.append((ber, shift, overlap, i, j))

    candidates.sort(key=lambda c: c[0])

    d_used = set()
    r_used = set()
    matches = []
    for ber, shift, overlap, i, j in candidates:
        if i in d_used or j in r_used:
            continue
        d_used.add(i)
        r_used.add(j)
        matches.append((ber, shift, overlap, i, j))

    # Report.
    print()
    print(f"# {len(matches)} matched pair(s)")
    header = (f"{'BER':>7}  {'shift':>5}  {'overlap':>7}  {'dt_ns':>14}  "
              f"{'df_hz':>10}  {'dir':>3}  d_id/r_id  files")
    print(header)
    print("# " + "-" * (len(header) - 2))
    for ber, shift, overlap, i, j in sorted(matches, key=lambda m: m[0]):
        db, rb = d_bursts[i], r_bursts[j]
        dt = db.timestamp_ns - rb.timestamp_ns
        df = db.center_freq_hz - rb.center_freq_hz
        dir_tag = db.direction if db.direction == rb.direction \
            else f"{db.direction}/{rb.direction}"
        print(f"{ber*100:6.2f}%  {shift:5d}  {overlap:7d}  {dt:14d}  "
              f"{df:10.1f}  {dir_tag:>3}  {db.burst_id}/{rb.burst_id}")
        print(f"           d: {os.path.basename(db.path)}")
        print(f"           r: {os.path.basename(rb.path)}")

    if args.show_unmatched:
        d_missing = [b for k, b in enumerate(d_bursts) if k not in d_used]
        r_missing = [b for k, b in enumerate(r_bursts) if k not in r_used]
        print(f"\n# {len(d_missing)} unmatched in '{args.dir_d}', "
              f"{len(r_missing)} unmatched in '{args.dir_r}'")
        for b in d_missing:
            print(f"  d-only  id={b.burst_id}  {b.direction}  "
                  f"{b.center_freq_hz:.0f} Hz  {os.path.basename(b.path)}")
        for b in r_missing:
            print(f"  r-only  id={b.burst_id}  {b.direction}  "
                  f"{b.center_freq_hz:.0f} Hz  {os.path.basename(b.path)}")


if __name__ == "__main__":
    main()
