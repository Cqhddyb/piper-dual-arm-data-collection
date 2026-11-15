"""Timing statistics preserve file order when counting regressions."""
import numpy as np


def describe(values):
    a = np.asarray(values, dtype=np.float64)
    a = a[np.isfinite(a)]
    if not len(a):
        return {"n": 0, "mean": None, "p95": None, "max": None, "min": None}
    return {"n": len(a), "mean": float(a.mean()), "p95": float(np.percentile(a, 95)),
            "max": float(a.max()), "min": float(a.min())}


def timestamp_stats(stamps):
    a = np.asarray(stamps, dtype=np.int64)
    if not len(a):
        return {"count": 0}
    dt = np.diff(a)
    span = int(a[-1]) - int(a[0])
    return {"count": len(a), "first_ns": int(a[0]), "last_ns": int(a[-1]),
            "span_s": span / 1e9,
            "zero_stamps": int(np.sum(a == 0)),
            "negative_stamps": int(np.sum(a < 0)),
            "unique_positive_stamps": len(np.unique(a[a > 0])),
            "rate_hz": (len(a) - 1) * 1e9 / span if span > 0 else None,
            "interval_ms": describe(dt / 1e6),
            "adjacent_duplicates": int(np.sum(dt == 0)),
            "duplicate_values": len(a) - len(np.unique(a)),
            "regressions": int(np.sum(dt < 0)),
            "gaps_over_69ms": int(np.sum(dt > 69_000_000))}


def nearest_offsets_ns(reference, query):
    """Absolute nearest-neighbor offsets; not mutual pairing or hardware skew."""
    a = np.sort(np.asarray(reference, dtype=np.int64))
    b = np.asarray(query, dtype=np.int64)
    if not len(a) or not len(b):
        return np.array([], dtype=np.int64)
    i = np.searchsorted(a, b)
    left = np.maximum(0, i - 1)
    right = np.minimum(len(a) - 1, i)
    return np.minimum(np.abs(a[left] - b), np.abs(a[right] - b))
