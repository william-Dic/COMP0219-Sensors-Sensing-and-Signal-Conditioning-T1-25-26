#!/usr/bin/env python3
"""Offline calibration using stored CSV logs.

This script consumes the aligned CSV exports produced by
`Testing_mac.py` (saved under the `runs/` directory) and estimates an
updated `CALIBRATED_C_OVERRIDE` constant so the firmware's velocity
output matches the logged ground-truth measurements.

Usage example:

    python3 calibrate_offline.py --log runs/log-20250223-101530-aligned.csv \
        --current-c 0.001432 --min-speed 0.6 --snippet

"""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from pathlib import Path
from typing import Iterable, List, Sequence, Tuple


try:  # YAML output is optional
    import yaml  # type: ignore
except Exception:  # pragma: no cover - yaml is optional
    yaml = None  # type: ignore


def load_pairs(
    files: Sequence[Path],
    student_col: str,
    gt_col: str,
) -> List[Tuple[float, float]]:
    """Return {(student, ground_truth)} pairs with both values present."""

    pairs: List[Tuple[float, float]] = []
    for path in files:
        with path.open(newline="") as fh:
            reader = csv.DictReader(fh)
            if student_col not in reader.fieldnames or gt_col not in reader.fieldnames:
                raise ValueError(
                    f"{path} does not contain required columns '{student_col}' and '{gt_col}'."
                )
            for row in reader:
                s_raw = (row.get(student_col) or "").strip()
                g_raw = (row.get(gt_col) or "").strip()
                if not s_raw or not g_raw:
                    continue
                try:
                    s_val = float(s_raw)
                    g_val = float(g_raw)
                except ValueError:
                    continue
                pairs.append((s_val, g_val))
    if not pairs:
        raise RuntimeError("No usable samples found in the provided logs.")
    return pairs


def filter_pairs(
    pairs: Iterable[Tuple[float, float]],
    min_speed: float,
    max_speed: float | None,
) -> List[Tuple[float, float]]:
    result: List[Tuple[float, float]] = []
    for student_v, gt_v in pairs:
        gt_abs = abs(gt_v)
        if gt_abs < min_speed:
            continue
        if max_speed is not None and gt_abs > max_speed:
            continue
        if not math.isfinite(student_v) or not math.isfinite(gt_v):
            continue
        result.append((student_v, gt_v))
    if not result:
        raise RuntimeError(
            "No samples remain after filtering. Reduce --min-speed or collect more data."
        )
    return result


def compute_new_constant(
    samples: Sequence[Tuple[float, float]],
    current_c: float,
) -> Tuple[float, dict]:
    """Compute suggested CALIBRATED_C constant and supporting stats."""

    if current_c <= 0:
        raise ValueError("--current-c must be positive.")

    # Least-squares fit: minimize ||force_i - C * v_gt_i^2||
    numerator = 0.0
    denominator = 0.0
    ratios: List[float] = []
    force_vals: List[float] = []
    old_residuals: List[float] = []
    new_residuals: List[float] = []

    for student_v, gt_v in samples:
        v_gt_sq = gt_v * gt_v
        v_student_sq = student_v * student_v
        force = current_c * v_student_sq
        numerator += force * v_gt_sq
        denominator += v_gt_sq * v_gt_sq
        force_vals.append(force)
        ratios.append(student_v / gt_v)
        old_residuals.append(student_v - gt_v)

    if denominator <= 0:
        raise RuntimeError("Unable to compute calibration constant (denominator is zero).")

    suggested_c = numerator / denominator

    if suggested_c <= 0:
        raise RuntimeError("Computed calibration constant is non-positive; check input data.")

    # Recompute residuals using the suggested constant
    for (student_v, gt_v), force in zip(samples, force_vals):
        calibrated_v = math.sqrt(force / suggested_c)
        new_residuals.append(calibrated_v - gt_v)

    def rms(values: Sequence[float]) -> float:
        return math.sqrt(math.fsum(v * v for v in values) / len(values)) if values else float("nan")

    def percentile_abs(values: Sequence[float], pct: float) -> float:
        if not values:
            return float("nan")
        ordered = sorted(abs(v) for v in values)
        idx = min(len(ordered) - 1, max(0, int(round(pct * (len(ordered) - 1)))))
        return ordered[idx]

    scale_factor = math.sqrt(current_c / suggested_c)

    stats = {
        "samples_used": len(samples),
        "current_c": current_c,
        "suggested_c": suggested_c,
        "velocity_scale_factor": scale_factor,
        "median_student_over_gt": statistics.median(ratios),
        "mean_student_over_gt": statistics.mean(ratios),
        "rms_error_before": rms(old_residuals),
        "rms_error_after": rms(new_residuals),
        "mean_error_before": statistics.mean(old_residuals),
        "mean_error_after": statistics.mean(new_residuals),
        "p95_abs_error_before": percentile_abs(old_residuals, 0.95),
        "p95_abs_error_after": percentile_abs(new_residuals, 0.95),
    }

    return suggested_c, stats


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Calibrate CALIBRATED_C using stored aligned logs."
    )
    parser.add_argument(
        "--log",
        dest="logs",
        action="append",
        required=True,
        help="Path to an aligned CSV file produced by Testing_mac.py. Repeatable.",
    )
    parser.add_argument("--current-c", type=float, required=True, help="Current CALIBRATED_C value.")
    parser.add_argument(
        "--student-column",
        default="student_mps",
        help="CSV column containing the student velocity (default: student_mps)",
    )
    parser.add_argument(
        "--gt-column",
        default="ground_truth_mps",
        help="CSV column containing the ground-truth velocity (default: ground_truth_mps)",
    )
    parser.add_argument(
        "--min-speed",
        type=float,
        default=0.5,
        help="Minimum |ground truth| speed (m/s) to include in the fit (default: 0.5)",
    )
    parser.add_argument(
        "--max-speed",
        type=float,
        default=None,
        help="Optional maximum |ground truth| speed (m/s) to include.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="Optional JSON/YAML path to write the summary statistics.",
    )
    parser.add_argument(
        "--snippet",
        action="store_true",
        help="Print a ready-to-paste #define line for the suggested constant.",
    )
    return parser.parse_args(argv)


def write_summary(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.suffix.lower() in {".yml", ".yaml"} and yaml is not None:
        path.write_text(yaml.safe_dump(payload, sort_keys=False))
    else:
        path.write_text(json.dumps(payload, indent=2))


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)

    log_paths = [Path(p).expanduser().resolve() for p in args.logs]
    samples = load_pairs(log_paths, args.student_column, args.gt_column)
    filtered = filter_pairs(samples, args.min_speed, args.max_speed)

    suggested_c, stats = compute_new_constant(filtered, args.current_c)

    print("\n=== Offline Calibration Summary ===")
    print(f"Log files         : {', '.join(p.name for p in log_paths)}")
    print(f"Samples used      : {stats['samples_used']}")
    print(f"Current C         : {stats['current_c']:.9f}")
    print(f"Suggested C       : {stats['suggested_c']:.9f}")
    print(f"Velocity scale    : ×{stats['velocity_scale_factor']:.6f} (multiply student data)")
    print(f"Median student/GT : {stats['median_student_over_gt']:.6f}")
    print(f"Mean student/GT   : {stats['mean_student_over_gt']:.6f}")
    print(f"RMS error before  : {stats['rms_error_before']:.6f} m/s")
    print(f"RMS error after   : {stats['rms_error_after']:.6f} m/s")
    print(f"Mean error before : {stats['mean_error_before']:.6f} m/s")
    print(f"Mean error after  : {stats['mean_error_after']:.6f} m/s")
    print(f"95th pct |err| before: {stats['p95_abs_error_before']:.6f} m/s")
    print(f"95th pct |err| after : {stats['p95_abs_error_after']:.6f} m/s")

    if args.snippet:
        print("\nPaste into firmware:")
        print(f"#define CALIBRATED_C_OVERRIDE  {suggested_c:.9f}f")

    if args.output:
        write_summary(args.output, stats)
        print(f"[i] Wrote summary to {args.output}")

    return 0


if __name__ == "__main__":  # pragma: no cover - CLI entry point
    import sys

    try:
        raise SystemExit(main(sys.argv[1:]))
    except Exception as exc:
        print(f"[!] Offline calibration failed: {exc}")
        raise SystemExit(1)
