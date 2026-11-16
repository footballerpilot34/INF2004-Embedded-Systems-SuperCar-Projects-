"""Fit an effective ultrasonic beam half-angle from measured reference scans.

python3 tests/calibrate_obstacle_width.py --examples
python3 tests/calibrate_obstacle_width.py measurements.csv
CSV columns: distance_mm,left_deg,right_deg,width_mm (absolute servo angles).
Use independently measured target widths; reserve separate scans for validation.
"""
import argparse
import csv
import math


def estimate(row, beta, centre):
    left = row["left_deg"] - centre + beta
    right = row["right_deg"] - centre - beta
    return row["distance_mm"] * (math.tan(math.radians(right)) -
                                  math.tan(math.radians(left)))


def fit(rows, centre=90):
    if not rows:
        raise ValueError("At least one measured reference scan is required")
    for row in rows:
        if not all(math.isfinite(v) for v in row.values()):
            raise ValueError("Measurements must be finite")
        if (row["distance_mm"] <= 0 or row["width_mm"] <= 0 or
                not -89 < row["left_deg"] - centre < row["right_deg"] - centre < 89):
            raise ValueError("Invalid reference dimensions or edge angles")
    # Angular erosion must not invert the measured interval. A 0.01-degree
    # search is much finer than the servo's 2-degree sampling resolution.
    maximum = min((r["right_deg"] - r["left_deg"]) / 2 for r in rows)
    candidates = (i / 100 for i in range(math.ceil(maximum * 100)))
    beta = min(candidates, key=lambda b:
               sum((estimate(r, b, centre) - r["width_mm"]) ** 2 for r in rows))
    return round(beta * 10) / 10  # Same 0.1-degree precision as config.h.


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", nargs="?")
    parser.add_argument("--centre", type=float, default=90)
    parser.add_argument("--examples", action="store_true")
    args = parser.parse_args()
    if args.examples:
        print("Approximate supplied summaries: residuals below are fit errors, not validation.")
        rows = [dict(distance_mm=300, left_deg=67, right_deg=105, width_mm=80),
                dict(distance_mm=220, left_deg=71, right_deg=105, width_mm=80)]
    elif args.csv:
        with open(args.csv, newline="") as stream:
            rows = [{key: float(row[key]) for key in
                     ("distance_mm", "left_deg", "right_deg", "width_mm")}
                    for row in csv.DictReader(stream)]
    else:
        parser.error("Supply a measurement CSV or --examples")
    beta = fit(rows, args.centre)
    print(f"OBSTACLE_WIDTH_BEAM_HALF_DEG_X10 = {round(beta * 10)} ({beta:.1f} degrees)")
    for row in rows:
        width = estimate(row, beta, args.centre)
        print(f"distance={row['distance_mm']:g} mm: estimate={width:.1f} mm; "
              f"reference={row['width_mm']:g} mm; error={width-row['width_mm']:+.1f} mm")


if __name__ == "__main__":
    main()
