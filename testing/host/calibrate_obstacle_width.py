"""Generate a small lookup table from PHYSICAL rectangular-obstacle scans.

CSV: nominal_distance_mm,distance_mm,left_deg,right_deg,width_mm,lst,rst
Angles are absolute servo degrees (centre=90). lst/rst must be echo-bracketed.
Collect >=3 usable scans per width at each nominal distance (150,200,250).
Keep independent scans aside to validate the generated table on the car.
No examples or synthetic calibration constants are installed by this script.
"""
import argparse
import csv
import math


def calibrate(rows, centre=90):
    groups = {distance: {150: [], 250: []} for distance in (150, 200, 250)}
    for row in rows:
        nominal, width = float(row['nominal_distance_mm']), float(row['width_mm'])
        if nominal not in groups or width not in groups[nominal]:
            raise ValueError('Nominal distances must be 150/200/250; widths 150/250 mm')
        distance, left, right = (float(row[key]) for key in
                                 ('distance_mm', 'left_deg', 'right_deg'))
        if not all(math.isfinite(v) for v in (distance, left, right)):
            raise ValueError('Measurements must be finite')
        if distance <= 0 or not centre - 30 < left < centre < right < centre + 30:
            raise ValueError('Invalid distance or unobserved/clipped scan edges')
        if row['lst'] != 'echo-bracketed' or row['rst'] != 'echo-bracketed':
            continue  # Missing/weak edges cannot train a reliable classifier.
        groups[nominal][width].append((distance, round((right - left) * 10)))
    result = []
    for nominal, sizes in groups.items():
        small, large = sizes[150], sizes[250]
        if len(small) < 3 or len(large) < 3:
            result.append((0, 0, 0, 0, 0, 0))
            continue
        smin, smax = min(p[1] for p in small), max(p[1] for p in small)
        lmin, lmax = min(p[1] for p in large), max(p[1] for p in large)
        if smax >= lmin:
            result.append((0, 0, 0, 0, 0, 0))  # Observed classes overlap: unknown.
            continue
        distances = [p[0] for p in small + large]
        # Do not extrapolate beyond actually observed distances.
        result.append((math.ceil(min(distances)), math.floor(max(distances)),
                       smin, smax, lmin, lmax))
    for i, row in enumerate(result):
        if row[0] and any(other[0] and max(row[0], other[0]) <= min(row[1], other[1])
                          for other in result[:i] + result[i+1:]):
            raise ValueError('Measured distance bands overlap; inspect test placement')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv')
    parser.add_argument('--centre', type=float, default=90)
    args = parser.parse_args()
    try:
        with open(args.csv, newline='') as stream:
            rows = calibrate(list(csv.DictReader(stream)), args.centre)
    except (ValueError, KeyError) as error:
        parser.error(str(error))
    print('OBSTACLE_WIDTH_CALIBRATION_ROWS (paste into config/config.h):')
    for nominal, row in zip((150, 200, 250), rows):
        print('{' + ', '.join(map(str, row)) + '}, /* ' + str(nominal) + ' mm */')
        if row[0] == 0:
            print('  /* UNKNOWN: fewer than 3 usable scans/class, or overlapping spans. */')
    print('These are training envelopes, not demonstrated accuracy. Validate on separate scans.')


if __name__ == '__main__':
    main()
