#!/usr/bin/env python3
"""Static calibration of an ICM-20948 (accel +/-4 g, gyro +/-2000 deg/s, as in the SLAM driver).

  calibrate_icm20948.py gyro  [--seconds 30]
  calibrate_icm20948.py accel [--seconds 3]    (auto-detects the 6 orientations)
  calibrate_icm20948.py show

Results are merged into icm20948_calibration.yaml (OpenCV YAML) with keys
IMU.GyroBias [rad/s], IMU.AccelBias [m/s^2], IMU.AccelMatrix (row-major 3x3),
applied by the driver as accel = M * (raw - bias), gyro = raw - bias.
Stop SLAM first: the I2C device must be free. Keep the board still.
"""
import argparse
import fcntl
import math
import os
import re
import sys
import time

G = 9.80665
ACCEL_SCALE = G / 8192.0  # +/-4 g, as in the SLAM driver
GYRO_SCALE = math.pi / 180.0 / 16.4  # +/-2000 deg/s, as in the SLAM driver
I2C_SLAVE = 0x0703
DEFAULT_OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "icm20948_calibration.yaml")


class Imu:
    def __init__(self, device, address=0x68):
        self.fd = os.open(device, os.O_RDWR)
        fcntl.ioctl(self.fd, I2C_SLAVE, address)
        self.write(0x7F, 0x00)
        self.write(0x06, 0x01)
        self.write(0x07, 0x00)
        time.sleep(0.1)
        self.write(0x7F, 0x20)  # bank 2
        self.write(0x14, 0x03)  # ACCEL_CONFIG: +/-4 g, DLPF on
        self.write(0x01, 0x07)  # GYRO_CONFIG_1: +/-2000 deg/s, DLPF on
        if (self.read(0x14, 1)[0] >> 1) & 3 != 1:
            raise SystemExit("Failed to set accel range to +/-4 g")
        if (self.read(0x01, 1)[0] >> 1) & 3 != 3:
            raise SystemExit("Failed to set gyro range to +/-2000 deg/s")
        self.write(0x7F, 0x00)
        who = self.read(0x00, 1)[0]
        if who != 0xEA:
            raise SystemExit("Unexpected WHO_AM_I 0x%02x" % who)

    def write(self, reg, value):
        os.write(self.fd, bytes([reg, value]))

    def read(self, reg, n):
        os.write(self.fd, bytes([reg]))
        return os.read(self.fd, n)

    def sample(self):
        d = self.read(0x2D, 12)
        v = [int.from_bytes(d[i:i + 2], "big", signed=True) for i in range(0, 12, 2)]
        return ([x * ACCEL_SCALE for x in v[:3]], [x * GYRO_SCALE for x in v[3:]])

    def average(self, seconds):
        acc, gyr, n = [0.0] * 3, [0.0] * 3, 0
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            a, g = self.sample()
            for i in range(3):
                acc[i] += a[i]
                gyr[i] += g[i]
            n += 1
            time.sleep(0.005)
        if n == 0:
            raise SystemExit("No samples read")
        return [x / n for x in acc], [x / n for x in gyr]


def solve(a, b):
    """Gaussian elimination with partial pivoting."""
    n = len(b)
    m = [row[:] + [b[i]] for i, row in enumerate(a)]
    for c in range(n):
        p = max(range(c, n), key=lambda r: abs(m[r][c]))
        if abs(m[p][c]) < 1e-12:
            raise ValueError("Singular system: orientations are not distinct")
        m[c], m[p] = m[p], m[c]
        for r in range(c + 1, n):
            f = m[r][c] / m[c][c]
            for k in range(c, n + 1):
                m[r][k] -= f * m[c][k]
    x = [0.0] * n
    for r in range(n - 1, -1, -1):
        x[r] = (m[r][n] - sum(m[r][k] * x[k] for k in range(r + 1, n))) / m[r][r]
    return x


def invert3(m):
    cols = [solve([m[i * 3:i * 3 + 3] for i in range(3)], [1.0 if i == j else 0.0 for i in range(3)])
            for j in range(3)]
    return [cols[j][i] for i in range(3) for j in range(3)]


def fit_accel(readings, truths):
    """Least squares affine model truth = A * raw + c; returns (bias, matrix)
    such that truth = matrix * (raw - bias)."""
    rows = [r + [1.0] for r in readings]
    ata = [[sum(r[i] * r[j] for r in rows) for j in range(4)] for i in range(4)]
    params = []
    for axis in range(3):
        atb = [sum(rows[k][i] * truths[k][axis] for k in range(len(rows))) for i in range(4)]
        params.append(solve(ata, atb))
    a = [params[i][j] for i in range(3) for j in range(3)]
    c = [params[i][3] for i in range(3)]
    inv = invert3(a)
    bias = [-sum(inv[i * 3 + j] * c[j] for j in range(3)) for i in range(3)]
    return bias, a


def load_yaml(path):
    data = {}
    if os.path.exists(path):
        text = open(path).read()
        for key in ("IMU.GyroBias", "IMU.AccelBias", "IMU.AccelMatrix"):
            m = re.search(re.escape(key) + r":\s*\[([^\]]*)\]", text)
            if m:
                data[key] = [float(x) for x in m.group(1).replace("\n", " ").split(",")]
    return data


def save_yaml(path, data):
    lines = ["%YAML:1.0", "---"]
    for key in ("IMU.GyroBias", "IMU.AccelBias", "IMU.AccelMatrix"):
        if key in data:
            lines.append("%s: [%s]" % (key, ", ".join("%.9g" % x for x in data[key])))
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        f.write("\n".join(lines) + "\n")
    os.replace(tmp, path)


ORIENTATIONS = [
    ("+X up (X axis pointing up)", (G, 0, 0)),
    ("-X up (X axis pointing down)", (-G, 0, 0)),
    ("+Y up", (0, G, 0)),
    ("-Y up", (0, -G, 0)),
    ("+Z up (board flat, chip facing up)", (0, 0, G)),
    ("-Z up (board upside down)", (0, 0, -G)),
]


def detect_orientation(acc):
    """Index into ORIENTATIONS if one axis reads 1 g +/-30% and the others are
    small (board roughly axis-aligned), otherwise None."""
    axis = max(range(3), key=lambda i: abs(acc[i]))
    if not 0.7 * G <= abs(acc[axis]) <= 1.3 * G:
        return None
    if any(abs(acc[i]) > 0.3 * G for i in range(3) if i != axis):
        return None
    return axis * 2 + (0 if acc[axis] > 0 else 1)


def collect_orientations(imu, hold_seconds, max_std=0.08):
    """Auto-detect orientations; returns when all 6 are captured."""
    captured = {}
    current, window, start = None, [], 0.0
    print("Rotate the board through all 6 orientations (each axis up and down).")
    print("Hold each still for %.1f s. Ctrl+C aborts without saving." % hold_seconds)
    while len(captured) < len(ORIENTATIONS):
        acc, _ = imu.sample()
        idx = detect_orientation(acc)
        if idx != current:
            current, window, start = idx, [], time.monotonic()
            if idx is None:
                print("No axis detected (moving or tilted)")
            elif idx in captured:
                print("%s: already captured, rotate to another orientation"
                      % ORIENTATIONS[idx][0])
            else:
                print("Detected: %s - hold still..." % ORIENTATIONS[idx][0])
        if idx is not None and idx not in captured:
            window.append(acc)
            if time.monotonic() - start >= hold_seconds:
                mean = [sum(w[i] for w in window) / len(window) for i in range(3)]
                std = max(math.sqrt(sum((w[i] - mean[i]) ** 2 for w in window) / len(window))
                          for i in range(3))
                if std > max_std:
                    print("  Too much motion (std %.3f m/s^2), restarting hold" % std)
                    window, start = [], time.monotonic()
                else:
                    captured[idx] = mean
                    print("  Captured %s: raw [%s] (%d/6)" % (
                        ORIENTATIONS[idx][0], ", ".join("%.4f" % x for x in mean),
                        len(captured)))
                    if len(captured) < len(ORIENTATIONS):
                        left = [ORIENTATIONS[i][0] for i in range(6) if i not in captured]
                        print("  Remaining: " + "; ".join(left))
        time.sleep(0.005)
    order = sorted(captured)
    return [captured[i] for i in order], [list(ORIENTATIONS[i][1]) for i in order]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("mode", choices=["gyro", "accel", "show"])
    ap.add_argument("--device", default="/dev/i2c-1")
    ap.add_argument("--seconds", type=float)
    ap.add_argument("--output", default=DEFAULT_OUT)
    args = ap.parse_args()

    data = load_yaml(args.output)
    if args.mode == "show":
        print(data or "no calibration at " + args.output)
        return

    imu = Imu(args.device)
    if args.mode == "gyro":
        seconds = args.seconds or 30.0
        print("Keep the board completely still for %.0f s..." % seconds)
        _, gyro = imu.average(seconds)
        data["IMU.GyroBias"] = gyro
        print("Gyro bias [rad/s]: %s" % ", ".join("%.6f" % x for x in gyro))
    else:
        seconds = args.seconds or 3.0
        readings, truths = collect_orientations(imu, seconds)
        bias, matrix = fit_accel(readings, truths)
        data["IMU.AccelBias"] = bias
        data["IMU.AccelMatrix"] = matrix
        worst = 0.0
        for r, t in zip(readings, truths):
            c = [sum(matrix[i * 3 + j] * (r[j] - bias[j]) for j in range(3)) for i in range(3)]
            worst = max(worst, max(abs(c[i] - t[i]) for i in range(3)))
        print("Accel bias [m/s^2]: %s" % ", ".join("%.5f" % x for x in bias))
        print("Accel matrix: %s" % ", ".join("%.5f" % x for x in matrix))
        print("Worst residual after calibration: %.4f m/s^2" % worst)
    save_yaml(args.output, data)
    print("Saved " + args.output)


if __name__ == "__main__":
    sys.exit(main())
