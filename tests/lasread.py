"""Read LAS/LAZ point coordinates with LAStools' las2las, which TBC installs, so the tests need no
Python packages. Set TBC_LAS2LAS to use another las2las."""
import hashlib
import os
import struct
import subprocess
import tempfile

LAS2LAS = os.environ.get("TBC_LAS2LAS",
                         r"C:\Program Files\Trimble\Trimble Business Center\Inpho_UAS\bin\las2las64.exe")


def las_header(path):
    """Public header fields of a LAS/LAZ file (LAZ compresses only the points, not the header)."""
    with open(path, "rb") as f:
        h = f.read(375)
    if h[:4] != b"LASF":
        raise ValueError(f"{path} is not a LAS/LAZ file")
    minor = h[25]
    n = struct.unpack("<I", h[107:111])[0]
    if minor >= 4 and len(h) >= 255:
        n = struct.unpack("<Q", h[247:255])[0] or n
    sx, sy, sz, ox, oy, oz, maxx, minx, maxy, miny, maxz, minz = struct.unpack("<12d", h[131:227])
    return {"version": f"1.{minor}", "point_format": h[104] & 0x3F, "points": n,
            "min": (minx, miny, minz), "max": (maxx, maxy, maxz)}


def run_las2las(args):
    if not os.path.exists(LAS2LAS):
        raise FileNotFoundError(f"las2las not found at {LAS2LAS}; set TBC_LAS2LAS")
    r = subprocess.run([LAS2LAS] + [str(a) for a in args], capture_output=True, text=True)
    if r.returncode != 0:
        raise RuntimeError(f"las2las failed ({r.returncode}): {(r.stderr or r.stdout).strip()}")


def load_xyz(path):
    """Points of a LAS/LAZ file as a list of (x, y, z), decoded once to a cached text file."""
    key = (os.path.abspath(path), os.path.getmtime(path), os.path.getsize(path))
    out = os.path.join(tempfile.gettempdir(), "lasread_%s.txt" % hashlib.md5(repr(key).encode()).hexdigest()[:12])
    if not os.path.exists(out):
        part = out[:-4] + ".part.txt"  # renamed when complete so an interrupted run is never reused
        run_las2las(["-i", path, "-otxt", "-oparse", "xyz", "-o", part])
        os.replace(part, out)
    pts = []
    with open(out) as f:
        for line in f:
            parts = line.split()
            if len(parts) >= 3:
                pts.append((float(parts[0]), float(parts[1]), float(parts[2])))
    return pts


def load_relative(path):
    """(points relative to the first point, first point): the macros work this way too, so sums
    of squares keep their precision with large grid coordinates."""
    pts = load_xyz(path)
    o = pts[0]
    return [(x - o[0], y - o[1], z - o[2]) for x, y, z in pts], o
