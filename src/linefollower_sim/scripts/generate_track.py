#!/usr/bin/env python3
"""Generate the track line as GEOMETRY: models/track_line/model.sdf gets one
short black box segment per centerline sample, laid flat on the floor.

Why geometry and not a floor texture: under this laptop's mandatory software
rendering (LIBGL_ALWAYS_SOFTWARE=1), ogre2 blurs floor textures viewed at the
camera's shallow angle so hard that a 5 cm line vanishes entirely (tested
2026-07-12 at 1024px and 256px; albedo_map also only loads from absolute
paths). Plain colored geometry — like the old empty_test strip — renders
crisply, so the line is boxes.

Three modes:
  python3 generate_track.py            fixed oval (the Phase 2 PID benchmark)
  python3 generate_track.py --seed 7   random closed loop; also writes
                                       worlds/track_random.sdf with the robot
                                       spawned on the new line. Same seed ->
                                       same track, always.
  python3 generate_track.py --pool 16  worlds/track_pool.sdf: 16 random tracks
                                       on a grid of floor panels in ONE world.
                                       Training teleports the robot to a random
                                       track each episode — no world reloads.
                                       Does NOT touch models/track_line (the
                                       oval benchmark stays loaded/intact).
Add --preview out.png to the oval/seed modes for a top-down centerline image.

Random loops are star-shaped: the distance from the center varies smoothly
with the angle (a few random cosine waves), so the loop can never cross
itself. A candidate is redrawn (deterministically, from the seed) until it
fits the floor panel, keeps every curve radius >= MIN_TURN_RADIUS_M (the
oval's tightest curve — known drivable), and keeps far-apart parts of the
loop from squeezing close together (the camera must never see two lines).
"""
import argparse
import math
import random
import re
from pathlib import Path

LINE_WIDTH_M = 0.05   # same width as the empty_test strip was
LINE_THICK_M = 0.002
SEGMENT_Z = 0.003     # floor panel top is at 0.002; sit 1 mm above it
N_POINTS = 128

PANEL_HALF_M = 2.0        # track_floor is 4 x 4 m
PANEL_MARGIN_M = 0.30     # keep the line this far inside the panel edge
MIN_TURN_RADIUS_M = 0.5
MIN_CLEARANCE_M = 0.55    # between parts of the loop that aren't neighbors
CLEARANCE_SKIP = 16       # "neighbors" = within this many samples along loop

# amplitude cap per cosine wave, as a fraction of the base radius:
# higher-frequency waves bend the curve harder (curvature grows with k^2),
# so they get smaller caps
WAVES = ((2, 0.35), (3, 0.15), (4, 0.08))

POOL_PITCH_M = 6.0     # grid spacing between pool panel centers (4 m panel
                       # + 2 m gap; off-line ends an episode at 0.2 m, so the
                       # robot can never get near a neighboring track)
POOL_SEED_BASE = 100000  # pool track seeds start here — "unseen track" evals
                         # use small seeds (1, 2, 3...) and can never collide


def oval_points():
    """Fixed oval, radii 1.2 x 0.8 m — the PID benchmark track."""
    return [
        (1.2 * math.cos(2 * math.pi * i / N_POINTS),
         0.8 * math.sin(2 * math.pi * i / N_POINTS))
        for i in range(N_POINTS)
    ]


def _star_points(rng):
    base = rng.uniform(0.9, 1.3)
    waves = [(k, rng.uniform(0.0, cap * base), rng.uniform(0, 2 * math.pi))
             for k, cap in WAVES]
    pts = []
    for i in range(N_POINTS):
        t = 2 * math.pi * i / N_POINTS
        r = base + sum(a * math.cos(k * t + p) for k, a, p in waves)
        pts.append((r * math.cos(t), r * math.sin(t)))
    return pts


def _circumradius(a, b, c):
    """Radius of the circle through three points = local turn radius."""
    ab, bc, ca = math.dist(a, b), math.dist(b, c), math.dist(c, a)
    area2 = abs((b[0] - a[0]) * (c[1] - a[1])
                - (b[1] - a[1]) * (c[0] - a[0]))
    if area2 < 1e-12:
        return math.inf
    return ab * bc * ca / (2 * area2)


def _min_turn_radius(pts):
    n = len(pts)
    return min(_circumradius(pts[i - 1], pts[i], pts[(i + 1) % n])
               for i in range(n))


def _constraint_failure(pts):
    """None if the candidate track is usable, else the reason it isn't."""
    n = len(pts)
    if max(max(abs(x), abs(y)) for x, y in pts) > PANEL_HALF_M - PANEL_MARGIN_M:
        return 'outside panel'
    if _min_turn_radius(pts) < MIN_TURN_RADIUS_M:
        return 'turn too tight'
    for i in range(n):
        for j in range(i + 1, n):
            if min(j - i, n - (j - i)) <= CLEARANCE_SKIP:
                continue
            if math.dist(pts[i], pts[j]) < MIN_CLEARANCE_M:
                return 'loop squeezes itself'
    return None


def random_points(seed):
    for attempt in range(500):
        rng = random.Random(f'{seed}:{attempt}')
        pts = _star_points(rng)
        if _constraint_failure(pts) is None:
            print(f'seed {seed}: accepted candidate {attempt + 1}')
            return pts
    raise SystemExit(f'seed {seed}: no valid track in 500 candidates')


def spawn_pose(pts):
    """On the first centerline point, facing along the loop."""
    (x0, y0), (x1, y1) = pts[0], pts[1]
    return x0, y0, math.atan2(y1 - y0, x1 - x0)


def pool_track_seed(pool_seed, i):
    return POOL_SEED_BASE + pool_seed * 1000 + i


def pool_centers(n):
    """Panel centers for n pool tracks: a near-square grid around the origin."""
    cols = math.ceil(math.sqrt(n))
    rows = math.ceil(n / cols)
    return [(((i % cols) - (cols - 1) / 2) * POOL_PITCH_M,
             ((i // cols) - (rows - 1) / 2) * POOL_PITCH_M)
            for i in range(n)]


def pool_tracks(pool_seed, n):
    """The n pool centerlines, each already shifted to its panel center.
    line_env.py calls this too — the world and the reward math must agree."""
    centers = pool_centers(n)
    return [[(x + cx, y + cy)
             for x, y in random_points(pool_track_seed(pool_seed, i))]
            for i, (cx, cy) in enumerate(centers)]


def segments(points):
    """One box per consecutive point pair, 30% overlap to hide seams."""
    for i, (x0, y0) in enumerate(points):
        x1, y1 = points[(i + 1) % len(points)]
        length = math.hypot(x1 - x0, y1 - y0) * 1.3
        yaw = math.atan2(y1 - y0, x1 - x0)
        yield (x0 + x1) / 2, (y0 + y1) / 2, length, yaw


def write_model(pkg_dir, points):
    visuals = []
    for i, (cx, cy, length, yaw) in enumerate(segments(points)):
        visuals.append(f'''      <visual name="seg_{i}">
        <pose>{cx:.4f} {cy:.4f} {SEGMENT_Z} 0 0 {yaw:.4f}</pose>
        <geometry>
          <box><size>{length:.4f} {LINE_WIDTH_M} {LINE_THICK_M}</size></box>
        </geometry>
        <material>
          <ambient>0 0 0 1</ambient>
          <diffuse>0 0 0 1</diffuse>
        </material>
      </visual>''')

    sdf = '''<?xml version="1.0"?>
<!-- GENERATED by scripts/generate_track.py — edit that script, not this file. -->
<sdf version="1.8">
  <model name="track_line">
    <static>true</static>
    <link name="link">
{visuals}
    </link>
  </model>
</sdf>
'''.format(visuals='\n'.join(visuals))

    out = pkg_dir / 'models' / 'track_line' / 'model.sdf'
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(sdf)
    print(f'wrote {out}: {len(visuals)} segments, line {LINE_WIDTH_M} m wide')


def write_random_world(pkg_dir, pts):
    """track_random.sdf = the oval world with the robot moved onto the
    generated line (both worlds include model://track_line, which this
    script just overwrote)."""
    x, y, yaw = spawn_pose(pts)
    world = (pkg_dir / 'worlds' / 'track_oval.sdf').read_text()
    world = world.replace('?>\n',
                          '?>\n<!-- GENERATED by scripts/generate_track.py '
                          '--seed N — edit track_oval.sdf, not this file. -->\n',
                          1)
    world = world.replace('<world name="track_oval">',
                          '<world name="track_random">')
    world = re.sub(
        r'(<uri>model://linefollower_bot</uri>\s*<pose>)[^<]*(</pose>)',
        lambda m: f'{m.group(1)}{x:.4f} {y:.4f} 0 0 0 {yaw:.4f}{m.group(2)}',
        world)
    out = pkg_dir / 'worlds' / 'track_random.sdf'
    out.write_text(world)
    print(f'wrote {out} (robot spawns at {x:.2f}, {y:.2f})')


def write_pool_world(pkg_dir, pool_seed, n):
    """worlds/track_pool.sdf = the oval world with n floor panels on a grid,
    every pool track's line inlined as one static model, and the robot
    spawned on track 0. The world KEEPS the name "track_oval" so the env's
    set_pose service path never changes (same convention as track_random)."""
    tracks = pool_tracks(pool_seed, n)
    centers = pool_centers(n)

    world = (pkg_dir / 'worlds' / 'track_oval.sdf').read_text()
    world = world.replace('?>\n',
                          '?>\n<!-- GENERATED by scripts/generate_track.py '
                          'in pool mode; edit track_oval.sdf, not this file. '
                          '(No double hyphen in this comment: XML forbids it.) '
                          '-->\n',
                          1)

    # ground plane large enough for the whole grid
    span = math.ceil(math.sqrt(n)) * POOL_PITCH_M + 8
    assert world.count('<size>20 20</size>') == 2, 'ground plane template drifted'
    world = world.replace('<size>20 20</size>', f'<size>{span} {span}</size>')

    # one floor panel per track (visual-only model, so n panels cost nothing)
    floor_old = '''    <include>
      <uri>model://track_floor</uri>
      <pose>0 0 0.001 0 0 0</pose>
    </include>'''
    floors = '\n'.join(f'''    <include>
      <name>track_floor_{i}</name>
      <uri>model://track_floor</uri>
      <pose>{cx:.1f} {cy:.1f} 0.001 0 0 0</pose>
    </include>''' for i, (cx, cy) in enumerate(centers))
    assert floor_old in world, 'floor include template drifted'
    world = world.replace(floor_old, floors)

    # all pool lines inlined as one static model (points are pre-shifted,
    # so segment poses are already in world coordinates)
    visuals = []
    for t, pts in enumerate(tracks):
        for i, (cx, cy, length, yaw) in enumerate(segments(pts)):
            visuals.append(
                f'      <visual name="t{t}_seg_{i}">\n'
                f'        <pose>{cx:.4f} {cy:.4f} {SEGMENT_Z} 0 0 {yaw:.4f}</pose>\n'
                f'        <geometry>\n'
                f'          <box><size>{length:.4f} {LINE_WIDTH_M} {LINE_THICK_M}</size></box>\n'
                f'        </geometry>\n'
                f'        <material>\n'
                f'          <ambient>0 0 0 1</ambient>\n'
                f'          <diffuse>0 0 0 1</diffuse>\n'
                f'        </material>\n'
                f'      </visual>')
    line_old = '''    <include>
      <uri>model://track_line</uri>
      <pose>0 0 0 0 0 0</pose>
    </include>'''
    lines_model = ('    <model name="track_pool_lines">\n'
                   '      <static>true</static>\n'
                   '      <link name="link">\n'
                   + '\n'.join(visuals) + '\n'
                   '      </link>\n'
                   '    </model>')
    assert line_old in world, 'track_line include template drifted'
    world = world.replace(line_old, lines_model)

    # robot starts on track 0 (reset() teleports anyway; this just avoids
    # spawning in the empty gap between panels)
    x, y, yaw = spawn_pose(tracks[0])
    world = re.sub(
        r'(<uri>model://linefollower_bot</uri>\s*<pose>)[^<]*(</pose>)',
        lambda m: f'{m.group(1)}{x:.4f} {y:.4f} 0 0 0 {yaw:.4f}{m.group(2)}',
        world)

    out = pkg_dir / 'worlds' / 'track_pool.sdf'
    out.write_text(world)
    print(f'wrote {out}: {n} tracks, {len(visuals)} segments, '
          f'ground plane {span} x {span} m')


def write_preview(pts, path):
    """Top-down PNG of the centerline on the panel, stdlib only.
    Red dot = robot spawn point."""
    import struct
    import zlib
    size = 400
    px = bytearray(b'\xff' * (size * size * 3))

    def put(x, y, rgb):
        c = int((x + PANEL_HALF_M) / (2 * PANEL_HALF_M) * (size - 1))
        r = int((PANEL_HALF_M - y) / (2 * PANEL_HALF_M) * (size - 1))
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                rr, cc = r + dr, c + dc
                if 0 <= rr < size and 0 <= cc < size:
                    i = (rr * size + cc) * 3
                    px[i:i + 3] = bytes(rgb)

    n = len(pts)
    for i in range(n):
        x0, y0 = pts[i]
        x1, y1 = pts[(i + 1) % n]
        for s in range(25):
            f = s / 25
            put(x0 + f * (x1 - x0), y0 + f * (y1 - y0), (0, 0, 0))
    sx, sy, _ = spawn_pose(pts)
    put(sx, sy, (220, 0, 0))

    def chunk(tag, data):
        return (struct.pack('>I', len(data)) + tag + data
                + struct.pack('>I', zlib.crc32(tag + data)))

    raw = b''.join(b'\x00' + bytes(px[y * size * 3:(y + 1) * size * 3])
                   for y in range(size))
    path = Path(path)
    path.write_bytes(
        b'\x89PNG\r\n\x1a\n'
        + chunk(b'IHDR', struct.pack('>IIBBBBB', size, size, 8, 2, 0, 0, 0))
        + chunk(b'IDAT', zlib.compress(raw))
        + chunk(b'IEND', b''))
    print(f'wrote {path}')


def main():
    ap = argparse.ArgumentParser(
        description='Generate the track line model (oval or random).')
    ap.add_argument('--seed', type=int,
                    help='random track from this seed; omit to restore '
                         'the fixed oval benchmark')
    ap.add_argument('--pool', type=int, metavar='N',
                    help='write worlds/track_pool.sdf with N random tracks '
                         'on a panel grid (for random-track-per-episode '
                         'training); does not touch models/track_line')
    ap.add_argument('--pool-seed', type=int, default=0,
                    help='which pool of N tracks (default 0)')
    ap.add_argument('--preview', type=Path,
                    help='also write a top-down PNG of the centerline here')
    args = ap.parse_args()

    pkg_dir = Path(__file__).resolve().parent.parent

    if args.pool:
        if args.seed is not None:
            ap.error('--pool and --seed are separate modes')
        write_pool_world(pkg_dir, args.pool_seed, args.pool)
        return

    points = oval_points() if args.seed is None else random_points(args.seed)

    ext = max(max(abs(x), abs(y)) for x, y in points)
    print(f'min turn radius {_min_turn_radius(points):.2f} m, '
          f'extent +/-{ext:.2f} m')

    write_model(pkg_dir, points)
    if args.seed is not None:
        write_random_world(pkg_dir, points)
        print('NOTE: models/track_line now holds this random track; '
              'run with no --seed to restore the oval.')
    if args.preview:
        write_preview(points, args.preview)


if __name__ == '__main__':
    main()
