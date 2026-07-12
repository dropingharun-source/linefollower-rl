#!/usr/bin/env bash
# Phase 1 step 4 done-check: three seeds give three visibly different random
# tracks, and each one loads in Gazebo with the robot spawned on its line —
# verified the same way as step 3 (dark line pixels in the camera's center
# band; a shadow can't fake that). Restores the oval benchmark at the end.
# Previews land in docs/track_previews/ (they double as evidence material).
set -e
PKG=$(cd "$(dirname "$0")/.." && pwd)
export IGN_GAZEBO_RESOURCE_PATH="$PKG/models"
export LIBGL_ALWAYS_SOFTWARE=1   # mandatory on this laptop, GUI and headless
source /opt/ros/humble/setup.bash

SEEDS="1 2 3"
mkdir -p "$PKG/docs/track_previews"

cleanup() {
  pkill -f '[i]gn gazebo' 2>/dev/null || true
  pkill -f '[p]arameter_bridge' 2>/dev/null || true
}
trap cleanup EXIT

for SEED in $SEEDS; do
  echo "=== seed $SEED: generate ==="
  python3 "$PKG/scripts/generate_track.py" --seed "$SEED" \
    --preview "$PKG/docs/track_previews/seed_$SEED.png"
  cp "$PKG/models/track_line/model.sdf" "/tmp/track_seed_$SEED.sdf"

  echo "=== seed $SEED: load in Gazebo, grab a camera frame ==="
  cleanup           # no ghost servers eating the launch (2026-07-11 trap)
  sleep 1
  ign gazebo -s -r -v3 "$PKG/worlds/track_random.sdf" \
    > "/tmp/gz_server_seed_$SEED.log" 2>&1 &
  ros2 run ros_gz_bridge parameter_bridge \
    /camera@sensor_msgs/msg/Image@ignition.msgs.Image \
    > /tmp/gz_bridge.log 2>&1 &
  sleep 10
  python3 "$PKG/scripts/save_camera_frame.py" "/tmp/track_random_$SEED.ppm"
  cleanup

  echo "--- seed $SEED: line must be in the camera's center band ---"
  python3 - "/tmp/track_random_$SEED.ppm" <<'EOF'
import sys
with open(sys.argv[1], 'rb') as f:
    assert f.readline().strip() == b'P6'
    w, h = map(int, f.readline().split())
    f.readline()
    data = f.read()

def dark(r, c):
    i = (r * w + c) * 3
    return (data[i] + data[i + 1] + data[i + 2]) / 3 < 80

rows_with_line = sum(
    1 for r in range(20, 60)
    if any(dark(r, c) for c in range(16, 48)))
print(f'rows 20-59 with dark pixels in center band (cols 16-47): '
      f'{rows_with_line}/40')
if rows_with_line < 25:
    print('FAIL: track line not visible in the camera frame')
    sys.exit(1)
print('OK: line visible')
EOF
done

echo "=== the three tracks must differ from each other ==="
python3 - /tmp/track_seed_1.sdf /tmp/track_seed_2.sdf /tmp/track_seed_3.sdf <<'EOF'
import itertools, math, re, sys

def pts(path):
    out = []
    for m in re.finditer(r'<pose>([-\d.e ]+)</pose>', open(path).read()):
        x, y = map(float, m.group(1).split()[:2])
        out.append((x, y))
    return out

tracks = {p: pts(p) for p in sys.argv[1:]}
for a, b in itertools.combinations(tracks, 2):
    d = (sum(min(math.dist(p, q) for q in tracks[b]) for p in tracks[a])
         / len(tracks[a]))
    print(f'{a.split("_")[-1]} vs {b.split("_")[-1]}: '
          f'mean nearest-segment distance {d:.3f} m')
    if d < 0.10:
        print('FAIL: two tracks are (nearly) the same shape')
        sys.exit(1)
print('OK: all three tracks differ')
EOF

echo "=== restoring the oval benchmark ==="
python3 "$PKG/scripts/generate_track.py"
echo "ALL CHECKS PASSED"
