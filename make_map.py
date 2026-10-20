import os

import numpy as np

RES = 0.05                 # meters per pixel
X0, Y0 = -10.0, -7.0       # world coords of the map's lower-left corner
W, H = 400, 280            # pixels -> 20 m x 14 m
FREE, OCC, UNK = 254, 0, 128

img = np.full((H, W), UNK, dtype=np.uint8)


def box(cx, cy, sx, sy, value):
    c0 = int(round((cx - sx / 2 - X0) / RES))
    c1 = int(round((cx + sx / 2 - X0) / RES))
    r0 = H - int(round((cy + sy / 2 - Y0) / RES))
    r1 = H - int(round((cy - sy / 2 - Y0) / RES))
    img[r0:r1, c0:c1] = value


# Room interior is free
box(0, 0, 18, 12, FREE)

# Outer walls (same as make_world.py)
box(0, 6.1, 18.4, 0.2, OCC)
box(0, -6.1, 18.4, 0.2, OCC)
box(9.1, 0, 0.2, 12, OCC)
box(-9.1, 0, 0.2, 12, OCC)

# Shelves: 6.0 x 0.6 m, four rows, two per row
for y in (4.2, 1.2, -1.2, -4.2):
    box(-4.5, y, 6, 0.6, OCC)
    box(4.5, y, 6, 0.6, OCC)

out_dir = os.path.expanduser('~/ugp_ws/src/ugp_warehouse/config')
os.makedirs(out_dir, exist_ok=True)

with open(os.path.join(out_dir, 'warehouse_map.pgm'), 'wb') as f:
    f.write(f'P5\n{W} {H}\n255\n'.encode())
    f.write(img.tobytes())

with open(os.path.join(out_dir, 'warehouse_map.yaml'), 'w') as f:
    f.write('image: warehouse_map.pgm\n')
    f.write('mode: trinary\n')
    f.write(f'resolution: {RES}\n')
    f.write(f'origin: [{X0}, {Y0}, 0.0]\n')
    f.write('negate: 0\n')
    f.write('occupied_thresh: 0.65\n')
    f.write('free_thresh: 0.25\n')

print('free pixels:', int((img == FREE).sum()),
      'occupied pixels:', int((img == OCC).sum()))
print('wrote map to', out_dir)
