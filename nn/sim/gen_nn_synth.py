"""Synthetic stress ops: points in a small cube so that slots fill up (exercises replace-farthest and downsample)."""
import os, random, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "model"))
import nn_model as N
random.seed(5); M = N.Map(); ops = []; exp = []; nfull = nds = 0
for i in range(int(sys.argv[1]) if len(sys.argv) > 1 else 6000):
    p = tuple(random.randint(-2 << 16, 2 << 16) + c for c in (7 << 16, -3 << 16, 1 << 15))   # 4 m cube
    r = random.random()
    if r < 0.5:
        v = (p[0] >> 15, p[1] >> 15, p[2] >> 15); s = M.find(v, False)
        if s is not None and len(M.pts[s]) == N.CAP: nfull += 1
        ops.append((0, 0) + p); M.add(p, 0)
    elif r < 0.65:
        nds += 1; ops.append((0, 1) + p); M.add(p, 1)
    else:
        ops.append((1, 0) + p); exp.append(M.search(p))
with open("nn_ops.mem", "w") as f:
    for o in ops: f.write(" ".join(f"{v & 0xFFFFFFFF:08x}" for v in o) + "\n")
with open("nn_expect.txt", "w") as f:
    for r in exp: f.write(f"{len(r)} " + " ".join(f"{c & 0xFFFFFFFF:08x}" for _, p in r for c in p) + "\n")
print(f"{len(ops)} ops: {len(exp)} searches, {nds} downsample adds, {nfull} adds into a full slot, voxels {len(M.tag)}, dropped {M.dropped}")
