"""Forgetting stress: a small table (2^gbits groups) and a sensor that wanders away, so groups fill up and far-away
voxels are forgotten (keep in voxels). Usage: gen_nn_forget.py [n_ops gbits keep]; run tb_nn with -P tb_nn.CFG=<printed>."""
import os, random, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "model"))
import nn_model as N
n_ops = int(sys.argv[1]) if len(sys.argv) > 1 else 8000
gbits = int(sys.argv[2]) if len(sys.argv) > 2 else 4
keep = int(sys.argv[3]) if len(sys.argv) > 3 else 6
random.seed(11); M = N.Map(gbits, keep); ops = []; exp = []
c = [3 << 16, -2 << 16, 1 << 16]
for i in range(n_ops):
    if i % 40 == 0:                                   # the "sensor" moves up to 0.5 m per step, mostly along +x / -y
        c[0] += random.randint(-1 << 13, 1 << 15); c[1] += random.randint(-1 << 15, 1 << 13); c[2] += random.randint(-1 << 12, 1 << 12)
    p = tuple(random.randint(-3 << 16, 3 << 16) + c[k] for k in range(3))      # 6 m cube around the sensor
    r = random.random()
    if r < 0.45:   ops.append((0, 0) + p); M.add(p, 0)
    elif r < 0.6:  ops.append((0, 1) + p); M.add(p, 1)
    else:          ops.append((1, 0) + p); exp.append(M.search(p))
with open("nn_ops.mem", "w") as f:
    for o in ops: f.write(" ".join(f"{v & 0xFFFFFFFF:08x}" for v in o) + "\n")
with open("nn_expect.txt", "w") as f:
    for r in exp: f.write(f"{len(r)} " + " ".join(f"{c & 0xFFFFFFFF:08x}" for _, p in r for c in p) + "\n")
print(f"{len(ops)} ops: {len(exp)} searches, voxels {len(M.tag)}, forgotten {M.evicted}, dropped {M.dropped}; CFG={(gbits & 15) << 12 | keep}")
