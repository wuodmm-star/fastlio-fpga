"""Ops from the recorded FAST-LIO dump -> nn_ops.mem (5 hex words per op: op, ds, x, y, z) and the golden results."""
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "model"))
import nn_model as N
MAX_SEARCH = int(sys.argv[1]) if len(sys.argv) > 1 else 400
EVERY = int(sys.argv[2]) if len(sys.argv) > 2 else 10
M = N.Map(); ops = []; exp = []; ns = 0; seen = 0
for rec in N.read_dump("nn.dump"):
    if rec[0] == "add":
        ops.append((0, rec[1]) + rec[2]); M.add(rec[2], rec[1])
    else:
        seen += 1
        if seen % EVERY:
            continue
        ops.append((1, 0) + rec[1]); exp.append(M.search(rec[1])); ns += 1
        if ns == MAX_SEARCH:
            break
with open("nn_ops.mem", "w") as f:
    for o in ops:
        f.write(" ".join(f"{v & 0xFFFFFFFF:08x}" for v in o) + "\n")
with open("nn_expect.txt", "w") as f:
    for r in exp:
        f.write(f"{len(r)} " + " ".join(f"{c & 0xFFFFFFFF:08x}" for _, p in r for c in p) + "\n")
print(f"{len(ops)} ops ({len(ops) - ns} adds, {ns} searches), golden dropped {M.dropped}, voxels {len(M.tag)}")
