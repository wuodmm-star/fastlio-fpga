"""Whole-chip stimulus: ops -> UDP request frames (<= 72 ops, <= 18 searches each) + expected reply payloads."""
import os, random, struct, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "model")); sys.path.insert(0, os.path.join(HERE, "..", "..", "common"))
import nn_model as N
from frames import build
PC_MAC, PC_IP, PC_PORT = bytes.fromhex("02005e000001"), bytes([192, 168, 137, 1]), 51000
# gen_nn_sys.py [n_ops clear_in_the_middle(0/1) gbits keep] : with gbits/keep the first packet is CLEAR with that
# configuration, the points wander away (so voxels get forgotten) and the last packet asks for the statistics
MID = len(sys.argv) > 2 and sys.argv[2] != "0"
GBITS = int(sys.argv[3]) if len(sys.argv) > 3 else 16
KEEP = int(sys.argv[4]) if len(sys.argv) > 4 else 0
FORGET = len(sys.argv) > 3
CFG = ((GBITS & 15) << 12) | KEEP
random.seed(9); M = N.Map(GBITS, KEEP); ops = []; ctr = [7 << 16, -3 << 16, 1 << 15]
for i in range(int(sys.argv[1]) if len(sys.argv) > 1 else 600):
    if FORGET and i % 30 == 0: ctr[0] += random.randint(0, 1 << 15); ctr[1] -= random.randint(0, 1 << 14)
    p = tuple(random.randint(-2 << 16, 2 << 16) + c for c in ctr)
    r = random.random()
    ops.append((0, 0, p) if r < 0.45 else (0, 1, p) if r < 0.6 else (1, 0, p))
pkts, exp, cur = [], [], []
if FORGET: pkts.append(struct.pack(">I", 0xA5130000 | CFG)); exp.append(struct.pack(">I", 0xA5120000))
def flush():
    global M
    if not cur: return
    body = b"".join(struct.pack(">I", (ds << 1) | s) + struct.pack(">3i", *p) for s, ds, p in cur)
    pkts.append(struct.pack(">I", 0xA5110000 | len(cur)) + body)
    res = []
    for s, ds, p in cur:
        if s:
            r = M.search(p); w = [len(r)]
            for k in range(5):
                w += list(r[k][1]) if k < len(r) else [0, 0, 0]
            res.append(struct.pack(">16I", *[x & 0xFFFFFFFF for x in w]))
        else:
            M.add(p, ds)
    exp.append(struct.pack(">I", 0xA5120000 | len(res)) + b"".join(res)); cur.clear()
for i, o in enumerate(ops):
    if i == len(ops) // 2 and MID:                      # CLEAR in the middle: board and golden model both start over
        flush()
        pkts.append(struct.pack(">I", 0xA5130000 | CFG)); exp.append(struct.pack(">I", 0xA5120000)); M = N.Map(GBITS, KEEP)
    cur.append(o)
    if len(cur) == 72 or sum(x[0] for x in cur) == 18 or random.random() < 0.08: flush()
flush()
if FORGET: pkts.append(struct.pack(">I", 0xA5140000)); exp.append(struct.pack(">3I", 0xA5120000, M.dropped, M.evicted))
frames = [build(PC_MAC, PC_IP, PC_PORT, p, ident=k + 1) for k, p in enumerate(pkts)]
open(os.path.join(HERE, "sys_stim.mem"), "w").writelines(f"{b:02x}\n" for fr in frames for b in fr)
open(os.path.join(HERE, "sys_lens.mem"), "w").writelines(f"{len(fr):04x}\n" for fr in frames)
open(os.path.join(HERE, "sys_expect.txt"), "w").writelines(e.hex() + "\n" for e in exp)
print(f"{len(ops)} ops in {len(frames)} packets, {sum(o[0] for o in ops)} searches, forgotten {M.evicted}, dropped {M.dropped}")
