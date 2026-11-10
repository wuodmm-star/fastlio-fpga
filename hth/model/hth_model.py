"""Golden model of the FPGA HTH engine (see ../SPEC.md).

Three things live here:
  1. quantize()/golden_fixed(): the exact integer math the FPGA must reproduce bit for bit
  2. reference_float(): the same quantity in float64, straight from the FAST-LIO formulas (for error analysis)
  3. make_microcode(): the per-point instruction list the FPGA sequencer runs, plus a tiny simulator of that
     instruction list, so the table itself is checked against (1) before any Verilog runs.
  + packet encode/decode for the UDP protocol.
"""
import struct
import numpy as np

Q_ROT, Q_POS, Q_N, Q_PD2 = 30, 16, 30, 24
H_SCALE = [Q_N, Q_N, Q_N, Q_POS, Q_POS, Q_POS]          # scale (power of two) of H0..H5
PAIRS = [(i, j) for i in range(6) for j in range(i, 6)]  # the 21 upper-triangle entries, row-major
NSUM = 27
SUM_BITS = 80
MAGIC = 0xA5
T_BEGIN, T_POINTS, T_END = 1, 2, 3
MAX_PTS_PER_PKT = 41                                      # 4 + 41*28 = 1152 bytes <= 1172
VERSION_WORD = 0x48544831                                 # "HTH1"
I32_MIN, I32_MAX = -(1 << 31), (1 << 31) - 1


def q(x, frac):
    """float -> nearest integer in Q(frac) (vectorised), checked to fit int32."""
    v = np.rint(np.asarray(x, dtype=np.float64) * (1 << frac)).astype(np.int64)
    assert v.min(initial=0) >= I32_MIN and v.max(initial=0) <= I32_MAX, "value does not fit int32"
    return v


def quantize(R, RL, tL, pb, n, pd2):
    """floats -> the integers that are sent to the board."""
    return dict(R=q(R, Q_ROT).reshape(9), RL=q(RL, Q_ROT).reshape(9), tL=q(tL, Q_POS).reshape(3),
                pb=q(pb, Q_POS).reshape(-1, 3), n=q(n, Q_N).reshape(-1, 3), pd2=q(pd2, Q_PD2).reshape(-1))


def _i32(v):
    assert I32_MIN <= v <= I32_MAX, f"intermediate {v} does not fit int32"
    return v


def golden_point(R, RL, tL, pb, n, pd2):
    """One point, pure Python ints. Returns (H list of 6 ints, pd2 int)."""
    R = [int(x) for x in R]; RL = [int(x) for x in RL]; tL = [int(x) for x in tL]
    pb = [int(x) for x in pb]; n = [int(x) for x in n]; pd2 = int(pd2)
    p = [_i32((RL[3 * i] * pb[0] + RL[3 * i + 1] * pb[1] + RL[3 * i + 2] * pb[2] + (tL[i] << 30)) >> 30)
         for i in range(3)]
    C = [_i32((R[0 + i] * n[0] + R[3 + i] * n[1] + R[6 + i] * n[2]) >> 30) for i in range(3)]
    A = [_i32((p[1] * C[2] - p[2] * C[1]) >> 30),
         _i32((p[2] * C[0] - p[0] * C[2]) >> 30),
         _i32((p[0] * C[1] - p[1] * C[0]) >> 30)]
    return n + A, pd2


def golden_fixed(qd):
    """All points of one iteration -> list of 27 exact integer sums (what the FPGA must return)."""
    S = [0] * NSUM
    for k in range(len(qd["pd2"])):
        H, pd2 = golden_point(qd["R"], qd["RL"], qd["tL"], qd["pb"][k], qd["n"][k], qd["pd2"][k])
        for s, (i, j) in enumerate(PAIRS):
            S[s] += H[i] * H[j]
        for i in range(6):
            S[21 + i] -= H[i] * pd2
    lim = 1 << (SUM_BITS - 1)
    assert all(-lim <= s < lim for s in S), "sum overflows 80 bits"
    return S


def sums_to_float(S):
    """27 integer sums -> (HTH 6x6, HTh 6) in floats."""
    HTH = np.zeros((6, 6))
    for s, (i, j) in enumerate(PAIRS):
        HTH[i, j] = HTH[j, i] = S[s] / 2.0 ** (H_SCALE[i] + H_SCALE[j])
    HTh = np.array([S[21 + i] / 2.0 ** (H_SCALE[i] + Q_PD2) for i in range(6)])
    return HTH, HTh


def reference_float(R, RL, tL, pb, n, pd2):
    """FAST-LIO formulas in float64 (no quantization)."""
    R = np.asarray(R, float).reshape(3, 3); RL = np.asarray(RL, float).reshape(3, 3); tL = np.asarray(tL, float)
    p = pb @ RL.T + tL
    C = n @ R                      # rows: (R^T n_i)^T = n_i^T R
    A = np.cross(p, C)
    H = np.hstack([n, A])
    h = -np.asarray(pd2, float)
    return H.T @ H, H.T @ h


# ----------------------------------------------------------------------------------------------
# Microcode.  Register file (6-bit index):
#   0-8 R (row-major) | 9-17 R_LI | 18-20 t_LI | 21-23 pb | 24-26 n | 27 pd2 | 28-30 p | 31-33 C | 34-36 A | 37 ONE=2^30
# Instruction: wb(1) mode(3) dst(6) a(6) b(6)  -> 22 bits.  prod = rf[a]*rf[b] (signed 32x32 -> 64)
#   mode 0 NOP | 1 acc=prod | 2 acc+=prod | 3 acc-=prod | 4 sum[dst]+=prod | 5 sum[dst]-=prod
#   wb=1 (modes 1-3): rf[dst] = (new acc) >>> 30   (lower 32 bits)
# Pipeline: issue (read rf) -> multiply -> accumulate/write back. A result written by the op issued at cycle t can be
# read by an op issued at cycle t+LAT or later.
# ----------------------------------------------------------------------------------------------
RF = dict(R=0, RL=9, tL=18, pb=21, n=24, pd2=27, p=28, C=31, A=34, ONE=37)
NOP, ACC_START, ACC_ADD, ACC_SUB, SUM_ADD, SUM_SUB = range(6)
H_REG = [RF["n"], RF["n"] + 1, RF["n"] + 2, RF["A"], RF["A"] + 1, RF["A"] + 2]


def _chain(terms, dst):
    """terms = [(sign, a, b)...] -> accumulate chain writing rf[dst]."""
    ops = []
    for k, (sg, a, b) in enumerate(terms):
        mode = ACC_START if k == 0 else (ACC_ADD if sg > 0 else ACC_SUB)
        assert k > 0 or sg > 0
        ops.append(dict(wb=int(k == len(terms) - 1), mode=mode, dst=dst if k == len(terms) - 1 else 0, a=a, b=b))
    return ops


def make_microcode(lat=3):
    R, RL, tL, pb, n, pd2, p, C, A, ONE = (RF[k] for k in ["R", "RL", "tL", "pb", "n", "pd2", "p", "C", "A", "ONE"])
    groups = []
    for i in range(3):   # C_i = sum_j R[j][i] n_j
        groups.append(_chain([(1, R + 3 * j + i, n + j) for j in range(3)], C + i))
    for i in range(3):   # p_i = sum_j RL[i][j] pb_j + tL_i * ONE
        groups.append(_chain([(1, RL + 3 * i + j, pb + j) for j in range(3)] + [(1, tL + i, ONE)], p + i))
    sum_idx = {pr: s for s, pr in enumerate(PAIRS)}
    nn = [dict(wb=0, mode=SUM_ADD, dst=sum_idx[(i, j)], a=H_REG[i], b=H_REG[j]) for (i, j) in PAIRS if j < 3]
    groups += [[op] for op in nn]
    groups.append(_chain([(1, p + 0, C + 1), (-1, p + 1, C + 0)], A + 2))   # A2 first: needs only p0,p1
    groups.append(_chain([(1, p + 1, C + 2), (-1, p + 2, C + 1)], A + 0))
    groups.append(_chain([(1, p + 2, C + 0), (-1, p + 0, C + 2)], A + 1))
    groups += [[dict(wb=0, mode=SUM_SUB, dst=21 + i, a=H_REG[i], b=pd2)] for i in range(3)]
    rest = [dict(wb=0, mode=SUM_ADD, dst=sum_idx[(i, j)], a=H_REG[i], b=H_REG[j]) for (i, j) in PAIRS if j >= 3]
    groups += [[op] for op in rest]
    groups += [[dict(wb=0, mode=SUM_SUB, dst=21 + i, a=H_REG[i], b=pd2)] for i in range(3, 6)]
    # schedule in order, inserting NOPs where an operand is not written back yet
    ready = {}            # rf index -> first cycle it can be read
    prog = []
    for g in groups:
        for op in g:
            need = max(ready.get(op["a"], 0), ready.get(op["b"], 0))
            while len(prog) < need:
                prog.append(dict(wb=0, mode=NOP, dst=0, a=0, b=0))
            if op["wb"]:
                ready[op["dst"]] = len(prog) + lat
            prog.append(op)
    # the sums live in a RAM with read-modify-write over 2 clocks: the same sum may not be hit twice within 2 clocks
    last = {}
    for t, op in enumerate(prog + prog):          # program runs back-to-back for consecutive points
        if op["mode"] in (SUM_ADD, SUM_SUB):
            assert t - last.get(op["dst"], -99) > 2, f"sum {op['dst']} reused too soon at {t}"
            last[op["dst"]] = t
    return prog


def encode_uop(op):
    return (op["wb"] << 21) | (op["mode"] << 18) | (op["dst"] << 12) | (op["a"] << 6) | op["b"]


def simulate_microcode(prog, qd, lat=3):
    """Cycle-level model of the sequencer pipeline (checks the table + the hazard rule)."""
    S = [0] * NSUM
    for k in range(len(qd["pd2"])):
        rf = [0] * 38
        rf[0:9] = [int(x) for x in qd["R"]]; rf[9:18] = [int(x) for x in qd["RL"]]; rf[18:21] = [int(x) for x in qd["tL"]]
        rf[21:24] = [int(x) for x in qd["pb"][k]]; rf[24:27] = [int(x) for x in qd["n"][k]]; rf[27] = int(qd["pd2"][k])
        rf[37] = 1 << 30
        pending = []     # (cycle when visible, idx, value)
        acc = 0
        for t, op in enumerate(prog + [dict(wb=0, mode=NOP, dst=0, a=0, b=0)] * lat):
            for (c, i, v) in [x for x in pending if x[0] <= t]:
                rf[i] = v
            pending = [x for x in pending if x[0] > t]
            if op["mode"] == NOP:
                continue
            prod = rf[op["a"]] * rf[op["b"]]
            m = op["mode"]
            if m == ACC_START: acc = prod
            elif m == ACC_ADD: acc += prod
            elif m == ACC_SUB: acc -= prod
            elif m == SUM_ADD: S[op["dst"]] += prod
            elif m == SUM_SUB: S[op["dst"]] -= prod
            if op["wb"]:
                v = acc >> 30
                v = ((v + (1 << 31)) & 0xFFFFFFFF) - (1 << 31)   # keep 32 bits like the hardware
                pending.append((t + lat, op["dst"], v))
    return S


def write_verilog_ucode(prog, path):
    with open(path, "w") as f:
        f.write("// generated by model/hth_model.py - do not edit\n")
        f.write(f"localparam NUOPS = {len(prog)};\n")
        f.write("function [21:0] ucode;\n    input [6:0] pc;\n    case (pc)\n")
        for i, op in enumerate(prog):
            f.write(f"        7'd{i}: ucode = 22'h{encode_uop(op):06x};\n")
        f.write("        default: ucode = 22'h000000;\n    endcase\nendfunction\n")


# ----------------------------------------------------------------------------------------------
# packets
# ----------------------------------------------------------------------------------------------
def _hdr(t, cnt):
    return struct.pack(">I", (MAGIC << 24) | (t << 16) | (cnt & 0xFFFF))


def _words(vals):
    return b"".join(struct.pack(">i", int(v)) for v in vals)


def pkt_begin(qd, iter_id):
    return _hdr(T_BEGIN, iter_id) + _words(list(qd["R"]) + list(qd["RL"]) + list(qd["tL"]))


def pkts_points(qd):
    N = len(qd["pd2"])
    out = []
    for s in range(0, N, MAX_PTS_PER_PKT):
        e = min(N, s + MAX_PTS_PER_PKT)
        body = b"".join(_words(list(qd["pb"][k]) + list(qd["n"][k]) + [qd["pd2"][k]]) for k in range(s, e))
        out.append(_hdr(T_POINTS, e - s) + body)
    return out


def pkt_end():
    return _hdr(T_END, 0)


def decode_reply(data):
    w = struct.unpack(">%dI" % (len(data) // 4), data)
    assert len(w) == 4 + 3 * NSUM, f"reply has {len(w)} words"
    S = []
    for k in range(NSUM):
        v = (w[4 + 3 * k] << 64) | (w[5 + 3 * k] << 32) | w[6 + 3 * k]
        if v >> 95:
            v -= 1 << 96
        S.append(v)
    return dict(magic=w[0] >> 24, type=(w[0] >> 16) & 0xFF, iter_id=w[0] & 0xFFFF, npts=w[1],
                bad_hdr=w[2] >> 16, ovf=w[2] & 1, version=w[3], S=S)


def same_reply(a, b):
    """Replies are equal except word 3, which carries the board's receive counters (diagnostics)."""
    return len(a) == len(b) and a[:12] == b[:12] and a[16:] == b[16:]


def diag_counters(reply):
    w3 = struct.unpack(">I", reply[12:16])[0]
    return (w3 >> 20) & 1023, (w3 >> 10) & 1023, w3 & 1023        # frames started, SFD ok, accepted


def expected_reply(S, iter_id, npts):
    w = [(MAGIC << 24) | (T_END << 16) | iter_id, npts, 0, VERSION_WORD]
    for v in S:
        u = v & ((1 << 96) - 1)
        w += [u >> 64, (u >> 32) & 0xFFFFFFFF, u & 0xFFFFFFFF]
    return struct.pack(">%dI" % len(w), *w)


# ----------------------------------------------------------------------------------------------
# synthetic data that looks like a FAST-LIO iteration
# ----------------------------------------------------------------------------------------------
def random_rotation(rng, max_angle=np.pi):
    axis = rng.normal(size=3); axis /= np.linalg.norm(axis)
    ang = rng.uniform(-max_angle, max_angle)
    K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + np.sin(ang) * K + (1 - np.cos(ang)) * K @ K


def synthetic_iteration(rng, npts, max_range=60.0):
    R = random_rotation(rng)
    RL = random_rotation(rng, 0.05)                    # extrinsic: small rotation
    tL = rng.uniform(-0.1, 0.1, 3)                    # extrinsic: few cm
    d = rng.normal(size=(npts, 3)); d /= np.linalg.norm(d, axis=1, keepdims=True)
    pb = d * rng.uniform(1.0, max_range, (npts, 1))
    n = rng.normal(size=(npts, 3)); n /= np.linalg.norm(n, axis=1, keepdims=True)
    pd2 = rng.normal(0, 0.05, npts)
    return R, RL, tL, pb, n, pd2
