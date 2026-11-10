"""Stimulus + expected replies for the HTH engine testbenches.

engine test : words straight into the FIFO   -> engine_words.mem, engine_expect.txt
system test : the same packets as real Ethernet/IPv4/UDP frames into the RMII pins -> sys_stim.mem, sys_lens.mem,
              sys_expect.txt
"""
import os, struct, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "model"))
sys.path.insert(0, os.path.join(HERE, "..", "..", "common"))
import hth_model as M
from frames import build

rng = np.random.default_rng(11)
PC_MAC, PC_IP, PC_PORT = bytes.fromhex("02005e000001"), bytes([192, 168, 137, 1]), 50999


def iteration(npts, iter_id, junk=False, max_range=60.0):
    R, RL, tL, pb, n, pd2 = M.synthetic_iteration(rng, npts, max_range)
    qd = M.quantize(R, RL, tL, pb, n, pd2)
    pkts = [M.pkt_begin(qd, iter_id)] + M.pkts_points(qd)
    if junk:                                   # a stray non-header word between two packets
        pkts.insert(2, struct.pack(">I", 0x12345678))
    pkts.append(M.pkt_end())
    S = M.golden_fixed(qd)
    exp = bytearray(M.expected_reply(S, iter_id, npts))
    if junk:
        exp[8:12] = struct.pack(">I", 1 << 16)  # bad-header count 1 reported in w2
    return pkts, bytes(exp)


def write_engine(its, tag):
    words = b"".join(b"".join(p) for p, _ in its)
    with open(os.path.join(HERE, f"{tag}_words.mem"), "w") as f:
        for k in range(0, len(words), 4):
            f.write(words[k:k + 4].hex() + "\n")
    with open(os.path.join(HERE, f"{tag}_expect.txt"), "w") as f:
        for _, e in its:
            f.write(e.hex() + "\n")
    return len(words) // 4


def write_system(its):
    frames = []
    for pkts, _ in its:
        for k, p in enumerate(pkts):
            if len(p) == 4 and struct.unpack(">I", p)[0] == 0x12345678:
                continue                        # junk word only makes sense inside one byte stream
            frames.append(build(PC_MAC, PC_IP, PC_PORT, p, ident=len(frames) + 1))
    with open(os.path.join(HERE, "sys_stim.mem"), "w") as f:
        for fr in frames:
            f.writelines(f"{b:02x}\n" for b in fr)
    with open(os.path.join(HERE, "sys_lens.mem"), "w") as f:
        f.writelines(f"{len(fr):04x}\n" for fr in frames)
    with open(os.path.join(HERE, "sys_expect.txt"), "w") as f:
        for _, e in its:
            f.write(e.hex() + "\n")
    return len(frames), sum(map(len, frames))


if __name__ == "__main__":
    # engine: 4 iterations incl. 1 point, 41 (exactly one packet), 42 (41+1), 997 points, junk word, far points
    eng = [iteration(1, 1), iteration(41, 2), iteration(42, 3, junk=True), iteration(997, 4),
           iteration(300, 5, max_range=8000.0)]
    nw = write_engine(eng, "engine")
    print(f"engine test: {len(eng)} iterations, {nw} words")
    sysi = [iteration(83, 21), iteration(250, 22)]
    nf, nb = write_system(sysi)
    print(f"system test: {len(sysi)} iterations, {nf} frames, {nb} bytes")
