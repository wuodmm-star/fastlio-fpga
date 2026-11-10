"""Step 4: replay iterations recorded from FAST-LIO (FPGA_HTH_DUMP) .

  python replay_dump.py stats  run.dump            golden fixed-point vs FAST-LIO's own float64 HTH/HTh
  python replay_dump.py board  run.dump [--max N]  send every iteration to the board, compare bit-exact with golden
"""
import argparse, os, socket, struct, sys, time
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "model"))
import hth_model as M


def read_dump(path, max_iters=None):
    with open(path, "rb") as f:
        data = f.read()
    off, out = 0, []
    while off + 4 <= len(data):
        (n,) = struct.unpack_from("<i", data, off)
        if off + 4 + 168 + 28 * n + 288 + 48 > len(data):
            break                                   # last record cut off when FAST-LIO was stopped
        off += 4
        m = np.frombuffer(data, "<f8", 21, off); off += 168
        pts = np.frombuffer(data, "<f4", 7 * n, off).reshape(n, 7).astype(np.float64); off += 28 * n
        A = np.frombuffer(data, "<f8", 36, off).reshape(6, 6).T; off += 288
        b = np.frombuffer(data, "<f8", 6, off); off += 48
        out.append(dict(R=m[0:9].reshape(3, 3), RL=m[9:18].reshape(3, 3), tL=m[18:21], pb=pts[:, 0:3], n=pts[:, 3:6],
                        pd2=pts[:, 6], HTH_cpu=A, HTh_cpu=b))
        if max_iters and len(out) >= max_iters:
            break
    return out


def quantize(it):
    return M.quantize(it["R"], it["RL"], it["tL"], it["pb"], it["n"], it["pd2"])


def stats(its):
    rel_A, rel_b, npts = [], [], []
    for it in its:
        HTH, HTh = M.sums_to_float(M.golden_fixed(quantize(it)))
        rel_A.append(np.abs(HTH - it["HTH_cpu"]).max() / np.abs(it["HTH_cpu"]).max())
        rel_b.append(np.abs(HTh - it["HTh_cpu"]).max() / max(np.abs(it["HTh_cpu"]).max(), 1e-300))
        npts.append(len(it["pd2"]))
    rel_A, rel_b, npts = map(np.array, (rel_A, rel_b, npts))
    print(f"{len(its)} real iterations, points per iteration: median {np.median(npts):.0f}, min {npts.min()}, max {npts.max()}")
    print(f"fixed-point vs FAST-LIO float64, max relative error per iteration:")
    print(f"  HTH: median {np.median(rel_A):.2e}, 99% {np.percentile(rel_A, 99):.2e}, worst {rel_A.max():.2e}")
    print(f"  HTh: median {np.median(rel_b):.2e}, 99% {np.percentile(rel_b, 99):.2e}, worst {rel_b.max():.2e}")
    allp = np.concatenate([it["pb"] for it in its]); print(f"  point range max |coord| {np.abs(allp).max():.1f} m, max |pd2| "
                                                            f"{max(np.abs(it['pd2']).max() for it in its):.3f} m")


def board(its, bind, warmup):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind((bind, 1234)); s.settimeout(0.5)
    ok = bad = lost = 0; t_sum = 0.0; pts_sum = 0; worst = 0.0
    for k, it in enumerate(its):
        qd = quantize(it); iid = k & 0xFFFF
        exp = M.expected_reply(M.golden_fixed(qd), iid, len(qd["pd2"]))
        pk = [M._hdr(M.T_POINTS, 0)] * warmup + [M.pkt_begin(qd, iid)] + M.pkts_points(qd) + [M.pkt_end()]
        t0 = time.perf_counter()
        for p in pk:
            s.sendto(p, ("192.168.137.234", 1234))
        try:
            while True:
                got, src = s.recvfrom(4096)
                if src[0] == "192.168.137.234":
                    break
        except socket.timeout:
            lost += 1; continue
        dt = time.perf_counter() - t0
        if M.same_reply(got, exp):
            ok += 1; t_sum += dt; pts_sum += len(qd["pd2"]); worst = max(worst, dt)
        else:
            bad += 1
    print(f"{len(its)} real iterations on the board: bit-exact {ok}, wrong {bad}, no reply {lost}")
    if ok:
        print(f"  {t_sum / ok * 1e3:.2f} ms per iteration on average (worst {worst * 1e3:.1f} ms), "
              f"{pts_sum / t_sum:,.0f} points/s, {pts_sum / ok:.0f} points per iteration")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["stats", "board"]); ap.add_argument("dump")
    ap.add_argument("--max", type=int, default=None); ap.add_argument("--bind", default="192.168.137.1")
    ap.add_argument("--warmup", type=int, default=1)
    a = ap.parse_args()
    its = read_dump(a.dump, a.max)
    stats(its) if a.cmd == "stats" else board(its, a.bind, a.warmup)
