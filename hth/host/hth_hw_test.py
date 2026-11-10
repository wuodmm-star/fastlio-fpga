"""Hardware test: random FAST-LIO-like iterations -> board, compare every reply bit-exact with the golden model.
Usage: python hth_hw_test.py [--iters 50] [--bind 192.168.137.1]
"""
import argparse, os, socket, sys, time
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "model"))
import hth_model as M

BOARD = ("192.168.137.234", 1234)


GAP = 0.0
SENT = 0
WARMUP = 0


def run_iteration(sock, qd, iter_id, timeout=0.5):
    pkts = [M.pkt_begin(qd, iter_id)] + M.pkts_points(qd) + [M.pkt_end()]
    pkts = [M._hdr(M.T_POINTS, 0)] * WARMUP + pkts          # header-only packets, ignored by the engine
    if GAP:
        time.sleep(GAP)
    t0 = time.perf_counter()
    global SENT
    for p in pkts:
        sock.sendto(p, BOARD)
        SENT += 1
    sock.settimeout(timeout)
    while True:
        data, src = sock.recvfrom(4096)
        if src[0] == BOARD[0]:
            break
    return data, time.perf_counter() - t0, len(pkts)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--iters", type=int, default=50)
    ap.add_argument("--bind", default="192.168.137.1")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--gap", type=float, default=0.0, help="seconds to wait before each BEGIN")
    ap.add_argument("--warmup", type=int, default=0, help="header-only packets sent before each BEGIN")
    a = ap.parse_args()
    global GAP, WARMUP
    GAP = a.gap
    WARMUP = a.warmup
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind((a.bind, 1234))
    rng = np.random.default_rng(a.seed)
    ok = lost = bad = 0
    prev = None; tot_sent = 0; tot = [0, 0, 0]; lossy = []
    pts_total, t_total = 0, 0.0
    for it in range(a.iters):
        npts = int(rng.integers(300, 3000))
        R, RL, tL, pb, n, pd2 = M.synthetic_iteration(rng, npts)
        qd = M.quantize(R, RL, tL, pb, n, pd2)
        exp = M.expected_reply(M.golden_fixed(qd), it & 0xFFFF, npts)
        try:
            got, dt, nsent = run_iteration(s, qd, it & 0xFFFF)
        except socket.timeout:
            lost += 1
            print(f"iter {it}: no reply")
            continue
        if len(got) == 340:
            c = M.diag_counters(got)
            if prev is not None:
                d = [(c[i] - prev[i]) % 1024 for i in range(3)]
                tot_sent += nsent; tot = [tot[i] + d[i] for i in range(3)]
                if d[2] != nsent:
                    lossy.append((it, nsent, d))
            prev = c
        if M.same_reply(got, exp):
            ok += 1; pts_total += npts; t_total += dt
        else:
            bad += 1
            d = M.decode_reply(got) if len(got) == 340 else None
            print(f"iter {it}: MISMATCH len {len(got)} " + (f"id {d['iter_id']} pts {d['npts']}/{npts} bad_hdr {d['bad_hdr']} ovf {d['ovf']}" if d else ""))
    print(f"{a.iters} iterations: bit-exact {ok}, wrong {bad}, no reply {lost}; python sent {SENT} datagrams")
    if ok:
        print(f"points/s incl. PC send time: {pts_total / t_total:,.0f}  (avg {pts_total / ok:.0f} points, "
              f"{t_total / ok * 1e3:.2f} ms per iteration)")
    print(f"board receive counters over {a.iters - 1} iterations: sent {tot_sent} datagrams | frames started {tot[0]}, "
          f"preamble+SFD ok {tot[1]}, accepted {tot[2]}")
    for it, n, d in lossy[:12]:
        print(f"   iter {it}: sent {n}, board saw start {d[0]}, SFD ok {d[1]}, accepted {d[2]}")
    print("PASS" if ok == a.iters else "FAIL")


if __name__ == "__main__":
    main()
