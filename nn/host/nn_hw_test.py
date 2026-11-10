"""Hardware test of the FPGA voxel map / nearest-neighbour search.
  python nn_hw_test.py <ops.mem> <expect.txt>      (files produced for the simulations: 5 hex words per op line)
Packets: <= 72 ops and <= 18 searches each; every packet gets a reply; stop-and-wait.
"""
import socket, struct, sys, time

BOARD = ("192.168.137.234", 1234)


def main():
    ops = [[int(x, 16) for x in l.split()] for l in open(sys.argv[1]) if l.strip()]
    exp = [l.split() for l in open(sys.argv[2]) if l.strip()]
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("0.0.0.0", 0)); s.settimeout(1.0)
    got, pk, nsr, lost = [], [], 0, 0
    t_search = 0.0
    t0 = time.perf_counter()

    def flush():
        nonlocal pk, nsr, lost, t_search
        if not pk:
            return
        body = b"".join(struct.pack(">4I", (o[1] << 1) | o[0], o[2], o[3], o[4]) for o in pk)
        ta = time.perf_counter()
        s.sendto(struct.pack(">I", 0xA5110000 | len(pk)) + body, BOARD)
        try:
            data, _ = s.recvfrom(4096)
        except socket.timeout:
            lost += 1; print("no reply"); pk, nsr = [], 0; return
        if nsr:
            t_search += time.perf_counter() - ta
        w = struct.unpack(">%dI" % (len(data) // 4), data)
        assert w[0] >> 16 == 0xA512 and (w[0] & 0xFFFF) == nsr, f"bad reply header {w[0]:08x} (expected {nsr} results)"
        for k in range(nsr):
            r = w[1 + 16 * k: 17 + 16 * k]
            n = r[0]
            got.append([str(n)] + [f"{r[1 + i]:08x}" for i in range(3 * n)])
        pk, nsr = [], 0

    for o in ops:
        pk.append(o); nsr += o[0]
        if len(pk) == 72 or nsr == 18:
            flush()
    flush()
    dt = time.perf_counter() - t0
    bad = sum(a != b for a, b in zip(exp, got))
    print(f"{len(ops)} ops, {len(exp)} searches expected, {len(got)} results, {bad} mismatching, {lost} lost packets")
    for k, (a, b) in enumerate(zip(exp, got)):
        if a != b:
            print("first mismatch at search", k, "\n exp", " ".join(a), "\n got", " ".join(b)); break
    print(f"total {dt:.2f} s; packets containing searches: {t_search / max(1, len(got)) * 1e6:.0f} us per search incl. PC round trip")
    print("ALL BIT-EXACT" if bad == 0 and len(got) == len(exp) and not lost else "FAIL")


if __name__ == "__main__":
    main()
