"""Golden model of the FPGA voxel-hash map + 5-nearest-neighbour search (same rules as MapFx in voxmap.hpp).

  coordinates : int32 Q16.16 metres;  voxel index = coord >> 15  (0.5 m)
  table       : 2^gbits groups x 8 slots (gbits = 16 on the board by default);
                group = bits 8..(8+gbits-1) of (uint32)(x + 1021 y + 262147 z) * 2654435761;
                a voxel takes the first free slot of its group (tag = voxel index). If the group is full, the slot
                whose voxel is farthest from the new one (Chebyshev, in voxels; lowest slot on ties) is reused when
                that distance is > keep (keep = 0: never); otherwise the add is dropped.
  add(p, ds)  : ds=1 -> voxel keeps only the point closest to its centre; ds=0 -> append, or replace the farthest
  search(q)   : 27 voxels around q (dx, dy, dz = -1..1, dz fastest), K=5 nearest, a later point wins only if
                strictly closer
python nn_model.py replay <dump> [gbits keep]   replays a FAST-LIO FPGA_NN_DUMP and checks every search result
"""
import struct
import sys

NGROUP, WAYS, CAP, K = 1 << 16, 8, 4, 5


def group(v, gbits=16):
    s = (v[0] + v[1] * 1021 + v[2] * 262147) & 0xFFFFFFFF
    return (((s * 2654435761) & 0xFFFFFFFF) >> 8) & ((1 << gbits) - 1)


def dc2(p, v):
    return sum((((v[i] << 15) + (1 << 14)) - p[i]) ** 2 for i in range(3))


class Map:
    def __init__(self, gbits=16, keep=0):
        self.tag = {}      # slot -> voxel tuple
        self.pts = {}      # slot -> list of points
        self.dropped = 0
        self.evicted = 0
        self.gbits, self.keep = gbits, keep

    def find(self, v, create):
        g = group(v, self.gbits) * WAYS
        for k in range(WAYS):
            s = g + k
            if s in self.tag and self.tag[s] == v:
                return s
        if not create:
            return None
        for k in range(WAYS):
            s = g + k
            if s not in self.tag:
                self.tag[s] = v
                self.pts[s] = []
                return s
        if self.keep:
            far, fd = 0, -1
            for k in range(WAYS):
                t = self.tag[g + k]
                d = max(abs(t[0] - v[0]), abs(t[1] - v[1]), abs(t[2] - v[2]))
                if d > fd:
                    far, fd = k, d
            if fd > self.keep:
                s = g + far
                self.tag[s] = v
                self.pts[s] = []
                self.evicted += 1
                return s
        self.dropped += 1
        return None

    def add(self, p, ds):
        v = (p[0] >> 15, p[1] >> 15, p[2] >> 15)
        s = self.find(v, True)
        if s is None:
            return
        L = self.pts[s]
        if not L:
            L.append(p)
        elif ds:
            best, bp = dc2(p, v), p
            for q in L:
                d = dc2(q, v)
                if d < best:
                    best, bp = d, q
            L[:] = [bp]
        elif len(L) < CAP:
            L.append(p)
        else:
            far, fd = 0, -1
            for i, q in enumerate(L):
                d = dc2(q, v)
                if d > fd:
                    far, fd = i, d
            L[far] = p

    def search(self, q):
        v0 = (q[0] >> 15, q[1] >> 15, q[2] >> 15)
        best = []          # list of (d2, point), sorted, stable
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    s = self.find((v0[0] + dx, v0[1] + dy, v0[2] + dz), False)
                    if s is None:
                        continue
                    for p in self.pts[s]:
                        d = (p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2 + (p[2] - q[2]) ** 2
                        if len(best) == K and d >= best[-1][0]:
                            continue
                        j = len(best)
                        while j > 0 and best[j - 1][0] > d:
                            j -= 1
                        best.insert(j, (d, p))
                        del best[K:]
        return best


def read_dump(path):
    data = open(path, "rb").read()
    off = 0
    while off < len(data):
        (op,) = struct.unpack_from("<i", data, off)
        if op == 1:
            if off + 20 > len(data):
                break
            _, ds, x, y, z = struct.unpack_from("<5i", data, off); off += 20
            yield ("add", ds, (x, y, z))
        elif op == 2:
            if off + 120 > len(data):
                break
            r = struct.unpack_from("<30i", data, off); off += 120
            n = r[4]
            res = [(((r[8 + 5 * k] << 32) | (r[9 + 5 * k] & 0xFFFFFFFF)), (r[5 + 5 * k], r[6 + 5 * k], r[7 + 5 * k])) for k in range(n)]
            yield ("search", (r[1], r[2], r[3]), res)
        else:
            raise ValueError(f"bad op {op} at {off}")


if __name__ == "__main__" and sys.argv[1] == "replay":
    M = Map(*(int(a) for a in sys.argv[3:5])); n_add = n_s = bad = 0
    for rec in read_dump(sys.argv[2]):
        if rec[0] == "add":
            M.add(rec[2], rec[1]); n_add += 1
        else:
            got = M.search(rec[1]); n_s += 1
            # the dump stores the result after a float round trip: points are exact while |coord| < 256 m,
            # squared distances only approximately -> compare the points (and their order) exactly
            if [p for _, p in got] != [p for _, p in rec[2]] or any(abs(a[0] - b[0]) > max(1, a[0] >> 20) for a, b in zip(got, rec[2])):
                bad += 1
                if bad <= 3:
                    print("mismatch", rec[1], got, rec[2])
    print(f"{n_add} adds, {n_s} searches, {bad} mismatches vs FAST-LIO's MapFx, voxels {len(M.tag)}, forgotten {M.evicted}, dropped {M.dropped}")
