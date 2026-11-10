"""Compare replies produced by a testbench with the golden model's expected replies (bit exact)."""
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "model"))
import hth_model as M
exp_file, out_file = sys.argv[1], sys.argv[2]
exp = [bytes.fromhex(l.strip()) for l in open(exp_file) if l.strip()]
out = [bytes.fromhex(l.strip()) for l in open(out_file) if l.strip()]
ok = len(exp) == len(out)
print(f"expected {len(exp)} replies, got {len(out)}")
for k, (e, o) in enumerate(zip(exp, out)):
    de, do = M.decode_reply(e), M.decode_reply(o)
    same = M.same_reply(e, o)
    nbad = sum(a != b for a, b in zip(de["S"], do["S"]))
    print(f"  reply {k}: id {do['iter_id']} points {do['npts']} (exp {de['npts']}) bad_hdr {do['bad_hdr']} "
          f"ovf {do['ovf']} version {do['version']:08x} | 27 sums bit-exact: {'yes' if nbad == 0 else f'NO ({nbad} differ)'}"
          f" | whole reply identical: {'yes' if same else 'NO'}")
    ok &= same
print("ALL BIT-EXACT" if ok else "MISMATCH")
