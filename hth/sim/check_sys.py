import os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "model")); sys.path.insert(0, os.path.join(HERE, "..", "..", "common"))
import hth_model as M
from frames import parse, BOARD_MAC, BOARD_IP
exp = [bytes.fromhex(l.strip()) for l in open(os.path.join(HERE, "sys_expect.txt")) if l.strip()]
tx = [l.split()[1] for l in open(os.path.join(HERE, "sys_out.txt")) if l.startswith("TX")]
ok = len(tx) == len(exp)
print(f"expected {len(exp)} reply frames, board sent {len(tx)}")
for k, (e, t) in enumerate(zip(exp, tx)):
    r = parse(bytes.fromhex(t))
    hdr = (r["crc_ok"] and r["iph_ok"] and r["src_mac"] == BOARD_MAC and r["src_ip"] == BOARD_IP and
           r["dst_mac"].hex() == "02005e000001" and r["dst_ip"] == bytes([192, 168, 137, 1]) and r["dst_port"] == 50999)
    same = M.same_reply(r["payload"], e)
    d = M.decode_reply(r["payload"])
    print(f"  frame {k}: FCS {r['crc_ok']} IPcsum {r['iph_ok']} to {r['dst_ip'][3]}:{r['dst_port']} | id {d['iter_id']} "
          f"points {d['npts']} | payload bit-exact: {same}")
    ok &= hdr and same
print("ALL PASS" if ok else "FAIL")
