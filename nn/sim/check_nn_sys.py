import os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "common"))
from frames import parse
exp = [bytes.fromhex(l.strip()) for l in open(os.path.join(HERE, "sys_expect.txt")) if l.strip()]
tx = [parse(bytes.fromhex(l.split()[1])) for l in open(os.path.join(HERE, "sys_out.txt")) if l.startswith("TX")]
ok = len(tx) == len(exp); nbad = 0
for k, (e, r) in enumerate(zip(exp, tx)):
    good = r["crc_ok"] and r["iph_ok"] and r["dst_port"] == 51000 and r["payload"][:len(e)] == e
    if not good:
        nbad += 1
        if nbad <= 3: print(f"reply {k}: crc {r['crc_ok']} ip {r['iph_ok']} len {len(r['payload'])}/{len(e)}\n exp {e[:40].hex()}\n got {r['payload'][:40].hex()}")
    ok &= good
print(f"expected {len(exp)} replies, got {len(tx)}, bad {nbad}")
print("ALL BIT-EXACT" if ok else "FAIL")
