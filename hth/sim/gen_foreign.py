"""System stimulus with foreign broadcast frames whose last bytes are 0x55 (look like a preamble start)."""
import os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import gen_hth_stim as G
from frames import build

def foreign(k):
    fr = bytearray(build(bytes.fromhex("02005e000001"), bytes([192, 168, 137, 1]), 137, bytes(range(60)),
                         dst_mac=bytes.fromhex("ffffffffffff"), dst_ip=bytes([192, 168, 137, 255]), dst_port=137, ident=900 + k))
    fr[-7:] = b"\x55" * 7          # payload tail + FCS look like preamble bytes
    return bytes(fr)

its = [G.iteration(83, 31), G.iteration(130, 32)]
frames = []
for pkts, _ in its:
    for k, p in enumerate(pkts):
        if k % 2 == 0:
            frames.append(foreign(len(frames)))
        frames.append(build(G.PC_MAC, G.PC_IP, G.PC_PORT, p, ident=len(frames) + 1))
with open(os.path.join(HERE, "sys_stim.mem"), "w") as f:
    for fr in frames:
        f.writelines(f"{b:02x}\n" for b in fr)
with open(os.path.join(HERE, "sys_lens.mem"), "w") as f:
    f.writelines(f"{len(fr):04x}\n" for fr in frames)
with open(os.path.join(HERE, "sys_expect.txt"), "w") as f:
    for _, e in its:
        f.write(e.hex() + "\n")
print(len(frames), "frames incl. foreign ones")
