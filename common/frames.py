"""Build/parse Ethernet+IPv4+UDP frames for the reply-to-sender testbench."""
import struct, zlib

BOARD_MAC = bytes.fromhex("123456789abc")
BOARD_IP = bytes([192, 168, 137, 234])
BOARD_PORT = 1234


def ip_checksum(h):
    s = sum(struct.unpack("!10H", h))
    while s >> 16:
        s = (s & 0xFFFF) + (s >> 16)
    return (~s) & 0xFFFF


def build(src_mac, src_ip, src_port, payload, dst_mac=BOARD_MAC, dst_ip=BOARD_IP, dst_port=BOARD_PORT, ident=1,
          flags=0x4000, proto=17):
    udp = struct.pack("!HHHH", src_port, dst_port, 8 + len(payload), 0) + payload
    iph = struct.pack("!BBHHHBBH4s4s", 0x45, 0, 20 + len(udp), ident, flags, 64, proto, 0, src_ip, dst_ip)
    iph = iph[:10] + struct.pack("!H", ip_checksum(iph)) + iph[12:]
    eth = dst_mac + src_mac + b"\x08\x00" + iph + udp
    fcs = struct.pack("<I", zlib.crc32(eth) & 0xFFFFFFFF)
    return b"\x55" * 7 + b"\xd5" + eth + fcs


def parse(raw):
    """raw = bytes seen on the wire (preamble included). Returns dict or raises."""
    i = raw.index(b"\xd5") + 1
    f = raw[i:]
    eth, fcs = f[:-4], f[-4:]
    crc_ok = struct.pack("<I", zlib.crc32(eth) & 0xFFFFFFFF) == fcs
    dst_mac, src_mac, etype = eth[0:6], eth[6:12], eth[12:14]
    iph = eth[14:34]
    ihl_ok = iph[0] == 0x45
    iph_ok = ip_checksum(iph[:10] + b"\0\0" + iph[12:]) == struct.unpack("!H", iph[10:12])[0]
    src_ip, dst_ip = iph[12:16], iph[16:20]
    sp, dp, ulen, _ = struct.unpack("!HHHH", eth[34:42])
    payload = eth[42:42 + ulen - 8]
    return dict(dst_mac=dst_mac, src_mac=src_mac, etype=etype, src_ip=src_ip, dst_ip=dst_ip,
                src_port=sp, dst_port=dp, payload=payload, crc_ok=crc_ok, iph_ok=iph_ok and ihl_ok)
