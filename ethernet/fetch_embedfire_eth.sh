#!/bin/bash
# Downloads the Ethernet UDP files of Embedfire's EP4CE10 (征途Pro) tutorial, example 60_ethernet_udp_rmii,
# and applies our changes (reply to sender, drop IP fragments / non-UDP, reset the receive state machine between
# frames, preamble/SFD pulse for diagnostics). Embedfire publishes the code without a license file, so it is not
# copied into this repository. Source: https://github.com/Embedfire-altera/ebf_ep4ce10_pro_tutorial_code
set -euo pipefail
cd "$(dirname "$0")"
R=https://raw.githubusercontent.com/Embedfire-altera/ebf_ep4ce10_pro_tutorial_code/master/60_ethernet_udp_rmii/rtl
rm -rf rtl && mkdir -p a/eth_udp_mii a/eth_udp_rmii
for f in eth_udp_mii.v ip_receive.v ip_send.v crc32_d4.v; do curl -sfL -o a/eth_udp_mii/$f $R/eth_udp_mii/$f; done
for f in rmii_to_mii.v mii_to_rmii.v; do curl -sfL -o a/eth_udp_rmii/$f $R/eth_udp_rmii/$f; done
patch -s -p1 -d a < embedfire_eth.patch
mv a rtl
echo "Ethernet files ready in ethernet/rtl/"
