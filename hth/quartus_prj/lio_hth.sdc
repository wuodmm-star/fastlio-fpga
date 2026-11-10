# RMII reference clock from the PHY: 50 MHz
create_clock -name eth_clk -period 20.000 [get_ports {eth_clk}]
# MII clock: toggle flip-flop on the falling edge of eth_clk (divide by 2)
create_generated_clock -name clk_25m -source [get_ports {eth_clk}] -edges {2 4 6} [get_registers {clk_25m}]
derive_clock_uncertainty
# Clock-domain crossings that are handled by design (dual-clock FIFO, synchronisers, transmit buffer written
# long before it is read) are still between related clocks, so TimeQuest analyses them normally - no false paths.
