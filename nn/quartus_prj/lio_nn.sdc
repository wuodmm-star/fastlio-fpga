create_clock -name sys_clk -period 20.000 [get_ports {sys_clk}]
create_clock -name eth_clk -period 20.000 [get_ports {eth_clk}]
create_generated_clock -name clk_25m -source [get_ports {eth_clk}] -edges {2 4 6} [get_registers {clk_25m}]
derive_pll_clocks
derive_clock_uncertainty
# PHY clock and board oscillator are unrelated crystals; all crossings use the dual-clock FIFO, the transmit RAM
# (written long before it is read) and toggle synchronisers.
set_clock_groups -asynchronous -group {eth_clk clk_25m} -group {sys_clk clk_gen_inst|altpll_component|auto_generated|pll1|clk[0] clk_gen_inst|altpll_component|auto_generated|pll1|clk[1] clk_gen_inst|altpll_component|auto_generated|pll1|clk[2]}
