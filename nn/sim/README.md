# NN-search simulations

The testbenches use Micron's SDRAM model `mt48lc16m16a2.v` (256 Mbit, x16 - same geometry as the W9825G6 on the
征途Pro board). It is not included here; download Micron's Verilog model for MT48LC16M16A2 and put the file in
this folder. iverilog 12 compiles it as is.

| test | command | what it checks |
|---|---|---|
| SDRAM controller | `iverilog -g2012 -DRDLAT=3 -o t.vvp tb_sdram.v mt48lc16m16a2.v ../rtl/sdram_rw.v && vvp -n t.vvp` | random burst writes/reads, refresh interleaving |
| core, synthetic | `python3 gen_nn_synth.py 6000`, then `tb_nn.v` (see below), `python3 check_nn.py` | all add cases (append, closest-to-centre, replace-farthest) + 2110 searches |
| core, real FAST-LIO ops | `python3 gen_nn_stim.py 400 10` (needs a FAST-LIO `FPGA_NN_DUMP` file named `nn.dump`) | recorded adds/searches |
| whole chip | `python3 gen_nn_sys.py 600`, `tb_nn_sys.v` with `+W=4` | Ethernet frames in, reply frames out, 4 packets in flight |

    iverilog -g2012 -o t.vvp tb_nn.v mt48lc16m16a2.v ../rtl/sdram_rw.v ../rtl/nn_core.v && vvp -n t.vvp +N=$(wc -l < nn_ops.mem)
    E=../../ethernet/rtl; iverilog -g2012 -o s.vvp -s tb_nn_sys tb_nn_sys.v clk_gen_sim.v mt48lc16m16a2.v ../../common/rx_fifo_sim.v \
      ../rtl/lio_nn_top.v ../rtl/nn_engine.v ../rtl/nn_core.v ../rtl/sdram_rw.v $E/eth_udp_mii/*.v $E/eth_udp_rmii/*.v
    vvp -n s.vvp +NF=$(wc -l < sys_lens.mem) +W=4 && python3 check_nn_sys.py
