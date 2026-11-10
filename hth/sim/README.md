# HᵀH engine simulations

    cd ../model && python3 test_model.py            # fixed-point error vs float64 + microcode check; writes ../rtl/hth_ucode.vh
    cd ../sim && python3 gen_hth_stim.py            # engine + system stimulus from random Jacobians
    # engine alone (5 iterations), bit-exact against the Python golden model
    iverilog -g2012 -I ../rtl -o e.vvp -s tb_engine tb_engine.v ../../common/rx_fifo_sim.v ../rtl/hth_engine.v
    vvp -n e.vvp +N=$(wc -l < engine_words.mem) +R=5 && python3 check_hth.py engine_expect.txt engine_out.txt
    # whole chip: Ethernet frames in -> reply frames out (needs ../../ethernet/fetch_embedfire_eth.sh first)
    E=../../ethernet/rtl
    iverilog -g2012 -I ../rtl -o s.vvp -s tb_system tb_system.v ../../common/rx_fifo_sim.v ../rtl/lio_hth_top.v \
      ../rtl/hth_engine.v ../rtl/hth_txbuf.v $E/eth_udp_mii/*.v $E/eth_udp_rmii/*.v
    vvp -n s.vvp +NF=$(wc -l < sys_lens.mem) && python3 check_sys.py

`gen_foreign.py` adds foreign broadcast frames between requests (the case that exposed the receive state machine bug).
On real hardware, `../host/hth_hw_test.py` sends recorded FAST-LIO iterations to the board and compares the replies.
