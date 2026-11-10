`timescale 1ns/1ps
// whole chip: Ethernet frames on the RMII pins, Micron SDRAM model on the SDRAM pins; stop-and-wait per packet
module tb_nn_sys;
    reg eth_clk = 0, sys_clk = 0, sys_rst_n = 1, rxdv = 0; reg [1:0] rxd = 0;
    always #10 eth_clk = ~eth_clk;
    always #10 sys_clk = ~sys_clk;
    wire tx_en; wire [1:0] txd; wire phy_rst_n;
    wire sdram_clk, sdram_cke, cs_n, ras_n, cas_n, we_n; wire [1:0] ba, dqm; wire [12:0] sa; wire [15:0] dq;
    lio_nn_top dut (.sys_clk(sys_clk), .sys_rst_n(sys_rst_n), .eth_clk(eth_clk), .eth_rxdv_r(rxdv), .eth_rx_data_r(rxd),
        .eth_tx_en_r(tx_en), .eth_tx_data_r(txd), .eth_rst_n(phy_rst_n), .sdram_clk(sdram_clk), .sdram_cke(sdram_cke),
        .sdram_cs_n(cs_n), .sdram_ras_n(ras_n), .sdram_cas_n(cas_n), .sdram_we_n(we_n), .sdram_ba(ba),
        .sdram_addr(sa), .sdram_dqm(dqm), .sdram_dq(dq));
    mt48lc16m16a2 mem (.Dq(dq), .Addr(sa), .Ba(ba), .Clk(sdram_clk), .Cke(sdram_cke), .Cs_n(cs_n), .Ras_n(ras_n),
        .Cas_n(cas_n), .We_n(we_n), .Dqm(dqm));
    reg [7:0] stim [0:1048575]; reg [15:0] lens [0:4095];
    integer nf, f, b, k, base, fo, ntx = 0, t, W;
    task send_frame(input integer start, input integer len);
        begin
            for (b = 0; b < len; b = b + 1)
                for (k = 0; k < 4; k = k + 1) begin @(negedge eth_clk); rxdv = 1; rxd = stim[start + b] >> (2 * k); end
            @(negedge eth_clk); rxdv = 0; rxd = 0;
        end
    endtask
    reg [7:0] cur; integer nd = 0; reg in_frame = 0;
    always @(posedge eth_clk) begin
        if (tx_en) begin
            if (!in_frame) begin in_frame = 1; nd = 0; $fwrite(fo, "TX "); end
            cur = {txd, cur[7:2]}; nd = nd + 1;
            if (nd == 4) begin $fwrite(fo, "%02x", cur); nd = 0; end
        end else if (in_frame) begin in_frame = 0; ntx = ntx + 1; $fwrite(fo, "\n"); end
    end
    initial begin
        if (!$value$plusargs("NF=%d", nf)) nf = 1;
        if (!$value$plusargs("W=%d", W)) W = 1;               // packets in flight (1 = stop-and-wait)
        $readmemh("sys_stim.mem", stim); $readmemh("sys_lens.mem", lens);
        fo = $fopen("sys_out.txt", "w");
        #50 sys_rst_n = 0; #150 sys_rst_n = 1;
        wait (dut.nn_core_inst.st == 5'd0 && dut.nn_core_inst.clr_g != 0);   // SDRAM initialised and table cleared
        $display("map cleared at %0t ns", $time);
        base = 0;
        for (f = 0; f < nf; f = f + 1) begin
            t = 0; while (ntx < f + 1 - W && t < 2000000) begin #100; t = t + 1; end
            send_frame(base, lens[f]); base = base + lens[f];
            #1000;
        end
        t = 0; while (ntx < nf && t < 200000) begin #100; t = t + 1; end
        #20000;
        $display("sent %0d request frames, board sent %0d replies, end %0t ns", nf, ntx, $time);
        $fclose(fo); $finish;
    end
endmodule
