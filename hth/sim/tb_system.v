`timescale 1ns/1ns
// Whole-chip test: real Ethernet frames on the RMII pins in, frames on the RMII pins out.
module tb_system;
    reg eth_clk = 0, sys_rst_n = 1, rxdv = 0; reg [1:0] rxd = 0;   // reset is pulsed low below (needs a real 1->0 edge)
    wire tx_en; wire [1:0] txd; wire phy_rst_n;
    always #10 eth_clk = ~eth_clk;
    lio_hth_top dut (.sys_rst_n(sys_rst_n), .eth_clk(eth_clk), .eth_rxdv_r(rxdv), .eth_rx_data_r(rxd),
                     .eth_tx_en_r(tx_en), .eth_tx_data_r(txd), .eth_rst_n(phy_rst_n));
    reg [7:0] stim [0:65535]; reg [15:0] lens [0:255];
    integer nf, f, b, k, base, fo, ntx = 0;
    task send_frame(input integer start, input integer len);
        begin
            for (b = 0; b < len; b = b + 1)
                for (k = 0; k < 4; k = k + 1) begin
                    @(negedge eth_clk); rxdv = 1; rxd = stim[start + b] >> (2 * k);
                end
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
        $readmemh("sys_stim.mem", stim); $readmemh("sys_lens.mem", lens);
        fo = $fopen("sys_out.txt", "w");
        #50 sys_rst_n = 0; #150 sys_rst_n = 1; #2000;
        base = 0;
        for (f = 0; f < nf; f = f + 1) begin
            send_frame(base, lens[f]);
            base = base + lens[f];
            #2000;                                // 2 us gap between frames
        end
        #500000;
        $display("sent %0d frames, board transmitted %0d frames", nf, ntx);
        $fclose(fo); $finish;
    end
endmodule
