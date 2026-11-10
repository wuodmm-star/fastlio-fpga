`timescale 1ns/1ps
// Random write / read-back test of sdram_rw against the Micron mt48lc16m16a2 model.
// Addresses are limited to rows 0..63 so a shadow copy of the memory fits in the testbench.
module tb_sdram;
    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;                                   // 100 MHz
    reg sdram_clk = 0;                                      // SDRAM clock leads by 0.833 ns (-30 deg, as the Embedfire PLL)
    always @(clk) sdram_clk <= #9.167 clk;                  // transport delay (a continuous-assign delay would swallow the 5 ns pulses)
    wire init_done, ready, done, wr_take, rd_valid, dq_oe;
    wire [8:0] wr_idx; wire [15:0] rd_data, dq_out; wire [3:0] cmd; wire [1:0] ba, dqm; wire [12:0] sa;
    wire [15:0] dq = dq_oe ? dq_out : 16'hzzzz;
    reg req = 0, we = 0; reg [23:0] addr = 0; reg [8:0] len = 0;
    reg [15:0] seed = 0;
    wire [15:0] wr_data = (addr[15:0] + wr_idx) ^ (seed * 16'h9E37);  // data pattern: function of address + request

    sdram_rw #(.RD_LAT(`RDLAT)) dut (.clk(clk), .rst_n(rst_n), .init_done(init_done), .req(req), .we(we), .addr(addr),
        .len(len), .ready(ready), .done(done), .wr_idx(wr_idx), .wr_take(wr_take), .wr_data(wr_data),
        .rd_valid(rd_valid), .rd_data(rd_data), .cmd(cmd), .ba(ba), .sa(sa), .dqm(dqm), .dq_out(dq_out),
        .dq_oe(dq_oe), .dq_in(dq));
    mt48lc16m16a2 mem (.Dq(dq), .Addr(sa), .Ba(ba), .Clk(sdram_clk), .Cke(1'b1), .Cs_n(cmd[3]), .Ras_n(cmd[2]),
        .Cas_n(cmd[1]), .We_n(cmd[0]), .Dqm(dqm));

    reg [15:0] shadow [0:131071];                           // rows 0..63 x 4 banks x 512 columns
    reg        known  [0:131071];
    integer i, n_wr = 0, n_rd = 0, n_words = 0, n_bad = 0, n_unknown = 0, rcount;
    always @(posedge clk) if (wr_take) begin shadow[addr[16:0] + wr_idx] <= wr_data; known[addr[16:0] + wr_idx] <= 1'b1; end
    always @(posedge clk) if (rd_valid) begin
        if (!known[addr[16:0] + rcount]) n_unknown = n_unknown + 1;
        else if (rd_data !== shadow[addr[16:0] + rcount]) begin
            n_bad = n_bad + 1;
            if (n_bad <= 5) $display("BAD read addr %h word %0d: got %h expected %h", addr, rcount, rd_data, shadow[addr[16:0] + rcount]);
        end
        rcount = rcount + 1; n_words = n_words + 1;
    end
    task run(input w, input [23:0] a, input [8:0] l);
        begin
            @(negedge clk); while (!ready) @(negedge clk);
            req = 1; we = w; addr = a; len = l; seed = seed + 1; rcount = 0;
            @(negedge clk); req = 0;
            while (!done) @(negedge clk);
            if (!w && rcount != l) begin n_bad = n_bad + 1; $display("read %0d words, expected %0d", rcount, l); end
        end
    endtask
    reg [8:0] col, l;
    initial begin
        for (i = 0; i < 131072; i = i + 1) known[i] = 0;
        #100 rst_n = 1;
        wait (init_done);
        $display("init done at %0t ns", $time);
        // first fill a few complete rows, then random mixed traffic
        for (i = 0; i < 16; i = i + 1) run(1, {7'd0, i[5:0], 2'd0, 9'd0} | (i << 9), 9'd256);
        for (i = 0; i < 4000; i = i + 1) begin
            col = $random; l = 1 + ($random & 63);
            if (col + l > 512) col = 512 - l;
            if (($random & 1) == 0) begin run(1, {7'd0, 2'b00, 4'($random), 2'($random), col}, l); n_wr = n_wr + 1; end
            else                    begin run(0, {7'd0, 2'b00, 4'($random), 2'($random), col}, l); n_rd = n_rd + 1; end
        end
        $display("%0d writes, %0d reads, %0d words checked, %0d wrong, %0d never written, time %0t ns",
                 n_wr, n_rd, n_words, n_bad, n_unknown, $time);
        $finish;
    end
endmodule
