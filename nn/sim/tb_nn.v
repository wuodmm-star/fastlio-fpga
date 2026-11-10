`timescale 1ns/1ps
// nn_core + sdram_rw + Micron SDRAM model: run the ops in nn_ops.mem, print every search result to nn_out.txt
module tb_nn;
    parameter CFG = 16'h0000;       // {gbits (0 = 16), keep in voxels}: iverilog -P tb_nn.CFG=...
    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;
    reg sdram_clk = 0;
    always @(clk) sdram_clk <= #9.167 clk;
    wire init_done, ready, ack, done, wr_take, rd_valid, dq_oe;
    wire [8:0] wr_idx; wire [15:0] rd_data, dq_out, wr_data; wire [3:0] cmd; wire [1:0] ba, dqm; wire [12:0] sa;
    wire [15:0] dq = dq_oe ? dq_out : 16'hzzzz;
    wire sd_req, sd_we; wire [23:0] sd_addr; wire [8:0] sd_len;
    sdram_rw #(.RD_LAT(3)) ctl (.clk(clk), .rst_n(rst_n), .init_done(init_done), .req(sd_req), .we(sd_we), .addr(sd_addr),
        .len(sd_len), .ready(ready), .ack(ack), .done(done), .wr_idx(wr_idx), .wr_take(wr_take), .wr_data(wr_data),
        .rd_valid(rd_valid), .rd_data(rd_data), .cmd(cmd), .ba(ba), .sa(sa), .dqm(dqm), .dq_out(dq_out),
        .dq_oe(dq_oe), .dq_in(dq));
    mt48lc16m16a2 mem (.Dq(dq), .Addr(sa), .Ba(ba), .Clk(sdram_clk), .Cke(1'b1), .Cs_n(cmd[3]), .Ras_n(cmd[2]),
        .Cas_n(cmd[1]), .We_n(cmd[0]), .Dqm(dqm));

    reg op_valid = 0, op_search = 0, op_ds = 0; reg [31:0] op_x = 0, op_y = 0, op_z = 0;
    wire op_ready, res_valid; wire [2:0] res_found; wire [479:0] res_pts; wire [31:0] drop_cnt, evict_cnt;
    nn_core #(.DEF_CFG(CFG)) core (.clk(clk), .rst_n(rst_n), .op_valid(op_valid), .op_ready(op_ready), .op_search(op_search), .op_ds(op_ds),
        .op_x(op_x), .op_y(op_y), .op_z(op_z), .clr_req(1'b0), .clr_cfg(16'd0), .evict_cnt(evict_cnt), .res_valid(res_valid), .res_found(res_found), .res_pts(res_pts),
        .drop_cnt(drop_cnt), .sd_ack(ack), .sd_done(done), .sd_req(sd_req), .sd_we(sd_we), .sd_addr(sd_addr),
        .sd_len(sd_len), .sd_wr_idx(wr_idx), .sd_wr_data(wr_data), .sd_rd_valid(rd_valid), .sd_rd_data(rd_data));

    reg [31:0] ops [0:5*200000-1];
    integer nops, i, k, fo, nres = 0; real t0;
    always @(posedge clk) if (res_valid) begin
        $fwrite(fo, "%0d", res_found);
        for (k = 0; k < res_found; k = k + 1)
            $fwrite(fo, " %08x %08x %08x", res_pts[96*k+64 +: 32], res_pts[96*k+32 +: 32], res_pts[96*k +: 32]);
        $fwrite(fo, "\n"); nres = nres + 1;
    end
    initial begin
        if (!$value$plusargs("N=%d", nops)) nops = 0;
        $readmemh("nn_ops.mem", ops);
        fo = $fopen("nn_out.txt", "w");
        #100 rst_n = 1;
        wait (init_done); t0 = $realtime;
        for (i = 0; i < nops; i = i + 1) begin
            @(negedge clk); while (!op_ready) @(negedge clk);
            op_valid = 1; op_search = ops[5*i][0]; op_ds = ops[5*i+1][0]; op_x = ops[5*i+2]; op_y = ops[5*i+3]; op_z = ops[5*i+4];
            @(negedge clk); op_valid = 0;
        end
        @(negedge clk); while (!op_ready) @(negedge clk);
        #1000;
        $display("%0d ops, %0d search results, dropped %0d, forgotten %0d, %.1f us after init", nops, nres, drop_cnt, evict_cnt, ($realtime - t0) / 1000.0);
        $fclose(fo); $finish;
    end
endmodule
