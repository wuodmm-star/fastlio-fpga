`timescale 1ns/1ps
// per-search cycle accounting for nn_core (bound to tb_nn); counts clocks, not $time
module probe_perf;
    integer cyc = 0, c0 = 0, cyc_sum = 0, nsearch = 0, nreq = 0, nhdr = 0, nhit = 0, npts = 0, sd_busy = 0;
    reg in_search = 0;
    always @(posedge tb_nn.clk) begin
        cyc = cyc + 1;
        if (tb_nn.core.st == 5'd0 && tb_nn.op_valid && tb_nn.op_search) begin c0 = cyc; in_search = 1; end
        else if (in_search) begin
            if (tb_nn.ctl.ack) nreq = nreq + 1;
            if (tb_nn.ctl.st != 4'd4) sd_busy = sd_busy + 1;
            if (tb_nn.core.st == 5'd7) nhdr = nhdr + 1;
            if (tb_nn.core.st == 5'd10) nhit = nhit + 1;
            if (tb_nn.core.st == 5'd12) npts = npts + 1;
            if (tb_nn.core.res_valid) begin
                cyc_sum = cyc_sum + (cyc - c0); nsearch = nsearch + 1; in_search = 0;
                if (nsearch <= 3) $display("search %0d: %0d cycles", nsearch, cyc - c0);
            end
        end
    end
    final $display("PERF searches %0d | %.0f cycles each | per search: %.1f SDRAM requests, %.1f groups, %.1f hits, %.1f points | SDRAM busy %.0f %%",
        nsearch, cyc_sum * 1.0 / nsearch, nreq * 1.0 / nsearch, nhdr * 1.0 / nsearch, nhit * 1.0 / nsearch, npts * 1.0 / nsearch,
        100.0 * sd_busy / cyc_sum);
endmodule
