`timescale 1ns/1ns
// Engine-only test: words from engine_words.mem are pushed into the FIFO (as fast as the FIFO accepts),
// every reply the engine writes into the transmit buffer is printed as one hex line to engine_out.txt.
module tb_engine;
    reg clk = 0, rst_n = 0;
    always #10 clk = ~clk;                       // 50 MHz
    reg  [31:0] words [0:65535];
    integer nwords, wi = 0, fo, i, nrep = 0, nexp;
    reg  wrreq = 0; reg [31:0] wdata = 0;
    wire [31:0] q; wire empty, full, rdreq;
    wire tx_we; wire [6:0] tx_addr; wire [31:0] tx_wdata; wire tgl;
    reg  tgl_d = 0;
    reg  [31:0] rep [0:84];

    rx_fifo fifo (.aclr(~rst_n), .wrclk(clk), .wrreq(wrreq), .data(wdata), .wrfull(full),
                  .rdclk(clk), .rdreq(rdreq), .q(q), .rdempty(empty));
    hth_engine dut (.clk(clk), .rst_n(rst_n), .fifo_q(q), .fifo_empty(empty), .fifo_rdreq(rdreq), .fifo_ovf(1'b0), .diag(32'h0),
                    .tx_we(tx_we), .tx_addr(tx_addr), .tx_wdata(tx_wdata), .result_tgl(tgl));

    always @(posedge clk) if (tx_we) rep[tx_addr] <= tx_wdata;
    always @(posedge clk) begin
        tgl_d <= tgl;
        if (tgl != tgl_d) begin                  // reply finished: dump it
            for (i = 0; i < 85; i = i + 1) $fwrite(fo, "%08x", rep[i]);
            $fwrite(fo, "\n"); nrep = nrep + 1;
        end
    end
    // feeder
    always @(negedge clk) begin
        if (rst_n && wi < nwords && !full) begin wrreq <= 1; wdata <= words[wi]; wi <= wi + 1; end
        else wrreq <= 0;
    end
    initial begin
        if (!$value$plusargs("N=%d", nwords)) nwords = 0;
        if (!$value$plusargs("R=%d", nexp)) nexp = 1;
        $readmemh("engine_words.mem", words);
        fo = $fopen("engine_out.txt", "w");
        #105 rst_n = 1;
        wait (wi == nwords);
        fork
            wait (nrep == nexp);
            #20000000;                           // 20 ms safety timeout
        join_any
        #1000;
        $display("fed %0d words, got %0d replies, %0t ns", wi, nrep, $time);
        $fclose(fo); $finish;
    end
endmodule
