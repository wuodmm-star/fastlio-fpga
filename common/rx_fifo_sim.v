`timescale 1ns/1ns
// Simulation stand-in for rx_fifo (dcfifo): two clocks, show-ahead OFF, empty/full flags without sync delay.
module rx_fifo (input aclr, input wrclk, input wrreq, input [31:0] data, output wrfull,
                input rdclk, input rdreq, output reg [31:0] q, output rdempty);
    reg [31:0] mem [0:2047];
    reg [11:0] wp = 0, rp = 0;
    assign rdempty = (wp == rp);
    assign wrfull  = ((wp - rp) == 12'd2048);
    always @(posedge wrclk or posedge aclr) if (aclr) wp <= 0; else if (wrreq && !wrfull) begin mem[wp[10:0]] <= data; wp <= wp + 1; end
    always @(posedge rdclk or posedge aclr) if (aclr) begin rp <= 0; q <= 0; end
        else if (rdreq && !rdempty) begin q <= mem[rp[10:0]]; rp <= rp + 1; end
endmodule
