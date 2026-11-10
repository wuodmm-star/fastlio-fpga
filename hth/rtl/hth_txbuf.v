`timescale 1ns/1ns
// Transmit buffer: 128 x 32 bit RAM. Written by the engine (50 MHz), read by ip_send (25 MHz).
// Read side behaves like the FIFO it replaces (show-ahead OFF): q changes one clock after rdreq.
// rst_ptr (one 25 MHz clock, at the start of every reply) rewinds the read pointer to word 0.
module hth_txbuf (
    input   wire            wclk,
    input   wire            we,
    input   wire    [6:0]   waddr,
    input   wire    [31:0]  wdata,
    input   wire            rclk,
    input   wire            rst_ptr,
    input   wire            rdreq,
    output  reg     [31:0]  q
);
    reg [31:0] mem [0:127];
    reg [6:0]  rptr;
    always @(posedge wclk) if (we) mem[waddr] <= wdata;
    always @(posedge rclk) if (rdreq) q <= mem[rptr];
    always @(posedge rclk)
        if (rst_ptr)    rptr <= 7'd0;
        else if (rdreq) rptr <= rptr + 7'd1;
endmodule
