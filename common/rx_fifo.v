`timescale 1ns/1ns
// Receive FIFO: 2048 x 32 bit, write side 25 MHz (MII), read side 50 MHz (engine).
// Same Altera dcfifo settings as the Embedfire example, but empty/full are brought out.
module rx_fifo (
    input   wire            aclr,
    input   wire            wrclk,
    input   wire            wrreq,
    input   wire    [31:0]  data,
    output  wire            wrfull,
    input   wire            rdclk,
    input   wire            rdreq,
    output  wire    [31:0]  q,
    output  wire            rdempty
);
    dcfifo dcfifo_component (
        .aclr (aclr), .data (data), .rdclk (rdclk), .rdreq (rdreq), .wrclk (wrclk), .wrreq (wrreq),
        .q (q), .rdempty (rdempty), .rdfull (), .rdusedw (), .wrempty (), .wrfull (wrfull), .wrusedw ());
    defparam
        dcfifo_component.intended_device_family = "Cyclone IV E",
        dcfifo_component.lpm_numwords = 2048,
        dcfifo_component.lpm_showahead = "OFF",
        dcfifo_component.lpm_type = "dcfifo",
        dcfifo_component.lpm_width = 32,
        dcfifo_component.lpm_widthu = 11,
        dcfifo_component.overflow_checking = "ON",
        dcfifo_component.rdsync_delaypipe = 4,
        dcfifo_component.read_aclr_synch = "OFF",
        dcfifo_component.underflow_checking = "ON",
        dcfifo_component.use_eab = "ON",
        dcfifo_component.write_aclr_synch = "OFF",
        dcfifo_component.wrsync_delaypipe = 4;
endmodule
