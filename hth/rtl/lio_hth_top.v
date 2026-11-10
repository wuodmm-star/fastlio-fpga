`timescale 1ns/1ns
////////////////////////////////////////////////////////////////////////
// lio_hth_top : 征途Pro (EP4CE10F17C8) top level
//
//   RMII PHY --rmii_to_mii--> eth_udp_mii (ip_receive) --25 MHz--> rx_fifo --50 MHz--> hth_engine
//   hth_engine --50 MHz--> hth_txbuf --25 MHz--> eth_udp_mii (ip_send) --mii_to_rmii--> RMII PHY
//
// Ethernet/UDP blocks are the Embedfire example (60_ethernet_udp_rmii) with our
// reply-to-sender and fragment-drop changes. Pins are the same as the example.
////////////////////////////////////////////////////////////////////////
module  lio_hth_top
(
    input   wire            sys_rst_n       ,   // key, active low
    input   wire            eth_clk         ,   // 50 MHz RMII reference clock from the PHY
    input   wire            eth_rxdv_r      ,
    input   wire    [1:0]   eth_rx_data_r   ,
    output  wire            eth_tx_en_r     ,
    output  wire    [1:0]   eth_tx_data_r   ,
    output  wire            eth_rst_n
);

parameter   BOARD_MAC   = 48'h12_34_56_78_9a_bc ;
parameter   BOARD_IP    = 32'hC0_A8_89_EA       ;   // 192.168.137.234
parameter   BOARD_PORT  = 16'd1234              ;
localparam  REPLY_BYTES = 16'd340               ;   // 85 words

wire            rec_end, rec_en, send_end, read_data_req, sfd_ok;
wire   [31:0]   rec_data, send_data;
wire   [15:0]   rec_data_num;
wire            eth_rxdv, eth_tx_en;
wire    [3:0]   eth_rx_data, eth_tx_data;
reg             clk_25m;

// 25 MHz MII clock = 50 MHz / 2 (falling edge), exactly as in the example
always @(negedge eth_clk or negedge sys_rst_n)
    if (!sys_rst_n) clk_25m <= 1'b0;
    else            clk_25m <= ~clk_25m;

rmii_to_mii rmii_to_mii_inst (
    .eth_rmii_clk (eth_clk), .eth_mii_clk (clk_25m), .sys_rst_n (sys_rst_n),
    .rx_dv (eth_rxdv_r), .rx_data (eth_rx_data_r), .eth_rx_dv (eth_rxdv), .eth_rx_data (eth_rx_data));

mii_to_rmii mii_to_rmii_inst (
    .eth_mii_clk (clk_25m), .eth_rmii_clk (eth_clk), .sys_rst_n (sys_rst_n),
    .tx_dv (eth_tx_en), .tx_data (eth_tx_data), .eth_tx_dv (eth_tx_en_r), .eth_tx_data (eth_tx_data_r));

//---------------------------------------------------------------- reply trigger: 50 MHz toggle -> 25 MHz pulse
wire            result_tgl;
reg     [2:0]   tgl_sync;           // two flip-flops for metastability + one for edge detection
always @(posedge clk_25m or negedge sys_rst_n)
    if (!sys_rst_n) tgl_sync <= 3'b000;
    else            tgl_sync <= {tgl_sync[1:0], result_tgl};
wire            send_pulse = tgl_sync[2] ^ tgl_sync[1];

eth_udp_mii #(.BOARD_MAC (BOARD_MAC), .BOARD_IP (BOARD_IP), .BOARD_PORT (BOARD_PORT)) eth_udp_mii_inst (
    .eth_rx_clk (clk_25m), .sys_rst_n (sys_rst_n), .eth_rxdv (eth_rxdv), .eth_rx_data (eth_rx_data),
    .eth_tx_clk (clk_25m),
    .send_en (send_pulse), .send_data (send_data), .send_data_num (REPLY_BYTES),
    .send_end (send_end), .read_data_req (read_data_req),
    .rec_end (rec_end), .rec_en (rec_en), .rec_data (rec_data), .rec_data_num (rec_data_num),
    .eth_tx_en (eth_tx_en), .eth_tx_data (eth_tx_data), .eth_rst_n (eth_rst_n), .sfd_ok (sfd_ok));

//---------------------------------------------------------------- receive diagnostics (25 MHz, free running, mod 1024)
// starts : frames seen on the wire (MII rx_dv rising edges)  | sfd : preamble + SFD found | acc : accepted UDP to us
// Read by the engine at END time, when no frame is arriving (quasi-static, so no synchroniser needed).
reg             rxdv_d;
reg     [9:0]   cnt_start, cnt_sfd, cnt_acc;
always @(posedge clk_25m or negedge sys_rst_n)
    if (!sys_rst_n) begin rxdv_d <= 1'b0; cnt_start <= 10'd0; cnt_sfd <= 10'd0; cnt_acc <= 10'd0; end
    else begin
        rxdv_d <= eth_rxdv;
        if (eth_rxdv && !rxdv_d) cnt_start <= cnt_start + 10'd1;
        if (sfd_ok)              cnt_sfd   <= cnt_sfd + 10'd1;
        if (rec_end)             cnt_acc   <= cnt_acc + 10'd1;
    end
wire    [31:0]  diag_word = {2'b00, cnt_start, cnt_sfd, cnt_acc};

//---------------------------------------------------------------- receive FIFO (25 MHz in, 50 MHz out)
wire    [31:0]  fifo_q;
wire            fifo_empty, fifo_rdreq, fifo_full;
rx_fifo rx_fifo_inst (
    .aclr (~sys_rst_n), .wrclk (clk_25m), .wrreq (rec_en), .data (rec_data), .wrfull (fifo_full),
    .rdclk (eth_clk), .rdreq (fifo_rdreq), .q (fifo_q), .rdempty (fifo_empty));

// sticky overflow flag (25 MHz) -> synchronised to 50 MHz; it only ever goes 0 -> 1, so 2 flip-flops are enough
reg             ovf_25;
reg     [1:0]   ovf_sync;
always @(posedge clk_25m or negedge sys_rst_n)
    if (!sys_rst_n)               ovf_25 <= 1'b0;
    else if (rec_en && fifo_full) ovf_25 <= 1'b1;
always @(posedge eth_clk or negedge sys_rst_n)
    if (!sys_rst_n) ovf_sync <= 2'b00;
    else            ovf_sync <= {ovf_sync[0], ovf_25};

//---------------------------------------------------------------- engine + transmit buffer
wire            tx_we;
wire    [6:0]   tx_addr;
wire    [31:0]  tx_wdata;

hth_engine hth_engine_inst (
    .clk (eth_clk), .rst_n (sys_rst_n),
    .fifo_q (fifo_q), .fifo_empty (fifo_empty), .fifo_rdreq (fifo_rdreq), .fifo_ovf (ovf_sync[1]), .diag (diag_word),
    .tx_we (tx_we), .tx_addr (tx_addr), .tx_wdata (tx_wdata), .result_tgl (result_tgl));

hth_txbuf hth_txbuf_inst (
    .wclk (eth_clk), .we (tx_we), .waddr (tx_addr), .wdata (tx_wdata),
    .rclk (clk_25m), .rst_ptr (send_pulse), .rdreq (read_data_req), .q (send_data));

endmodule
