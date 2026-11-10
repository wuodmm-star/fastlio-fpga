`timescale 1ns/1ps
////////////////////////////////////////////////////////////////////////
// lio_nn_top : FAST-LIO nearest-neighbour search on 征途Pro (EP4CE10F17C8)
//
//   RMII PHY -> ip_receive (25 MHz) -> rx_fifo -> nn_engine (100 MHz) <-> nn_core <-> sdram_rw <-> SDRAM (32 MB)
//   nn_engine -> nn_txbuf (2 x 512 words) -> ip_send (25 MHz) -> RMII PHY
// Clocks: eth_clk 50 MHz from the PHY (Ethernet side); sys_clk 50 MHz board oscillator -> PLL -> 100 MHz logic and
// 100 MHz SDRAM clock shifted by -833 ps (Embedfire clk_gen settings).
////////////////////////////////////////////////////////////////////////
module lio_nn_top
(
    input   wire            sys_clk,
    input   wire            sys_rst_n,
    input   wire            eth_clk,
    input   wire            eth_rxdv_r,
    input   wire    [1:0]   eth_rx_data_r,
    output  wire            eth_tx_en_r,
    output  wire    [1:0]   eth_tx_data_r,
    output  wire            eth_rst_n,
    // SDRAM
    output  wire            sdram_clk,
    output  wire            sdram_cke,
    output  wire            sdram_cs_n,
    output  wire            sdram_ras_n,
    output  wire            sdram_cas_n,
    output  wire            sdram_we_n,
    output  wire    [1:0]   sdram_ba,
    output  wire    [12:0]  sdram_addr,
    output  wire    [1:0]   sdram_dqm,
    inout   wire    [15:0]  sdram_dq
);

parameter   BOARD_MAC   = 48'h12_34_56_78_9a_bc;
parameter   BOARD_IP    = 32'hC0_A8_89_EA;          // 192.168.137.234
parameter   BOARD_PORT  = 16'd1234;
parameter   NCLR        = 65536;                    // groups cleared after reset (all)

//---------------------------------------------------------------- clocks / resets
wire            clk100, pll_locked, c0_unused;
clk_gen clk_gen_inst (.areset(~sys_rst_n), .inclk0(sys_clk), .c0(c0_unused), .c1(clk100), .c2(sdram_clk), .locked(pll_locked));
wire            rst100_n = sys_rst_n & pll_locked;

reg             clk_25m;
always @(negedge eth_clk or negedge sys_rst_n)
    if (!sys_rst_n) clk_25m <= 1'b0; else clk_25m <= ~clk_25m;

//---------------------------------------------------------------- Ethernet (25 MHz MII side)
wire            rec_end, rec_en, send_end, read_data_req, sfd_ok, eth_rxdv, eth_tx_en;
wire   [31:0]   rec_data, send_data;
wire   [15:0]   rec_data_num;
wire    [3:0]   eth_rx_data, eth_tx_data;

rmii_to_mii rmii_to_mii_inst (.eth_rmii_clk (eth_clk), .eth_mii_clk (clk_25m), .sys_rst_n (sys_rst_n),
    .rx_dv (eth_rxdv_r), .rx_data (eth_rx_data_r), .eth_rx_dv (eth_rxdv), .eth_rx_data (eth_rx_data));
mii_to_rmii mii_to_rmii_inst (.eth_mii_clk (clk_25m), .eth_rmii_clk (eth_clk), .sys_rst_n (sys_rst_n),
    .tx_dv (eth_tx_en), .tx_data (eth_tx_data), .eth_tx_dv (eth_tx_en_r), .eth_tx_data (eth_tx_data_r));

// reply announce: 100 MHz toggle -> 25 MHz pulse; the reply length is stable while the toggle travels
wire            result_tgl;
wire   [15:0]   reply_bytes;
reg     [2:0]   tgl_sync;
always @(posedge clk_25m or negedge sys_rst_n)
    if (!sys_rst_n) tgl_sync <= 3'b000; else tgl_sync <= {tgl_sync[1:0], result_tgl};
wire            send_pulse = tgl_sync[2] ^ tgl_sync[1];
// Replies are queued here: a reply can be announced while the previous one is still on the wire (ip_send ignores a
// start pulse while busy - found when the PC kept 4 packets in flight and a short reply followed a long one).
reg             rd_half;                            // which half of the transmit buffer is being sent
reg     [1:0]   q_cnt;                              // announced replies not started yet (<= 2 by the engine's rule)
reg             tx_busy, send_go;
wire            launch = (q_cnt != 2'd0) && !tx_busy && !send_go;
always @(posedge clk_25m or negedge sys_rst_n)
    if (!sys_rst_n) begin rd_half <= 1'b1; q_cnt <= 2'd0; tx_busy <= 1'b0; send_go <= 1'b0; end
    else begin
        send_go <= launch;
        if (launch) begin tx_busy <= 1'b1; rd_half <= ~rd_half; end
        else if (send_end) tx_busy <= 1'b0;
        q_cnt <= q_cnt + (send_pulse ? 2'd1 : 2'd0) - (launch ? 2'd1 : 2'd0);
    end
// reply_bytes changes together with result_tgl and stays put for microseconds; ip_send latches it at send_pulse,
// which comes 2-3 clocks after the toggle (a register here would hand ip_send the PREVIOUS length - bug found in sim)
wire   [15:0]   send_bytes = reply_bytes;
// "reply sent" : 25 MHz send_end pulse -> toggle -> synchronised into the 100 MHz engine
reg             sent_tgl25;
always @(posedge clk_25m or negedge sys_rst_n)
    if (!sys_rst_n) sent_tgl25 <= 1'b0; else if (send_end) sent_tgl25 <= ~sent_tgl25;
reg     [1:0]   sent_sync;
always @(posedge clk100 or negedge rst100_n)
    if (!rst100_n) sent_sync <= 2'b00; else sent_sync <= {sent_sync[0], sent_tgl25};

eth_udp_mii #(.BOARD_MAC (BOARD_MAC), .BOARD_IP (BOARD_IP), .BOARD_PORT (BOARD_PORT)) eth_udp_mii_inst (
    .eth_rx_clk (clk_25m), .sys_rst_n (sys_rst_n), .eth_rxdv (eth_rxdv), .eth_rx_data (eth_rx_data),
    .eth_tx_clk (clk_25m), .send_en (send_go), .send_data (send_data), .send_data_num (send_bytes),
    .send_end (send_end), .read_data_req (read_data_req),
    .rec_end (rec_end), .rec_en (rec_en), .rec_data (rec_data), .rec_data_num (rec_data_num),
    .eth_tx_en (eth_tx_en), .eth_tx_data (eth_tx_data), .eth_rst_n (eth_rst_n), .sfd_ok (sfd_ok));

//---------------------------------------------------------------- receive FIFO (25 MHz in, 100 MHz out)
wire    [31:0]  fifo_q;
wire            fifo_empty, fifo_rdreq, fifo_full;
rx_fifo rx_fifo_inst (.aclr (~sys_rst_n), .wrclk (clk_25m), .wrreq (rec_en), .data (rec_data), .wrfull (fifo_full),
    .rdclk (clk100), .rdreq (fifo_rdreq), .q (fifo_q), .rdempty (fifo_empty));

//---------------------------------------------------------------- engine, core, SDRAM controller (100 MHz)
wire            tx_we;
wire    [9:0]   tx_addr;
wire    [31:0]  tx_wdata;
wire   [15:0]   bad_hdr_cnt;
wire            op_ready, op_valid, op_search, op_ds, res_valid, clr_req;
wire   [31:0]   op_x, op_y, op_z, drop_cnt;
wire    [2:0]   res_found;
wire  [479:0]   res_pts;

nn_engine nn_engine_inst (.clk (clk100), .rst_n (rst100_n),
    .fifo_q (fifo_q), .fifo_empty (fifo_empty), .fifo_rdreq (fifo_rdreq),
    .tx_we (tx_we), .tx_addr (tx_addr), .tx_wdata (tx_wdata), .result_tgl (result_tgl), .reply_bytes (reply_bytes),
    .sent_tgl (sent_sync[1]), .bad_hdr_cnt (bad_hdr_cnt),
    .op_ready (op_ready), .op_valid (op_valid), .op_search (op_search), .op_ds (op_ds),
    .op_x (op_x), .op_y (op_y), .op_z (op_z), .clr_req (clr_req),
    .res_valid (res_valid), .res_found (res_found), .res_pts (res_pts));

wire            sd_init_done, sd_ready, sd_ack, sd_done, sd_req, sd_we, sd_wr_take, sd_rd_valid, sd_dq_oe;
wire   [23:0]   sd_addr;
wire    [8:0]   sd_len, sd_wr_idx;
wire   [15:0]   sd_wr_data, sd_rd_data, sd_dq_out;
wire    [3:0]   sd_cmd;

nn_core #(.NCLR (NCLR)) nn_core_inst (.clk (clk100), .rst_n (rst100_n & sd_init_done),
    .op_valid (op_valid), .op_ready (op_ready), .op_search (op_search), .op_ds (op_ds),
    .op_x (op_x), .op_y (op_y), .op_z (op_z), .clr_req (clr_req),
    .res_valid (res_valid), .res_found (res_found), .res_pts (res_pts),
    .drop_cnt (drop_cnt), .sd_ack (sd_ack), .sd_done (sd_done), .sd_req (sd_req), .sd_we (sd_we),
    .sd_addr (sd_addr), .sd_len (sd_len), .sd_wr_idx (sd_wr_idx), .sd_wr_data (sd_wr_data),
    .sd_rd_valid (sd_rd_valid), .sd_rd_data (sd_rd_data));

sdram_rw #(.RD_LAT (3)) sdram_rw_inst (.clk (clk100), .rst_n (rst100_n), .init_done (sd_init_done),
    .req (sd_req), .we (sd_we), .addr (sd_addr), .len (sd_len), .ready (sd_ready), .ack (sd_ack), .done (sd_done),
    .wr_idx (sd_wr_idx), .wr_take (sd_wr_take), .wr_data (sd_wr_data), .rd_valid (sd_rd_valid), .rd_data (sd_rd_data),
    .cmd (sd_cmd), .ba (sdram_ba), .sa (sdram_addr), .dqm (sdram_dqm), .dq_out (sd_dq_out), .dq_oe (sd_dq_oe),
    .dq_in (sdram_dq));

assign  sdram_cke = 1'b1;
assign  {sdram_cs_n, sdram_ras_n, sdram_cas_n, sdram_we_n} = sd_cmd;
assign  sdram_dq  = sd_dq_oe ? sd_dq_out : 16'hzzzz;

//---------------------------------------------------------------- transmit buffer (written at 100 MHz, read at 25 MHz)
reg     [31:0]  txmem [0:1023];
reg     [8:0]   txptr;
reg     [31:0]  txq;
always @(posedge clk100) if (tx_we) txmem[tx_addr] <= tx_wdata;
always @(posedge clk_25m) if (read_data_req) txq <= txmem[{rd_half, txptr}];
always @(posedge clk_25m)
    if (send_go) txptr <= 9'd0;
    else if (read_data_req) txptr <= txptr + 9'd1;
assign  send_data = txq;

endmodule
