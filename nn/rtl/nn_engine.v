`timescale 1ns/1ps
////////////////////////////////////////////////////////////////////////
// nn_engine : UDP payload <-> nn_core, 100 MHz
//
// Request payload : 0xA5 | 0x11 | count, then count ops x 4 words: {.., ds, search}, x, y, z  (Q16.16)
//                   (client limit: <= 72 ops and <= 18 searches per packet, so the reply fits one datagram)
//                   0xA5 | 0x13 | cfg : clear the map; cfg = {gbits (0 = 16), keep[11:0] in voxels} -> empty reply
//                   0xA5 | 0x14       : statistics -> reply 0xA5120000, dropped adds, forgotten voxels
// Reply payload   : 0xA5 | 0x12 | number of searches, then per search 16 words: found, 5 x (x, y, z)
//                   (unused entries are 0). Every request gets a reply, so the client knows it was processed.
// Replies are built in one half of a 2 x 512-word transmit buffer while the Ethernet side may still be sending
// the other half; at most one reply may be waiting to be sent.
////////////////////////////////////////////////////////////////////////
module nn_engine
(
    input   wire            clk,
    input   wire            rst_n,
    // receive FIFO (show-ahead OFF)
    input   wire    [31:0]  fifo_q,
    input   wire            fifo_empty,
    output  wire            fifo_rdreq,
    // transmit buffer (2 x 512 words) + handshake with the 25 MHz side
    output  reg             tx_we,
    output  reg     [9:0]   tx_addr,
    output  reg     [31:0]  tx_wdata,
    output  reg             result_tgl,     // toggles when a reply is ready (in half `cur_buf` before the toggle)
    output  reg     [15:0]  reply_bytes,    // stable until the next toggle
    input   wire            sent_tgl,       // toggles when the Ethernet side finished sending a reply (synchronised)
    output  reg     [15:0]  bad_hdr_cnt,
    // nn_core
    input   wire            op_ready,
    output  reg             op_valid,
    output  reg             op_search,
    output  reg             op_ds,
    output  reg     [31:0]  op_x, op_y, op_z,
    output  reg             clr_req,
    output  reg     [15:0]  clr_cfg,
    input   wire    [31:0]  drop_cnt,
    input   wire    [31:0]  evict_cnt,
    input   wire            res_valid,
    input   wire    [2:0]   res_found,
    input   wire    [479:0] res_pts
);

localparam  S_HDR = 4'd0, S_OPW = 4'd1, S_ISSUE = 4'd2, S_WAITC = 4'd3, S_RES = 4'd4, S_FIN = 4'd5, S_SEND = 4'd6,
            S_CLRW = 4'd7, S_CLRX = 4'd8, S_ST0 = 4'd9, S_ST1 = 4'd10;

reg     [3:0]   st;
reg     [1:0]   clr_dly;
reg             stat;                       // the reply being built is a statistics reply (2 extra words)
reg     [15:0]  nops, ns;
reg     [1:0]   w;
reg     [31:0]  opw0;
reg             cur_buf;
reg     [4:0]   ridx;
reg     [2:0]   r_found;
reg     [479:0] r_pts;
reg     [7:0]   ann_cnt, sent_cnt;          // replies announced / replies the Ethernet side has finished sending
reg             sent_d;
wire    [7:0]   pending = ann_cnt - sent_cnt;

// a new packet is only started when at most one reply is still waiting (the other buffer half)
wire            want_word = ((st == S_HDR) && (pending <= 8'd1)) || (st == S_OPW);
reg             rd_pending;
assign  fifo_rdreq = want_word && !fifo_empty && !rd_pending;
wire            word_valid = rd_pending;
always @(posedge clk or negedge rst_n)
    if (!rst_n) rd_pending <= 1'b0; else rd_pending <= fifo_rdreq;

// word k of the current search result: 0 = found, then x/y/z of point rp (counters instead of /3 and %3,
// which were the slowest path at 100 MHz)
reg     [2:0]   rp;
reg     [1:0]   rc;
wire    [95:0]  rpt = r_pts[96*rp +: 96];
wire    [31:0]  rword = (ridx == 5'd0) ? {29'd0, r_found} :
                        (rp >= r_found) ? 32'd0 :
                        (rc == 2'd0) ? rpt[95:64] : (rc == 2'd1) ? rpt[63:32] : rpt[31:0];

always @(posedge clk or negedge rst_n)
    if (!rst_n) begin
        st <= S_HDR; nops <= 0; ns <= 0; w <= 0; opw0 <= 0; cur_buf <= 1'b0; ridx <= 0; r_found <= 0; r_pts <= 0;
        rp <= 3'd0; rc <= 2'd0;
        tx_we <= 1'b0; tx_addr <= 0; tx_wdata <= 0; result_tgl <= 1'b0; reply_bytes <= 16'd4; bad_hdr_cnt <= 0;
        op_valid <= 1'b0; op_search <= 1'b0; op_ds <= 1'b0; op_x <= 0; op_y <= 0; op_z <= 0;
ann_cnt <= 8'd0; sent_cnt <= 8'd0; sent_d <= 1'b0; clr_req <= 1'b0; clr_dly <= 2'd0; clr_cfg <= 16'd0; stat <= 1'b0;
    end else begin
        tx_we <= 1'b0; op_valid <= 1'b0; clr_req <= 1'b0;
        sent_d <= sent_tgl;
        if (sent_tgl != sent_d) sent_cnt <= sent_cnt + 8'd1;

        case (st)
        S_HDR:
            if (word_valid) begin
                if (fifo_q[31:16] == 16'hA513) begin            // CLEAR: empty the map, then an empty reply
                    ns <= 16'd0; clr_cfg <= fifo_q[15:0]; st <= S_CLRW;
                end else if (fifo_q[31:16] == 16'hA514) begin   // statistics
                    ns <= 16'd0; stat <= 1'b1; st <= S_ST0;
                end else if (fifo_q[31:16] == 16'hA511) begin
                    nops <= fifo_q[15:0]; ns <= 0; w <= 0;
                    st <= (fifo_q[15:0] == 16'd0) ? S_FIN : S_OPW;
                end else bad_hdr_cnt <= bad_hdr_cnt + 16'd1;
            end
        S_OPW:
            if (word_valid) begin
                case (w)
                    2'd0: opw0 <= fifo_q;
                    2'd1: op_x <= fifo_q;
                    2'd2: op_y <= fifo_q;
                    default: op_z <= fifo_q;
                endcase
                w <= w + 2'd1;
                if (w == 2'd3) st <= S_ISSUE;
            end
        S_ISSUE:
            if (op_ready) begin
                op_valid <= 1'b1; op_search <= opw0[0]; op_ds <= opw0[1];
                st <= S_WAITC;
            end
        S_WAITC:                                         // nn_core busy with the op
            if (op_search && res_valid) begin
                r_found <= res_found; r_pts <= res_pts; ridx <= 0; rp <= 3'd0; rc <= 2'd0; st <= S_RES;
            end else if (!op_search && op_ready && !op_valid) begin
                nops <= nops - 16'd1;
                st <= (nops == 16'd1) ? S_FIN : S_OPW;
            end
        S_RES: begin                                     // 16 words of this search into the reply
            tx_we <= 1'b1; tx_addr <= {cur_buf, 9'd1 + {ns[4:0], 4'd0} + {4'd0, ridx}}; tx_wdata <= rword;
            ridx <= ridx + 5'd1;
            if (ridx != 5'd0) begin
                if (rc == 2'd2) begin rc <= 2'd0; rp <= rp + 3'd1; end else rc <= rc + 2'd1;
            end
            if (ridx == 5'd15) begin
                ns <= ns + 16'd1; nops <= nops - 16'd1;
                st <= (nops == 16'd1) ? S_FIN : S_OPW;
            end
        end
        S_FIN: begin                                     // header word of the reply
            tx_we <= 1'b1; tx_addr <= {cur_buf, 9'd0}; tx_wdata <= {16'hA512, ns};
            st <= S_SEND;
        end
        S_SEND: begin                                    // announce one clock after the last write
            reply_bytes <= stat ? 16'd12 : (16'd4 + {ns[9:0], 6'd0}); stat <= 1'b0;
            result_tgl <= ~result_tgl; cur_buf <= ~cur_buf; ann_cnt <= ann_cnt + 8'd1;
            st <= S_HDR;
        end
        S_CLRW: if (op_ready) begin clr_req <= 1'b1; clr_dly <= 2'd3; st <= S_CLRX; end
        S_CLRX: if (clr_dly != 2'd0) clr_dly <= clr_dly - 2'd1;        // let the core leave S_IDLE first
                else if (op_ready) st <= S_FIN;
        S_ST0: begin tx_we <= 1'b1; tx_addr <= {cur_buf, 9'd1}; tx_wdata <= drop_cnt;  st <= S_ST1; end
        S_ST1: begin tx_we <= 1'b1; tx_addr <= {cur_buf, 9'd2}; tx_wdata <= evict_cnt; st <= S_FIN; end
        default: st <= S_HDR;
        endcase
    end

endmodule
