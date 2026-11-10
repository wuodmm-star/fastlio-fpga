`timescale 1ns/1ps
////////////////////////////////////////////////////////////////////////
// nn_core (v2, "summary" layout) : voxel-hash map in SDRAM + 5-nearest-neighbour search
// Same results as v1 / model/nn_model.py; only the SDRAM layout and the access pattern changed.
//
//  op = {ADD | SEARCH, downsample flag}, x, y, z  (int32 Q16.16 metres), |coordinate| < 16 km
//  voxel v = coord >>> 15 (16-bit tag) ; offset = coord[14:0]
//  h = (uint32)(vx + 1021 vy + 262147 vz) * 2654435761 ; group g = h[23:8] ; fingerprint fp = h[31:24]
//  group = 128 SDRAM words in one row:
//     word 0 : counts of slots 0..4 (3 bits each)       word 1 : counts of slots 5..7
//     words 2..5 : fingerprints, two slots per word (slot 2i in [7:0], 2i+1 in [15:8])
//     slot k at word 8 + 15 k : tag x, tag y, tag z, then 4 points x 3 offsets
//  A lookup reads the 6 summary words; only slots whose fingerprint matches are read (tag + points in one burst).
//  v1 read all 8 slot headers (32 words) for every one of the 27 neighbour voxels.
// Clock: 100 MHz. One operation at a time.
////////////////////////////////////////////////////////////////////////
module nn_core
#(
    parameter   NCLR = 65536                // groups cleared after reset (all of them on hardware)
)
(
    input   wire            clk,
    input   wire            rst_n,
    input   wire            op_valid,
    output  wire            op_ready,
    input   wire            op_search,
    input   wire            op_ds,
    input   wire    [31:0]  op_x, op_y, op_z,
    input   wire            clr_req,
    output  reg             res_valid,
    output  reg     [2:0]   res_found,
    output  reg     [479:0] res_pts,
    output  reg     [31:0]  drop_cnt,
    input   wire            sd_ack,
    input   wire            sd_done,
    output  reg             sd_req,
    output  reg             sd_we,
    output  reg     [23:0]  sd_addr,
    output  reg     [8:0]   sd_len,
    input   wire    [8:0]   sd_wr_idx,
    output  wire    [15:0]  sd_wr_data,
    input   wire            sd_rd_valid,
    input   wire    [15:0]  sd_rd_data
);

localparam  K = 5, CAP = 4;
localparam  S_IDLE = 5'd0, S_NB = 5'd1, S_HASH = 5'd2, S_HASH1 = 5'd3, S_HASH2 = 5'd4, S_RDSUM = 5'd5,
            S_SUMW = 5'd6, S_SUMDEC = 5'd7, S_CAND = 5'd8, S_SLOTW = 5'd9, S_TAGCHK = 5'd10, S_NOHIT = 5'd11,
            S_DIST0 = 5'd12, S_DIST1 = 5'd13, S_DIST2 = 5'd14, S_INS = 5'd15, S_NEXT = 5'd16, S_DONE = 5'd17,
            S_DC0 = 5'd18, S_DC1 = 5'd19, S_DC2 = 5'd20, S_DCSEL = 5'd21, S_ADDNEW = 5'd22, S_WR_P = 5'd23,
            S_WR_S = 5'd24, S_CLR = 5'd25;

reg     [4:0]   st;
reg             is_search, ds, busy;
reg signed [31:0] qx, qy, qz;
reg signed [16:0] vx, vy, vz;
reg     [4:0]   nb;
reg     [15:0]  g;
reg     [7:0]   fp;
reg     [16:0]  clr_g;
reg     [15:0]  sw [0:5];                   // the 6 summary words of group g
reg     [15:0]  pb [0:14];                  // tag (3) + up to 4 points (12) of the slot being examined
reg     [4:0]   rcnt;
reg     [7:0]   cand;                       // slots whose fingerprint matches
reg     [7:0]   empt;                       // slots with count 0
reg     [2:0]   c;                          // candidate being read
reg     [2:0]   slot, n, j;
reg signed [31:0] hs_ar, hs_br, hs_r;
reg signed [15:0] ncx, ncy, ncz, ox, oy, oz;
reg     [30:0]  dcn, dc_old_r, best_d, far_d;
reg     [2:0]   best_j, far_j, wpoint;
reg signed [31:0] bx [0:K-1], by [0:K-1], bz [0:K-1];
reg     [39:0]  bd [0:K-1];
reg     [2:0]   found;
reg signed [31:0] px, py, pz;
reg signed [17:0] ddx, ddy, ddz;
reg     [35:0]  sqx, sqy, sqz;
reg     [39:0]  d2;
reg     [15:0]  wbuf [0:5];
reg             new_slot;

assign  op_ready   = (st == S_IDLE);
assign  sd_wr_data = wbuf[sd_wr_idx[2:0]];

//---------------------------------------------------------------- hash (3 clocks)
wire signed [31:0] ex = {{15{vx[16]}}, vx}, ey = {{15{vy[16]}}, vy}, ez = {{15{vz[16]}}, vz};
wire signed [31:0] hs_a = ex + (ey <<< 10) - (ey <<< 1) - ey;          // x + 1021 y
wire signed [31:0] hs_b = (ez <<< 18) + (ez <<< 1) + ez;               // 262147 z
wire    [31:0]  hm = hs_r * 32'h9E37_79B1;

//---------------------------------------------------------------- neighbour offsets (dz fastest)
wire signed [1:0] ndx = (nb < 9) ? -2'sd1 : (nb < 18) ? 2'sd0 : 2'sd1;
wire    [4:0]   nr  = (nb < 9) ? nb : (nb < 18) ? nb - 5'd9 : nb - 5'd18;
wire signed [1:0] ndy = (nr < 3) ? -2'sd1 : (nr < 6) ? 2'sd0 : 2'sd1;
wire    [4:0]   nz  = (nr < 3) ? nr : (nr < 6) ? nr - 5'd3 : nr - 5'd6;
wire signed [1:0] ndz = (nz == 0) ? -2'sd1 : (nz == 1) ? 2'sd0 : 2'sd1;
wire signed [16:0] qvx = qx >>> 15, qvy = qy >>> 15, qvz = qz >>> 15;

//---------------------------------------------------------------- summary fields
wire    [2:0]   cnt_of [0:7];
assign  cnt_of[0] = sw[0][2:0];   assign cnt_of[1] = sw[0][5:3];   assign cnt_of[2] = sw[0][8:6];
assign  cnt_of[3] = sw[0][11:9];  assign cnt_of[4] = sw[0][14:12]; assign cnt_of[5] = sw[1][2:0];
assign  cnt_of[6] = sw[1][5:3];   assign cnt_of[7] = sw[1][8:6];
wire    [7:0]   fp_of [0:7];
genvar gi;
generate for (gi = 0; gi < 8; gi = gi + 1) begin : fps
    assign fp_of[gi] = gi[0] ? sw[2 + gi/2][15:8] : sw[2 + gi/2][7:0];
end endgenerate

// lowest set bit of a mask
function [2:0] lsb8; input [7:0] m; integer b; begin lsb8 = 3'd0; for (b = 7; b >= 0; b = b - 1) if (m[b]) lsb8 = b[2:0]; end endfunction

wire    [23:0]  g_base = {1'b0, g[15:4], g[3:2], g[1:0], 7'd0};
wire    [23:0]  slot_base_c = g_base + 24'd8 + ({21'd0, c} << 4) - {21'd0, c};          // 8 + 15 c
wire    [23:0]  slot_base_s = g_base + 24'd8 + ({21'd0, slot} << 4) - {21'd0, slot};

wire    [30:0]  dc_new = ncx * ncx + ncy * ncy + ncz * ncz;
wire    [30:0]  dc_old = ox * ox + oy * oy + oz * oz;

reg     [2:0]   ipos;
integer t;
always @(*) begin
    ipos = 3'd0;
    for (t = 0; t < K; t = t + 1)
        if (t < found && bd[t] <= d2) ipos = t + 1;
end

wire    [3:0]   pj  = 4'd3 + {j, 1'b0} + {1'b0, j};      // pb index of point j = 3 + 3 j

integer u, k2;
always @(posedge clk or negedge rst_n)
    if (!rst_n) begin
        st <= S_CLR; clr_g <= 17'd0; is_search <= 1'b0; ds <= 1'b0; busy <= 1'b0; qx <= 0; qy <= 0; qz <= 0;
        vx <= 0; vy <= 0; vz <= 0; nb <= 0; g <= 0; fp <= 0; rcnt <= 0; cand <= 0; empt <= 0; c <= 0;
        slot <= 0; n <= 0; j <= 0; found <= 0; hs_ar <= 0; hs_br <= 0; hs_r <= 0; ncx <= 0; ncy <= 0; ncz <= 0;
        ox <= 0; oy <= 0; oz <= 0; dcn <= 0; dc_old_r <= 0; best_d <= 0; far_d <= 0; best_j <= 0; far_j <= 0;
        wpoint <= 0; px <= 0; py <= 0; pz <= 0; ddx <= 0; ddy <= 0; ddz <= 0; sqx <= 0; sqy <= 0; sqz <= 0; d2 <= 0;
        new_slot <= 1'b0; res_valid <= 1'b0; res_found <= 0; res_pts <= 0; drop_cnt <= 0;
        sd_req <= 1'b0; sd_we <= 1'b0; sd_addr <= 0; sd_len <= 0;
    end else begin
        res_valid <= 1'b0;
        if (sd_ack) sd_req <= 1'b0;
        if (sd_rd_valid) begin
            if (st == S_SUMW) sw[rcnt[2:0]] <= sd_rd_data;
            else              pb[rcnt[3:0]] <= sd_rd_data;
            rcnt <= rcnt + 5'd1;
        end

        case (st)
        S_CLR:                                               // zero the 6 summary words of every group
            if (!busy) begin
                if (clr_g == NCLR) st <= S_IDLE;
                else begin
                    for (k2 = 0; k2 < 6; k2 = k2 + 1) wbuf[k2] <= 16'd0;
                    sd_req <= 1'b1; sd_we <= 1'b1; sd_addr <= {1'b0, clr_g[15:4], clr_g[3:2], clr_g[1:0], 7'd0};
                    sd_len <= 9'd6; busy <= 1'b1;
                end
            end else if (sd_done) begin busy <= 1'b0; clr_g <= clr_g + 17'd1; end
        S_IDLE:
            if (clr_req) begin clr_g <= 17'd0; drop_cnt <= 32'd0; st <= S_CLR; end
            else if (op_valid) begin
                is_search <= op_search; ds <= op_ds; qx <= op_x; qy <= op_y; qz <= op_z;
                found <= 3'd0; nb <= 5'd0;
                vx <= op_x >>> 15; vy <= op_y >>> 15; vz <= op_z >>> 15;
                st <= op_search ? S_NB : S_HASH;
            end
        S_NB: begin vx <= qvx + ndx; vy <= qvy + ndy; vz <= qvz + ndz; st <= S_HASH; end
        S_HASH: begin
            hs_ar <= hs_a; hs_br <= hs_b; st <= S_HASH1;
            ncx <= {1'b0, qx[14:0]} - 16'sd16384; ncy <= {1'b0, qy[14:0]} - 16'sd16384; ncz <= {1'b0, qz[14:0]} - 16'sd16384;
        end
        S_HASH1: begin hs_r <= hs_ar + hs_br; st <= S_HASH2; end
        S_HASH2: begin g <= hm[23:8]; fp <= hm[31:24]; dcn <= dc_new; st <= S_RDSUM; end
        S_RDSUM: begin                                       // read the 6 summary words
            sd_req <= 1'b1; sd_we <= 1'b0; sd_addr <= g_base; sd_len <= 9'd6; rcnt <= 5'd0; st <= S_SUMW;
        end
        S_SUMW: if (sd_done) st <= S_SUMDEC;
        S_SUMDEC: begin                                      // candidate slots (registered)
            for (k2 = 0; k2 < 8; k2 = k2 + 1) begin
                cand[k2] <= (cnt_of[k2] != 3'd0) && (fp_of[k2] == fp);
                empt[k2] <= (cnt_of[k2] == 3'd0);
            end
            st <= S_CAND;
        end
        S_CAND:
            if (cand == 8'd0) st <= S_NOHIT;
            else begin c <= lsb8(cand); st <= S_SLOTW; end   // read tag + points of the lowest candidate
        S_SLOTW:
            if (!busy) begin
                sd_req <= 1'b1; sd_we <= 1'b0; sd_addr <= slot_base_c;
                sd_len <= 9'd3 + ({6'd0, cnt_of[c]} << 1) + {6'd0, cnt_of[c]};
                rcnt <= 5'd0; busy <= 1'b1;
            end else if (sd_done) begin busy <= 1'b0; st <= S_TAGCHK; end
        S_TAGCHK:
            if (pb[0] == vx[15:0] && pb[1] == vy[15:0] && pb[2] == vz[15:0]) begin
                slot <= c; n <= cnt_of[c]; j <= 3'd0; new_slot <= 1'b0;
                best_d <= dcn; best_j <= 3'd7; far_d <= 31'd0; far_j <= 3'd0;
                st <= is_search ? S_DIST0 : (ds ? S_DC0 : (cnt_of[c] < CAP ? S_ADDNEW : S_DC0));
            end else begin
                cand[c] <= 1'b0; st <= S_CAND;              // fingerprint collision: try the next candidate
            end
        S_NOHIT:                                             // the voxel is not in its group
            if (is_search) st <= S_NEXT;
            else if (empt != 8'd0) begin
                slot <= lsb8(empt); n <= 3'd0; new_slot <= 1'b1; st <= S_ADDNEW;
            end else begin
                drop_cnt <= drop_cnt + 32'd1; st <= S_IDLE;  // group full: point dropped
            end
        //------------------------------------------------ SEARCH
        S_DIST0: begin
            px = {vx, pb[pj][14:0]}; py = {vy, pb[pj+4'd1][14:0]}; pz = {vz, pb[pj+4'd2][14:0]};
            ddx <= px - qx; ddy <= py - qy; ddz <= pz - qz;
            st <= S_DIST1;
        end
        S_DIST1: begin sqx <= ddx * ddx; sqy <= ddy * ddy; sqz <= ddz * ddz; st <= S_DIST2; end
        S_DIST2: begin d2 <= {4'd0, sqx} + {4'd0, sqy} + {4'd0, sqz}; st <= S_INS; end
        S_INS: begin
            if (!(found == K && d2 >= bd[K-1])) begin
                for (u = K - 1; u > 0; u = u - 1)
                    if (u > ipos) begin bd[u] <= bd[u-1]; bx[u] <= bx[u-1]; by[u] <= by[u-1]; bz[u] <= bz[u-1]; end
                bd[ipos] <= d2; bx[ipos] <= px; by[ipos] <= py; bz[ipos] <= pz;
                if (found < K) found <= found + 3'd1;
            end
            if (j + 3'd1 < n) begin j <= j + 3'd1; st <= S_DIST0; end
            else st <= S_NEXT;
        end
        S_NEXT:
            if (nb == 5'd26) st <= S_DONE;
            else begin nb <= nb + 5'd1; st <= S_NB; end
        S_DONE: begin
            res_valid <= 1'b1; res_found <= found;
            for (u = 0; u < K; u = u + 1) res_pts[96*u +: 96] <= {bx[u], by[u], bz[u]};
            st <= S_IDLE;
        end
        //------------------------------------------------ ADD into an existing slot
        S_DC0: begin
            ox <= {1'b0, pb[pj][14:0]} - 16'sd16384; oy <= {1'b0, pb[pj+4'd1][14:0]} - 16'sd16384;
            oz <= {1'b0, pb[pj+4'd2][14:0]} - 16'sd16384;
            st <= S_DC1;
        end
        S_DC1: begin dc_old_r <= dc_old; st <= S_DC2; end
        S_DC2: begin
            if (dc_old_r < best_d) begin best_d <= dc_old_r; best_j <= j; end
            if (dc_old_r > far_d || j == 0) begin far_d <= dc_old_r; far_j <= j; end
            if (j + 3'd1 < n) begin j <= j + 3'd1; st <= S_DC0; end
            else st <= S_DCSEL;
        end
        S_DCSEL: begin
            if (ds) begin                                    // keep one point: the best (the new one if best_j == 7)
                n <= 3'd1; wpoint <= 3'd0;
                if (best_j == 3'd7) begin wbuf[0] <= {1'b0, qx[14:0]}; wbuf[1] <= {1'b0, qy[14:0]}; wbuf[2] <= {1'b0, qz[14:0]}; end
                else begin
                    wbuf[0] <= pb[4'd3 + {best_j, 1'b0} + {1'b0, best_j}];
                    wbuf[1] <= pb[4'd4 + {best_j, 1'b0} + {1'b0, best_j}];
                    wbuf[2] <= pb[4'd5 + {best_j, 1'b0} + {1'b0, best_j}];
                end
            end else begin                                   // slot full: replace the farthest point
                wpoint <= far_j;
                wbuf[0] <= {1'b0, qx[14:0]}; wbuf[1] <= {1'b0, qy[14:0]}; wbuf[2] <= {1'b0, qz[14:0]};
            end
            st <= S_WR_P;
        end
        S_ADDNEW: begin                                      // append at index n (new slot: tag + point 0)
            wpoint <= n; n <= n + 3'd1;
            if (new_slot) begin
                wbuf[0] <= vx[15:0]; wbuf[1] <= vy[15:0]; wbuf[2] <= vz[15:0];
                wbuf[3] <= {1'b0, qx[14:0]}; wbuf[4] <= {1'b0, qy[14:0]}; wbuf[5] <= {1'b0, qz[14:0]};
            end else begin
                wbuf[0] <= {1'b0, qx[14:0]}; wbuf[1] <= {1'b0, qy[14:0]}; wbuf[2] <= {1'b0, qz[14:0]};
            end
            st <= S_WR_P;
        end
        S_WR_P:                                              // write the point (and the tag for a new slot)
            if (!busy) begin
                sd_req <= 1'b1; sd_we <= 1'b1; busy <= 1'b1;
                if (new_slot) begin sd_addr <= slot_base_s; sd_len <= 9'd6; end
                else begin sd_addr <= slot_base_s + 24'd3 + ({21'd0, wpoint} << 1) + {21'd0, wpoint}; sd_len <= 9'd3; end
            end else if (sd_done) begin
                busy <= 1'b0;
                // updated summary: count n of `slot`, and its fingerprint for a new slot
                for (k2 = 0; k2 < 6; k2 = k2 + 1) wbuf[k2] <= sw[k2];
                case (slot)
                    3'd0: wbuf[0][2:0]   <= n;  3'd1: wbuf[0][5:3]   <= n;  3'd2: wbuf[0][8:6] <= n;
                    3'd3: wbuf[0][11:9]  <= n;  3'd4: wbuf[0][14:12] <= n;  3'd5: wbuf[1][2:0] <= n;
                    3'd6: wbuf[1][5:3]   <= n;  default: wbuf[1][8:6] <= n;
                endcase
                if (new_slot) begin
                    if (slot[0]) wbuf[2 + slot/2][15:8] <= fp; else wbuf[2 + slot/2][7:0] <= fp;
                end
                st <= S_WR_S;
            end
        S_WR_S:                                              // write the 6 summary words back
            if (!busy) begin
                sd_req <= 1'b1; sd_we <= 1'b1; sd_addr <= g_base; sd_len <= 9'd6; busy <= 1'b1;
            end else if (sd_done) begin busy <= 1'b0; st <= S_IDLE; end
        default: st <= S_IDLE;
        endcase
    end

endmodule
