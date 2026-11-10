`timescale 1ns/1ps
////////////////////////////////////////////////////////////////////////
// nn_core : voxel-hash map in SDRAM + 5-nearest-neighbour search (rules = model/nn_model.py, DESIGN.md)
//
//  op = {ADD | SEARCH, downsample flag}, x, y, z  (int32 Q16.16 metres)
//  voxel v = coord >>> 15 ; offset = coord[14:0]
//  group  g = bits 8..23 of (uint32)(vx + 1021 vy + 262147 vz) * 2654435761 ; group = 128 SDRAM words
//  group layout: words 0..31 = 8 slot headers (4 words: tag x[15:0], tag y[15:0], tag z[15:0],
//                {tx[16], ty[16], tz[16], 10'b0, count[2:0]}), words 32 + 12 k + 3 j = point j of slot k (3 offsets)
//  SDRAM word address of group g = {1'b0, g[15:4], g[3:2], g[1:0], 7'b0}  (row, bank, column)
//
// Results of a SEARCH: found (0..5) and up to 5 points (Q16.16), nearest first; ties keep the earlier point.
// Clock: 100 MHz (same as the SDRAM controller). One operation at a time.
////////////////////////////////////////////////////////////////////////
module nn_core
#(
    parameter   NCLR = 65536                // groups whose headers are cleared after reset (all of them on hardware)
)
(
    input   wire            clk,
    input   wire            rst_n,
    // operation in
    input   wire            op_valid,
    output  wire            op_ready,
    input   wire            op_search,      // 1 = SEARCH, 0 = ADD
    input   wire            op_ds,          // ADD only: 1 = keep the point closest to the voxel centre
    input   wire    [31:0]  op_x, op_y, op_z,
    input   wire            clr_req,        // pulse while op_ready: clear the whole map again (op_ready low ~29 ms)
    // search result out (one clock pulse)
    output  reg             res_valid,
    output  reg     [2:0]   res_found,
    output  reg     [479:0] res_pts,        // point k at [96k +: 96] = {x, y, z}
    output  reg     [31:0]  drop_cnt,
    // SDRAM controller
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
localparam  S_IDLE = 5'd0, S_NB = 5'd1, S_HASH = 5'd2, S_HDR = 5'd3, S_FIND = 5'd4, S_PTS = 5'd5,
            S_DIST0 = 5'd6, S_DIST1 = 5'd7, S_INS = 5'd8, S_NEXT = 5'd9, S_DONE = 5'd10,
            S_ADDNEW = 5'd11, S_WR_H = 5'd12, S_WR_P = 5'd13, S_DC0 = 5'd14, S_DC1 = 5'd15, S_DCSEL = 5'd16,
            S_WAIT = 5'd17, S_CLR = 5'd18, S_HASH2 = 5'd19, S_DC2 = 5'd21, S_HASH1 = 5'd22, S_FIND0 = 5'd23,
            S_DIST2 = 5'd24;

reg     [4:0]   st, st_after;
reg             is_search, ds;
reg signed [31:0] qx, qy, qz;               // operation point
reg signed [16:0] vx, vy, vz;               // voxel being visited
reg     [4:0]   nb;                         // neighbour index 0..26
reg     [15:0]  g;
reg     [15:0]  hdr [0:31];
reg     [14:0]  pts [0:11];                 // up to 4 points x 3 offsets
reg     [5:0]   rcnt;
reg     [2:0]   slot;
reg     [2:0]   n;                          // points in the slot
reg             hit;
reg     [2:0]   j;                          // point index inside the slot
// top-K list
reg signed [31:0] bx [0:K-1], by [0:K-1], bz [0:K-1];
reg     [39:0]  bd [0:K-1];
reg     [2:0]   found;
// distance pipeline
reg signed [31:0] px, py, pz;
reg signed [17:0] ddx, ddy, ddz;            // |difference| < 2 voxels = 2^16 -> 18-bit signed is enough
reg     [39:0]  d2;
reg     [35:0]  sqx, sqy, sqz;              // squared differences (registered: 100 MHz)
// ADD bookkeeping
reg     [2:0]   wslot;
reg     [30:0]  best_d;                     // distances to the voxel centre are < 3 * 2^28 -> 31 bits
reg     [2:0]   best_j;                     // 7 = the new point
reg     [2:0]   far_j;
reg     [30:0]  far_d;
reg     [15:0]  wbuf [0:3];
reg signed [31:0] hs_r;                     // first hash stage
reg signed [15:0] ox, oy, oz;               // offsets of the stored point j relative to the voxel centre
reg     [30:0]  dc_old_r, dcn;              // registered distances to the voxel centre (stored point / new point)
reg     [2:0]   wpoint;                     // index of the point slot to write
reg             busy;                       // an SDRAM request of this state is in flight
reg     [16:0]  clr_g;                      // group being cleared after reset

assign  op_ready  = (st == S_IDLE);         // also low while the table is cleared after reset
assign  sd_wr_data = wbuf[sd_wr_idx[1:0]];

//---------------------------------------------------------------- hash
// x + 1021 y + 262147 z with shifts (1021 = 1024 - 2 - 1, 262147 = 262144 + 2 + 1), in two clocks
wire signed [31:0] ex = {{15{vx[16]}}, vx}, ey = {{15{vy[16]}}, vy}, ez = {{15{vz[16]}}, vz};
wire signed [31:0] hs_a = ex + (ey <<< 10) - (ey <<< 1) - ey;
wire signed [31:0] hs_b = (ez <<< 18) + (ez <<< 1) + ez;
reg  signed [31:0] hs_ar, hs_br;
wire signed [31:0] hs = hs_ar + hs_br;
wire    [31:0]  hm = hs_r * 32'h9E37_79B1;
wire    [15:0]  g_next = hm[23:8];

//---------------------------------------------------------------- neighbour offsets: dx, dy, dz = -1..1, dz fastest
wire    [4:0]   nbi = nb;
wire signed [1:0] ndx = (nbi < 9) ? -2'sd1 : (nbi < 18) ? 2'sd0 : 2'sd1;
wire    [4:0]   nr  = (nbi < 9) ? nbi : (nbi < 18) ? nbi - 5'd9 : nbi - 5'd18;
wire signed [1:0] ndy = (nr < 3) ? -2'sd1 : (nr < 6) ? 2'sd0 : 2'sd1;
wire    [4:0]   nz  = (nr < 3) ? nr : (nr < 6) ? nr - 5'd3 : nr - 5'd6;
wire signed [1:0] ndz = (nz == 0) ? -2'sd1 : (nz == 1) ? 2'sd0 : 2'sd1;

wire signed [16:0] qvx = qx >>> 15, qvy = qy >>> 15, qvz = qz >>> 15;

//---------------------------------------------------------------- slot search in the 8 headers (combinational)
reg     [2:0]   m_slot, e_slot;
reg             m_hit, e_hit;
reg     [2:0]   m_slot_r, e_slot_r, m_cnt_r;   // tag search registered (S_FIND0) to keep 100 MHz
reg             m_hit_r, e_hit_r;
integer s;
always @(*) begin
    m_hit = 1'b0; m_slot = 3'd0; e_hit = 1'b0; e_slot = 3'd0;
    for (s = 7; s >= 0; s = s - 1) begin                 // descending, so the lowest index wins
        if (hdr[4*s+3][2:0] != 3'd0 && hdr[4*s] == vx[15:0] && hdr[4*s+1] == vy[15:0] && hdr[4*s+2] == vz[15:0]
            && hdr[4*s+3][15] == vx[16] && hdr[4*s+3][14] == vy[16] && hdr[4*s+3][13] == vz[16]) begin
            m_hit = 1'b1; m_slot = s[2:0];
        end
        if (hdr[4*s+3][2:0] == 3'd0) begin e_hit = 1'b1; e_slot = s[2:0]; end
    end
end

wire    [23:0]  g_base = {1'b0, g[15:4], g[3:2], g[1:0], 7'd0};

// distance of point j of the current slot to the voxel centre (ADD) : offsets relative to 2^14
reg  signed [15:0] ncx, ncy, ncz;          // new point relative to the voxel centre (registered in S_HASH)
wire    [30:0]  dc_new = ncx * ncx + ncy * ncy + ncz * ncz;
wire    [30:0]  dc_old = ox * ox + oy * oy + oz * oz;

// insertion position for d2 in the sorted list (after all entries <= d2)
reg     [2:0]   ipos;
integer t;
always @(*) begin
    ipos = 3'd0;
    for (t = 0; t < K; t = t + 1)
        if (t < found && bd[t] <= d2) ipos = t + 1;
end

integer u;
always @(posedge clk or negedge rst_n)
    if (!rst_n) begin
        st <= S_CLR; clr_g <= 17'd0; st_after <= S_IDLE; is_search <= 1'b0; ds <= 1'b0; qx <= 0; qy <= 0; qz <= 0;
        vx <= 0; vy <= 0; vz <= 0; nb <= 5'd0; g <= 16'd0; rcnt <= 6'd0; slot <= 3'd0; n <= 3'd0; hit <= 1'b0;
        j <= 3'd0; found <= 3'd0; px <= 0; py <= 0; pz <= 0; ddx <= 0; ddy <= 0; ddz <= 0; d2 <= 0;
        wslot <= 0; best_d <= 0; best_j <= 0; far_j <= 0; far_d <= 0; wpoint <= 0; busy <= 1'b0;
        res_valid <= 1'b0; res_found <= 3'd0; res_pts <= 480'd0; drop_cnt <= 32'd0;
        sd_req <= 1'b0; sd_we <= 1'b0; sd_addr <= 24'd0; sd_len <= 9'd0;
    end else begin
        res_valid <= 1'b0;
        if (sd_ack) sd_req <= 1'b0;                          // request held until the controller takes it
        if (sd_rd_valid) begin                               // collect read words
            if (st == S_HDR) hdr[rcnt[4:0]] <= sd_rd_data;
            else             pts[rcnt[3:0]] <= sd_rd_data[14:0];
            rcnt <= rcnt + 6'd1;
        end

        case (st)
        S_IDLE:
            if (clr_req) begin clr_g <= 17'd0; drop_cnt <= 32'd0; st <= S_CLR; end
            else if (op_valid) begin
                is_search <= op_search; ds <= op_ds;
                qx <= op_x; qy <= op_y; qz <= op_z;
                found <= 3'd0; nb <= 5'd0;
                st <= op_search ? S_NB : S_HASH;
                vx <= op_x >>> 15; vy <= op_y >>> 15; vz <= op_z >>> 15;
            end
        S_NB: begin                                          // voxel of neighbour nb
            vx <= qvx + ndx; vy <= qvy + ndy; vz <= qvz + ndz;
            st <= S_HASH;
        end
        S_HASH: begin                                        // hash in three clocks (100 MHz)
            hs_ar <= hs_a; hs_br <= hs_b; st <= S_HASH1;
            ncx <= {1'b0, qx[14:0]} - 16'sd16384; ncy <= {1'b0, qy[14:0]} - 16'sd16384; ncz <= {1'b0, qz[14:0]} - 16'sd16384;
        end
        S_HASH1: begin hs_r <= hs; st <= S_HASH2; end
        S_HASH2: begin                                       // then read the 8 headers
            g <= g_next; st <= S_WAIT; st_after <= S_HDR;
            dcn <= dc_new;
        end
        S_WAIT: begin                                        // read the 8 slot headers of group g
            sd_req <= 1'b1; sd_we <= 1'b0; sd_addr <= g_base; sd_len <= 9'd32; rcnt <= 6'd0; st <= S_HDR;
        end
        S_HDR:
            if (sd_done) st <= S_FIND0;
        S_FIND0: begin
            m_hit_r <= m_hit; m_slot_r <= m_slot; e_hit_r <= e_hit; e_slot_r <= e_slot;
            m_cnt_r <= hdr[4*m_slot+3][2:0];
            st <= S_FIND;
        end
        S_FIND: begin
            if (is_search) begin
                if (m_hit_r) begin
                    slot <= m_slot_r; n <= m_cnt_r; hit <= 1'b1; st <= S_PTS;
                    sd_we <= 1'b0; sd_addr <= g_base + 24'd32 + ({21'd0, m_slot_r} << 3) + ({21'd0, m_slot_r} << 2); sd_len <= ({6'd0, m_cnt_r} << 1) + {6'd0, m_cnt_r};
                end else st <= S_NEXT;
            end else begin
                if (m_hit_r) begin
                    slot <= m_slot_r; n <= m_cnt_r; hit <= 1'b1; st <= S_PTS;
                    sd_we <= 1'b0; sd_addr <= g_base + 24'd32 + ({21'd0, m_slot_r} << 3) + ({21'd0, m_slot_r} << 2); sd_len <= ({6'd0, m_cnt_r} << 1) + {6'd0, m_cnt_r};
                end else if (e_hit_r) begin
                    slot <= e_slot_r; n <= 3'd0; hit <= 1'b0; st <= S_ADDNEW;
                end else begin
                    drop_cnt <= drop_cnt + 32'd1; st <= S_IDLE;  // group full: point dropped
                end
            end
        end
        S_PTS:                                               // read the points of the slot (address set in S_FIND)
            if (!busy) begin sd_req <= 1'b1; busy <= 1'b1; rcnt <= 6'd0; end
            else if (sd_done) begin
                busy <= 1'b0; j <= 3'd0;
                st <= is_search ? S_DIST0 : (ds ? S_DC0 : (n < CAP ? S_ADDNEW : S_DC0));
                best_d <= dcn; best_j <= 3'd7; far_d <= 31'd0; far_j <= 3'd0;
            end
        //------------------------------------------------ SEARCH: distance of point j, insert into top-K
        S_DIST0: begin
            px = {vx, pts[3*j]}; py = {vy, pts[3*j+1]}; pz = {vz, pts[3*j+2]};
            ddx <= px - qx; ddy <= py - qy; ddz <= pz - qz;     // fits 18 bits: both points within 2 voxels
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
        S_NEXT: begin
            hit <= 1'b0;
            if (nb == 5'd26) st <= S_DONE;
            else begin nb <= nb + 5'd1; st <= S_NB; end
        end
        S_DONE: begin
            res_valid <= 1'b1; res_found <= found;
            for (u = 0; u < K; u = u + 1) res_pts[96*u +: 96] <= {bx[u], by[u], bz[u]};
            st <= S_IDLE;
        end
        //------------------------------------------------ ADD
        S_DC0: begin                                         // scan the stored points: closest to centre / farthest
            ox <= {1'b0, pts[3*j]} - 16'sd16384; oy <= {1'b0, pts[3*j+1]} - 16'sd16384; oz <= {1'b0, pts[3*j+2]} - 16'sd16384;
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
                if (best_j != 3'd7) begin pts[0] <= pts[3*best_j]; pts[1] <= pts[3*best_j+1]; pts[2] <= pts[3*best_j+2]; end
                else begin pts[0] <= qx[14:0]; pts[1] <= qy[14:0]; pts[2] <= qz[14:0]; end
            end else begin                                   // slot full: replace the farthest point
                wpoint <= far_j;
                pts[3*far_j] <= qx[14:0]; pts[3*far_j+1] <= qy[14:0]; pts[3*far_j+2] <= qz[14:0];
            end
            st <= S_WR_H;
        end
        S_ADDNEW: begin                                      // append the new point at index n
            wpoint <= n; n <= n + 3'd1;
            pts[3*n] <= qx[14:0]; pts[3*n+1] <= qy[14:0]; pts[3*n+2] <= qz[14:0];
            st <= S_WR_H;
        end
        S_WR_H:                                              // write the 4 header words of the slot
            if (!busy) begin
                wbuf[0] <= vx[15:0]; wbuf[1] <= vy[15:0]; wbuf[2] <= vz[15:0];
                wbuf[3] <= {vx[16], vy[16], vz[16], 10'd0, n};
                sd_req <= 1'b1; sd_we <= 1'b1; sd_addr <= g_base + {slot, 2'b00}; sd_len <= 9'd4; busy <= 1'b1;
            end else if (sd_done) begin busy <= 1'b0; st <= S_WR_P; end
        S_WR_P:                                              // write the changed point (3 words)
            if (!busy) begin
                wbuf[0] <= {1'b0, pts[3*wpoint]}; wbuf[1] <= {1'b0, pts[3*wpoint+1]}; wbuf[2] <= {1'b0, pts[3*wpoint+2]};
                sd_req <= 1'b1; sd_we <= 1'b1; sd_addr <= g_base + 24'd32 + ({21'd0, slot} << 3) + ({21'd0, slot} << 2) + 24'd3 * wpoint; sd_len <= 9'd3;
                busy <= 1'b1;
            end else if (sd_done) begin busy <= 1'b0; st <= S_IDLE; end
        S_CLR:                                               // SDRAM powers up with random data: zero all slot headers
            if (!busy) begin
                if (clr_g == NCLR) st <= S_IDLE;
                else begin
                    wbuf[0] <= 16'd0; wbuf[1] <= 16'd0; wbuf[2] <= 16'd0; wbuf[3] <= 16'd0;
                    sd_req <= 1'b1; sd_we <= 1'b1; sd_addr <= {1'b0, clr_g[15:4], clr_g[3:2], clr_g[1:0], 7'd0};
                    sd_len <= 9'd32; busy <= 1'b1;
                end
            end else if (sd_done) begin busy <= 1'b0; clr_g <= clr_g + 17'd1; end
        default: st <= S_IDLE;
        endcase
    end

endmodule
