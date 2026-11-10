`timescale 1ns/1ps
////////////////////////////////////////////////////////////////////////
// sdram_rw : random-access burst controller for the 征途Pro SDRAM (W9825G6, 32 MB, x16), 100 MHz
//
// One request at a time: "read LEN words" or "write LEN words" starting at a 24-bit word address
// {row[12:0], bank[1:0], col[8:0]}. A request must stay inside one row (col + LEN <= 512), LEN = 1..256.
// Every access opens the row, transfers a full-page burst that is cut after LEN words, and closes the row,
// so there are no open-row rules to track. Auto-refresh is inserted between requests (every 7.8 us).
// Timing at 100 MHz: tRCD 2, CL 3, tRP 2, tWR 2, tRC 7, tMRD 3, tRAS >= 5 clocks (values of the Embedfire example).
//
// Write data: the controller shows the index of the word it needs on wr_idx and samples wr_data in the same clock
// while wr_take is high (so wr_data must be a combinational function of wr_idx, e.g. from a register array).
// Read data: rd_valid / rd_data, LEN words in order. done pulses when the request is finished.
////////////////////////////////////////////////////////////////////////
module sdram_rw
#(
    parameter   RD_LAT  = 4,            // clocks from READ command to the clock in which dq_in holds word 0
    parameter   T_INIT  = 20000         // 200 us power-up wait
)
(
    input   wire            clk,
    input   wire            rst_n,
    output  reg             init_done,
    // request
    input   wire            req,
    input   wire            we,
    input   wire    [23:0]  addr,
    input   wire    [8:0]   len,
    output  wire            ready,      // idle (informational)
    output  reg             ack,        // request latched: requester drops req after seeing ack
    output  reg             done,
    // write data
    output  reg     [8:0]   wr_idx,
    output  reg             wr_take,
    input   wire    [15:0]  wr_data,
    // read data
    output  reg             rd_valid,
    output  reg     [15:0]  rd_data,
    // SDRAM pins
    output  reg     [3:0]   cmd,        // {cs_n, ras_n, cas_n, we_n}
    output  reg     [1:0]   ba,
    output  reg     [12:0]  sa,
    output  reg     [1:0]   dqm,
    output  wire    [15:0]  dq_out,
    output  reg             dq_oe,
    input   wire    [15:0]  dq_in
);

localparam  C_NOP = 4'b0111, C_ACT = 4'b0011, C_RD = 4'b0101, C_WR = 4'b0100, C_BST = 4'b0110,
            C_PRE = 4'b0010, C_REF = 4'b0001, C_MRS = 4'b0000;
localparam  MODE  = 13'b000_0_00_011_0_111;     // burst write, CL 3, sequential, full-page burst
localparam  T_REFI = 10'd600;                   // 6 us: leaves room for one request (<= ~180 clocks) before 7.8 us

localparam  S_WAIT = 4'd0, S_IPRE = 4'd1, S_IREF = 4'd2, S_IMRS = 4'd3, S_IDLE = 4'd4, S_ACT = 4'd5,
            S_RW = 4'd6, S_XFER = 4'd7, S_PRE = 4'd8, S_REF = 4'd9, S_GAP = 4'd10;

reg     [3:0]   st;
reg     [14:0]  cnt;            // generic wait counter
reg     [3:0]   nref;           // refresh count during init
reg     [9:0]   refi;           // refresh interval counter
reg             ref_due;
reg             r_we;
reg     [23:0]  r_addr;
reg     [8:0]   r_len;
reg     [8:0]   k;              // words issued (write) / clocks since READ (read)
reg     [8:0]   nrx;            // words received

assign  ready  = (st == S_IDLE) && init_done;
assign  dq_out = wr_data;


always @(posedge clk or negedge rst_n)
    if (!rst_n) begin
        refi <= 10'd0; ref_due <= 1'b0;
    end else begin
        if (st == S_REF) begin refi <= 10'd0; ref_due <= 1'b0; end
        else if (refi == T_REFI) ref_due <= 1'b1;
        else refi <= refi + 10'd1;
    end

always @(posedge clk or negedge rst_n)
    if (!rst_n) begin
        st <= S_WAIT; cnt <= 15'd0; nref <= 4'd0; init_done <= 1'b0; done <= 1'b0; ack <= 1'b0;
        cmd <= C_NOP; ba <= 2'd0; sa <= 13'd0; dqm <= 2'b11; dq_oe <= 1'b0;
        r_we <= 1'b0; r_addr <= 24'd0; r_len <= 9'd0; k <= 9'd0; nrx <= 9'd0;
        wr_idx <= 9'd0; wr_take <= 1'b0; rd_valid <= 1'b0; rd_data <= 16'd0;
    end else begin
        cmd <= C_NOP; done <= 1'b0; ack <= 1'b0; wr_take <= 1'b0; rd_valid <= 1'b0;
        cnt <= (cnt == 15'd0) ? 15'd0 : cnt - 15'd1;

        // read data arrives independently of the command state machine
        // k = 1 + clocks since the READ command was on the pins; word 0 is sampled RD_LAT clocks after it
        if (!r_we && (st == S_XFER || st == S_PRE || st == S_GAP) && k > RD_LAT && nrx < r_len) begin
            rd_valid <= 1'b1; rd_data <= dq_in; nrx <= nrx + 9'd1;
        end

        case (st)
        //------------------------------------------------ power-up initialisation
        S_WAIT: if (cnt == 15'd0 && nref == 4'd0) begin cnt <= T_INIT; nref <= 4'd1; end
                else if (cnt == 15'd1) begin cmd <= C_PRE; sa[10] <= 1'b1; cnt <= 15'd3; nref <= 4'd0; st <= S_IPRE; end
        S_IPRE: if (cnt == 15'd1) begin cmd <= C_REF; cnt <= 15'd8; st <= S_IREF; end
        S_IREF: if (cnt == 15'd1) begin
                    if (nref == 4'd7) begin cmd <= C_MRS; ba <= 2'b00; sa <= MODE; cnt <= 15'd4; st <= S_IMRS; end
                    else begin cmd <= C_REF; cnt <= 15'd8; nref <= nref + 4'd1; end
                end
        S_IMRS: if (cnt == 15'd1) begin init_done <= 1'b1; dqm <= 2'b00; st <= S_IDLE; end
        //------------------------------------------------ normal operation
        S_IDLE:
            if (req && !ack) begin                                 // a pending request goes first ...
                r_we <= we; r_addr <= addr; r_len <= len; ack <= 1'b1;
                cmd <= C_ACT; ba <= addr[10:9]; sa <= addr[23:11];
                cnt <= 15'd2; st <= S_ACT;
            end else if (ref_due) begin cmd <= C_REF; cnt <= 15'd8; st <= S_REF; end   // ... refresh in between
        S_ACT:  if (cnt == 15'd1) begin                          // tRCD satisfied in the next clock
                    ba <= r_addr[10:9]; sa <= {4'b0000, r_addr[8:0]};
                    k <= 9'd0; nrx <= 9'd0;
                    if (r_we) begin                               // word 0 goes out together with WRITE
                        cmd <= C_WR; dq_oe <= 1'b1; wr_idx <= 9'd0; wr_take <= 1'b1; k <= 9'd1; st <= S_XFER;
                    end else begin
                        cmd <= C_RD; k <= 9'd1; st <= S_XFER;
                    end
                end
        S_XFER:
            if (r_we) begin
                if (k < r_len) begin                              // word k in the next clock
                    wr_take <= 1'b1; wr_idx <= k; k <= k + 9'd1;
                end else begin                                    // all words sent: stop the burst
                    cmd <= C_BST; dq_oe <= 1'b0; cnt <= 15'd2; st <= S_PRE;
                end
            end else begin
                k <= k + 9'd1;
                if (k >= r_len && k >= 9'd4) begin                // cut the read burst (tRAS >= 5 from ACT)
                    cmd <= C_PRE; sa[10] <= 1'b1; cnt <= RD_LAT + 2; st <= S_GAP;
                end
            end
        S_PRE:  if (cnt == 15'd1) begin cmd <= C_PRE; sa[10] <= 1'b1; cnt <= 15'd3; st <= S_GAP; end   // after tWR
        S_GAP:  begin
                    if (!r_we) k <= k + 9'd1;
                    if (cnt == 15'd1) begin done <= 1'b1; st <= S_IDLE; end
                end
        S_REF:  if (cnt == 15'd1) st <= S_IDLE;
        default: st <= S_IDLE;
        endcase
    end

endmodule
