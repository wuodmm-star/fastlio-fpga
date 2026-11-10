`timescale 1ns/1ps
// simulation stand-in for the altpll clk_gen: c1 = 100 MHz, c2 = c1 leading by 0.833 ns, locked after 1 us
module clk_gen (input areset, input inclk0, output reg c0 = 0, output reg c1 = 0, output reg c2 = 0, output reg locked = 0);
    always #5 c1 = ~c1;
    always @(c1) c2 <= #9.167 c1;
    always #10 c0 = ~c0;
    initial #1000 locked = 1;
endmodule
