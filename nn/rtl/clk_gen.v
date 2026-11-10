`timescale 1ns/1ps
// 50 MHz board oscillator -> c0 50 MHz, c1 100 MHz (logic), c2 100 MHz shifted -833 ps (SDRAM clock pin)
module clk_gen (input wire areset, input wire inclk0, output wire c0, output wire c1, output wire c2, output wire locked);
    wire [4:0] clk;
    assign {c2, c1, c0} = clk[2:0];
    altpll altpll_component (.areset (areset), .inclk ({1'b0, inclk0}), .clk (clk), .locked (locked));
    defparam
        altpll_component.intended_device_family = "Cyclone IV E",
        altpll_component.lpm_type = "altpll",
        altpll_component.operation_mode = "NORMAL",
        altpll_component.pll_type = "AUTO",
        altpll_component.inclk0_input_frequency = 20000,
        altpll_component.clk0_multiply_by = 1, altpll_component.clk0_divide_by = 1, altpll_component.clk0_duty_cycle = 50,
        altpll_component.clk0_phase_shift = "0",
        altpll_component.clk1_multiply_by = 2, altpll_component.clk1_divide_by = 1, altpll_component.clk1_duty_cycle = 50,
        altpll_component.clk1_phase_shift = "0",
        altpll_component.clk2_multiply_by = 2, altpll_component.clk2_divide_by = 1, altpll_component.clk2_duty_cycle = 50,
        altpll_component.clk2_phase_shift = "-833",
        altpll_component.port_areset = "PORT_USED", altpll_component.port_locked = "PORT_USED",
        altpll_component.port_clk0 = "PORT_USED", altpll_component.port_clk1 = "PORT_USED", altpll_component.port_clk2 = "PORT_USED",
        altpll_component.port_clk3 = "PORT_UNUSED", altpll_component.port_clk4 = "PORT_UNUSED",
        altpll_component.width_clock = 5;
endmodule
