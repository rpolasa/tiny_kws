/*
 * tb_fpga - co-simulation of the FPGA build (top_icesugar + unchanged chip RTL) with the Pico 2 W
 * Tiny Tapeout QSPI Pmod emulator (flash CS0, PSRAM A CS1, PSRAM B CS2; r15). The Pico side (PIO state machines executing the assembled qspi_psram.pio
 * opcodes, DMA re-arm, core-1 server) is modelled in Python (pio_model.py) and drives pico_sd_*.
 * Board delays: FPGA pad -> Pico pin T_F2P, Pico pin -> FPGA flip-flop T_P2F.
 */
`timescale 1ns / 1ps
`default_nettype none

module tb_fpga #(
    parameter real T_F2P = 6.0,
    parameter real T_P2F = 8.0
) ();
    reg        clk = 1'b0;                 // chip clock (PLL output) driven by cocotb
    reg        host_rst_n = 1'b0;
    reg        spi_sck = 1'b0, spi_mosi = 1'b0, spi_csn = 1'b1;
    reg        pdm_dat = 1'b0;
    wire       pdm_clk, spi_miso, detect, frame, led_r, led_g, led_b;
    wire [3:0] det_class;
    wire [7:0] uio;

    // ---- Pico side of the QSPI bus (driven by the Python PIO model)
    reg  [3:0] pico_sd_out = 4'h0, pico_sd_oe = 4'h0;
    wire [3:0] pico_drv;
    assign #(T_P2F) pico_drv[0] = pico_sd_oe[0] ? pico_sd_out[0] : 1'bz;
    assign #(T_P2F) pico_drv[1] = pico_sd_oe[1] ? pico_sd_out[1] : 1'bz;
    assign #(T_P2F) pico_drv[2] = pico_sd_oe[2] ? pico_sd_out[2] : 1'bz;
    assign #(T_P2F) pico_drv[3] = pico_sd_oe[3] ? pico_sd_out[3] : 1'bz;
    assign uio[1] = pico_drv[0];           // SD0
    assign uio[2] = pico_drv[1];           // SD1
    assign uio[4] = pico_drv[2];           // SD2
    assign uio[5] = pico_drv[3];           // SD3
    // pulls are resolved explicitly (iverilog loses pull strength through the inout port chain)

    // what the Pico pins see
    wire [3:0] p_sd;
    wire       p_sck, p_cs0, p_cs1, p_cs2, p_csall;
    assign #(T_F2P) p_sd  = {uio[5], uio[4], uio[2], uio[1]};
    assign #(T_F2P) p_sck = (uio[3] === 1'bz) ? 1'b0 : uio[3];   // Pico GP14 pull-down
    assign #(T_F2P) p_cs0 = (uio[0] === 1'bz) ? 1'b1 : uio[0];   // Pico GP7 pull-up (flash)
    assign #(T_F2P) p_cs1 = (uio[6] === 1'bz) ? 1'b1 : uio[6];   // Pico GP8 pull-up (PSRAM A)
    assign #(T_F2P) p_cs2 = (uio[7] === 1'bz) ? 1'b1 : uio[7];   // Pico GP9 pull-up (PSRAM B)
    assign p_csall = p_cs0 & p_cs1 & p_cs2;                      // testbench only: "no transaction"

    top_icesugar dut (
        .clk_12m(clk), .pdm_dat(pdm_dat), .pdm_clk(pdm_clk),
        .spi_sck(spi_sck), .spi_mosi(spi_mosi), .spi_csn(spi_csn), .spi_miso(spi_miso),
        .detect(detect), .frame(frame), .det_class(det_class), .uio(uio),
        .host_rst_n(host_rst_n), .led_r(led_r), .led_g(led_g), .led_b(led_b)
    );

    // ---- bus contention: FPGA and Pico driving an SD line at the same time
    integer contention = 0;
`ifdef GATE_SIM
    wire [3:0] fpga_oe = 4'b0000;          // netlist: internal names not kept
`else
    wire [3:0] fpga_oe = {dut.uio_oe[5], dut.uio_oe[4], dut.uio_oe[2], dut.uio_oe[1]};
`endif
    always @(posedge clk) if (|(fpga_oe & pico_sd_oe)) contention = contention + 1;
endmodule

`ifndef GATE_SIM
// minimal SB_IO model (PIN_TYPE 6'b1010_01: tristate output, plain input), pull-up option
module SB_IO #(
    parameter [5:0] PIN_TYPE = 6'b000000,
    parameter [0:0] PULLUP = 1'b0
) (
    inout  wire PACKAGE_PIN,
    input  wire OUTPUT_ENABLE,
    input  wire D_OUT_0,
    output wire D_IN_0
);
    assign PACKAGE_PIN = OUTPUT_ENABLE ? D_OUT_0 : 1'bz;
    assign D_IN_0 = (PACKAGE_PIN === 1'bz) ? PULLUP : PACKAGE_PIN;   // pad pull-up when undriven
endmodule
`endif
