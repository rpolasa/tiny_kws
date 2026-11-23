/*
 * top_icesugar - tiny_kws rev B (tt_um_rakesh_tiny_kws, RTL unchanged in rtl/) on the
 *                Muse Lab iCESugar v1.5 (iCE40UP5K-SG48) for FPGA emulation.
 *
 *   12 MHz (iCELink MCO, pin 35) -> PLL 16.3125 MHz = chip clk (fs -0.44 % with PDM_DIV 7)
 *   PMOD2 : uio[7:0] in Tiny Tapeout QSPI-Pmod order (CS0 SD0 SD1 SCK / SD2 SD3 CS1 CS2):
 *           Pico 2 W PSRAM emulator now, the real TT QSPI Pmod later - same connector.
 *   PMOD3 : PDM mic + host SPI (Pico) + DETECT + FRAME
 *   PMOD1 : DET_CLASS[3:0] (only the 4 pins not shared with iCELink USB/UART)
 *   PMOD4 pin 21 : host reset (active low, 4.7k pull-up on the board - keep DIP switch 1 OFF)
 *   RGB LED : green = detection (stretched 0.2 s), blue = frame heartbeat, red = chip not running
 * Define SIM to bypass the PLL (testbench drives clk_12m at the chip clock).
 */
`default_nettype none

module top_icesugar (
    input  wire       clk_12m,
    // PMOD3
    input  wire       pdm_dat,
    output wire       pdm_clk,
    input  wire       spi_sck,
    input  wire       spi_mosi,
    input  wire       spi_csn,
    output wire       spi_miso,
    output wire       detect,
    output wire       frame,
    // PMOD1
    output wire [3:0] det_class,
    // PMOD2
    inout  wire [7:0] uio,
    // PMOD4 / SW1
    input  wire       host_rst_n,
    // RGB LED (open-drain current sinks)
    output wire       led_r,
    output wire       led_g,
    output wire       led_b
);
    // ---------------------------------------------------------------- clock
    wire clk;
`ifdef SIM
    assign clk = clk_12m;
`elsif CLK12
    assign clk = clk_12m;                 // diagnostic build: chip clock = 12 MHz oscillator, no PLL
`else
    // icepll -i 12 -o 16.3125 : DIVR 0, DIVF 86, DIVQ 6, FILTER_RANGE 1 (VCO 1044 MHz)
    SB_PLL40_PAD #(
        .FEEDBACK_PATH("SIMPLE"),
        .DIVR(4'b0000),
        .DIVF(7'b1010110),
        .DIVQ(3'b110),
        .FILTER_RANGE(3'b001)
    ) u_pll (
        .RESETB(1'b1),
        .BYPASS(1'b0),
        .PACKAGEPIN(clk_12m),
        .PLLOUTGLOBAL(clk)
    );
`endif

    // ---------------------------------------------------------------- reset
    // power-on: ~4 ms after configuration (FFs start at 0 on iCE40), then follow the host pin
`ifdef SIM
    localparam POR_BIT = 6;
`else
    localparam POR_BIT = 15;
`endif
    reg [15:0] por_cnt = 16'd0;
    wire       por_done = por_cnt[POR_BIT];
    always @(posedge clk) if (!por_done) por_cnt <= por_cnt + 16'd1;
    reg [1:0]  hr_sync = 2'b00;
    always @(posedge clk) hr_sync <= {hr_sync[0], host_rst_n};
    wire rst_n = por_done & hr_sync[1];          // the chip adds its own 2-FF synchroniser

    // ---------------------------------------------------------------- the chip (unchanged RTL)
    wire [7:0] ui_in = {4'b0000, spi_csn, spi_mosi, spi_sck, pdm_dat};
    wire [7:0] uo_out, uio_out, uio_oe, uio_in;

    tt_um_rakesh_tiny_kws u_kws (
        .ui_in(ui_in), .uo_out(uo_out),
        .uio_in(uio_in), .uio_out(uio_out), .uio_oe(uio_oe),
        .ena(1'b1), .clk(clk), .rst_n(rst_n)
    );

    assign pdm_clk   = uo_out[0];
    assign spi_miso  = uo_out[1];
    assign detect    = uo_out[2];
    assign frame     = uo_out[3];
    assign det_class = uo_out[7:4];

    // ---------------------------------------------------------------- uio pads (tristate)
    // CS0/CS1/CS2 get the pad pull-up so they read high while the chip releases the bus (RUN = 0)
    genvar i;
    generate
        for (i = 0; i < 8; i = i + 1) begin : g_uio
            SB_IO #(
                .PIN_TYPE(6'b1010_01),               // output with tristate enable, plain input
                .PULLUP((i == 0 || i == 6 || i == 7) ? 1'b1 : 1'b0)
            ) u_io (
                .PACKAGE_PIN(uio[i]),
                .OUTPUT_ENABLE(uio_oe[i]),
                .D_OUT_0(uio_out[i]),
                .D_IN_0(uio_in[i])
            );
        end
    endgenerate

    // ---------------------------------------------------------------- LEDs
    reg [21:0] det_stretch = 22'd0;                  // 2^22 / 16.3 MHz = 0.26 s
    always @(posedge clk)
        if (detect) det_stretch <= {22{1'b1}};
        else if (det_stretch != 22'd0) det_stretch <= det_stretch - 22'd1;
    wire running = uio_oe[7];                        // CS2 is driven only while CTRL.RUN = 1

`ifdef SIM
    assign led_g = ~(det_stretch != 22'd0);
    assign led_b = ~frame;
    assign led_r = running;
`else
    SB_RGBA_DRV #(
        .CURRENT_MODE("0b1"),                        // half current
        .RGB0_CURRENT("0b000001"),                   // 2 mA per LED is plenty
        .RGB1_CURRENT("0b000001"),
        .RGB2_CURRENT("0b000001")
    ) u_rgb (
        .CURREN(1'b1),
        .RGBLEDEN(1'b1),
        .RGB0PWM(frame),                             // RGB0 = pin 39 = blue
        .RGB1PWM(~running),                          // RGB1 = pin 40 = red
        .RGB2PWM(det_stretch != 22'd0),              // RGB2 = pin 41 = green
        .RGB0(led_b),
        .RGB1(led_r),
        .RGB2(led_g)
    );
`endif
endmodule
