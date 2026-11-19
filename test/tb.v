`default_nettype none
`timescale 1ns / 1ps

/* Testbench: DUT + QSPI Pmod (flash on CS0, PSRAM A on CS1, PSRAM B on CS2) + PDM microphone.
 * Driven from cocotb (test/test_kws.py). */
module tb ();
    initial begin
        if ($test$plusargs("DUMP")) begin
            $dumpfile("tb.fst");
            $dumpvars(0, tb);
        end
    end

    reg        clk, rst_n, ena;
    reg  [7:0] ui_ctl;          // ui_in driven by cocotb (bit 0 replaced by mic when pdm_on)
    reg        pdm_on;
    wire [7:0] ui_in;
    wire [7:0] uo_out, uio_out, uio_oe;
    wire [7:0] uio;
    wire [7:0] uio_in = uio;

`ifdef GL_TEST
    wire VPWR = 1'b1;
    wire VGND = 1'b0;
`endif

    tt_um_rakesh_tiny_kws user_project (
`ifdef GL_TEST
        .VPWR(VPWR),
        .VGND(VGND),
`endif
        .ui_in  (ui_in),
        .uo_out (uo_out),
        .uio_in (uio_in),
        .uio_out(uio_out),
        .uio_oe (uio_oe),
        .ena    (ena),
        .clk    (clk),
        .rst_n  (rst_n)
    );

    // bidirectional pads
    genvar gi;
    generate
        for (gi = 0; gi < 8; gi = gi + 1) begin : pad
            assign uio[gi] = uio_oe[gi] ? uio_out[gi] : 1'bz;
        end
    endgenerate
    pullup (uio[0]);   // CS lines have pull-ups on the Pmod
    pullup (uio[6]);
    pullup (uio[7]);
    pulldown (uio[3]); // SCK

    wire drv_f, drv_a, drv_b;
    psram_model #(.T_CEM(0.0), .CHECK_PAGE(0), .WRITABLE(0)) flash (.csn(uio[0]), .sck(uio[3]), .sd({uio[5], uio[4], uio[2], uio[1]}), .drive(drv_f));
    psram_model                               ramA  (.csn(uio[6]), .sck(uio[3]), .sd({uio[5], uio[4], uio[2], uio[1]}), .drive(drv_a));
    psram_model                               ramB  (.csn(uio[7]), .sck(uio[3]), .sd({uio[5], uio[4], uio[2], uio[1]}), .drive(drv_b));

    // bus contention monitor
    integer contention = 0;
    always @(posedge clk) begin
        if ((drv_f | drv_a | drv_b) && (uio_oe[1] | uio_oe[2] | uio_oe[4] | uio_oe[5])) contention = contention + 1;
        if ((drv_f + drv_a + drv_b) > 1) contention = contention + 1;
    end

    // PDM microphone: new bit T_DV after PDM_CLK falling edge (data for rising-edge capture)
    reg        mic_bit;
    reg  [0:0] pdm_mem [0:1048575];
    integer    pdm_ptr = 0;
    always @(negedge uo_out[0]) if (pdm_on) begin
        #60 mic_bit = pdm_mem[pdm_ptr][0];
        pdm_ptr = pdm_ptr + 1;
    end
    reg pdm_load;
    always @(posedge pdm_load) $readmemb("pdm_bits.mem", pdm_mem);
    initial begin mic_bit = 0; pdm_on = 0; pdm_load = 0; end

    assign ui_in = {ui_ctl[7:1], pdm_on ? mic_bit : ui_ctl[0]};
endmodule
