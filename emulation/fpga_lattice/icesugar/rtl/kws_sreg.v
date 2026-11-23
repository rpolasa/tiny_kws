/*
 * kws_sreg - W-bit register with a common load enable (the storage element of the rings).
 *
 * Default: plain enable flip-flops (synthesised as mux2 + dfxtp, 32.5 um2 per bit in sky130_fd_sc_hd).
 * With KWS_SKY130_SCANFF defined: one sky130_fd_sc_hd__sdfxtp_1 per bit, using the cell's built-in
 * scan multiplexer as the load multiplexer (D = hold, SCD = new data, SCE = enable):
 * 26.3 um2 per bit.  Functionally identical; the SCD path is the slower input, which helps hold.
 * The power pins are left to the flow (LibreLane connects them like those of every other cell);
 * tying them here makes Odb.SetPowerConnections fail.
 */
`default_nettype none

module kws_sreg #(
    parameter W = 8
) (
    input  wire         clk,
    input  wire         en,
    input  wire [W-1:0] d,
    output wire [W-1:0] q
);
`ifdef KWS_SKY130_SCANFF
    genvar i;
    generate
        for (i = 0; i < W; i = i + 1) begin : g_bit
            (* keep *) sky130_fd_sc_hd__sdfxtp_1 ff (
                .CLK(clk), .D(q[i]), .SCD(d[i]), .SCE(en), .Q(q[i])
            );
        end
    endgenerate
`else
    reg [W-1:0] r;
    always @(posedge clk) if (en) r <= d;
    assign q = r;
`endif
endmodule
