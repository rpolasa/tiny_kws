// Simulation stand-in for sky130_fd_sc_hd__sdfxtp_1 (scan flip-flop): used only to run the RTL
// regression with KWS_SKY130_SCANFF defined when the PDK Verilog models are not available.
`default_nettype none
module sky130_fd_sc_hd__sdfxtp_1 (
    input  wire CLK, D, SCD, SCE,
    output reg  Q
);
    always @(posedge CLK) Q <= SCE ? SCD : D;
endmodule
