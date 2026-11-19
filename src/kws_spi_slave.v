/*
 * kws_spi_slave - host register interface (SPI mode 0, MSB first)
 *
 *   byte 0      : {R/nW, addr[6:0]}
 *   byte 1..n   : write data, or read data on MISO; address auto-increments
 *                 except for the stream ports (NOINC_A / NOINC_B)
 *
 * All SPI pins are oversampled by clk (2-FF synchronisers):  f_SCK <= f_clk/8.
 */
`default_nettype none

module kws_spi_slave #(
    parameter [6:0] NOINC_A = 7'h10,
    parameter [6:0] NOINC_B = 7'h11
) (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       sck_pin,
    input  wire       mosi_pin,
    input  wire       csn_pin,
    output reg        miso,
    output wire       active,      // chip select asserted (synchronised)
    // register bus
    output reg        wr_stb,
    output reg  [6:0] addr,
    output reg  [7:0] wdata,
    input  wire [7:0] rdata
);
    reg [2:0] sck_s, csn_s;
    reg [1:0] mosi_s;
    always @(posedge clk) begin
        if (!rst_n) begin
            sck_s  <= 3'b000;
            csn_s  <= 3'b111;
            mosi_s <= 2'b00;
        end else begin
            sck_s  <= {sck_s[1:0], sck_pin};
            csn_s  <= {csn_s[1:0], csn_pin};
            mosi_s <= {mosi_s[0], mosi_pin};
        end
    end
    wire sck_rise = (sck_s[2:1] == 2'b01);
    wire sck_fall = (sck_s[2:1] == 2'b10);
    wire cs_act   = ~csn_s[1];
    assign active = cs_act;
    wire   _unused_s = &{csn_s[2], 1'b0};

    reg [2:0] bitcnt;
    reg [6:0] shreg;
    reg       first, is_read, load_tx;
    reg [7:0] txsh;

    wire [7:0] rx_byte = {shreg, mosi_s[1]};
    wire       noinc   = (addr == NOINC_A) || (addr == NOINC_B);

    always @(posedge clk) begin
        if (!rst_n) begin
            bitcnt  <= 3'd0;
            shreg   <= 7'd0;
            first   <= 1'b1;
            is_read <= 1'b0;
            load_tx <= 1'b0;
            txsh    <= 8'd0;
            miso    <= 1'b0;
            wr_stb  <= 1'b0;
            addr    <= 7'd0;
            wdata   <= 8'd0;
        end else begin
            wr_stb <= 1'b0;
            if (load_tx) begin
                txsh    <= rdata;   // rdata reflects 'addr' (updated last cycle)
                load_tx <= 1'b0;
            end
            if (!cs_act) begin
                bitcnt <= 3'd0;
                first  <= 1'b1;
                miso   <= 1'b0;
            end else begin
                if (sck_rise) begin
                    shreg  <= rx_byte[6:0];
                    bitcnt <= bitcnt + 3'd1;
                    if (bitcnt == 3'd7) begin
                        if (first) begin
                            first   <= 1'b0;
                            is_read <= rx_byte[7];
                            addr    <= rx_byte[6:0];
                            load_tx <= rx_byte[7];
                        end else if (!is_read) begin
                            wr_stb <= 1'b1;
                            wdata  <= rx_byte;
                        end else begin
                            if (!noinc) addr <= addr + 7'd1;
                            load_tx <= 1'b1;
                        end
                    end
                end
                if (sck_fall) begin
                    miso <= is_read & ~first & txsh[7];
                    txsh <= {txsh[6:0], 1'b0};
                end
                // write-address auto increment happens the cycle after the strobe
                if (wr_stb && !noinc) addr <= addr + 7'd1;
            end
        end
    end
endmodule
