/*
 * kws_qspi - QSPI master for the Tiny Tapeout QSPI Pmod
 *            (W25Q128 flash on CS0, APS6404L PSRAM on CS1/CS2)
 *
 * Read  (stream): CS | 0xEB on SD0 (8) | addr quad (6) | mode 00 (2) | dummy hi-Z (4) | data...
 *                 6 wait clocks for both PSRAM and flash. Bursts end at aligned
 *                 2^(burst+2)-byte boundaries (4..128 B for flash, max 32 B for PSRAM) to respect
 *                 PSRAM tCEM = 8 us and the 1 kB page; the stream resumes at the next address.
 * Write (1 byte, PSRAM CS2 only): CS | 0x38 on SD0 (8) | addr quad (6) | data quad (2)
 *                 Serviced between read bursts (or when idle).
 * SCK = clk/(2*div).  Read nibbles are captured rd_dly+1 clk after the launching falling edge.
 */
`default_nettype none

module kws_qspi (
    input  wire        clk,
    input  wire        rst_n,
    input  wire [2:0]  div,        // SCK half period in clk (0 treated as 1)
    input  wire [2:0]  rd_dly,
    input  wire [2:0]  burst,      // n: 2^(n+2) bytes, n>5 treated as 5
    // read stream
    input  wire        start,      // pulse (only accepted when idle)
    input  wire [23:0] addr,
    input  wire [1:0]  cs_sel,     // chip select for this stream
    input  wire        stop,       // pulse / level: end stream
    output reg         byte_valid,
    output reg  [7:0]  byte_data,
    output wire        busy,
    // single-byte write to PSRAM B (CS2)
    input  wire        wr_req,     // level, held until wr_ack
    input  wire [23:0] wr_addr,
    input  wire [7:0]  wr_data,
    output reg         wr_ack,     // pulse when the write transaction has completed
    // pads
    output reg         sck,
    output reg         cs_act,
    output reg  [1:0]  cs_cur,
    output reg  [3:0]  sd_out,
    output reg  [3:0]  sd_oe,
    input  wire [3:0]  sd_in
);
    localparam S_IDLE = 2'd0, S_GAP = 2'd1, S_SHIFT = 2'd2, S_DRAIN = 2'd3;

    reg [1:0]  state;
    reg        streaming;      // a read stream is active (resumes after writes)
    reg        tx_wr;          // current transaction is a write
    reg [23:0] cur;            // address of next read burst
    reg [1:0]  rcs;            // stream chip select
    reg [8:0]  pulse;
    reg [8:0]  npulse;
    reg [2:0]  hc;
    reg [3:0]  gap;
    reg [7:0]  launch;
    reg        nib_hi;
    reg [3:0]  hi_nib;
    reg [7:0]  bytes_in_burst;

    wire [2:0] hdiv  = (div == 3'd0) ? 3'd1 : div;
    wire       half  = (hc == hdiv - 3'd1);
    // flash (CS0): up to 128 B; PSRAM (CS1/CS2): never more than 32 B so CS stays < 8 us (tCEM)
    wire [2:0] bmax  = (rcs == 2'd0) ? 3'd5 : 3'd3;
    wire [2:0] bcode = (burst > bmax) ? bmax : burst;
    wire [7:0] blen  = (8'd4 << bcode);
    wire [7:0] boff  = cur[7:0] & (blen - 8'd1);
    wire [7:0] bcnt  = blen - boff;                        // bytes to the aligned boundary (1..128)

    assign busy = (state != S_IDLE);
    wire capture = launch[rd_dly];

    // command / address / data nibble for SCK pulse 'pulse+1'
    wire [7:0]  cmd     = tx_wr ? 8'h38 : 8'hEB;
    wire [7:0]  cmd_sh  = cmd << pulse[2:0];
    wire        cmd_bit = cmd_sh[7];
    wire [23:0] a_cur   = tx_wr ? wr_addr : cur;
    reg  [3:0]  addr_nib;
    always @(*) begin
        case (pulse[3:0])
            4'd8:    addr_nib = a_cur[23:20];
            4'd9:    addr_nib = a_cur[19:16];
            4'd10:   addr_nib = a_cur[15:12];
            4'd11:   addr_nib = a_cur[11:8];
            4'd12:   addr_nib = a_cur[7:4];
            default: addr_nib = a_cur[3:0];
        endcase
    end
    reg [3:0] nib_next, oe_next;
    always @(*) begin
        if (pulse < 9'd8) begin
            nib_next = {2'b11, 1'b0, cmd_bit};
            oe_next  = 4'b1101;
        end else if (pulse < 9'd14) begin
            nib_next = addr_nib;
            oe_next  = 4'b1111;
        end else if (pulse < 9'd16) begin
            nib_next = tx_wr ? ((pulse == 9'd14) ? wr_data[7:4] : wr_data[3:0]) : 4'b0000;
            oe_next  = 4'b1111;
        end else begin
            nib_next = 4'b0000;
            oe_next  = 4'b0000;
        end
    end
    wire _unused_q = &{cmd_sh[6:0], 1'b0};

    always @(posedge clk) begin
        if (!rst_n) begin
            state <= S_IDLE; streaming <= 1'b0; tx_wr <= 1'b0; cur <= 24'd0; rcs <= 2'd1;
            pulse <= 9'd0; npulse <= 9'd0; hc <= 3'd0; gap <= 4'd0;
            launch <= 8'd0; nib_hi <= 1'b1; hi_nib <= 4'd0; bytes_in_burst <= 8'd0;
            sck <= 1'b0; cs_act <= 1'b0; cs_cur <= 2'd1; sd_out <= 4'b1100; sd_oe <= 4'b0000;
            byte_valid <= 1'b0; byte_data <= 8'd0; wr_ack <= 1'b0;
        end else begin
            byte_valid <= 1'b0;
            wr_ack     <= 1'b0;
            launch     <= {launch[6:0], 1'b0};
            if (capture && !tx_wr && streaming) begin
                if (nib_hi) begin
                    hi_nib <= sd_in;
                    nib_hi <= 1'b0;
                end else begin
                    byte_data  <= {hi_nib, sd_in};
                    byte_valid <= 1'b1;
                    nib_hi     <= 1'b1;
                end
            end
            if (stop) streaming <= 1'b0;
            case (state)
                S_IDLE: begin
                    sck <= 1'b0; cs_act <= 1'b0; sd_oe <= 4'b0000;
                    if (start && !stop) begin
                        cur <= addr; rcs <= cs_sel; streaming <= 1'b1;
                        gap <= 4'd0; state <= S_GAP;
                    end else if (wr_req) begin
                        gap <= 4'd0; state <= S_GAP;
                    end
                end
                S_GAP: begin                           // CS high time, then begin a transaction
                    cs_act <= 1'b0;
                    sck    <= 1'b0;
                    sd_oe  <= 4'b0000;
                    gap    <= gap + 4'd1;
                    if (gap == {hdiv, 1'b1}) begin
                        if (wr_req) begin
                            tx_wr  <= 1'b1;
                            cs_cur <= 2'd2;
                            npulse <= 9'd16;
                            sd_out <= 4'b1100;              // 0x38 MSB = 0
                        end else if (streaming && !stop) begin
                            tx_wr  <= 1'b0;
                            cs_cur <= rcs;
                            npulse <= 9'd19 + {bcnt, 1'b0};
                            bytes_in_burst <= bcnt;
                            sd_out <= 4'b1101;              // 0xEB MSB = 1
                        end else begin
                            state <= S_IDLE;
                        end
                        if (wr_req || (streaming && !stop)) begin
                            cs_act <= 1'b1;
                            pulse  <= 9'd0;
                            hc     <= 3'd0;
                            nib_hi <= 1'b1;
                            sd_oe  <= 4'b1101;
                            state  <= S_SHIFT;
                        end
                    end
                end
                S_SHIFT: begin
                    hc <= half ? 3'd0 : hc + 3'd1;
                    if (stop && !tx_wr) begin
                        state <= S_IDLE; cs_act <= 1'b0; sck <= 1'b0; sd_oe <= 4'b0000;
                    end else if (half) begin
                        if (!sck) begin                     // rising edge of pulse+1
                            sck   <= 1'b1;
                            pulse <= pulse + 9'd1;
                        end else begin                      // falling edge of 'pulse'
                            sck    <= 1'b0;
                            sd_out <= nib_next;
                            sd_oe  <= oe_next;
                            if (!tx_wr && pulse >= 9'd20) launch[0] <= 1'b1;
                            if (pulse == npulse) state <= S_DRAIN;
                        end
                    end
                end
                S_DRAIN: begin                              // last capture done -> CS high
                    if (launch == 8'd0 && !capture) begin
                        cs_act <= 1'b0;
                        sd_oe  <= 4'b0000;
                        if (tx_wr) wr_ack <= 1'b1;
                        else       cur    <= cur + {16'd0, bytes_in_burst};
                        tx_wr <= 1'b0;
                        gap   <= 4'd0;
                        state <= S_GAP;
                    end
                end
            endcase
        end
    end
endmodule
