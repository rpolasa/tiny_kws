/*
 * kws_frontend - audio front end (bit-exact with model/kws_fe.c)          [area-optimised, rev B]
 *
 *   PDM mic  -> CIC3 (R=64, 19-bit, pipelined integrators) -> int16 ┐
 *   host PCM (int16 via SPI) ───────────────────────────────────────┴-> x0
 *   x0 -> N (16) x band-pass biquad (DF1, shared x delay line, bit-serial multiply-accumulate)
 *      -> |y|>>4 accumulated per hop (256/512 samples) -> log2 (p*8 + 3-bit mantissa) -> u8
 *
 * Channel state lives in an N-entry rotating shift register (no address decode):
 *   entry = {y1[16:0], y2[16:0], e[19:0]} (y in Q.1), head = channel being processed.
 *
 * Multiply-accumulate (rev B).  y = sat17((A*(2d) + C1*y1 - C2*y2 + 2^15) >>> 16) is evaluated
 * LSB-first over the 18 coefficient bits with a right-shifting 22-bit accumulator:
 *     for b = 0..15 :  R = (R + A[b]*2d + C1[b]*y1 - C2[b]*y2 + (b==15)) >>> 1
 *     b = 16        :  R =  R + (A[b]*2d + C1[b]*y1 - C2[b]*y2)
 *     b = 17        :  R =  R - 2*(A[b]*2d + C1[b]*y1 - C2[b]*y2)        (sign bit, weight -2^17)
 * The bits shifted out can never change the result again (floor((floor(p/2)+t)/2) = floor((p+2t)/4)),
 * and the +1 at b = 15 is the 2^15 rounding constant, so R equals (acc >>> 16) of the reference
 * exactly.  One 22-bit adder, one product term per clock: 3*18 + 1 = 55 clk per channel.
 * The operands are read straight from the x registers / ring head and the coefficient bit
 * straight from the ROM, so no operand, coefficient or write-back staging registers exist.
 *
 * Features: after the last sample of a hop the ring makes one extra turn (N clk); each entry
 * passing the head emits log2(e) and has e cleared.
 */
`default_nettype none

module kws_frontend #(
    parameter N = 16             // filterbank channels
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        clear,        // zero all state (pulse)
    input  wire        run,
    input  wire        src_pcm,      // 1: samples from host, 0: PDM microphone
    input  wire        hop512,
    input  wire [7:0]  pdm_div,      // PDM clk = clk / (2*(pdm_div+1))
    input  wire        pdm_edge,     // 0: capture at PDM_CLK rising, 1: falling
    input  wire        pdm_pin,
    output reg         pdm_clk_o,
    input  wire        pcm_stb,
    input  wire [15:0] pcm_data,     // must be stable in the cycle pcm_stb is high
    input  wire        feat_en,      // deliver features to the NN
    input  wire        nn_busy,
    output reg         feat_valid,
    output reg  [7:0]  feat_data,
    output reg         overrun,      // pulse: sample or frame dropped
    output wire        busy
);
    localparam W = 54;
    localparam integer NLI = N - 1;
    localparam [3:0] NL = NLI[3:0];

    // ------------------------------------------------------------------ PDM + CIC3
    reg [1:0]  pdm_s;
    reg [7:0]  pdm_cnt;
    reg [5:0]  dec_cnt;
    reg [18:0] ci0, ci1, ci2;
    reg [18:0] cc0, cc1, cc2, cv;
    reg [1:0]  comb_st;          // 0 idle, 1..3 comb stages
    reg        cic_stb;          // cv holds a new sample (cv is then stable until the next one)

    wire pdm_tick  = run & ~src_pcm & (pdm_cnt == pdm_div);
    wire pdm_capt  = pdm_tick & (pdm_clk_o == pdm_edge);  // toggling into the capture edge
    wire [18:0] pin_v = pdm_s[1] ? 19'd1 : {19{1'b1}};

    wire [18:0] cc_sel = (comb_st == 2'd2) ? cc1 : cc2;
    wire [18:0] comb_d = cv - cc_sel;
    // 19-bit result >>> 3 -> exactly 16 bits (full scale 2^18 -> 2^15), no saturation needed
    wire [15:0] cic_out = cv[18:3];

    always @(posedge clk) begin
        if (!rst_n || clear) begin
            pdm_s <= 2'b00; pdm_cnt <= 8'd0; pdm_clk_o <= 1'b0; dec_cnt <= 6'd0;
            ci0 <= 19'd0; ci1 <= 19'd0; ci2 <= 19'd0;
            cc0 <= 19'd0; cc1 <= 19'd0; cc2 <= 19'd0; cv <= 19'd0;
            comb_st <= 2'd0; cic_stb <= 1'b0;
        end else begin
            pdm_s   <= {pdm_s[0], pdm_pin};
            cic_stb <= 1'b0;
            if (!run || src_pcm) begin
                pdm_cnt   <= 8'd0;
                pdm_clk_o <= 1'b0;
            end else if (pdm_tick) begin
                pdm_cnt   <= 8'd0;
                pdm_clk_o <= ~pdm_clk_o;
            end else begin
                pdm_cnt <= pdm_cnt + 8'd1;
            end
            if (pdm_capt) begin
                ci0 <= ci0 + pin_v;
                ci1 <= ci1 + ci0;
                ci2 <= ci2 + ci1;
                dec_cnt <= dec_cnt + 6'd1;
                if (dec_cnt == 6'd63) comb_st <= 2'd1;  // ci2 is updated this cycle; comb starts next
            end
            case (comb_st)
                2'd1: begin cv <= ci2 - cc0; cc0 <= ci2; comb_st <= 2'd2; end
                2'd2: begin cv <= comb_d;    cc1 <= cv;  comb_st <= 2'd3; end
                2'd3: begin cv <= comb_d;    cc2 <= cv;  comb_st <= 2'd0; cic_stb <= 1'b1; end
                default: ;
            endcase
        end
    end

    // ------------------------------------------------------------------ biquad engine
    // Channel state: an N-entry ring that advances by one entry per processed channel
    // (enable flip-flops; the head is always the channel being processed -> no address decode).
    wire [W*N-1:0] ring;
    reg  [W-1:0]   ring_in;      // entry entering at the top
    reg            ring_en;      // rotate by one entry
    kws_sreg #(.W(W*N)) u_ring (.clk(clk), .en(ring_en), .d({ring_in, ring[W*N-1:W]}), .q(ring));
    wire [W-1:0] head = ring[W-1:0];
    wire [16:0] hy1 = head[53:37];
    wire [16:0] hy2 = head[36:20];
    wire [19:0] he  = head[19:0];

    localparam ST_IDLE = 2'd0, ST_MAC = 2'd1, ST_WR = 2'd2, ST_EMIT = 2'd3;
    reg  [15:0] x0, x1, x2;
    reg  [3:0]  ch;
    reg  [1:0]  mph;           // product term 0: A*(2d)  1: C1*y1  2: C2*y2
    reg  [4:0]  mb;            // coefficient bit 0..17
    reg  [1:0]  st;
    reg  [21:0] acc;           // right-shifting accumulator (= reference acc >>> 16 when done)
    reg  [8:0]  smp_cnt;
    reg         last, emit;
    reg  [4:0]  clr_cnt;
    reg         clr_run;

    wire [17:0] rom_q;
    kws_coef_rom rom (.ch(ch), .sel(mph), .coef(rom_q));

    wire signed [16:0] d17 = $signed({x0[15], x0}) - $signed({x2[15], x2});
    reg  [21:0] opnd;
    always @(*) begin
        case (mph)
            2'd0:    opnd = {{4{d17[16]}}, d17, 1'b0};          // (x0-x2) << YF(1)
            2'd1:    opnd = {{5{hy1[16]}}, hy1};
            default: opnd = {{5{hy2[16]}}, hy2};
        endcase
    end
    wire        b17    = (mb == 5'd17);                         // coefficient sign bit: weight -2 (after 16 shifts)
    wire [21:0] op_s   = b17 ? {opnd[20:0], 1'b0} : opnd;
    wire [17:0] rom_sh = rom_q >> mb;
    wire        cbit   = rom_sh[0];
    wire        neg    = (mph == 2'd2) ^ b17;
    wire [21:0] addend = cbit ? (neg ? ~op_s : op_s) : 22'd0;
    wire        cin    = (cbit & neg) | ((mb == 5'd15) & (mph == 2'd0));   // two's complement / 2^15 rounding
    wire [21:0] sum    = acc + addend + {21'd0, cin};

    // finish: y = sat17(acc)
    wire        ovf    = ~((&acc[21:16]) | ~(|acc[21:16]));
    wire [16:0] y_new  = ovf ? (acc[21] ? 17'h10000 : 17'h0FFFF) : acc[16:0];
    // |y| >> 4 without a negation carry chain: for y < 0, (-y) >> 4 = ~(y >> 4) + (y[3:0] == 0),
    // so the +1 becomes the carry-in of the envelope adder (|-2^16| >> 4 = 4096 is covered).
    wire [12:0] y_mag  = y_new[16] ? ~y_new[16:4] : y_new[16:4];
    wire        y_cin  = y_new[16] & ~(|y_new[3:0]);
    wire [20:0] e_sum  = {1'b0, he} + {8'd0, y_mag} + {20'd0, y_cin};
    wire [19:0] e_new  = e_sum[20] ? 20'hFFFFF : e_sum[19:0];

    // log2 compressor (of the head entry's e)
    reg  [4:0]  lp;
    integer i;
    always @(*) begin
        lp = 5'd0;
        for (i = 0; i < 20; i = i + 1)
            if (he[i]) lp = i[4:0];
    end
    wire [19:0] e_norm = he << (5'd19 - lp);
    wire [7:0]  e_log  = (he == 20'd0) ? 8'd0 : {lp, e_norm[18:16]};
    wire        _unused_fe = &{cv[2:0], e_norm[19], e_norm[15:0], rom_sh[17:1], 1'b0};

    always @(*) begin
        ring_in = {W{1'b0}};                                   // clear run: shift zeros through
        ring_en = clr_run & rst_n & ~clear;
        if (!clr_run && rst_n && !clear) begin
            if (st == ST_WR)   begin ring_en = 1'b1; ring_in = {y_new, hy1, e_new}; end
            if (st == ST_EMIT) begin ring_en = 1'b1; ring_in = {hy1, hy2, 20'd0};  end
        end
    end

    wire smp_in  = src_pcm ? pcm_stb : cic_stb;
    wire hop_end = (smp_cnt == (hop512 ? 9'd511 : 9'd255));
    assign busy = (st != ST_IDLE) | clr_run;

    always @(posedge clk) begin
        feat_valid <= 1'b0;
        overrun    <= 1'b0;
        if (!rst_n || clear) begin
            st <= ST_IDLE; ch <= 4'd0; mph <= 2'd0; mb <= 5'd0;
            x0 <= 16'd0; x1 <= 16'd0; x2 <= 16'd0;
            smp_cnt <= 9'd0; last <= 1'b0; emit <= 1'b0;
            acc <= 22'd0;
            clr_cnt <= 5'd0; clr_run <= 1'b1;
            feat_data <= 8'd0;
        end else if (clr_run) begin
            clr_cnt <= clr_cnt + 5'd1;
            if (clr_cnt == {1'b0, NL}) clr_run <= 1'b0;
        end else begin
            case (st)
                ST_IDLE: if (smp_in && run) begin
                    x0   <= src_pcm ? pcm_data : cic_out;
                    last <= hop_end;
                    emit <= hop_end & feat_en & ~nn_busy;
                    if (hop_end & feat_en & nn_busy) overrun <= 1'b1;
                    smp_cnt <= hop_end ? 9'd0 : smp_cnt + 9'd1;
                    ch  <= 4'd0;
                    mph <= 2'd0;
                    mb  <= 5'd0;
                    acc <= 22'd0;
                    st  <= ST_MAC;
                end
                ST_MAC: begin                   // one product term per clock, LSB first
                    if (mph == 2'd2) begin
                        acc <= (mb < 5'd16) ? {sum[21], sum[21:1]} : sum;
                        mph <= 2'd0;
                        mb  <= mb + 5'd1;
                        if (b17) st <= ST_WR;
                    end else begin
                        acc <= sum;
                        mph <= mph + 2'd1;
                    end
                end
                ST_WR: begin                    // write back = rotate the ring by one entry
                    mph  <= 2'd0;
                    mb   <= 5'd0;
                    acc  <= 22'd0;
                    ch   <= ch + 4'd1;
                    if (ch == NL) begin
                        ch <= 4'd0;
                        x2 <= x1;
                        x1 <= x0;
                        st <= last ? ST_EMIT : ST_IDLE;
                    end else st <= ST_MAC;
                end
                default: begin                  // ST_EMIT: one more turn, emit log2(e) and clear e
                    if (emit) begin
                        feat_valid <= 1'b1;
                        feat_data  <= e_log;
                    end
                    ch <= ch + 4'd1;
                    if (ch == NL) begin ch <= 4'd0; st <= ST_IDLE; end
                end
            endcase
            if (smp_in && run && st != ST_IDLE) overrun <= 1'b1;   // sample arrived while busy: dropped
        end
    end
endmodule
