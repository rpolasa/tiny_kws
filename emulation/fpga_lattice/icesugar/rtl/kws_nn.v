/*
 * kws_nn - streaming integer GRU + linear classifier (bit-exact with model/gru_int.py)
 *
 * Weights/biases are streamed once per frame from external QSPI memory in exactly the
 * order they are consumed.  All vectors live in rotating byte rings, so the datapath has
 * no RAM, no address decoder and a single 9x9 multiplier.
 *
 *   xring  : F features (u8)          hring : H hidden (s8)
 * The new hidden state h'_j is written to PSRAM B scratch (1-byte quad writes between weight
 * bursts) and read back into hring before the classifier, so no second hidden buffer exists.
 *
 * rev B (area): one accumulator.  In the n row the x part (Bnx + Wn_x.x) is finished first; its
 * shifted value, clamped to 9 bits (exact, because |(r*qh)>>>7| <= 128 and n saturates to int8),
 * is parked in 'axs' and the accumulator is reloaded with Bnh, which is still in the bias shift
 * register.  The stream address is a 3-way mux instead of a register, and the scratch write index
 * doubles as the write address counter.
 */
`default_nettype none

module kws_nn #(
    parameter H = 48,
    parameter F = 16,
    parameter [23:0] SCRATCH = 24'h7FFF00      // PSRAM B (CS2) scratch for h'
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        clear,        // zero hidden state (pulse)
    input  wire        run,
    // feature input (frontend or host)
    input  wire        fin_valid,
    input  wire [7:0]  fin_data,
    // detection config
    input  wire [3:0]  nfill,        // classes < nfill are non-keywords
    input  wire [7:0]  det_thr,      // min top1-top2 margin
    input  wire [3:0]  det_n,        // consecutive frames
    // model stream
    input  wire [23:0] base,         // model base address
    input  wire [1:0]  wcs,          // chip select of the model memory
    output reg         m_start,
    output wire [23:0] m_addr,
    output reg  [1:0]  m_cs,
    output reg         m_stop,
    input  wire        m_valid,
    input  wire [7:0]  m_data,
    input  wire        m_busy,
    output reg         w_req,
    output wire [23:0] w_addr,
    output reg  [7:0]  w_data,
    input  wire        w_ack,
    // status / results
    output wire        busy,
    output reg         model_err,    // pulse
    output reg         frame_done,   // pulse
    output reg  [3:0]  top1_idx,
    output reg  [7:0]  top1_val,
    output reg  [7:0]  top2_val,
    output reg         det,          // level, valid for the frame
    output reg         det_pulse,
    output reg  [3:0]  det_class,
    input  wire [3:0]  feat_sel,
    output wire [7:0]  feat_rd
);
    localparam D = F + H;
    localparam HB = 8 * H;
    localparam integer FL = F - 1;
    localparam [3:0] FLAST = FL[3:0];
    localparam [23:0] FC_OFF = 8 + H * (3 * D + 12);   // byte offset of the classifier section

    // ------------------------------------------------------------------ rings
    wire [8*F-1:0] xring;
    wire [HB-1:0]  hring;
    reg  [7:0]     xring_in, hring_in;
    reg            xring_en, hring_en;
    kws_sreg #(.W(8*F)) u_xring (.clk(clk), .en(xring_en), .d({xring_in, xring[8*F-1:8]}), .q(xring));
    kws_sreg #(.W(HB))  u_hring (.clk(clk), .en(hring_en), .d({hring_in, hring[HB-1:8]}),  .q(hring));
    wire [7:0] x_head  = xring[7:0];
    wire [7:0] h_head  = hring[7:0];

    // feature readback (valid while idle)
    generate
        if (F == 16) begin : g_rd16
            assign feat_rd = xring[8*feat_sel +: 8];
        end else begin : g_rd
            wire [31:0] fsel = {28'd0, feat_sel};
            assign feat_rd = (fsel < F) ? xring[8*feat_sel +: 8] : 8'd0;
        end
    endgenerate

    // ------------------------------------------------------------------ sequencer
    localparam S_IDLE = 4'd0, S_HDR = 4'd1, S_BIAS = 4'd2, S_W = 4'd3, S_FCB = 4'd4, S_FCW = 4'd5,
               S_FLUSH = 4'd6, S_DET = 4'd8, S_CLR = 4'd9,
               S_WDONE = 4'd10, S_HSTART = 4'd11, S_HRD = 4'd12, S_FSTART = 4'd13;
    reg [3:0]  state;
    reg [2:0]  hcnt;           // header byte
    reg [2:0]  bcnt;           // bias byte (0..5)
    reg [1:0]  row;            // 0:r 1:z 2:n
    reg [6:0]  pos;            // element index within row
    reg [6:0]  unit;           // GRU unit j  (also FC class / copy counter)
    reg [3:0]  fcnt;           // features received
    reg [6:0]  hn_idx;         // index of the next h' write (= scratch address offset)
    reg [1:0]  msel;           // stream start address: 0 model base, 1 scratch, 2 classifier section
    reg [1:0]  wcnt;           // settle counter for stream restarts
    reg [23:0] bsr;
    // header
    reg [4:0]  nclass;
    reg [7:0]  xoff;
    reg [4:0]  s_r, s_z, s_n, s_fc;

    // datapath regs
    reg signed [25:0] acc;
    reg signed [17:0] prod;
    reg        prod_v;         // prod holds a MAC product to accumulate
    reg [1:0]  ax_cap;         // n row: x part complete -> park it, reload acc with Bnh
    reg signed [8:0] axs;      // clamp9((Bnx + Wn_x.x) >>> Sn)
    reg [7:0]  r7, z7;         // gate values 0..128
    reg signed [7:0] hj, qh, n8;
    reg [2:0]  post;           // post-processing step after a row
    reg [1:0]  post_kind;      // 0 r, 1 z, 2 n, 3 fc
    // detection
    reg [3:0]  last_idx;
    reg [3:0]  run_cnt;
    reg [3:0]  c_idx;
    reg [7:0]  t1, t2;
    reg [3:0]  t1i;

    assign busy   = (state != S_IDLE);
    assign m_addr = (msel == 2'd0) ? base : (msel == 2'd1) ? SCRATCH : base + FC_OFF;
    assign w_addr = SCRATCH + {17'd0, hn_idx};

    // operand for the current element
    wire        in_x     = (pos < F);
    wire signed [8:0] xs = $signed({1'b0, x_head}) - $signed({1'b0, xoff});
    wire signed [7:0] x8 = (xs > 9'sd127) ? 8'sd127 : (xs < -9'sd128) ? -8'sd128 : xs[7:0];
    wire signed [7:0] v8 = (in_x && state != S_FCW) ? x8 : $signed(h_head);

    // shared shifter + saturation
    reg  [4:0]         sh_amt;
    wire signed [25:0] sh_out = acc >>> sh_amt;
    wire signed [8:0]  sh_sat9 = (sh_out > 26'sd255) ? 9'sd255 : (sh_out < -26'sd256) ? -9'sd256 : sh_out[8:0];
    wire signed [7:0]  sh_sat = (sh_out > 26'sd127) ? 8'sd127 : (sh_out < -26'sd128) ? -8'sd128 : sh_out[7:0];

    // shared multiplier (signed 9x9)
    reg  signed [8:0]  mul_a, mul_b;
    wire signed [17:0] mul_p = mul_a * mul_b;

    // post-processing combinational pieces
    wire signed [8:0]  gate_q = sh_sat + 9'sd64;
    wire [7:0]         gate7  = (gate_q < 9'sd0) ? 8'd0 : (gate_q > 9'sd128) ? 8'd128 : gate_q[7:0];
    wire signed [17:0] p7     = prod >>> 7;
    wire signed [9:0]  n_t    = {axs[8], axs} + p7[9:0];        // p7 = (r*qh)>>>7 is within [-128, 127]
    wire signed [7:0]  n_sat  = (n_t > 10'sd127) ? 8'sd127 : (n_t < -10'sd128) ? -8'sd128 : n_t[7:0];
    wire signed [8:0]  hdiff  = hj - n8;
    wire signed [17:0] h_new  = {{10{n8[7]}}, n8} + p7;   // always within int8 by construction
    wire signed [8:0]  topd   = $signed({t1[7], t1}) - $signed({t2[7], t2});   // >= 0
    // detection bookkeeping
    wire        kw      = (t1i >= nfill) && (topd >= $signed({1'b0, det_thr}));
    wire [3:0]  run_inc = (t1i == last_idx) ? ((run_cnt == 4'd15) ? 4'd15 : run_cnt + 4'd1) : 4'd1;
    wire [3:0]  det_req = (det_n == 4'd0) ? 4'd1 : det_n;
    wire        fire    = kw && (run_inc == det_req) && !(t1i == last_idx && run_cnt == 4'd15);

    always @(*) begin
        sh_amt = s_r;
        mul_a  = {m_data[7], m_data};
        mul_b  = {v8[7], v8};
        case (post_kind)
            2'd0: sh_amt = s_r;
            2'd1: sh_amt = s_z;
            2'd2: sh_amt = s_n;
            default: sh_amt = s_fc;
        endcase
        if (ax_cap[1]) sh_amt = s_n;                             // parking the x part of the n row
        if (post == 3'd2) begin mul_a = {1'b0, r7}; mul_b = {qh[7], qh}; end
        if (post == 3'd4) begin mul_a = {1'b0, z7}; mul_b = hdiff; end
    end

    // ring controls
    wire nrst = rst_n & ~clear;
    always @(*) begin
        xring_en = 1'b0; xring_in = x_head;
        hring_en = 1'b0; hring_in = h_head;
        if (nrst) begin
            case (state)
                S_CLR:  begin hring_en = 1'b1; hring_in = 8'd0; end
                S_IDLE: begin xring_en = fin_valid; xring_in = fin_data; end
                S_W:    begin xring_en = m_valid & in_x; hring_en = m_valid & ~in_x; end
                S_HRD:  begin hring_en = m_valid; hring_in = m_data; end
                S_FCW:  hring_en = m_valid;
                default: ;
            endcase
        end
    end

    wire last_hdr = (hcnt == 3'd7);
    wire [2:0] nbias = (row == 2'd2) ? 3'd5 : 3'd2;
    wire [23:0] bias24 = {m_data, bsr[23:8]};
    wire        _unused_nn = &{h_new[17:8], p7[17:10], 1'b0};

    always @(posedge clk) begin
        m_start    <= 1'b0;
        m_stop     <= 1'b0;
        if (w_ack) begin w_req <= 1'b0; hn_idx <= hn_idx + 7'd1; end
        model_err  <= 1'b0;
        frame_done <= 1'b0;
        det_pulse  <= 1'b0;
        if (!rst_n || clear) begin
            state <= S_CLR; unit <= 7'd0; hcnt <= 3'd0; bcnt <= 3'd0; row <= 2'd0; pos <= 7'd0;
            fcnt <= 4'd0; post <= 3'd0; post_kind <= 2'd0; prod_v <= 1'b0; ax_cap <= 2'b00; axs <= 9'sd0;
            acc <= 26'sd0; prod <= 18'sd0; bsr <= 24'd0;
            nclass <= 5'd0; xoff <= 8'd0; s_r <= 5'd0; s_z <= 5'd0; s_n <= 5'd0; s_fc <= 5'd0;
            r7 <= 8'd0; z7 <= 8'd0; hj <= 8'sd0; qh <= 8'sd0; n8 <= 8'sd0;
            last_idx <= 4'd0; run_cnt <= 4'd0; c_idx <= 4'd0; t1 <= 8'd0; t2 <= 8'd0; t1i <= 4'd0;
            hn_idx <= 7'd0; wcnt <= 2'd0; w_req <= 1'b0; w_data <= 8'd0;
            msel <= 2'd0; m_cs <= 2'd1;
            if (!rst_n) begin
                top1_idx <= 4'd0; top1_val <= 8'd0; top2_val <= 8'd0; det <= 1'b0; det_class <= 4'd0;
            end
        end else begin
            // ---------------- MAC pipeline stage 2: accumulate
            if (prod_v) begin
                acc    <= acc + {{8{prod[17]}}, prod};
                prod_v <= 1'b0;
            end
            // n row, after the last x product has been accumulated: park ax, reload with Bnh.
            // (the next weight byte is at least 4 clk after the previous one, so acc is free)
            ax_cap <= {ax_cap[0], 1'b0};
            if (ax_cap[1]) begin
                axs <= sh_sat9;
                acc <= $signed({{2{bsr[23]}}, bsr});
            end

            // ---------------- post processing after a row (multiplier/shifter are free)
            case (post)
                3'd1: begin
                    if (post_kind == 2'd0) begin r7 <= gate7; post <= 3'd0; end
                    else if (post_kind == 2'd1) begin z7 <= gate7; post <= 3'd0; end
                    else if (post_kind == 2'd2) begin qh <= sh_sat; post <= 3'd2; end
                    else begin                                   // FC logit -> top-2
                        if ($signed(sh_sat) > $signed(t1)) begin
                            t2 <= t1; t1 <= sh_sat; t1i <= c_idx;
                        end else if ($signed(sh_sat) > $signed(t2)) begin
                            t2 <= sh_sat;
                        end
                        c_idx <= c_idx + 4'd1;
                        post  <= 3'd0;
                    end
                end
                3'd2: begin prod <= mul_p; post <= 3'd3; end            // r7 * qh
                3'd3: begin n8 <= n_sat; post <= 3'd4; end              // (ax>>>Sn) + (r*qh>>>7)
                3'd4: begin prod <= mul_p; post <= 3'd5; end            // z7 * (h - n)
                3'd5: if (!w_req) begin                         // h'_j -> PSRAM B scratch
                    w_req  <= 1'b1;
                    w_data <= h_new[7:0];
                    post   <= 3'd0;
                end
                3'd6: post <= 3'd1;
                default: ;
            endcase

            case (state)
                S_CLR: begin                                    // shift zeros through the h ring
                    unit   <= unit + 7'd1;
                    if (unit == H - 1) begin unit <= 7'd0; state <= S_IDLE; end
                end
                S_IDLE: begin
                    if (fin_valid) begin
                        fcnt  <= (fcnt == FLAST) ? 4'd0 : fcnt + 4'd1;
                        if (fcnt == FLAST && run) begin
                            m_start <= 1'b1;
                            msel    <= 2'd0;
                            m_cs    <= wcs;
                            hn_idx  <= 7'd0;
                            hcnt    <= 3'd0;
                            state   <= S_HDR;
                        end
                    end
                end
                S_HDR: if (m_valid) begin
                    hcnt <= hcnt + 3'd1;
                    case (hcnt)
                        3'd1: nclass <= m_data[4:0];
                        3'd2: xoff   <= m_data;
                        3'd3: s_r    <= m_data[4:0];
                        3'd4: s_z    <= m_data[4:0];
                        3'd5: s_n    <= m_data[4:0];
                        3'd6: s_fc   <= m_data[4:0];
                        default: ;
                    endcase
                    if (hcnt == 3'd0 && m_data != 8'hA5) begin
                        m_stop <= 1'b1; model_err <= 1'b1; state <= S_IDLE;
                    end
                    if (last_hdr) begin
                        if (m_data != H[7:0] || nclass == 5'd0 || nclass > 5'd16) begin
                            m_stop <= 1'b1; model_err <= 1'b1; state <= S_IDLE;
                        end else begin
                            unit <= 7'd0; row <= 2'd0; bcnt <= 3'd0; state <= S_BIAS;
                        end
                    end
                end
                S_BIAS: if (m_valid) begin
                    bsr  <= bias24;
                    bcnt <= bcnt + 3'd1;
                    if (bcnt == 3'd2) acc  <= $signed({{2{m_data[7]}}, m_data, bsr[23:8]});
                    if (bcnt == nbias) begin pos <= 7'd0; state <= S_W; end
                end
                S_W: if (m_valid) begin
                    prod     <= mul_p;
                    prod_v   <= 1'b1;
                    if (row == 2'd2 && pos == F - 1) ax_cap <= 2'b01;
                    if (row == 2'd0 && pos == unit + F) hj <= $signed(h_head);
                    pos <= pos + 7'd1;
                    if (pos == D - 1) begin
                        post      <= 3'd6;   // wait one cycle for the last accumulate
                        post_kind <= row;
                        bcnt      <= 3'd0;
                        if (row == 2'd2) begin
                            row <= 2'd0;
                            if (unit == H - 1) begin unit <= 7'd0; c_idx <= 4'd0; t1 <= 8'h80; t2 <= 8'h80; t1i <= 4'd0; state <= S_WDONE; end
                            else begin unit <= unit + 7'd1; state <= S_BIAS; end
                        end else begin
                            row <= row + 2'd1; state <= S_BIAS;
                        end
                    end
                end
                // ---- h' readback: wait for the last write, stop the weight stream,
                //      read H bytes of scratch into hring, restart at the classifier section
                S_WDONE: if (post == 3'd0 && !prod_v && !w_req) begin
                    m_stop <= 1'b1; wcnt <= 2'd0; state <= S_HSTART;
                end
                S_HSTART: begin
                    wcnt <= wcnt + 2'd1;
                    if (wcnt == 2'd3 && !m_busy) begin
                        m_start <= 1'b1; msel <= 2'd1; m_cs <= 2'd2; pos <= 7'd0; state <= S_HRD;
                    end
                end
                S_HRD: if (m_valid) begin
                    pos   <= pos + 7'd1;
                    if (pos == H - 1) begin m_stop <= 1'b1; wcnt <= 2'd0; state <= S_FSTART; end
                end
                S_FSTART: begin
                    wcnt <= wcnt + 2'd1;
                    if (wcnt == 2'd3 && !m_busy) begin
                        m_start <= 1'b1; msel <= 2'd2; m_cs <= wcs; bcnt <= 3'd0; state <= S_FCB;
                    end
                end
                S_FCB: if (m_valid) begin
                    bsr  <= bias24;
                    bcnt <= bcnt + 3'd1;
                    if (bcnt == 3'd2) begin acc <= $signed({{2{m_data[7]}}, m_data, bsr[23:8]}); pos <= 7'd0; state <= S_FCW; end
                end
                S_FCW: if (m_valid) begin
                    prod     <= mul_p;
                    prod_v   <= 1'b1;
                    pos      <= pos + 7'd1;
                    if (pos == H - 1) begin
                        post <= 3'd6; post_kind <= 2'd3; bcnt <= 3'd0;
                        if (unit == {2'b00, nclass} - 7'd1) begin unit <= 7'd0; m_stop <= 1'b1; state <= S_FLUSH; end
                        else begin unit <= unit + 7'd1; state <= S_FCB; end
                    end
                end
                S_FLUSH: if (post == 3'd0 && !prod_v) state <= S_DET;
                S_DET: begin
                    top1_idx <= t1i; top1_val <= t1; top2_val <= t2;
                    run_cnt  <= kw ? run_inc : 4'd0;
                    det      <= fire;
                    det_pulse <= fire;
                    if (fire) det_class <= t1i;
                    last_idx   <= t1i;
                    frame_done <= 1'b1;
                    fcnt       <= 4'd0;
                    state      <= S_IDLE;
                end
                default: state <= S_IDLE;
            endcase
        end
    end
endmodule
