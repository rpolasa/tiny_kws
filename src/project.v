/*
 * tiny_kws - keyword spotting accelerator for Tiny Tapeout (SKY130)
 * Copyright (c) 2026 Rakesh Polasa
 * SPDX-License-Identifier: Apache-2.0
 *
 * PDM mic (or host PCM) -> 16ch IIR filterbank log-energy features -> int8 GRU(H) + FC
 * Model (weights) streamed every frame from the QSPI PSRAM/flash Pmod on uio[7:0];
 * the new hidden state is kept in an H-byte scratch area at the top of PSRAM B (CS2).
 *
 * ui_in[0]  PDM_DAT        uo_out[0] PDM_CLK
 * ui_in[1]  SPI_SCK        uo_out[1] SPI_MISO
 * ui_in[2]  SPI_MOSI       uo_out[2] DETECT (high for the frame a keyword is detected)
 * ui_in[3]  SPI_CSn        uo_out[3] FRAME (toggles every processed frame)
 *                          uo_out[7:4] DET_CLASS (class of the last detection)
 * uio[0] CS0(flash) uio[1] SD0 uio[2] SD1 uio[3] SCK uio[4] SD2 uio[5] SD3 uio[6] CS1(RAM A) uio[7] CS2(RAM B)
 * All uio pins are released (input) while CTRL.RUN=0, so a host can load the memory.
 */
`default_nettype none

module tt_um_rakesh_tiny_kws #(
    parameter H = 48,
    parameter NCH = 16
) (
    input  wire [7:0] ui_in,
    output wire [7:0] uo_out,
    input  wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,
    input  wire       ena,
    input  wire       clk,
    input  wire       rst_n
);
    // ------------------------------------------------------------------ reset synchroniser
    reg [1:0] rst_sync;
    always @(posedge clk) rst_sync <= {rst_sync[0], rst_n};
    wire rst_ni = rst_sync[1];

    // ------------------------------------------------------------------ host SPI + registers
    wire       spi_miso, spi_active, wr_stb;
    wire [6:0] raddr;
    wire [7:0] wdata;
    reg  [7:0] rdata;

    kws_spi_slave u_spi (
        .clk(clk), .rst_n(rst_ni),
        .sck_pin(ui_in[1]), .mosi_pin(ui_in[2]), .csn_pin(ui_in[3]),
        .miso(spi_miso), .active(spi_active),
        .wr_stb(wr_stb), .addr(raddr), .wdata(wdata), .rdata(rdata)
    );

    reg  [7:0]  r_ctrl, r_pdmdiv, r_memcfg, r_burst, r_thr, r_detcfg;
    reg  [23:0] r_base;
    reg         clr_pulse;
    reg         st_ovr, st_err, st_det;
    reg  [15:0] frame_cnt;
    reg         frame_tgl;
    reg  [7:0]  pcm_lo;
    reg         pcm_hi_next, pcm_stb;
    reg         hf_valid;

    wire run      = r_ctrl[0];
    wire src_pcm  = r_ctrl[1];
    wire feat_host= r_ctrl[2];
    wire hop512   = r_ctrl[3];
    wire pdm_edge = r_ctrl[5];

    wire nn_busy, fe_busy, q_busy, model_err, frame_done, det, det_pulse;
    wire fe_ovr, fe_fvalid;
    wire [7:0] fe_fdata;
    wire [3:0] top1_idx, det_class;
    wire [7:0] top1_val, top2_val, feat_rd;

    always @(posedge clk) begin
        if (!rst_ni) begin
            r_ctrl   <= 8'h00;
            r_pdmdiv <= 8'd23;                 // 50 MHz / 48 = 1.042 MHz PDM, fs = 16.28 kHz
            r_memcfg <= {3'd2, 3'd2, 2'd1};    // rd_dly=2, SCK=clk/4, CS1 (RAM A)
            r_burst  <= 8'd3;                  // 32-byte bursts (PSRAM tCEM); max 5 = 128 B
            r_base   <= 24'd0;
            r_thr    <= 8'd16;
            r_detcfg <= {4'd2, 4'd3};          // nfill=2 (silence, unknown), N=3 frames
            clr_pulse <= 1'b0;
            st_ovr <= 1'b0; st_err <= 1'b0; st_det <= 1'b0;
            pcm_lo <= 8'd0; pcm_hi_next <= 1'b0; pcm_stb <= 1'b0;
            hf_valid <= 1'b0;
            frame_cnt <= 16'd0; frame_tgl <= 1'b0;
        end else begin
            clr_pulse <= 1'b0;
            pcm_stb   <= 1'b0;
            hf_valid  <= 1'b0;
            if (!spi_active) pcm_hi_next <= 1'b0;
            if (fe_ovr)    st_ovr <= 1'b1;
            if (model_err) st_err <= 1'b1;
            if (det_pulse) st_det <= 1'b1;
            if (frame_done) begin frame_cnt <= frame_cnt + 16'd1; frame_tgl <= ~frame_tgl; end
            if (wr_stb) begin
                case (raddr)
                    7'h00: begin r_ctrl <= {wdata[7:5], 1'b0, wdata[3:0]}; clr_pulse <= wdata[4]; end
                    7'h01: r_pdmdiv <= wdata;
                    7'h02: r_memcfg <= wdata;
                    7'h03: r_burst  <= wdata;
                    7'h04: r_base[7:0]   <= wdata;
                    7'h05: r_base[15:8]  <= wdata;
                    7'h06: r_base[23:16] <= wdata;
                    7'h07: r_thr    <= wdata;
                    7'h08: r_detcfg <= wdata;
                    7'h10: begin
                        if (pcm_hi_next) pcm_stb <= 1'b1;        // high byte stays in wdata (see below)
                        else pcm_lo <= wdata;
                        pcm_hi_next <= ~pcm_hi_next;
                    end
                    7'h11: hf_valid <= feat_host;
                    7'h20: begin
                        if (wdata[2]) st_ovr <= 1'b0;
                        if (wdata[3]) st_err <= 1'b0;
                        if (wdata[4]) st_det <= 1'b0;
                    end
                    default: ;
                endcase
            end
        end
    end

    always @(*) begin
        case (raddr)
            7'h00: rdata = r_ctrl;
            7'h01: rdata = r_pdmdiv;
            7'h02: rdata = r_memcfg;
            7'h03: rdata = r_burst;
            7'h04: rdata = r_base[7:0];
            7'h05: rdata = r_base[15:8];
            7'h06: rdata = r_base[23:16];
            7'h07: rdata = r_thr;
            7'h08: rdata = r_detcfg;
            7'h20: rdata = {2'b00, q_busy, st_det, st_err, st_ovr, fe_busy, nn_busy};
            7'h21: rdata = {top1_idx, det_class};
            7'h22: rdata = top1_val;
            7'h23: rdata = top2_val;
            7'h24: rdata = frame_cnt[7:0];
            7'h25: rdata = frame_cnt[15:8];
            7'h26: rdata = 8'h4B;              // 'K' : design id
            7'h27: rdata = H[7:0];
            7'h28: rdata = NCH[7:0];
            default: rdata = (raddr[6:4] == 3'b011) ? feat_rd : 8'h00;   // 0x30..0x3F features
        endcase
    end

    // pcm_stb / hf_valid are asserted the cycle after wr_stb; the SPI slave's wdata register still
    // holds that byte (the next one is at least 64 clk away), so it is used directly as the PCM
    // high byte and as the host feature byte - no copies are kept.
    // ------------------------------------------------------------------ front end
    wire pdm_clk;
    kws_frontend #(.N(NCH)) u_fe (
        .clk(clk), .rst_n(rst_ni), .clear(clr_pulse), .run(run), .src_pcm(src_pcm), .hop512(hop512),
        .pdm_div(r_pdmdiv), .pdm_edge(pdm_edge), .pdm_pin(ui_in[0]), .pdm_clk_o(pdm_clk),
        .pcm_stb(pcm_stb), .pcm_data({wdata, pcm_lo}),
        .feat_en(~feat_host), .nn_busy(nn_busy),
        .feat_valid(fe_fvalid), .feat_data(fe_fdata), .overrun(fe_ovr), .busy(fe_busy)
    );

    // ------------------------------------------------------------------ model memory
    wire       m_start, m_stop, m_valid;
    wire [7:0] m_data;
    wire [23:0] m_addr, w_addr;
    wire [1:0] m_cs, q_cs_cur;
    wire       w_req, w_ack;
    wire [7:0] w_data;
    wire       q_sck, q_cs;
    wire [3:0] q_sd_out, q_sd_oe;
    wire [3:0] q_sd_in = {uio_in[5], uio_in[4], uio_in[2], uio_in[1]};

    kws_qspi u_qspi (
        .clk(clk), .rst_n(rst_ni),
        .div(r_memcfg[4:2]), .rd_dly(r_memcfg[7:5]), .burst(r_burst[2:0]),
        .start(m_start), .addr(m_addr), .cs_sel(m_cs), .stop(m_stop | clr_pulse | ~run),
        .byte_valid(m_valid), .byte_data(m_data), .busy(q_busy),
        .wr_req(w_req), .wr_addr(w_addr), .wr_data(w_data), .wr_ack(w_ack),
        .sck(q_sck), .cs_act(q_cs), .cs_cur(q_cs_cur), .sd_out(q_sd_out), .sd_oe(q_sd_oe), .sd_in(q_sd_in)
    );

    // ------------------------------------------------------------------ GRU engine
    kws_nn #(.H(H), .F(NCH)) u_nn (
        .clk(clk), .rst_n(rst_ni), .clear(clr_pulse), .run(run),
        .fin_valid(feat_host ? hf_valid : fe_fvalid), .fin_data(feat_host ? wdata : fe_fdata),
        .nfill(r_detcfg[7:4]), .det_thr(r_thr), .det_n(r_detcfg[3:0]),
        .base(r_base), .wcs(r_memcfg[1:0]),
        .m_start(m_start), .m_addr(m_addr), .m_cs(m_cs), .m_stop(m_stop), .m_valid(m_valid), .m_data(m_data),
        .m_busy(q_busy), .w_req(w_req), .w_addr(w_addr), .w_data(w_data), .w_ack(w_ack),
        .busy(nn_busy), .model_err(model_err), .frame_done(frame_done),
        .top1_idx(top1_idx), .top1_val(top1_val), .top2_val(top2_val),
        .det(det), .det_pulse(det_pulse), .det_class(det_class),
        .feat_sel(raddr[3:0]), .feat_rd(feat_rd)
    );

    // ------------------------------------------------------------------ pins
    wire cs0_n = ~(q_cs && q_cs_cur == 2'd0);
    wire cs1_n = ~(q_cs && q_cs_cur == 2'd1);
    wire cs2_n = ~(q_cs && q_cs_cur == 2'd2);

    assign uio_out = {cs2_n, cs1_n, q_sd_out[3], q_sd_out[2], q_sck, q_sd_out[1], q_sd_out[0], cs0_n};
    assign uio_oe  = run ? {1'b1, 1'b1, q_sd_oe[3], q_sd_oe[2], 1'b1, q_sd_oe[1], q_sd_oe[0], 1'b1} : 8'h00;
    assign uo_out  = {det_class, frame_tgl, det, spi_miso, pdm_clk};

    wire _unused = &{ena, ui_in[7:4], uio_in[7:6], uio_in[3], uio_in[0], 1'b0};
endmodule
