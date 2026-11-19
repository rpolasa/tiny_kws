`timescale 1ns/1ps
`default_nettype none
module tb_fe;
  reg clk=0, rst_n=0; always #5 clk=~clk;
  reg pcm_stb=0; reg [15:0] pcm_data=0; reg hop512=0;
  wire fv; wire [7:0] fd; wire ovr, busy, pclk;
  kws_frontend #(.N(16)) dut(.clk(clk),.rst_n(rst_n),.clear(1'b0),.run(1'b1),.src_pcm(1'b1),.hop512(hop512),
    .pdm_div(8'd23),.pdm_edge(1'b0),.pdm_pin(1'b0),.pdm_clk_o(pclk),.pcm_stb(pcm_stb),.pcm_data(pcm_data),
    .feat_en(1'b1),.nn_busy(1'b0),.feat_valid(fv),.feat_data(fd),.overrun(ovr),.busy(busy));
  integer fi, fy, ff, r, n; reg [15:0] v;
  always @(posedge clk) begin
    if (fv) $fdisplay(ff,"%02x",fd);
    if (dut.ring_en && dut.st==2'd2) $fdisplay(fy,"%05x",dut.y_new);
    if (ovr) begin $display("OVERRUN"); $finish; end
  end
  initial begin
    if ($test$plusargs("HOP512")) hop512=1;
    fi=$fopen("stim.txt","r"); fy=$fopen("dut_y.txt","w"); ff=$fopen("dut_f.txt","w");
    repeat(4) @(posedge clk); rst_n=1; repeat(40) @(posedge clk);
    n=0;
    while (!$feof(fi)) begin
      r=$fscanf(fi,"%h\n",v);
      if (r==1) begin
        @(posedge clk); pcm_data<=v; pcm_stb<=1; @(posedge clk); pcm_stb<=0;
        @(posedge clk); while (busy) @(posedge clk);
        n=n+1;
      end
    end
    repeat(10) @(posedge clk);
    $display("samples %0d", n); $fclose(fy); $fclose(ff); $finish;
  end
endmodule
