/*
 * Behavioural model of APS6404L PSRAM / W25Q128 flash: Quad I/O fast read (0xEB) and,
 * for PSRAM (WRITABLE=1), quad write (0x38: cmd on SD0, addr quad, data quad, no wait).
 *  - command on SD0 (8 rising edges), address on SD[3:0] (6), 6 wait clocks, then data
 *  - data nibble launched T_CO after each SCK falling edge and driven X from T_OH after the
 *    next falling edge until the new nibble is valid -> catches sampling outside the window
 *  - error counters: CS low longer than T_CEM (PSRAM refresh limit), bad command,
 *    burst crossing a 1 kB page (PSRAM linear-burst rule)
 */
`timescale 1ns / 1ps
`default_nettype none

module psram_model #(
    parameter MEMBYTES = 65536,
    parameter real T_CO  = 7.0,     // clock-to-output incl. board/pad delay (ns)
    parameter real T_OH  = 1.5,
    parameter real T_CEM = 8000.0,  // ns, 0 = no limit (flash)
    parameter CHECK_PAGE = 1,
    parameter WRITABLE = 1
) (
    input  wire       csn,
    input  wire       sck,
    inout  wire [3:0] sd,
    output reg        drive
);
    reg [7:0]  mem [0:MEMBYTES-1];
    reg [3:0]  dout;
    assign sd = drive ? dout : 4'bzzzz;

    integer    cnt;
    reg [7:0]  cmd;
    reg [23:0] addr, start_addr;
    reg        nib_hi;
    realtime   t_fall_cs;
    integer    err_cem, err_cmd, err_page, nbursts, nbytes, nwrites;
    reg        wr_hi;
    reg [3:0]  wr_nib;

    initial begin
        drive = 0; dout = 4'h0; cnt = 0; err_cem = 0; err_cmd = 0; err_page = 0; nbursts = 0; nbytes = 0; nwrites = 0;
    end

    always @(negedge csn) begin
        cnt = 0; cmd = 0; addr = 0; nib_hi = 1; t_fall_cs = $realtime; nbursts = nbursts + 1;
    end
    always @(posedge csn) begin
        drive = 0;
        if (T_CEM > 0 && ($realtime - t_fall_cs) > T_CEM) begin
            err_cem = err_cem + 1;
            $display("%t MEM: tCEM violated (CS low %0.1f ns)", $realtime, $realtime - t_fall_cs);
        end
    end

    always @(posedge sck) if (!csn) begin
        cnt = cnt + 1;
        if (cnt <= 8) cmd = {cmd[6:0], sd[0]};
        else if (cnt <= 14) begin
            if (cnt == 9 && !(cmd == 8'hEB || (WRITABLE && cmd == 8'h38))) begin err_cmd = err_cmd + 1; $display("%t MEM: bad cmd %h", $realtime, cmd); end
            addr = {addr[19:0], sd};
            if (cnt == 14) begin start_addr = addr; wr_hi = 1; end
        end else if (cmd == 8'h38) begin
            if (wr_hi) wr_nib = sd;
            else begin mem[addr % MEMBYTES] = {wr_nib, sd}; addr = addr + 1; nwrites = nwrites + 1; end
            wr_hi = ~wr_hi;
        end
    end

    always @(negedge sck) if (!csn && cnt >= 20 && cmd == 8'hEB) begin
        if (cnt == 20) addr = start_addr;
        #(T_OH);
        if (!csn) begin drive = 1; dout = 4'bxxxx; end
        #(T_CO - T_OH);
        if (!csn) begin
            if (nib_hi) begin
                if (CHECK_PAGE && (addr % 1024 == 0) && (addr != start_addr)) begin
                    err_page = err_page + 1;
                    $display("%t MEM: burst crossed 1kB page at %h", $realtime, addr);
                end
                dout = mem[addr % MEMBYTES][7:4];
            end else begin
                dout = mem[addr % MEMBYTES][3:0];
                addr = addr + 1;
                nbytes = nbytes + 1;
            end
            nib_hi = ~nib_hi;
        end
    end
endmodule
