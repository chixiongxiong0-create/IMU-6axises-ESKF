`timescale 1ns/1ps
module spi_check #(parameter integer SPI_HZ=5000000, HALF_NS=100)
    (output reg finished=0);
    reg clk=0, rst_n=0, start=0;
    always #5 clk=~clk;
    reg [31:0] tx_word=0;
    wire [2:0] cs_n;
    wire sclk, mosi, busy, done;
    wire [31:0] rx0,rx1,rx2;
    reg [2:0] miso=0;
    localparam [31:0] R0=32'h89abcdef, R1=32'h76543210, R2=32'h80000001;
    spi_master_3ch #(.SPI_HZ(SPI_HZ)) dut(.*);
    integer rises=0, falls=0, completed=0;
    reg [31:0] captured=0, expected_tx;
    realtime cs_low, cs_high=-1000, last_edge, last_rise=-1000, last_mosi=-1000;
    task check(input bit ok, input string msg);
        if (!ok) begin $display("FAIL: SPI %0d %s",SPI_HZ,msg); $fatal(1); end
    endtask
    always @(mosi) if (rst_n && cs_n==0) begin
        if (rises>0) check($realtime-last_rise>=49,"MOSI hold");
        last_mosi=$realtime;
    end
    always @(negedge cs_n[0]) if (rst_n) begin
        check(cs_n===3'b000,"simultaneous CS assertion");
        check(sclk===0,"idle polarity");
        check($realtime-cs_high>=200,"CS high gap");
        cs_low=$realtime; last_edge=$realtime;
        rises=0; falls=0; captured=0; expected_tx=tx_word;
        // Exercise the maximum first-bit output access delay (20 ns).
        #20 miso={R2[31],R1[31],R0[31]};
    end
    always @(posedge sclk) if (rst_n) begin
        check(cs_n===0,"clock while selected");
        check($realtime-last_edge==HALF_NS,"low half period");
        check($realtime-last_mosi>=37,"MOSI setup");
        if (rises==0) check($realtime-cs_low>=HALF_NS,"CS lead");
        check(rises<32,"too many clocks");
        captured={captured[30:0],mosi}; rises=rises+1;
        last_edge=$realtime; last_rise=$realtime;
    end
    always @(negedge sclk) if (rst_n && cs_n==0) begin
        check($realtime-last_edge==HALF_NS,"high half period");
        falls=falls+1; last_edge=$realtime;
        // Model tV=40 ns, rather than zero-delay MISO changes.
        if (falls<32) begin
            #40 miso={R2[31-falls],R1[31-falls],R0[31-falls]};
        end
    end
    always @(posedge cs_n[0]) begin
        cs_high=$realtime;
        if (rst_n) begin
            check(cs_n===3'b111,"simultaneous CS release");
            check(rises==32 && falls==32,"exactly 32 clocks");
            check($realtime-last_edge>=HALF_NS,"CS lag");
            check(captured===expected_tx,"MSB first transmitted word");
            completed=completed+1;
        end
    end
    task exchange(input [31:0] word);
        @(negedge clk); tx_word=word; start=1;
        @(negedge clk); start=0;
        wait(busy);
        // A pulse while busy must not queue a second transfer.
        repeat(3) @(negedge clk); start=1;
        @(negedge clk); start=0;
        wait(done); #1;
        check(rx0===R0 && rx1===R1 && rx2===R2,"received words / last bit");
        @(posedge clk); #1; check(done==0,"single-cycle done");
    endtask
    initial begin
        repeat(30) @(negedge clk); rst_n=1;
        check(cs_n===7 && sclk===0 && mosi===0,"reset pins");
        exchange(32'ha55a1234);
        // Request immediately following done, then check output retention.
        exchange(32'h80000001);
        repeat(30) @(negedge clk);
        check(completed==2 && rx0==R0,"busy request ignored / results retained");
        @(negedge clk); start=1;
        @(negedge clk); start=0;
        repeat(15) @(negedge clk); rst_n=0;
        #1; check(cs_n===7 && sclk===0 && !busy && !done,"abort reset");
        repeat(30) @(negedge clk); rst_n=1;
        exchange(32'hffffffff);
        check(completed==3,"recovery");
        finished=1;
    end
endmodule

module tb_spi_master_3ch;
    wire f0,f1,f2;
    spi_check a(f0);
    spi_check #(.SPI_HZ(6000000),.HALF_NS(90)) b(f1);
    spi_large_gap c(f2);
    initial begin
        wait(f0 && f1 && f2);
        if ($test$plusargs("INJECT_FAILURE")) begin
            $display("FAIL: intentional runner check"); $finish;
        end
        $display("PASS: tb_spi_master_3ch"); $finish;
    end
    initial begin #100000; $display("FAIL: SPI watchdog"); $fatal(1); end
endmodule

// Prove that a large legal positive gap is not truncated to one cycle.
// Only observe its early busy interval; simulating the full 2 seconds is unnecessary.
module spi_large_gap(output reg finished=0);
    reg clk=0,rst_n=0,start=0;
    always #0.25 clk=~clk;
    wire [2:0] cs_n;
    wire busy,done;
    spi_master_3ch #(.CLK_HZ(2000000000),.CS_HIGH_NS(2000000000)) dut(
        .clk(clk),.rst_n(rst_n),.start(start),.tx_word(32'b0),.miso(3'b0),
        .cs_n(cs_n),.busy(busy),.done(done));
    initial begin
        #10; @(negedge clk); rst_n=1; start=1;
        @(negedge clk); start=0;
        @(posedge cs_n[0]);
        repeat(1000) @(negedge clk);
        if(!busy || done) begin $display("FAIL: large positive CS gap truncated"); $fatal(1); end
        finished=1;
    end
endmodule
