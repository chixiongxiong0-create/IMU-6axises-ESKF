`timescale 1ns/1ps
// Three ADXRS453s share MOSI/SCLK; MISO and CS remain separate pins.
// frame_valid reports completion, gyro_valid reports protocol/startup health.
// Neither signal proves a new internal MEMS conversion (no DRDY is available).
module adxrs453_acq #(
    parameter integer CLK_HZ=100000000,
    parameter integer SPI_HZ=5000000,
    parameter integer SAMPLE_HZ=485
)(
    input wire clk, input wire rst_n, input wire [2:0] miso,
    output wire sclk, output wire mosi, output wire [2:0] cs_n,
    output reg signed [15:0] gyro0, output reg signed [15:0] gyro1,
    output reg signed [15:0] gyro2,
    output reg [31:0] raw0, output reg [31:0] raw1, output reg [31:0] raw2,
    output reg [63:0] timestamp_cycles,
    output reg [2:0] frame_seq, output reg frame_valid,
    output reg [2:0] gyro_valid,
    output reg [4:0] err0, output reg [4:0] err1, output reg [4:0] err2,
    output reg startup_done, output reg [2:0] startup_ok, output reg overrun
);
    localparam integer WAIT100_RAW=(64'd1*CLK_HZ+9)/10;
    localparam integer WAIT50_RAW=(64'd1*CLK_HZ+19)/20;
    localparam integer WAIT100=(WAIT100_RAW>0)?WAIT100_RAW:1;
    localparam integer WAIT50=(WAIT50_RAW>0)?WAIT50_RAW:1;
    localparam [4:0] POWER_WAIT=0, CHK_ISSUE=1, CHK_WAIT=2,
        BUILD_WAIT=3, CLEAR_ISSUE=4, CLEAR_WAIT=5, DECAY_WAIT=6,
        REPORT1_ISSUE=7, REPORT1_WAIT=8, REPORT2_ISSUE=9, REPORT2_WAIT=10,
        RESTORE_ISSUE=11, RESTORE_WAIT=12, RUN_IDLE=13,
        A_ISSUE=14, A_WAIT=15, B_ISSUE=16, B_WAIT=17;
    reg [4:0] state;
    reg [31:0] wait_count;
    reg [2:0] startup_checks, tick_seq, active_seq;
    reg [63:0] cycles, latch_stamp, phase;
    wire [63:0] phase_sum=phase+((SAMPLE_HZ>0)?SAMPLE_HZ:1);
    wire sample_tick=startup_done && (phase_sum>=CLK_HZ);
    wire spi_start=(state==CHK_ISSUE || state==CLEAR_ISSUE ||
        state==REPORT1_ISSUE || state==REPORT2_ISSUE || state==RESTORE_ISSUE ||
        state==A_ISSUE || state==B_ISSUE);
    wire spi_busy, spi_done;
    wire [31:0] spi_rx0, spi_rx1, spi_rx2;

    function [31:0] sensor_command;
        input [2:0] sq;
        reg [31:0] word;
        begin
            word=32'h20000000;
            word[28]=sq[2]; word[31:30]=sq[1:0];
            word[0]=~^word[31:1]; sensor_command=word;
        end
    endfunction
    wire [31:0] spi_tx=(state==CHK_ISSUE)?32'h20000003:
        ((state==A_ISSUE || state==B_ISSUE)?sensor_command(active_seq):32'h20000000);

    spi_master_3ch #(.CLK_HZ(CLK_HZ),.SPI_HZ(SPI_HZ)) spi(
        .clk(clk),.rst_n(rst_n),.start(spi_start),.tx_word(spi_tx),.miso(miso),
        .sclk(sclk),.mosi(mosi),.cs_n(cs_n),.busy(spi_busy),.done(spi_done),
        .rx0(spi_rx0),.rx1(spi_rx1),.rx2(spi_rx2));

    function [4:0] response_errors;
        input [31:0] word; input [2:0] sq;
        begin
            response_errors[0]=!(^word[31:16]);
            response_errors[1]=!(^word);
            response_errors[2]=(word[31:29]!=sq);
            response_errors[3]=(word[27:26]!=2'b01);
            response_errors[4]=|word[7:1];
        end
    endfunction
    function chk_report_ok;
        input [31:0] word;
        begin
            chk_report_ok=(^word[31:16]) && (^word) &&
                word[31:29]==3'b000 && word[27:26]==2'b10 && word[7:1]==7'h7f;
        end
    endfunction
    wire [4:0] e0=response_errors(spi_rx0,active_seq);
    wire [4:0] e1=response_errors(spi_rx1,active_seq);
    wire [4:0] e2=response_errors(spi_rx2,active_seq);
    wire [2:0] chk_ok={chk_report_ok(spi_rx2),chk_report_ok(spi_rx1),chk_report_ok(spi_rx0)};
    wire [2:0] restore_ok={response_errors(spi_rx2,3'b000)==0,
        response_errors(spi_rx1,3'b000)==0,response_errors(spi_rx0,3'b000)==0};

    // Conservative complete A/B budget, including start/done handshakes.
    localparam integer SAFE_SPI=(SPI_HZ>0)?SPI_HZ:1;
    localparam integer SAFE_SAMPLE=(SAMPLE_HZ>0)?SAMPLE_HZ:1;
    localparam [63:0] HALF_CYCLES=(64'd1*CLK_HZ+64'd2*SAFE_SPI-1)/(64'd2*SAFE_SPI);
    localparam [63:0] GAP_CYCLES=(64'd1*CLK_HZ*200+999999999)/1000000000;
    localparam [63:0] FRAME_CYCLES=2*(65*HALF_CYCLES+GAP_CYCLES)+8;
    // synthesis translate_off
    initial begin
        if (CLK_HZ<=0 || SPI_HZ<=0 || SAMPLE_HZ<=0 ||
            (64'd1*CLK_HZ/SAFE_SAMPLE < FRAME_CYCLES)) begin
            $display("PARAM_REJECT: %m CLK_HZ=%0d SPI_HZ=%0d SAMPLE_HZ=%0d (A/B budget %0d cycles)",
                CLK_HZ,SPI_HZ,SAMPLE_HZ,FRAME_CYCLES);
            #1 $finish;
        end
    end
    // synthesis translate_on

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state<=POWER_WAIT; wait_count<=WAIT100-1; startup_checks<=7;
            startup_done<=0; startup_ok<=0; overrun<=0;
            cycles<=0; phase<=0; tick_seq<=0; active_seq<=0; latch_stamp<=0;
            gyro0<=0; gyro1<=0; gyro2<=0; raw0<=0; raw1<=0; raw2<=0;
            timestamp_cycles<=0; frame_seq<=0; frame_valid<=0; gyro_valid<=0;
            err0<=0; err1<=0; err2<=0;
        end else begin
            cycles<=cycles+1;
            frame_valid<=0;
            if(startup_done) begin
                if(sample_tick) begin
                    phase<=phase_sum-CLK_HZ; tick_seq<=tick_seq+1;
                    if(state!=RUN_IDLE) overrun<=1;
                end else phase<=phase_sum;
            end else begin phase<=0; tick_seq<=0; end
            case(state)
                POWER_WAIT: if(wait_count!=0) wait_count<=wait_count-1;
                    else state<=CHK_ISSUE;
                CHK_ISSUE: if(!spi_busy) state<=CHK_WAIT;
                CHK_WAIT: if(spi_done) begin wait_count<=WAIT50-1; state<=BUILD_WAIT; end
                BUILD_WAIT: if(wait_count!=0) wait_count<=wait_count-1;
                    else state<=CLEAR_ISSUE;
                CLEAR_ISSUE: if(!spi_busy) state<=CLEAR_WAIT;
                CLEAR_WAIT: if(spi_done) begin wait_count<=WAIT50-1; state<=DECAY_WAIT; end
                DECAY_WAIT: if(wait_count!=0) wait_count<=wait_count-1;
                    else state<=REPORT1_ISSUE;
                REPORT1_ISSUE: if(!spi_busy) state<=REPORT1_WAIT;
                REPORT1_WAIT: if(spi_done) begin
                    startup_checks<=startup_checks & chk_ok; state<=REPORT2_ISSUE;
                end
                REPORT2_ISSUE: if(!spi_busy) state<=REPORT2_WAIT;
                REPORT2_WAIT: if(spi_done) begin
                    startup_checks<=startup_checks & chk_ok; state<=RESTORE_ISSUE;
                end
                RESTORE_ISSUE: if(!spi_busy) state<=RESTORE_WAIT;
                RESTORE_WAIT: if(spi_done) begin
                    startup_ok<=startup_checks & restore_ok;
                    startup_done<=1; state<=RUN_IDLE;
                end
                RUN_IDLE: if(sample_tick) begin active_seq<=tick_seq; state<=A_ISSUE; end
                A_ISSUE: if(!spi_busy) begin
                    // The engine consumes start and asserts CS on this same edge.
                    // cycles+1 denotes this edge in the post-edge free-running counter.
                    latch_stamp<=cycles+1; state<=A_WAIT;
                end
                A_WAIT: if(spi_done) state<=B_ISSUE;
                B_ISSUE: if(!spi_busy) state<=B_WAIT;
                B_WAIT: if(spi_done) begin
                    raw0<=spi_rx0; raw1<=spi_rx1; raw2<=spi_rx2;
                    gyro0<=spi_rx0[25:10]; gyro1<=spi_rx1[25:10]; gyro2<=spi_rx2[25:10];
                    err0<=e0; err1<=e1; err2<=e2;
                    gyro_valid<=startup_ok & {e2==0,e1==0,e0==0};
                    timestamp_cycles<=latch_stamp; frame_seq<=active_seq;
                    frame_valid<=1; state<=RUN_IDLE;
                end
                default: begin startup_done<=0; startup_ok<=0;
                    state<=POWER_WAIT; wait_count<=WAIT100-1; startup_checks<=7; end
            endcase
        end
    end
endmodule
