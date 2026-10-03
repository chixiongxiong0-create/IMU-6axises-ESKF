`timescale 1ns/1ps
// Behavioral ADXRS453 Rev. B, deliberately independent of the RTL FSM/functions.
module adxrs453_model(
    input wire power_reset_n, input wire cs_n, input wire sclk, input wire mosi,
    output reg miso=0,
    input wire [15:0] rate,
    input wire [31:0] flip_mask,
    input wire override_sq, input wire [2:0] sq_value,
    input wire override_st, input wire [1:0] st_value,
    input wire [6:0] fault_value
);
    reg [31:0] pending=32'h00000001, shifted, command;
    reg [15:0] latched_rate;
    reg [1:0] latched_status;
    reg [6:0] latched_fault;
    reg [31:0] latched_flip;
    reg latched_sq_en, latched_st_en;
    reg [2:0] latched_sq;
    reg [1:0] latched_st;
    reg chk=0, forcing=0, sticky=0;
    realtime chk_time=0, clear_time=0;
    integer count=0;
    reg [31:0] next_response;
    reg odd;
    integer j;
    always @(negedge power_reset_n) begin
        pending=32'h00000001; miso=0; chk=0; forcing=0; sticky=0; count=0;
    end
    always @(negedge cs_n) if (power_reset_n) begin
        if (chk && $realtime-chk_time>=50000000) forcing=1;
        if (!chk && forcing && $realtime-clear_time>=50000000) forcing=0;
        if (forcing) sticky=1;
        latched_rate=rate;
        latched_status=(forcing || sticky) ? 2'b10 : 2'b01;
        latched_fault=((forcing || sticky) ? 7'h7f : 7'h00) | fault_value;
        latched_flip=flip_mask; latched_sq_en=override_sq; latched_sq=sq_value;
        latched_st_en=override_st; latched_st=st_value;
        shifted=pending; command=0; count=0;
        #20 miso=shifted[31];
    end
    always @(posedge sclk) if (!cs_n && power_reset_n) begin
        command={command[30:0],mosi}; count=count+1;
    end
    always @(negedge sclk) if (!cs_n && power_reset_n && count<32) begin
        shifted=shifted<<1;
        #40 miso=shifted[31];
    end
    always @(posedge cs_n) if (power_reset_n && count!=0) begin
        if (count!=32) begin
            // A controller reset can abort a word while sensor power remains on.
            // Real silicon queues a R/W error (SPI=1), not a healthy rate response.
            pending=32'h1e040000; count=0;
        end else begin
        if (^command!==1'b1 || command[29]!==1'b1 ||
            command[27:2]!==26'b0) begin
            $display("FAIL: model rejected command %h, count %0d",command,count);
            $fatal(1);
        end
        next_response=0;
        next_response[31:29]=latched_sq_en ? latched_sq : {command[28],command[31:30]};
        next_response[27:26]=latched_st_en ? latched_st : latched_status;
        next_response[25:10]=latched_rate;
        next_response[7:1]=latched_fault;
        // Set P0, then P1 via independent bit-by-bit counting.
        odd=0; for(j=16;j<32;j=j+1) odd=odd^next_response[j];
        next_response[28]=!odd;
        odd=0; for(j=1;j<32;j=j+1) odd=odd^next_response[j];
        next_response[0]=!odd;
        pending=next_response ^ latched_flip;
        if (!forcing) sticky=0;
        if (command[1] && !chk) begin chk=1; chk_time=$realtime; end
        if (!command[1] && chk) begin chk=0; clear_time=$realtime; end
        count=0;
        end
        #40 miso=0;
    end
endmodule
