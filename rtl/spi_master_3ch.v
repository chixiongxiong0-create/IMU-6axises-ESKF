`timescale 1ns/1ps
// Mode 0, MSB first, exactly 32 clocks; one transmitter, three receivers.
// rst_n may assert asynchronously; its release must be synchronized by the caller.
module spi_master_3ch #(
    parameter integer CLK_HZ = 100000000,
    parameter integer SPI_HZ = 5000000,
    parameter integer CS_HIGH_NS = 200
)(
    input wire clk, input wire rst_n,
    input wire start, input wire [31:0] tx_word,
    input wire [2:0] miso,
    output reg sclk, output reg mosi, output reg [2:0] cs_n,
    output reg busy, output reg done,
    output reg [31:0] rx0, output reg [31:0] rx1, output reg [31:0] rx2
);
    // 64-bit constant arithmetic avoids overflow in ns-to-cycle conversion.
    localparam integer SAFE_SPI = (SPI_HZ > 0) ? SPI_HZ : 1;
    localparam integer HALF_RAW = (64'd1 * CLK_HZ + 64'd2*SAFE_SPI-1)/(64'd2*SAFE_SPI);
    localparam integer HALF = (HALF_RAW > 0) ? HALF_RAW : 1;
    localparam [63:0] GAP_RAW = (64'd1 * CLK_HZ * CS_HIGH_NS + 999999999)/1000000000;
    localparam [63:0] GAP = (GAP_RAW > 0) ? GAP_RAW : 1;
    localparam [1:0] IDLE=0, SHIFT=1, LAG=2, HIGH_GAP=3;
    reg [1:0] state;
    reg [31:0] timer;
    reg [4:0] bit_index;
    reg [31:0] tx_shift, r0, r1, r2;

    // synthesis translate_off
    initial begin
        if (CLK_HZ<=0 || SPI_HZ<=0 ||
            (64'd1*HALF*1000000000 < 64'd49*CLK_HZ) ||
            SPI_HZ>8080000 || CS_HIGH_NS<100 || GAP>64'd4294967296) begin
            $display("PARAM_REJECT: %m CLK_HZ=%0d SPI_HZ=%0d CS_HIGH_NS=%0d",CLK_HZ,SPI_HZ,CS_HIGH_NS);
            #1 $finish;
        end
    end
    // synthesis translate_on

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state<=IDLE; timer<=0; bit_index<=0;
            tx_shift<=0; r0<=0; r1<=0; r2<=0;
            sclk<=0; mosi<=0; cs_n<=3'b111; busy<=0; done<=0;
            rx0<=0; rx1<=0; rx2<=0;
        end else begin
            done<=0;
            case (state)
                IDLE: if (start) begin
                    tx_shift<=tx_word; mosi<=tx_word[31]; cs_n<=0;
                    r0<=0; r1<=0; r2<=0; bit_index<=0;
                    busy<=1; timer<=HALF-1; state<=SHIFT;
                end
                SHIFT: if (timer!=0) timer<=timer-1; else begin
                    timer<=HALF-1;
                    if (!sclk) begin
                        sclk<=1;
                        r0<={r0[30:0],miso[0]};
                        r1<={r1[30:0],miso[1]};
                        r2<={r2[30:0],miso[2]};
                    end else begin
                        sclk<=0;
                        if (bit_index==31) state<=LAG;
                        else begin
                            bit_index<=bit_index+1;
                            tx_shift<={tx_shift[30:0],1'b0}; mosi<=tx_shift[30];
                        end
                    end
                end
                LAG: if (timer!=0) timer<=timer-1; else begin
                    cs_n<=3'b111; mosi<=0;
                    rx0<=r0; rx1<=r1; rx2<=r2;
                    timer<=GAP-1; state<=HIGH_GAP;
                end
                HIGH_GAP: if (timer!=0) timer<=timer-1; else begin
                    busy<=0; done<=1; state<=IDLE;
                end
                default: begin state<=IDLE; sclk<=0; mosi<=0; cs_n<=7; busy<=0; end
            endcase
        end
    end
endmodule
