`timescale 1ns/1ps
module tb_adxrs453_invalid;
    reg clk=0;
    always #5 clk=~clk;
    // Each diagnostic must name its offending instance. The runner checks all eleven.
    spi_master_3ch #(.CLK_HZ(0)) c_clk_zero(.clk(clk),.rst_n(1'b0),.start(1'b0),.tx_word(32'b0),.miso(3'b0));
    spi_master_3ch #(.CLK_HZ(-1)) c_clk_negative(.clk(clk),.rst_n(1'b0),.start(1'b0),.tx_word(32'b0),.miso(3'b0));
    spi_master_3ch #(.SPI_HZ(0)) c_spi_zero(.clk(clk),.rst_n(1'b0),.start(1'b0),.tx_word(32'b0),.miso(3'b0));
    spi_master_3ch #(.SPI_HZ(-1)) c_spi_negative(.clk(clk),.rst_n(1'b0),.start(1'b0),.tx_word(32'b0),.miso(3'b0));
    spi_master_3ch #(.SPI_HZ(9000000)) c_spi_high(.clk(clk),.rst_n(1'b0),.start(1'b0),.tx_word(32'b0),.miso(3'b0));
    spi_master_3ch #(.SPI_HZ(20000000)) c_half_short(.clk(clk),.rst_n(1'b0),.start(1'b0),.tx_word(32'b0),.miso(3'b0));
    spi_master_3ch #(.CS_HIGH_NS(50)) c_gap_short(.clk(clk),.rst_n(1'b0),.start(1'b0),.tx_word(32'b0),.miso(3'b0));
    spi_master_3ch #(.CLK_HZ(2147483647),.CS_HIGH_NS(2147483647)) c_gap_capacity(.clk(clk),.rst_n(1'b0),.start(1'b0),.tx_word(32'b0),.miso(3'b0));
    adxrs453_acq #(.SAMPLE_HZ(0)) c_sample_zero(.clk(clk),.rst_n(1'b0),.miso(3'b0));
    adxrs453_acq #(.SAMPLE_HZ(-1)) c_sample_negative(.clk(clk),.rst_n(1'b0),.miso(3'b0));
    adxrs453_acq #(.SAMPLE_HZ(100000)) c_capacity(.clk(clk),.rst_n(1'b0),.miso(3'b0));
    initial begin #1000; $display("FAIL: invalid parameters were accepted"); $finish; end
endmodule
